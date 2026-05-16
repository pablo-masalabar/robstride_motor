#!/usr/bin/env python3

import json
import os
import time
import tomllib
from os import environ
from threading import Thread

environ['PYGAME_HIDE_SUPPORT_PROMPT'] = '1'

import pygame
import socketio
from wsgiref.simple_server import WSGIServer, WSGIRequestHandler
from socketserver import ThreadingMixIn
import rclpy
import rclpy.logging
from rclpy.node import Node
from sensor_msgs.msg import Joy


def remap(x: float) -> float:
    return (max(-1.0, min(1.0, x)) + 1) / 2


class _ThreadingWSGIServer(ThreadingMixIn, WSGIServer):
    daemon_threads = True


class _SilentHandler(WSGIRequestHandler):
    def log_message(self, format, *args):
        pass


class WebSocketServer:
    def __init__(self, host: str, port: int, logger, gamepad_callback):
        self.host = host
        self.port = port
        self.logger = logger
        self.gamepad_callback = gamepad_callback
        self.sio = socketio.Server(cors_allowed_origins='*', async_mode='threading')
        self.app = socketio.WSGIApp(self.sio)
        self.connected_clients = set()
        self.running = True
        self._server = None

        @self.sio.event
        def connect(sid, environ):
            self.logger.info(f'Client connected: {sid}')
            self.connected_clients.add(sid)

        @self.sio.event
        def disconnect(sid):
            self.logger.info(f'Client disconnected: {sid}')
            self.connected_clients.discard(sid)

        @self.sio.on('gamepad_data')
        def on_gamepad_data(sid, data):
            if self.gamepad_callback:
                self.gamepad_callback(data)

    def start(self):
        self.logger.info(f'Starting WebSocket server on {self.host}:{self.port}')
        Thread(target=self._run, daemon=True).start()

    def _run(self):
        try:
            self._server = _ThreadingWSGIServer((self.host, self.port), _SilentHandler)
            self._server.set_app(self.app)
            self._server.serve_forever()
        except Exception as e:
            if self.running:
                self.logger.error(f'Server error: {e}')

    def stop(self):
        self.running = False
        for sid in list(self.connected_clients):
            self.sio.disconnect(sid)
        if self._server:
            self._server.shutdown()
        self.sio.shutdown()


class WebSocketClient:
    def __init__(self, server_url: str, logger):
        self.server_url = server_url
        self.logger = logger
        self.sio = socketio.Client(
            reconnection=True,
            reconnection_attempts=0,      # infinite retries
            reconnection_delay=1,         # start with 1s delay
            reconnection_delay_max=5,     # cap at 5s
        )
        self.connected = False

        @self.sio.event
        def connect():
            self.logger.info(f'Connected to server: {self.server_url}')
            self.connected = True

        @self.sio.event
        def disconnect():
            self.logger.info('Disconnected from server — will reconnect automatically')
            self.connected = False

        @self.sio.event
        def connect_error(data):
            self.logger.warn(f'Connection error: {data}')

    def connect(self):
        try:
            self.sio.connect(self.server_url, wait_timeout=10)
            return True
        except Exception as e:
            self.logger.error(f'Failed to connect to server: {e}')
            return False

    def send_gamepad_data(self, data: str):
        if self.connected:
            self.sio.emit('gamepad_data', data)

    def disconnect(self):
        try:
            self.sio.disconnect()
        except Exception:
            pass


class RemoteJoystickNode(Node):
    # Buttons array layout:
    #   [0]=A [1]=B [2]=Y [3]=X [4]=LB [5]=RB [6]=? [7]=? [8]=back
    #   [9]=start [10]=? [11]=leftStickClick [12]=rightStickClick
    #   [13]=dpadLeft [14]=dpadRight [15]=dpadUp [16]=dpadDown
    _NUM_BUTTONS = 17

    def __init__(self):
        super().__init__('remote_joystick_node')

        self.declare_parameter('config_path', '')
        config_path = self.get_parameter('config_path').get_parameter_value().string_value
        if not config_path:
            raise RuntimeError('config_path parameter is required')

        with open(config_path, 'rb') as f:
            cfg = tomllib.load(f)

        self._hz = max(20.0, float(cfg.get('hz', 20.0)))
        self._debug = cfg.get('debug', False)
        if self._debug:
            self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        self._joy_topic = cfg.get('joy_topic', 'joy')
        self._mode = cfg.get('mode', 'standalone')
        self._listen_host = cfg.get('listen_host', '0.0.0.0')
        self._listen_port = int(cfg.get('listen_port', 8765))
        self._server_url = cfg.get('server_url', '')

        if self._mode not in ('standalone', 'server', 'client'):
            raise RuntimeError(f"Invalid mode '{self._mode}': must be standalone, server, or client")
        if self._mode == 'client' and not self._server_url:
            raise RuntimeError("mode=client requires server_url in config")

        self._joy_pub = self.create_publisher(Joy, self._joy_topic, 10)

        # axes: [leftStickX, leftStickY, leftTrigger, rightStickX, rightStickY, rightTrigger]
        self._axes = [0.0] * 6
        self._buttons = [0] * self._NUM_BUTTONS

        # Server mode: last data received from WebSocket client
        self._recv_axes = None
        self._recv_buttons = None
        self._last_recv_time = 0.0

        self._ws_server = None
        self._ws_client = None
        self._running = True

        Thread(target=self._run, daemon=True).start()

        self.get_logger().info(f'RemoteJoystickNode started in {self._mode} mode at {self._hz} Hz')

    def _on_gamepad_received(self, data: str):
        try:
            d = json.loads(data)
            self._recv_axes = d['axes']
            self._recv_buttons = d['buttons']
            self._last_recv_time = time.time()
            if self._debug:
                self.get_logger().debug('Received gamepad data from client')
        except Exception as e:
            self.get_logger().error(f'Error parsing received gamepad data: {e}')

    def _publish_joy(self, axes, buttons):
        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.axes = [float(a) for a in axes]
        msg.buttons = [int(b) for b in buttons]
        self._joy_pub.publish(msg)

    def _run(self):
        if self._mode == 'server':
            self._ws_server = WebSocketServer(
                self._listen_host, self._listen_port,
                self.get_logger(), self._on_gamepad_received,
            )
            self._ws_server.start()
        elif self._mode == 'client':
            self._ws_client = WebSocketClient(self._server_url, self.get_logger())
            if not self._ws_client.connect():
                self.get_logger().error('Failed to connect to WebSocket server')

        if self._mode != 'server':
            os.environ['SDL_VIDEODRIVER'] = 'dummy'
            os.environ['SDL_AUDIODRIVER'] = 'dsp'
            pygame.init()
            pygame.joystick.init()

        joystick_ok = False

        if self._mode != 'server':
            count = pygame.joystick.get_count()
            if count == 0:
                self.get_logger().info('No joysticks connected')
            else:
                self.get_logger().info(f'{count} joystick(s) detected')
                joystick = pygame.joystick.Joystick(0)
                joystick.init()
                self.get_logger().info(f'Initialized joystick: {joystick.get_name()}')
                joystick_ok = True

        period = 1.0 / self._hz

        try:
            while self._running:
                t0 = time.monotonic()

                if self._mode != 'server':
                    for event in pygame.event.get():
                        if event.type == pygame.JOYDEVICEADDED:
                            if pygame.joystick.get_count() > 0:
                                j = pygame.joystick.Joystick(0)
                                j.init()
                                self.get_logger().info(f'Joystick connected: {j.get_name()}')
                                joystick_ok = True
                        elif event.type == pygame.JOYDEVICEREMOVED:
                            self.get_logger().warn('Joystick disconnected')
                            joystick_ok = False
                        elif event.type == pygame.JOYAXISMOTION:
                            if event.axis == 0:
                                self._axes[0] = event.value       # leftStickX
                            elif event.axis == 1:
                                self._axes[1] = event.value       # leftStickY
                            elif event.axis == 2:
                                self._axes[2] = remap(event.value) # leftTrigger
                            elif event.axis == 3:
                                self._axes[3] = event.value       # rightStickX
                            elif event.axis == 4:
                                self._axes[4] = event.value       # rightStickY
                            elif event.axis == 5:
                                self._axes[5] = remap(event.value) # rightTrigger
                        elif event.type == pygame.JOYBUTTONDOWN:
                            if event.button < 13:
                                self._buttons[event.button] = 1
                        elif event.type == pygame.JOYBUTTONUP:
                            if event.button < 13:
                                self._buttons[event.button] = 0
                        elif event.type == pygame.JOYHATMOTION:
                            self._buttons[13] = 1 if event.value[0] == -1 else 0  # dpadLeft
                            self._buttons[14] = 1 if event.value[0] == 1 else 0   # dpadRight
                            self._buttons[15] = 1 if event.value[1] == 1 else 0   # dpadUp
                            self._buttons[16] = 1 if event.value[1] == -1 else 0  # dpadDown

                if self._mode == 'server':
                    if self._recv_axes and (time.time() - self._last_recv_time < 1.0):
                        self._publish_joy(self._recv_axes, self._recv_buttons)
                        if self._debug:
                            self.get_logger().debug(f'axes={self._recv_axes}')
                    else:
                        if self._debug:
                            self.get_logger().debug('No recent gamepad data from clients')
                elif joystick_ok:
                    self._publish_joy(self._axes, self._buttons)
                    if self._ws_client:
                        data = json.dumps({'axes': self._axes, 'buttons': self._buttons})
                        self._ws_client.send_gamepad_data(data)
                    if self._debug:
                        self.get_logger().debug(
                            f'axes={self._axes} buttons={self._buttons}')

                elapsed = time.monotonic() - t0
                remaining = period - elapsed
                if remaining > 0:
                    time.sleep(remaining)

        except Exception as e:
            self.get_logger().error(f'Error in gamepad loop: {e}')
        finally:
            if self._ws_server:
                self._ws_server.stop()
            if self._ws_client:
                self._ws_client.disconnect()
            if self._mode != 'server':
                pygame.quit()

    def shutdown_cleanup(self):
        self._running = False
        self.get_logger().info('RemoteJoystickNode shutting down')


def main(args=None):
    rclpy.init(args=args)
    node = RemoteJoystickNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_cleanup()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
