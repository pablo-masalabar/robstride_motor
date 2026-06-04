"""
realsense_node.py – ROS 2 node for the Intel RealSense D405 camera.

ROS topics published
--------------------
  /{node_name}/color/image_raw          sensor_msgs/Image        BGR8
  /{node_name}/color/image_rect         sensor_msgs/Image        rectified BGR8 (if enabled)
  /{node_name}/color/camera_info        sensor_msgs/CameraInfo
  /{node_name}/depth/image_raw          sensor_msgs/Image        float32 metres

Config (`config/config.toml`)
------------------------------
  node_name                    str    ROS node name and topic namespace
  serial                       str    device serial; empty = first available
  enable_color                 bool   enable RGB stream (default true)
  enable_depth                 bool   enable depth stream (default true)
  color_fps                    float  colour frame rate (default 30.0)
  depth_fps                    float  depth frame rate (default 30.0)
  color_resolution             str    "480P" or "720P" (default "720P")
  depth_resolution             str    "480P" or "720P" (default "720P")
  depth_min_m                  float  min depth threshold in metres (default 0.07)
  depth_max_m                  float  max depth threshold in metres (default 0.50)
  align_to_color               bool   align depth to colour frame (default true)
  enable_color_rectification   bool   publish undistorted colour (default false)
  color_rectification_alpha    float  0.0 = no black borders, 1.0 = full FOV
  color_rectification_crop     bool   crop rectified image to valid ROI
  disconnect_timeout_s         float  seconds of consecutive timeouts before shutdown (default 5.0)
"""

import signal
import threading
import tomllib
import traceback

import cv2
import numpy as np
import pyrealsense2 as rs

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import CameraInfo, Image


# ── Constants ─────────────────────────────────────────────────────────────────

_RESOLUTION_MAP = {
    '480P': (848, 480),
    '720P': (1280, 720),
}

_RS_DISTORTION_MODEL = {
    rs.distortion.brown_conrady:          'plumb_bob',
    rs.distortion.modified_brown_conrady: 'plumb_bob',
    rs.distortion.inverse_brown_conrady:  'plumb_bob',
    rs.distortion.kannala_brandt4:        'equidistant',
}


# ── Helpers ───────────────────────────────────────────────────────────────────

def _np_to_ros_image(arr: np.ndarray, encoding: str, stamp, frame_id: str = '') -> Image:
    msg                 = Image()
    msg.header.stamp    = stamp
    msg.header.frame_id = frame_id
    if arr.ndim == 2:
        msg.height, msg.width = arr.shape
        msg.step = arr.shape[1] * arr.itemsize
    else:
        msg.height, msg.width, _ = arr.shape
        msg.step = arr.shape[1] * arr.shape[2] * arr.itemsize
    msg.encoding     = encoding
    msg.is_bigendian = False
    msg.data         = arr.tobytes()
    return msg


def _rs_intrinsics_to_camera_info(intr: rs.intrinsics, stamp, frame_id: str = '') -> CameraInfo:
    msg                 = CameraInfo()
    msg.header.stamp    = stamp
    msg.header.frame_id = frame_id
    msg.width           = intr.width
    msg.height          = intr.height
    msg.distortion_model = _RS_DISTORTION_MODEL.get(intr.model, 'plumb_bob')
    msg.d = list(intr.coeffs)
    msg.k = [
        intr.fx, 0.0,     intr.ppx,
        0.0,     intr.fy, intr.ppy,
        0.0,     0.0,     1.0,
    ]
    msg.r = [1.0, 0.0, 0.0,
             0.0, 1.0, 0.0,
             0.0, 0.0, 1.0]
    msg.p = [
        intr.fx, 0.0,     intr.ppx, 0.0,
        0.0,     intr.fy, intr.ppy, 0.0,
        0.0,     0.0,     1.0,      0.0,
    ]
    return msg


# ── Node ──────────────────────────────────────────────────────────────────────

class RealSenseNode(Node):

    def __init__(self):
        super().__init__('realsense_node')

        self.declare_parameter('config_path', '')
        config_path = self.get_parameter('config_path').value
        if not config_path:
            self.get_logger().fatal('config_path parameter is required')
            raise RuntimeError('config_path parameter is required')

        cfg = self._load_config(config_path)

        self._node_name:                  str   = cfg.get('node_name', 'realsense')
        self._serial:                     str   = cfg.get('serial', '')
        self._enable_color:               bool  = bool(cfg.get('enable_color', True))
        self._enable_depth:               bool  = bool(cfg.get('enable_depth', True))
        self._color_fps:                  float = float(cfg.get('color_fps', 30.0))
        self._depth_fps:                  float = float(cfg.get('depth_fps', 30.0))
        self._color_resolution:           tuple = _RESOLUTION_MAP[cfg.get('color_resolution', '720P')]
        self._depth_resolution:           tuple = _RESOLUTION_MAP[cfg.get('depth_resolution', '720P')]
        self._enable_color_rectification: bool  = bool(cfg.get('enable_color_rectification', False))
        self._color_rectification_alpha:  float = float(cfg.get('color_rectification_alpha', 0.0))
        self._color_rectification_crop:   bool  = bool(cfg.get('color_rectification_crop', False))
        self._align_to_color:             bool  = bool(cfg.get('align_to_color', True))
        self._depth_min_m:                float = float(cfg.get('depth_min_m', 0.07))
        self._depth_max_m:                float = float(cfg.get('depth_max_m', 0.50))
        self._disconnect_timeout_s:       float = float(cfg.get('disconnect_timeout_s', 5.0))
        self._ns:                         str   = f'/{self._node_name}'

        # RealSense pipeline state
        self._pipeline:   rs.pipeline        = None
        self._profile:    rs.pipeline_profile = None
        self._align:      rs.align            = None

        # Post-processing filters
        self._decimation   = None
        self._threshold    = None
        self._spatial      = None
        self._temporal     = None
        self._hole_filling = None

        # Intrinsics cache
        self._color_intr:       rs.intrinsics = None
        self._color_rect_props: dict          = {}

        self._ok   = threading.Event()
        self._stop: threading.Event = None

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self._color_pub:      object = None
        self._color_rect_pub: object = None
        self._cam_info_pub:   object = None
        self._depth_pub:      object = None

        if self._enable_color:
            self._color_pub    = self.create_publisher(Image,      f'{self._ns}/color/image_raw',   qos)
            self._cam_info_pub = self.create_publisher(CameraInfo, f'{self._ns}/color/camera_info', qos)
            if self._enable_color_rectification:
                self._color_rect_pub = self.create_publisher(Image, f'{self._ns}/color/image_rect', qos)
        if self._enable_depth:
            self._depth_pub = self.create_publisher(Image, f'{self._ns}/depth/image_raw', qos)

        self.get_logger().info(
            f'Starting RealSense D405 — serial={self._serial or "any"}  '
            f'color={self._enable_color}  depth={self._enable_depth}  '
            f'ns={self._ns}'
        )

        if not self._start_pipeline():
            self.get_logger().fatal('Failed to start RealSense pipeline')
            raise RuntimeError('Failed to start RealSense pipeline')

        self._capture_thread = threading.Thread(
            target=self._capture_loop,
            daemon=True,
            name=f'{self._node_name}_rs_capture',
        )
        self._capture_thread.start()
        self.get_logger().info('RealSense capture thread started')

    # ── Config ────────────────────────────────────────────────────────────────

    def _load_config(self, path: str) -> dict:
        try:
            with open(path, 'rb') as f:
                return tomllib.load(f)
        except FileNotFoundError:
            self.get_logger().fatal(f'Config not found: {path}')
            raise
        except Exception as e:
            self.get_logger().fatal(f'Failed to parse config: {e}')
            raise

    # ── Per-stream init (called after pipeline.start) ─────────────────────────

    def _init_color(self) -> None:
        self._color_intr = rs.video_stream_profile(
            self._profile.get_stream(rs.stream.color)
        ).get_intrinsics()

        if self._enable_color_rectification:
            intr = self._color_intr
            K    = np.array([
                [intr.fx, 0,       intr.ppx],
                [0,       intr.fy, intr.ppy],
                [0,       0,       1       ],
            ])
            dist = np.array(intr.coeffs)
            K_rect, roi = cv2.getOptimalNewCameraMatrix(
                cameraMatrix=K,
                distCoeffs=dist,
                imageSize=self._color_resolution,
                alpha=self._color_rectification_alpha,
            )
            self._color_rect_props = {'K': K, 'K_rect': K_rect, 'dist': dist, 'roi': roi}

    def _init_depth(self) -> None:
        self._decimation = rs.decimation_filter()
        self._decimation.set_option(rs.option.filter_magnitude, 1)

        self._threshold = rs.threshold_filter()
        self._threshold.set_option(rs.option.min_distance, self._depth_min_m)
        self._threshold.set_option(rs.option.max_distance, self._depth_max_m)

        self._spatial = rs.spatial_filter()
        self._spatial.set_option(rs.option.filter_magnitude, 2)
        self._spatial.set_option(rs.option.filter_smooth_alpha, 0.5)
        self._spatial.set_option(rs.option.filter_smooth_delta, 20)
        self._spatial.set_option(rs.option.holes_fill, 0)

        self._temporal = rs.temporal_filter()
        self._temporal.set_option(rs.option.filter_smooth_alpha, 0.4)
        self._temporal.set_option(rs.option.filter_smooth_delta, 20)

        self._hole_filling = rs.hole_filling_filter()

        self._align = rs.align(
            rs.stream.color if self._align_to_color else rs.stream.depth
        )

    # ── Per-frame callbacks ───────────────────────────────────────────────────

    def _color_cb(self, color_frame: rs.video_frame, stamp) -> None:
        img = np.asanyarray(color_frame.get_data())

        self._color_pub.publish(
            _np_to_ros_image(img, 'bgr8', stamp, f'{self._node_name}_color')
        )
        if self._color_intr is not None:
            self._cam_info_pub.publish(
                _rs_intrinsics_to_camera_info(self._color_intr, stamp, f'{self._node_name}_color')
            )

        if not self._enable_color_rectification or not self._color_rect_props:
            return

        props    = self._color_rect_props
        img_rect = cv2.undistort(
            src=img,
            cameraMatrix=props['K'],
            distCoeffs=props['dist'],
            newCameraMatrix=props['K_rect'],
        )
        if self._color_rectification_crop:
            x, y, w, h = props['roi']
            img_rect = img_rect[y:y + h, x:x + w]
        self._color_rect_pub.publish(
            _np_to_ros_image(img_rect, 'bgr8', stamp, f'{self._node_name}_color_rect')
        )

    def _depth_cb(self, depth_frame: rs.depth_frame, stamp) -> None:
        f = self._decimation.process(depth_frame)
        f = self._threshold.process(f)
        f = self._spatial.process(f)
        f = self._temporal.process(f)
        f = self._hole_filling.process(f).as_depth_frame()

        depth_m = np.asanyarray(f.get_data()).astype(np.float32) / 1000.0
        self._depth_pub.publish(
            _np_to_ros_image(depth_m, '32FC1', stamp, f'{self._node_name}_depth')
        )

    # ── Capture loop ──────────────────────────────────────────────────────────

    def _trigger_disconnect_shutdown(self) -> None:
        self.get_logger().fatal('Camera disconnected — triggering clean shutdown')
        self._ok.clear()
        if self._stop is not None:
            self._stop.set()

    def _capture_loop(self) -> None:
        _timeout_ms      = 1000
        _max_consecutive = max(1, int(self._disconnect_timeout_s * 1000 / _timeout_ms))
        consecutive_timeouts = 0

        while self._ok.is_set():
            try:
                frames = self._pipeline.wait_for_frames(timeout_ms=_timeout_ms)
                consecutive_timeouts = 0
            except RuntimeError:
                consecutive_timeouts += 1
                if consecutive_timeouts >= _max_consecutive:
                    self.get_logger().fatal(
                        f'No frames received for {self._disconnect_timeout_s:.0f}s '
                        f'— assuming camera disconnected'
                    )
                    self._trigger_disconnect_shutdown()
                    break
                self.get_logger().warning(
                    f'wait_for_frames timed out ({consecutive_timeouts}/{_max_consecutive}) — retrying'
                )
                continue
            except Exception:
                self.get_logger().error(f'Capture error: {traceback.format_exc()}')
                self._trigger_disconnect_shutdown()
                break

            if self._enable_depth and self._align is not None:
                frames = self._align.process(frames)

            stamp = self.get_clock().now().to_msg()

            if self._enable_color:
                cf = frames.get_color_frame()
                if cf and cf.is_valid():
                    try:
                        self._color_cb(cf, stamp)
                    except Exception:
                        self.get_logger().error(f'color_cb error: {traceback.format_exc()}')

            if self._enable_depth:
                df = frames.get_depth_frame()
                if df and df.is_valid():
                    try:
                        self._depth_cb(df, stamp)
                    except Exception:
                        self.get_logger().error(f'depth_cb error: {traceback.format_exc()}')

    # ── Pipeline lifecycle ────────────────────────────────────────────────────

    def _start_pipeline(self) -> bool:
        try:
            self.get_logger().info(
                f'Connecting to RealSense D405 (serial={self._serial or "any"}) ...'
            )
            self._pipeline = rs.pipeline()
            cfg = rs.config()

            if self._serial:
                cfg.enable_device(self._serial)

            if self._enable_color:
                w, h = self._color_resolution
                cfg.enable_stream(rs.stream.color, w, h, rs.format.bgr8, int(self._color_fps))

            if self._enable_depth:
                w, h = self._depth_resolution
                cfg.enable_stream(rs.stream.depth, w, h, rs.format.z16, int(self._depth_fps))
                cfg.enable_stream(rs.stream.infrared, 1, w, h, rs.format.y8, int(self._depth_fps))
                cfg.enable_stream(rs.stream.infrared, 2, w, h, rs.format.y8, int(self._depth_fps))

            self._profile = self._pipeline.start(cfg)

            dev = self._profile.get_device()
            self.get_logger().info(
                f'Connected: {dev.get_info(rs.camera_info.name)} '
                f'serial={dev.get_info(rs.camera_info.serial_number)} '
                f'fw={dev.get_info(rs.camera_info.firmware_version)}'
            )

            if self._enable_color:
                self._init_color()
            if self._enable_depth:
                self._init_depth()

            self._ok.set()
            self.get_logger().info('RealSense D405 pipeline started successfully')
            return True

        except RuntimeError:
            self._ok.clear()
            self.get_logger().error(f'Failed to connect: {traceback.format_exc()}')
            return False
        except Exception:
            self._ok.clear()
            self.get_logger().error(f'Unexpected error starting pipeline: {traceback.format_exc()}')
            return False

    # ── Shutdown ──────────────────────────────────────────────────────────────

    def shutdown(self) -> None:
        self.get_logger().info('Shutting down RealSense node …')
        self._ok.clear()

        if self._pipeline:
            try:
                self._pipeline.stop()
            except Exception:
                pass

        if hasattr(self, '_capture_thread'):
            self._capture_thread.join(timeout=3.0)

        self.get_logger().info('RealSense node shut down')


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = RealSenseNode()

    _stop = threading.Event()
    node._stop = _stop
    signal.signal(signal.SIGINT, lambda sig, frame: _stop.set())

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    _stop.wait()

    node.shutdown()
    node.destroy_node()
    rclpy.shutdown()
    spin_thread.join(timeout=3.0)


if __name__ == '__main__':
    main()
