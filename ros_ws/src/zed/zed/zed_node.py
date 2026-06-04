"""
zed_node.py – ROS 2 node for the Stereolabs ZED 2i camera.

ROS topics published
--------------------
  /{node_name}/color/image_raw          sensor_msgs/Image        BGR8 left camera
  /{node_name}/color/image_rect         sensor_msgs/Image        rectified BGR8 (if enabled)
  /{node_name}/color/camera_info        sensor_msgs/CameraInfo
  /{node_name}/right/image_raw          sensor_msgs/Image        BGR8 right camera
  /{node_name}/depth/image_raw          sensor_msgs/Image        float32 metres
  /{node_name}/imu                      sensor_msgs/Imu          (if enable_imu)

Config (`config/config.toml`)
------------------------------
  node_name                    str    ROS node name and topic namespace
  serial                       str    device serial; empty = first available
  enable_color                 bool   enable left RGB stream (default true)
  enable_depth                 bool   enable depth stream (default true)
  enable_imu                   bool   enable IMU stream (default true)
  resolution                   str    VGA | HD720 | HD1080 | HD2K (default HD1080)
  fps                          int    frame rate (default 30)
  depth_mode                   str    performance | quality | ultra | neural (default ultra)
  depth_min_m                  float  min depth threshold (default 0.3)
  depth_max_m                  float  max depth threshold (default 10.0)
  depth_confidence             int    0–100 (default 95)
  enable_fill_mode             bool   fill depth holes (default false)
  enable_color_rectification   bool   publish rectified colour (default false)
  color_rectification_alpha    float  0.0 = no black borders, 1.0 = full FOV
  color_rectification_crop     bool   crop rectified image to valid ROI

Notes
-----
  - ZED SDK returns BGRA — alpha channel is stripped before publishing.
  - IMU angular velocity is in deg/s from the SDK and converted to rad/s.
  - Disconnect detected via grab() error codes (CAMERA_NOT_DETECTED / CAMERA_REBOOTING).
"""

import signal
import threading
import tomllib
import traceback

import cv2
import numpy as np
import pyzed.sl as sl

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import CameraInfo, Image, Imu
from geometry_msgs.msg import Vector3, Quaternion
from std_msgs.msg import Header


# ── Constants ─────────────────────────────────────────────────────────────────

_RESOLUTION_MAP = {
    'VGA':    sl.RESOLUTION.VGA,
    'HD720':  sl.RESOLUTION.HD720,
    'HD1080': sl.RESOLUTION.HD1080,
    'HD2K':   sl.RESOLUTION.HD2K,
}

_DEPTH_MODE_MAP = {
    'performance': sl.DEPTH_MODE.PERFORMANCE,
    'quality':     sl.DEPTH_MODE.QUALITY,
    'ultra':       sl.DEPTH_MODE.ULTRA,
    'neural':      sl.DEPTH_MODE.NEURAL,
}

_DISCONNECT_CODES = frozenset({
    sl.ERROR_CODE.CAMERA_NOT_DETECTED,
    sl.ERROR_CODE.CAMERA_REBOOTING,
})


# ── Helpers ───────────────────────────────────────────────────────────────────

def _np_to_ros_image(arr: np.ndarray, encoding: str, stamp, frame_id: str = '') -> Image:
    msg              = Image()
    msg.header.stamp = stamp
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


def _build_camera_info(cam: sl.CameraParameters, stamp, frame_id: str = '') -> CameraInfo:
    msg              = CameraInfo()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.width        = int(cam.image_size.width)
    msg.height       = int(cam.image_size.height)
    msg.distortion_model = 'plumb_bob'
    msg.d = list(cam.disto)
    msg.k = [
        cam.fx, 0.0,    cam.cx,
        0.0,    cam.fy, cam.cy,
        0.0,    0.0,    1.0,
    ]
    msg.r = [1.0, 0.0, 0.0,
             0.0, 1.0, 0.0,
             0.0, 0.0, 1.0]
    msg.p = [
        cam.fx, 0.0,    cam.cx, 0.0,
        0.0,    cam.fy, cam.cy, 0.0,
        0.0,    0.0,    1.0,    0.0,
    ]
    return msg


# ── Node ──────────────────────────────────────────────────────────────────────

class ZedNode(Node):

    def __init__(self):
        super().__init__('zed_node')

        self.declare_parameter('config_path', '')
        config_path = self.get_parameter('config_path').value
        if not config_path:
            self.get_logger().fatal('config_path parameter is required')
            raise RuntimeError('config_path parameter is required')

        cfg = self._load_config(config_path)

        self._node_name:                  str  = cfg.get('node_name', 'zed')
        self._serial:                     int  = int(cfg['serial']) if cfg.get('serial') else 0
        self._enable_color:               bool = bool(cfg.get('enable_color', True))
        self._enable_depth:               bool = bool(cfg.get('enable_depth', True))
        self._enable_imu:                 bool = bool(cfg.get('enable_imu', True))
        self._resolution_key:             str  = cfg.get('resolution', 'HD1080')
        self._resolution:        sl.RESOLUTION = _RESOLUTION_MAP[self._resolution_key]
        self._fps:                        int  = int(cfg.get('fps', 30))
        self._depth_mode:       sl.DEPTH_MODE  = _DEPTH_MODE_MAP[cfg.get('depth_mode', 'ultra')]
        self._depth_min_m:                float = float(cfg.get('depth_min_m', 0.3))
        self._depth_max_m:                float = float(cfg.get('depth_max_m', 10.0))
        self._depth_confidence:           int  = int(cfg.get('depth_confidence', 95))
        self._enable_fill_mode:           bool = bool(cfg.get('enable_fill_mode', False))
        self._enable_color_rectification: bool = bool(cfg.get('enable_color_rectification', False))
        self._color_rectification_alpha:  float = float(cfg.get('color_rectification_alpha', 0.0))
        self._color_rectification_crop:   bool = bool(cfg.get('color_rectification_crop', False))
        self._ns:                         str  = f'/{self._node_name}'

        # ZED SDK objects
        self._zed            = sl.Camera()
        self._runtime_params = sl.RuntimeParameters()

        # Pre-allocated sl.Mat / SensorsData
        self._left_mat     = sl.Mat()
        self._right_mat    = sl.Mat()
        self._depth_mat    = sl.Mat()
        self._sensors_data = sl.SensorsData()

        # Calibration cache for rectification
        self._left_rect_props: dict            = {}
        self._left_cam_params: sl.CameraParameters = None

        self._ok   = threading.Event()
        self._stop: threading.Event = None

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        # Publishers — created here, populated after pipeline opens
        self._color_pub:      object = None
        self._color_rect_pub: object = None
        self._cam_info_pub:   object = None
        self._right_pub:      object = None
        self._depth_pub:      object = None
        self._imu_pub:        object = None

        if self._enable_color:
            self._color_pub    = self.create_publisher(Image,      f'{self._ns}/color/image_raw',  qos)
            self._cam_info_pub = self.create_publisher(CameraInfo, f'{self._ns}/color/camera_info', qos)
            if self._enable_color_rectification:
                self._color_rect_pub = self.create_publisher(Image, f'{self._ns}/color/image_rect', qos)
        if self._enable_depth:
            self._right_pub = self.create_publisher(Image, f'{self._ns}/right/image_raw', qos)
            self._depth_pub = self.create_publisher(Image, f'{self._ns}/depth/image_raw', qos)
        if self._enable_imu:
            self._imu_pub = self.create_publisher(Imu, f'{self._ns}/imu', qos)

        self.get_logger().info(
            f'Starting ZED 2i — serial={self._serial or "any"}  '
            f'color={self._enable_color}  depth={self._enable_depth}  '
            f'imu={self._enable_imu}  ns={self._ns}'
        )

        if not self._start_pipeline():
            self.get_logger().fatal('Failed to start ZED pipeline')
            raise RuntimeError('Failed to start ZED pipeline')

        self._capture_thread = threading.Thread(
            target=self._capture_loop,
            daemon=True,
            name=f'{self._node_name}_zed_capture',
        )
        self._capture_thread.start()
        self.get_logger().info('ZED capture thread started')

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

    # ── Per-stream init (called after zed.open) ───────────────────────────────

    def _init_color(self, calibration: sl.CalibrationParameters) -> None:
        self._left_cam_params = calibration.left_cam

        if self._enable_color_rectification:
            cam  = self._left_cam_params
            K    = np.array([[cam.fx, 0, cam.cx], [0, cam.fy, cam.cy], [0, 0, 1]])
            dist = np.array(cam.disto)
            w, h = int(cam.image_size.width), int(cam.image_size.height)
            K_rect, roi = cv2.getOptimalNewCameraMatrix(
                cameraMatrix=K, distCoeffs=dist,
                imageSize=(w, h), alpha=self._color_rectification_alpha,
            )
            self._left_rect_props = {'K': K, 'K_rect': K_rect, 'dist': dist, 'roi': roi}

    def _init_depth(self, calibration: sl.CalibrationParameters) -> None:
        if self._left_cam_params is None:
            self._left_cam_params = calibration.left_cam
        self._runtime_params.confidence_threshold = self._depth_confidence
        self._runtime_params.enable_fill_mode     = self._enable_fill_mode

    # ── Per-frame callbacks ───────────────────────────────────────────────────

    def _color_cb(self, stamp) -> None:
        left_img = cv2.cvtColor(self._left_mat.get_data(), cv2.COLOR_BGRA2BGR)

        self._color_pub.publish(
            _np_to_ros_image(left_img, 'bgr8', stamp, f'{self._node_name}_left')
        )

        if self._left_cam_params is not None:
            self._cam_info_pub.publish(
                _build_camera_info(self._left_cam_params, stamp, f'{self._node_name}_left')
            )

        if not self._enable_color_rectification or not self._left_rect_props:
            return

        props    = self._left_rect_props
        img_rect = cv2.undistort(
            src=left_img, cameraMatrix=props['K'],
            distCoeffs=props['dist'], newCameraMatrix=props['K_rect'],
        )
        if self._color_rectification_crop:
            x, y, w, h = props['roi']
            img_rect = img_rect[y:y + h, x:x + w]
        self._color_rect_pub.publish(
            _np_to_ros_image(img_rect, 'bgr8', stamp, f'{self._node_name}_left_rect')
        )

    def _depth_cb(self, stamp) -> None:
        right_img = cv2.cvtColor(self._right_mat.get_data(), cv2.COLOR_BGRA2BGR)
        depth_m   = self._depth_mat.get_data().copy()   # float32, metres

        self._right_pub.publish(
            _np_to_ros_image(right_img, 'bgr8', stamp, f'{self._node_name}_right')
        )
        self._depth_pub.publish(
            _np_to_ros_image(depth_m, '32FC1', stamp, f'{self._node_name}_left')
        )

    def _imu_cb(self, stamp) -> None:
        self._zed.get_sensors_data(self._sensors_data, sl.TIME_REFERENCE.IMAGE)
        imu_data  = self._sensors_data.get_imu_data()

        lin_accel = imu_data.get_linear_acceleration()
        ang_vel   = np.deg2rad(np.array(imu_data.get_angular_velocity(), dtype=float))

        orient = sl.Orientation()
        imu_data.get_pose().get_orientation(orient)
        q = orient.get()   # [ox, oy, oz, ow]

        try:
            lin_cov = list(np.array(imu_data.get_linear_acceleration_covariance().r).flatten())
            ang_cov = list(np.array(imu_data.get_angular_velocity_covariance().r).flatten())
        except Exception:
            lin_cov = [0.0] * 9
            ang_cov = [0.0] * 9

        msg                  = Imu()
        msg.header.stamp     = stamp
        msg.header.frame_id  = f'{self._node_name}_imu'
        msg.linear_acceleration.x  = float(lin_accel[0])
        msg.linear_acceleration.y  = float(lin_accel[1])
        msg.linear_acceleration.z  = float(lin_accel[2])
        msg.angular_velocity.x     = float(ang_vel[0])
        msg.angular_velocity.y     = float(ang_vel[1])
        msg.angular_velocity.z     = float(ang_vel[2])
        msg.orientation.x          = float(q[0])
        msg.orientation.y          = float(q[1])
        msg.orientation.z          = float(q[2])
        msg.orientation.w          = float(q[3])
        msg.linear_acceleration_covariance  = lin_cov
        msg.angular_velocity_covariance     = ang_cov
        msg.orientation_covariance          = [0.0] * 9
        self._imu_pub.publish(msg)

    # ── Capture loop ──────────────────────────────────────────────────────────

    def _trigger_disconnect_shutdown(self, reason: str) -> None:
        self.get_logger().fatal(f'Camera disconnected ({reason}) — triggering clean shutdown')
        self._ok.clear()
        if self._stop is not None:
            self._stop.set()

    def _capture_loop(self) -> None:
        while self._ok.is_set():
            err = self._zed.grab(self._runtime_params)

            if err == sl.ERROR_CODE.SUCCESS:
                stamp = self.get_clock().now().to_msg()

                if self._enable_color:
                    self._zed.retrieve_image(self._left_mat, sl.VIEW.LEFT, sl.MEM.CPU)
                    try:
                        self._color_cb(stamp)
                    except Exception:
                        self.get_logger().error(f'color_cb error: {traceback.format_exc()}')

                if self._enable_depth:
                    if not self._enable_color:
                        self._zed.retrieve_image(self._left_mat, sl.VIEW.LEFT, sl.MEM.CPU)
                    self._zed.retrieve_image(self._right_mat,   sl.VIEW.RIGHT,  sl.MEM.CPU)
                    self._zed.retrieve_measure(self._depth_mat, sl.MEASURE.DEPTH, sl.MEM.CPU)
                    try:
                        self._depth_cb(stamp)
                    except Exception:
                        self.get_logger().error(f'depth_cb error: {traceback.format_exc()}')

                if self._enable_imu:
                    try:
                        self._imu_cb(stamp)
                    except Exception:
                        self.get_logger().error(f'imu_cb error: {traceback.format_exc()}')

            elif err in _DISCONNECT_CODES:
                self._trigger_disconnect_shutdown(str(err))
                break

            else:
                self.get_logger().warning(f'grab() returned {err} — skipping frame')

    # ── Pipeline lifecycle ────────────────────────────────────────────────────

    def _start_pipeline(self) -> bool:
        try:
            self.get_logger().info(
                f'Connecting to ZED 2i (serial={self._serial or "any"}) ...'
            )
            init_params = sl.InitParameters()
            init_params.camera_resolution        = self._resolution
            init_params.camera_fps               = self._fps
            init_params.depth_mode               = (
                self._depth_mode if self._enable_depth else sl.DEPTH_MODE.NONE
            )
            init_params.coordinate_units         = sl.UNIT.METER
            init_params.depth_minimum_distance   = self._depth_min_m
            init_params.depth_maximum_distance   = self._depth_max_m
            init_params.enable_image_enhancement = True

            if self._serial:
                init_params.set_from_serial_number(self._serial)

            err = self._zed.open(init_params)
            if err != sl.ERROR_CODE.SUCCESS:
                self.get_logger().error(f'zed.open() failed: {err}')
                return False

            info        = self._zed.get_camera_information()
            calibration = info.camera_configuration.calibration_parameters

            self.get_logger().info(
                f'Connected: {info.camera_model}  '
                f'serial={info.serial_number}  '
                f'resolution={self._resolution_key}  '
                f'fps={self._fps}'
            )

            if self._enable_color:
                self._init_color(calibration)
            if self._enable_depth:
                self._init_depth(calibration)

            self._ok.set()
            self.get_logger().info('ZED 2i pipeline started successfully')
            return True

        except Exception:
            self._ok.clear()
            self.get_logger().error(f'Unexpected error starting pipeline: {traceback.format_exc()}')
            return False

    # ── Shutdown ──────────────────────────────────────────────────────────────

    def shutdown(self) -> None:
        self.get_logger().info('Shutting down ZED node …')
        self._ok.clear()

        try:
            self._zed.close()
        except Exception:
            pass

        if hasattr(self, '_capture_thread'):
            self._capture_thread.join(timeout=3.0)

        self.get_logger().info('ZED node shut down')


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = ZedNode()

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
