#!/usr/bin/env python3

import argparse
import logging
import threading
import traceback

import cv2
import zenoh
import numpy as np
import pyzed.sl as sl

import feather_pb2
from feather_pb2 import (
    MonoCameraFrame, StereoCameraFrame, CameraIntrinsics, Image, DType, DistortionModel,
    Imu, Vector3D, Quaternion,
)

# ─── module-level constants ───────────────────────────────────────────────────

_RESOLUTION_MAP = {
    "VGA":    sl.RESOLUTION.VGA,     # 672 × 376   @ up to 100 fps
    "HD720":  sl.RESOLUTION.HD720,   # 1280 × 720  @ up to 60 fps
    "HD1080": sl.RESOLUTION.HD1080,  # 1920 × 1080 @ up to 30 fps
    "HD2K":   sl.RESOLUTION.HD2K,    # 2208 × 1242 @ up to 15 fps
}

_DEPTH_MODE_MAP = {
    "performance": sl.DEPTH_MODE.PERFORMANCE,
    "quality":     sl.DEPTH_MODE.QUALITY,
    "ultra":       sl.DEPTH_MODE.ULTRA,
    "neural":      sl.DEPTH_MODE.NEURAL,
}

_NP_DTYPE_MAP = {
    np.dtype("uint8"):   DType.UINT8,
    np.dtype("uint16"):  DType.UINT16,
    np.dtype("float32"): DType.F32,
    np.dtype("float64"): DType.F64,
}

# grab() error codes that mean the device is gone.
_DISCONNECT_CODES = frozenset({
    sl.ERROR_CODE.CAMERA_NOT_DETECTED,
    sl.ERROR_CODE.CAMERA_REBOOTING,
})

# ─── helpers ─────────────────────────────────────────────────────────────────


def _to_image(arr: np.ndarray) -> Image:
    if arr.ndim == 2:
        h, w = arr.shape
        channels = 1
    else:
        h, w, channels = arr.shape
    return Image(
        channels=channels,
        height=h,
        width=w,
        dtype=_NP_DTYPE_MAP.get(arr.dtype, DType.UINT8),
        data=arr.tobytes(),
    )


def _zed_cam_params_to_proto(cam: sl.CameraParameters) -> CameraIntrinsics:
    # ZED SDK uses Brown-Conrady with up to 8 coefficients: [k1,k2,p1,p2,k3,k4,k5,k6].
    return CameraIntrinsics(
        f=cam.fx,
        cx=cam.cx,
        cy=cam.cy,
        distortion_model=DistortionModel.BROWN_CONRADY,
        distortion_parameters=list(cam.disto),
    )


# ─── wrapper class ───────────────────────────────────────────────────────────

class Zed2i:
    # ZED 2i: 120 mm baseline wide-FOV stereo camera with on-device neural depth
    # and a built-in 6-DOF IMU (accelerometer + gyroscope + orientation estimate).
    #
    # Depth is already registered to the left camera; no explicit align step
    # is needed (unlike RealSense). ZED SDK returns images as BGRA — they are
    # converted to BGR before being written into the proto Image.
    #
    # IMU: get_angular_velocity() returns deg/s — converted to rad/s here.
    # Orientation covariance is not separately exposed by the SDK; zeros are used.
    #
    # Disconnection is detected via grab() error codes (no SDK callback like
    # RealSense): CAMERA_NOT_DETECTED / CAMERA_REBOOTING → clean shutdown.

    def __init__(self, **kwargs):
        self.logger: logging.Logger = kwargs["logger"]
        self.serial: int = int(kwargs["serial"]) if kwargs["serial"] else 0
        self.enable_color: bool = kwargs["enable_color"]
        self.enable_depth: bool = kwargs["enable_depth"]
        self.fps: int = int(kwargs["fps"])
        self._resolution_key: str = kwargs["resolution"]
        self.resolution: sl.RESOLUTION = _RESOLUTION_MAP[kwargs["resolution"]]
        self.depth_mode: sl.DEPTH_MODE = _DEPTH_MODE_MAP[kwargs["depth_mode"]]
        self.depth_min_m: float = kwargs["depth_min_m"]
        self.depth_max_m: float = kwargs["depth_max_m"]
        self.depth_confidence: int = int(kwargs["depth_confidence"])
        self.enable_fill_mode: bool = kwargs["enable_fill_mode"]
        self.enable_imu: bool = kwargs["enable_imu"]
        self.enable_color_rectification: bool = kwargs["enable_color_rectification"]
        self.color_rectification_alpha: float = kwargs["color_rectification_alpha"]
        self.color_rectification_crop: bool = kwargs["color_rectification_crop"]
        self.topic_prefix: str = kwargs["topic_prefix"]
        self.topic_prefix = "" if not self.topic_prefix else f"{self.topic_prefix}/"
        self.topic_color            = kwargs.get("topic_color")            or f"{self.topic_prefix}color"
        self.topic_color_rectified  = kwargs.get("topic_color_rectified")  or f"{self.topic_prefix}color/rectified"
        self.topic_stereo           = kwargs.get("topic_stereo")           or f"{self.topic_prefix}stereo"
        self.topic_imu              = kwargs.get("topic_imu")              or f"{self.topic_prefix}imu"

        self.zed = sl.Camera()
        self._runtime_params = sl.RuntimeParameters()

        # Pre-allocated sl.Mat / SensorsData objects — reused each grab.
        self._left_mat      = sl.Mat()
        self._right_mat     = sl.Mat()
        self._depth_mat     = sl.Mat()
        self._sensors_data  = sl.SensorsData()

        self._left_intrinsics: CameraIntrinsics = None
        self._left_rect_props: dict = {}
        self._baseline_m: float = 0.12  # ZED 2i nominal 120 mm

        self._capture_thread: threading.Thread = None
        self.ok = threading.Event()

        self.init_comms()

    # ── comms / zenoh ────────────────────────────────────────────────────────

    def init_comms(self):
        zenoh.try_init_log_from_env()
        self.session = zenoh.open(zenoh.Config())
        self.left_pub            = None
        self.left_rectified_pub  = None
        self.stereo_pub          = None
        self.imu_pub             = None

    # ── per-stream init (called after zed.open) ──────────────────────────────

    def init_color(self, calibration: sl.CalibrationParameters):
        left_cam = calibration.left_cam
        self._left_intrinsics = _zed_cam_params_to_proto(left_cam)

        if self.enable_color_rectification:
            K = np.array([
                [left_cam.fx, 0,          left_cam.cx],
                [0,           left_cam.fy, left_cam.cy],
                [0,           0,           1          ],
            ])
            dist  = np.array(left_cam.disto)
            w     = int(left_cam.image_size.width)
            h     = int(left_cam.image_size.height)
            K_rect, roi = cv2.getOptimalNewCameraMatrix(
                cameraMatrix=K,
                distCoeffs=dist,
                imageSize=(w, h),
                alpha=self.color_rectification_alpha,
            )
            self._left_rect_props = {"K": K, "K_rect": K_rect, "dist": dist, "roi": roi}
            self.left_rectified_pub = self.session.declare_publisher(
                self.topic_color_rectified
            )

        self.left_pub = self.session.declare_publisher(self.topic_color)

    def init_depth(self, calibration: sl.CalibrationParameters):
        if self._left_intrinsics is None:
            self._left_intrinsics = _zed_cam_params_to_proto(calibration.left_cam)

        # Baseline from left-to-right stereo translation vector (mm → m).
        try:
            t = calibration.stereo_transform.get_translation().get()
            self._baseline_m = abs(float(t[0])) / 1000.0
        except Exception:
            self._baseline_m = 0.12  # ZED 2i nominal 120 mm

        self._runtime_params.confidence_threshold = self.depth_confidence
        self._runtime_params.enable_fill_mode     = self.enable_fill_mode

        self.stereo_pub = self.session.declare_publisher(self.topic_stereo)

    def init_imu(self):
        self.imu_pub = self.session.declare_publisher(self.topic_imu)

    # ── per-frame callbacks ───────────────────────────────────────────────────

    def color_cb(self):
        # ZED SDK yields BGRA; strip alpha channel before publishing.
        left_img = cv2.cvtColor(self._left_mat.get_data(), cv2.COLOR_BGRA2BGR)

        msg = MonoCameraFrame(
            intrinsics=self._left_intrinsics,
            image=_to_image(left_img),
        )
        if self.ok.is_set():
            self.left_pub.put(msg.SerializeToString())

        if not self.enable_color_rectification or not self._left_rect_props:
            return

        props = self._left_rect_props
        img_rect = cv2.undistort(
            src=left_img,
            cameraMatrix=props["K"],
            distCoeffs=props["dist"],
            newCameraMatrix=props["K_rect"],
        )
        if self.color_rectification_crop:
            x, y, w, h = props["roi"]
            img_rect = img_rect[y:y + h, x:x + w]

        K_r = props["K_rect"]
        msg_rect = MonoCameraFrame(
            intrinsics=CameraIntrinsics(
                f=K_r[0, 0],
                cx=K_r[0, 2],
                cy=K_r[1, 2],
                distortion_model=DistortionModel.NONE,
                distortion_parameters=[],
            ),
            image=_to_image(img_rect),
        )
        if self.ok.is_set():
            self.left_rectified_pub.put(msg_rect.SerializeToString())

    def depth_cb(self):
        left_img  = cv2.cvtColor(self._left_mat.get_data(),  cv2.COLOR_BGRA2BGR)
        right_img = cv2.cvtColor(self._right_mat.get_data(), cv2.COLOR_BGRA2BGR)
        # MEASURE.DEPTH → float32, metres; NaN where depth is invalid.
        depth_m   = self._depth_mat.get_data().copy()

        msg = StereoCameraFrame(
            image_left=_to_image(left_img),
            image_right=_to_image(right_img),
            depth=_to_image(depth_m),
            intrinsics=self._left_intrinsics,
            baseline=self._baseline_m,
        )
        if self.ok.is_set():
            self.stereo_pub.put(msg.SerializeToString())

    def imu_cb(self):
        self.zed.get_sensors_data(self._sensors_data, sl.TIME_REFERENCE.IMAGE)
        imu = self._sensors_data.get_imu_data()

        lin_accel = imu.get_linear_acceleration()   # m/s², sensor frame
        # ZED SDK returns angular velocity in deg/s — convert to rad/s.
        ang_vel = np.deg2rad(np.array(imu.get_angular_velocity(), dtype=float))

        orient = sl.Orientation()
        imu.get_pose().get_orientation(orient)
        q = orient.get()  # [ox, oy, oz, ow]

        try:
            lin_cov = list(np.array(imu.get_linear_acceleration_covariance().r).flatten())
            ang_cov = list(np.array(imu.get_angular_velocity_covariance().r).flatten())
        except Exception:
            lin_cov = [0.0] * 9
            ang_cov = [0.0] * 9

        msg = Imu(
            linear_acceleration=Vector3D(
                x=float(lin_accel[0]), y=float(lin_accel[1]), z=float(lin_accel[2])
            ),
            angular_velocity=Vector3D(
                x=float(ang_vel[0]), y=float(ang_vel[1]), z=float(ang_vel[2])
            ),
            orientation=Quaternion(
                x=float(q[0]), y=float(q[1]), z=float(q[2]), w=float(q[3])
            ),
            linear_acceleration_covariance=lin_cov,
            angular_velocity_covariance=ang_cov,
            orientation_covariance=[0.0] * 9,
        )
        if self.ok.is_set():
            self.imu_pub.put(msg.SerializeToString())

    # ── capture loop ─────────────────────────────────────────────────────────

    def _capture_loop(self):
        while self.ok.is_set():
            err = self.zed.grab(self._runtime_params)

            if err == sl.ERROR_CODE.SUCCESS:
                if self.enable_color:
                    self.zed.retrieve_image(self._left_mat, sl.VIEW.LEFT, sl.MEM.CPU)
                    try:
                        self.color_cb()
                    except Exception:
                        self.logger.error(f"color_cb error: {traceback.format_exc()}")

                if self.enable_depth:
                    # Left image is needed for StereoCameraFrame even when color
                    # publishing is disabled — retrieve it if not already done.
                    if not self.enable_color:
                        self.zed.retrieve_image(self._left_mat, sl.VIEW.LEFT, sl.MEM.CPU)
                    self.zed.retrieve_image(self._right_mat,  sl.VIEW.RIGHT,  sl.MEM.CPU)
                    self.zed.retrieve_measure(self._depth_mat, sl.MEASURE.DEPTH, sl.MEM.CPU)
                    try:
                        self.depth_cb()
                    except Exception:
                        self.logger.error(f"depth_cb error: {traceback.format_exc()}")

                if self.enable_imu:
                    try:
                        self.imu_cb()
                    except Exception:
                        self.logger.error(f"imu_cb error: {traceback.format_exc()}")

            elif err in _DISCONNECT_CODES:
                self.logger.warning(
                    f"ZED 2i disconnected ({err}) — initiating clean shutdown"
                )
                self.ok.clear()
                break

            else:
                # Transient SDK error (exposure settling, dropped frame, etc.) — skip.
                self.logger.warning(f"grab() returned {err} — skipping frame")

    # ── lifecycle ─────────────────────────────────────────────────────────────

    def start_pipeline(self) -> bool:
        try:
            self.logger.info(
                f"Connecting to ZED 2i (serial={self.serial or 'any'}) ..."
            )
            init_params = sl.InitParameters()
            init_params.camera_resolution      = self.resolution
            init_params.camera_fps             = self.fps
            init_params.depth_mode             = (
                self.depth_mode if self.enable_depth else sl.DEPTH_MODE.NONE
            )
            init_params.coordinate_units       = sl.UNIT.METER
            init_params.depth_minimum_distance = self.depth_min_m
            init_params.depth_maximum_distance = self.depth_max_m
            init_params.enable_image_enhancement = True

            if self.serial:
                init_params.set_from_serial_number(self.serial)

            err = self.zed.open(init_params)
            if err != sl.ERROR_CODE.SUCCESS:
                self.logger.error(f"zed.open() failed: {err}")
                return False

            info        = self.zed.get_camera_information()
            calibration = info.camera_configuration.calibration_parameters

            self.logger.info(
                f"Connected: {info.camera_model} "
                f"serial={info.serial_number} "
                f"resolution={self._resolution_key} "
                f"fps={self.fps}"
            )

            if self.enable_color:
                self.init_color(calibration)

            if self.enable_depth:
                self.init_depth(calibration)

            if self.enable_imu:
                self.init_imu()

            self.ok.set()
            self.logger.info("ZED 2i started successfully")
            return True

        except Exception:
            self.ok.clear()
            self.logger.error(
                f"Unexpected error starting pipeline: {traceback.format_exc()}"
            )
            return False

    def cleanup(self):
        try:
            self.zed.close()
        except Exception as exc:
            self.logger.debug(f"zed.close() raised (device already gone?): {exc}")

        for pub in (self.left_pub, self.left_rectified_pub, self.stereo_pub, self.imu_pub):
            if pub:
                pub.undeclare()

        self.session.close()

    def spin(self):
        self._capture_thread = threading.Thread(
            target=self._capture_loop, daemon=True
        )
        self._capture_thread.start()

        try:
            self._capture_thread.join()
        except KeyboardInterrupt:
            self.logger.info("Interrupt received, exiting ...")
            self.ok.clear()
        except Exception:
            self.logger.error(f"Error: {traceback.format_exc()}")
            self.ok.clear()
        finally:
            self.cleanup()
            self._capture_thread.join(timeout=2.0)
            self.logger.info("ZED 2i wrapper successfully destroyed.")


# ─── entry point ─────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="""
ZED 2i zenoh wrapper.

Supported models
1. Stereolabs ZED 2i
   Wide-baseline (120 mm) stereo camera with on-board neural depth.
   Sensors: left/right RGB (BGRA), depth (float32 m), 6-DOF IMU.

Published topics
1. <topic-prefix>/color           (feather:MonoCameraFrame  — left camera)
2. <topic-prefix>/color/rectified (feather:MonoCameraFrame, if --enable-color-rectification)
3. <topic-prefix>/stereo          (feather:StereoCameraFrame)
4. <topic-prefix>/imu             (feather:Imu, if --enable-imu)

Resolution / max fps
--------------------
VGA    — 672 × 376   @ 100 fps
HD720  — 1280 × 720  @  60 fps
HD1080 — 1920 × 1080 @  30 fps  ← default
HD2K   — 2208 × 1242 @  15 fps
""",
        formatter_class=argparse.RawTextHelpFormatter,
    )
    parser.add_argument(
        "--serial",
        type=str,
        default="33647906",
        help="Device serial number. Empty string connects to first available device.",
    )
    parser.add_argument(
        "--enable-color",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Enable left RGB stream (default: on).",
    )
    parser.add_argument(
        "--enable-depth",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Enable depth stream (default: off).",
    )
    parser.add_argument(
        "--resolution",
        choices=list(_RESOLUTION_MAP),
        default="HD1080",
        help="Camera resolution (applies to both colour and depth).",
    )
    parser.add_argument(
        "--fps",
        type=int,
        default=30,
        help="Frame rate. Must be compatible with the chosen resolution.",
    )
    parser.add_argument(
        "--depth-mode",
        choices=list(_DEPTH_MODE_MAP),
        default="ultra",
        help="ZED depth quality preset.",
    )
    parser.add_argument(
        "--depth-min-m",
        type=float,
        default=0.3,
        help="Minimum depth threshold in metres (ZED 2i lower limit ~0.2 m).",
    )
    parser.add_argument(
        "--depth-max-m",
        type=float,
        default=10.0,
        help="Maximum depth threshold in metres.",
    )
    parser.add_argument(
        "--depth-confidence",
        type=int,
        default=95,
        help="Depth confidence threshold (0–100). Higher = fewer but more accurate points.",
    )
    parser.add_argument(
        "--enable-fill-mode",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Fill depth holes via interpolation (default: off).",
    )
    parser.add_argument(
        "--enable-imu",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Enable IMU stream on <prefix>/imu (default: off).",
    )
    parser.add_argument(
        "--enable-color-rectification",
        action="store_true",
        help="Publish undistorted left frames on <prefix>/color/rectified.",
    )
    parser.add_argument(
        "--color-rectification-alpha",
        type=float,
        default=0.0,
        help="getOptimalNewCameraMatrix alpha (0 = no black borders, 1 = full FOV).",
    )
    parser.add_argument(
        "--color-rectification-crop",
        action="store_true",
        help="Crop rectified image to valid ROI.",
    )
    parser.add_argument(
        "--topic-prefix",
        type=str,
        default="zed2i",
        help="Zenoh topic prefix (used to build default topic names).",
    )
    parser.add_argument(
        "--topic-color",
        type=str,
        default="",
        help="Zenoh topic for left colour frames. Defaults to <prefix>/color.",
    )
    parser.add_argument(
        "--topic-color-rectified",
        type=str,
        default="",
        help="Zenoh topic for rectified left colour frames. Defaults to <prefix>/color/rectified.",
    )
    parser.add_argument(
        "--topic-stereo",
        type=str,
        default="",
        help="Zenoh topic for stereo+depth frames. Defaults to <prefix>/stereo.",
    )
    parser.add_argument(
        "--topic-imu",
        type=str,
        default="",
        help="Zenoh topic for IMU data. Defaults to <prefix>/imu.",
    )

    args = vars(parser.parse_args())

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s — %(message)s",
    )
    args["logger"] = logging.getLogger("zed_2i")

    wrapper = Zed2i(**args)
    if not wrapper.start_pipeline():
        return
    wrapper.spin()


if __name__ == "__main__":
    main()
