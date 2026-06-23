#!/usr/bin/env python3

import argparse
import logging
import threading
import traceback

import cv2
import zenoh
import numpy as np
import pyrealsense2 as rs

import feather_pb2
from feather_pb2 import (
    MonoCameraFrame, StereoCameraFrame, CameraIntrinsics, Image, DType, DistortionModel
)

# ─── helpers ─────────────────────────────────────────────────────────────────

_RS_DISTORTION_MAP = {
    rs.distortion.brown_conrady:          DistortionModel.BROWN_CONRADY,
    rs.distortion.modified_brown_conrady: DistortionModel.BROWN_CONRADY,
    rs.distortion.inverse_brown_conrady:  DistortionModel.BROWN_CONRADY,
    rs.distortion.kannala_brandt4:        DistortionModel.KANNALA_BRANDIT,
}

_NP_DTYPE_MAP = {
    np.dtype("uint8"):   DType.UINT8,
    np.dtype("uint16"):  DType.UINT16,
    np.dtype("float32"): DType.F32,
    np.dtype("float64"): DType.F64,
}


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


def _rs_intrinsics_to_proto(intr: rs.intrinsics) -> CameraIntrinsics:
    return CameraIntrinsics(
        f=intr.fx,
        cx=intr.ppx,
        cy=intr.ppy,
        distortion_model=_RS_DISTORTION_MAP.get(intr.model, DistortionModel.NONE),
        distortion_parameters=list(intr.coeffs),
    )


# ─── wrapper class ───────────────────────────────────────────────────────────

class RealSenseD405:
    # D405 is a short-range global-shutter stereo depth camera (0.07 – 0.5 m).
    # It has no IMU (unlike D435i). Depth is reported in millimetres (Z16);
    # we convert to metres before publishing.

    def __init__(self, **kwargs):
        self.logger: logging.Logger = kwargs["logger"]
        self.serial: str = kwargs["serial"]
        self.enable_color: bool = kwargs["enable_color"]
        self.enable_depth: bool = kwargs["enable_depth"]
        self.color_fps: float = kwargs["color_fps"]
        self.depth_fps: float = kwargs["depth_fps"]
        self.color_resolution: tuple = (int(kwargs["color_width"]), int(kwargs["color_height"]))
        self.depth_resolution: tuple = (int(kwargs["depth_width"]), int(kwargs["depth_height"]))
        self.enable_color_rectification: bool = kwargs["enable_color_rectification"]
        self.color_rectification_alpha: float = kwargs["color_rectification_alpha"]
        self.color_rectification_crop: bool = kwargs["color_rectification_crop"]
        self.align_to_color: bool = kwargs["align_to_color"] and kwargs["enable_color"]
        self.depth_min_m: float = kwargs["depth_min_m"]
        self.depth_max_m: float = kwargs["depth_max_m"]
        node_name: str = kwargs.get("node_name") or kwargs.get("topic_prefix") or "d405"
        self.topic_prefix = f"{node_name}/"
        self.topic_color           = kwargs.get("topic_color")           or f"{self.topic_prefix}color"
        self.topic_color_rectified = kwargs.get("topic_color_rectified") or f"{self.topic_prefix}color/rectified"
        self.topic_stereo          = kwargs.get("topic_stereo")          or f"{self.topic_prefix}stereo"
        self.zenoh_config: str = kwargs.get("zenoh_config", "")

        self.pipeline: rs.pipeline = None
        self.profile: rs.pipeline_profile = None

        self._color_intrinsics: CameraIntrinsics = None
        self._color_rect_props: dict = {}
        self._depth_intrinsics: CameraIntrinsics = None
        self._baseline_m: float = 0.0

        # post-processing filters (initialised in init_depth)
        self._decimation = None
        self._threshold = None
        self._spatial = None
        self._temporal = None
        self._hole_filling = None

        self._depth_scale: float = 0.001

        self._capture_thread: threading.Thread = None
        self.ok = threading.Event()

        self.init_comms()

    # ── comms / zenoh ────────────────────────────────────────────────────────

    def init_comms(self):
        zenoh.try_init_log_from_env()
        if self.zenoh_config:
            import os
            cfg = zenoh.Config.from_file(self.zenoh_config) if os.path.isfile(self.zenoh_config) \
                  else zenoh.Config()
        else:
            cfg = zenoh.Config()
        self.session = zenoh.open(cfg)
        self.color_pub = None
        self.color_rectified_pub = None
        self.stereo_pub = None

    # ── per-stream init (called after pipeline.start) ────────────────────────

    def init_color(self):
        intr = rs.video_stream_profile(
            self.profile.get_stream(rs.stream.color)
        ).get_intrinsics()
        self._color_intrinsics = _rs_intrinsics_to_proto(intr)

        if self.enable_color_rectification:
            K = np.array([
                [intr.fx, 0,       intr.ppx],
                [0,       intr.fy, intr.ppy],
                [0,       0,       1       ],
            ])
            dist = np.array(intr.coeffs)
            K_rect, roi = cv2.getOptimalNewCameraMatrix(
                cameraMatrix=K,
                distCoeffs=dist,
                imageSize=self.color_resolution,
                alpha=self.color_rectification_alpha,
            )
            self._color_rect_props = {"K": K, "K_rect": K_rect, "dist": dist, "roi": roi}
            self.color_rectified_pub = self.session.declare_publisher(
                self.topic_color_rectified
            )

        self.color_pub = self.session.declare_publisher(self.topic_color)

    def init_depth(self):
        depth_sensor = self.profile.get_device().first_depth_sensor()
        self._depth_scale = depth_sensor.get_depth_scale()
        self.logger.info(f"Depth scale: {self._depth_scale} m/unit")

        intr = rs.video_stream_profile(
            self.profile.get_stream(rs.stream.depth)
        ).get_intrinsics()
        self._depth_intrinsics = _rs_intrinsics_to_proto(intr)

        # Baseline from left-IR → right-IR extrinsics.
        try:
            left = self.profile.get_stream(rs.stream.infrared, 1)
            right = self.profile.get_stream(rs.stream.infrared, 2)
            self._baseline_m = abs(left.get_extrinsics_to(right).translation[0])
        except RuntimeError:
            self._baseline_m = 0.018  # D405 nominal ~18 mm

        # Post-processing chain (mirroring OAK-D depth pipeline).
        self._decimation = rs.decimation_filter()
        self._decimation.set_option(rs.option.filter_magnitude, 1)

        self._threshold = rs.threshold_filter()
        self._threshold.set_option(rs.option.min_distance, self.depth_min_m)
        self._threshold.set_option(rs.option.max_distance, self.depth_max_m)

        self._spatial = rs.spatial_filter()
        self._spatial.set_option(rs.option.filter_magnitude, 2)
        self._spatial.set_option(rs.option.filter_smooth_alpha, 0.5)
        self._spatial.set_option(rs.option.filter_smooth_delta, 20)
        self._spatial.set_option(rs.option.holes_fill, 0)

        self._temporal = rs.temporal_filter()
        self._temporal.set_option(rs.option.filter_smooth_alpha, 0.4)
        self._temporal.set_option(rs.option.filter_smooth_delta, 20)

        self._hole_filling = rs.hole_filling_filter()

        self.stereo_pub = self.session.declare_publisher(self.topic_stereo)

    # ── per-frame callbacks ───────────────────────────────────────────────────

    def color_cb(self, color_frame: rs.video_frame):
        img = np.asanyarray(color_frame.get_data())

        msg = MonoCameraFrame(
            intrinsics=self._color_intrinsics,
            image=_to_image(img),
        )
        if self.ok.is_set():
            self.color_pub.put(msg.SerializeToString())

        if not self.enable_color_rectification or not self._color_rect_props:
            return

        props = self._color_rect_props
        img_rect = cv2.undistort(
            src=img,
            cameraMatrix=props["K"],
            distCoeffs=props["dist"],
            newCameraMatrix=props["K_rect"],
        )
        K_r = props["K_rect"]
        cx = K_r[0, 2]
        cy = K_r[1, 2]
        if self.color_rectification_crop:
            x, y, w, h = props["roi"]
            img_rect = img_rect[y:y + h, x:x + w]
            cx -= x
            cy -= y

        msg_rect = MonoCameraFrame(
            intrinsics=CameraIntrinsics(
                f=K_r[0, 0],
                cx=cx,
                cy=cy,
                distortion_model=DistortionModel.NONE,
                distortion_parameters=[],
            ),
            image=_to_image(img_rect),
        )
        if self.ok.is_set():
            self.color_rectified_pub.put(msg_rect.SerializeToString())

    def depth_cb(self, depth_frame: rs.depth_frame,
                 ir_left_frame: rs.video_frame, ir_right_frame: rs.video_frame):
        # Post-processing chain: decimation → threshold → spatial → temporal → hole-filling.
        f = self._decimation.process(depth_frame)
        f = self._threshold.process(f)
        f = self._spatial.process(f)
        f = self._temporal.process(f)
        f = self._hole_filling.process(f).as_depth_frame()

        depth_m  = np.asanyarray(f.get_data()).astype(np.float32) * self._depth_scale
        ir_left  = np.asanyarray(ir_left_frame.get_data())
        ir_right = np.asanyarray(ir_right_frame.get_data())

        msg = StereoCameraFrame(
            image_left=_to_image(ir_left),
            image_right=_to_image(ir_right),
            depth=_to_image(depth_m),
            intrinsics=self._depth_intrinsics,
            baseline=self._baseline_m,
        )
        if self.ok.is_set():
            self.stereo_pub.put(msg.SerializeToString())

    # ── capture loop (single thread: poll → align → dispatch) ────────────────

    def _capture_loop(self):
        while self.ok.is_set():
            try:
                frames = self.pipeline.wait_for_frames(timeout_ms=1000)
            except RuntimeError:
                self.logger.warning("wait_for_frames timed out — retrying")
                continue
            except Exception:
                self.logger.error(f"Capture error: {traceback.format_exc()}")
                self.ok.clear()
                break

            if self.enable_color:
                cf = frames.get_color_frame()
                if cf:
                    try:
                        self.color_cb(cf)
                    except Exception:
                        self.logger.error(f"color_cb error: {traceback.format_exc()}")

            if self.enable_depth:
                df  = frames.get_depth_frame()
                ifl = frames.get_infrared_frame(1)
                ifr = frames.get_infrared_frame(2)
                if df and ifl and ifr:
                    try:
                        self.depth_cb(df, ifl, ifr)
                    except Exception:
                        self.logger.error(f"depth_cb error: {traceback.format_exc()}")

    # ── lifecycle ─────────────────────────────────────────────────────────────

    def start_pipeline(self) -> bool:
        try:
            self.logger.info(
                f"Connecting to RealSense D405 (serial={self.serial or 'any'}) ..."
            )
            self.pipeline = rs.pipeline()
            cfg = rs.config()

            if self.serial:
                cfg.enable_device(self.serial)

            if self.enable_color:
                w, h = self.color_resolution
                cfg.enable_stream(rs.stream.color, w, h, rs.format.bgr8, int(self.color_fps))

            if self.enable_depth:
                w, h = self.depth_resolution
                cfg.enable_stream(rs.stream.depth, w, h, rs.format.z16, int(self.depth_fps))
                # IR streams: needed for accurate baseline; not published by default.
                cfg.enable_stream(rs.stream.infrared, 1, w, h, rs.format.y8, int(self.depth_fps))
                cfg.enable_stream(rs.stream.infrared, 2, w, h, rs.format.y8, int(self.depth_fps))

            self.profile = self.pipeline.start(cfg)

            dev = self.profile.get_device()
            usb = dev.get_info(rs.camera_info.usb_type_descriptor)
            self.logger.info(
                f"Connected: {dev.get_info(rs.camera_info.name)} "
                f"serial={dev.get_info(rs.camera_info.serial_number)} "
                f"fw={dev.get_info(rs.camera_info.firmware_version)} "
                f"usb={usb}"
            )
            if not usb.startswith("3"):
                self.logger.warning(
                    f"USB {usb} detected — D405 requires USB 3.x to stream; "
                    f"frames will time out on USB 2.x"
                )

            if self.enable_color:
                self.init_color()

            if self.enable_depth:
                self.init_depth()

            self.ok.set()
            self.logger.info("RealSense D405 started successfully")
            return True

        except RuntimeError:
            self.ok.clear()
            self.logger.error(f"Failed to connect to device: {traceback.format_exc()}")
            return False
        except Exception:
            self.ok.clear()
            self.logger.error(f"Unexpected error starting pipeline: {traceback.format_exc()}")
            return False

    def cleanup(self):
        if self.pipeline:
            try:
                self.pipeline.stop()
            except Exception:
                pass

        for pub in (self.color_pub, self.color_rectified_pub, self.stereo_pub):
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
            self.logger.info("RealSense D405 wrapper successfully destroyed.")


# ─── entry point ─────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="""
RealSense D405 zenoh wrapper.

Supported models
1. Intel RealSense D405
   Short-range global-shutter stereo depth (0.07 – 0.5 m).
   Sensors: depth (Z16), colour (BGR8), infrared L/R (Y8). No IMU.

Published topics
1. <node-name>/color           (feather:MonoCameraFrame)
2. <node-name>/color/rectified (feather:MonoCameraFrame, if --enable-color-rectification)
3. <node-name>/stereo          (feather:StereoCameraFrame)
""",
        formatter_class=argparse.RawTextHelpFormatter,
    )
    parser.add_argument(
        "--serial",
        type=str,
        default="",
        help="Device serial number. Empty string connects to first available device.",
    )
    parser.add_argument(
        "--enable-color",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Enable RGB colour stream (default: on).",
    )
    parser.add_argument(
        "--enable-depth",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Enable depth stream (default: off).",
    )
    parser.add_argument(
        "--color-fps",
        type=float,
        default=5.0,
        help="Colour stream frame rate (fps).",
    )
    parser.add_argument(
        "--depth-fps",
        type=float,
        default=30.0,
        help="Depth stream frame rate (fps).",
    )
    parser.add_argument(
        "--color-width",
        type=int,
        default=640,
        help="Colour stream width in pixels (default: 640).",
    )
    parser.add_argument(
        "--color-height",
        type=int,
        default=480,
        help="Colour stream height in pixels (default: 480).",
    )
    parser.add_argument(
        "--depth-width",
        type=int,
        default=640,
        help="Depth stream width in pixels (default: 640).",
    )
    parser.add_argument(
        "--depth-height",
        type=int,
        default=480,
        help="Depth stream height in pixels (default: 480).",
    )
    parser.add_argument(
        "--enable-color-rectification",
        action="store_true",
        help="Publish undistorted colour frames on <node-name>/color/rectified.",
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
        "--align-to-color",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Align depth to colour frame (default: on).",
    )
    parser.add_argument(
        "--depth-min-m",
        type=float,
        default=0.07,
        help="Minimum depth threshold in metres (D405 lower limit ~0.07 m).",
    )
    parser.add_argument(
        "--depth-max-m",
        type=float,
        default=0.5,
        help="Maximum depth threshold in metres (D405 upper limit ~0.5 m).",
    )
    parser.add_argument(
        "--node-name",
        type=str,
        default="d405",
        help="Zenoh topic prefix (used to build default topic names).",
    )
    parser.add_argument(
        "--topic-color",
        type=str,
        default="",
        help="Zenoh topic for colour frames. Defaults to <node-name>/color.",
    )
    parser.add_argument(
        "--topic-color-rectified",
        type=str,
        default="",
        help="Zenoh topic for rectified colour frames. Defaults to <node-name>/color/rectified.",
    )
    parser.add_argument(
        "--topic-stereo",
        type=str,
        default="",
        help="Zenoh topic for stereo+depth frames. Defaults to <node-name>/stereo.",
    )
    parser.add_argument(
        "--zenoh-config",
        type=str,
        default="",
        help="Path to a zenoh config file. Falls back to default config if not found.",
    )

    args = vars(parser.parse_args())

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s — %(message)s",
    )
    args["logger"] = logging.getLogger("realsense_d405")

    wrapper = RealSenseD405(**args)
    if not wrapper.start_pipeline():
        return
    wrapper.spin()


if __name__ == "__main__":
    main()