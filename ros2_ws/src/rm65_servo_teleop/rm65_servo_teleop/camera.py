"""Independent D435i RGB-D acquisition, with explicit synthetic backend."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import math
import importlib.metadata
import time

import numpy as np

from .peripheral_clock import clock_identity


@dataclass(frozen=True)
class CameraConfig:
    serial: str = ""
    width: int = 640
    height: int = 480
    fps: int = 30
    timeout_ms: int = 1000
    queue_size: int = 8
    align_depth_to_color: bool = False

    def __post_init__(self):
        for name, low, high in (
            ("width", 16, 1920),
            ("height", 16, 1080),
            ("fps", 1, 60),
            ("timeout_ms", 100, 5000),
            ("queue_size", 1, 32),
        ):
            value = getattr(self, name)
            if type(value) is not int or not low <= value <= high:
                raise ValueError(f"invalid camera {name}")
        if not isinstance(self.serial, str) or any(c.isspace() for c in self.serial):
            raise ValueError("camera serial must be a string without whitespace")
        if type(self.align_depth_to_color) is not bool:
            raise ValueError("align_depth_to_color must be boolean")
        if self.width * self.height * 7 * self.queue_size > 256 * 1024**2:
            raise ValueError("camera queue exceeds the 256 MiB raw frame budget")


@dataclass(frozen=True)
class RGBDFrame:
    color: np.ndarray
    depth: np.ndarray
    metadata: dict
    aligned_depth: np.ndarray | None = None

    def validate(self):
        h, w = self.depth.shape if self.depth.ndim == 2 else (0, 0)
        if (
            not h
            or not w
            or self.depth.dtype != np.uint16
            or self.color.dtype != np.uint8
            or self.color.shape != (h, w, 3)
        ):
            raise ValueError("RGB uint8 (H,W,3) and native Z16 uint16 (H,W) required")
        if self.aligned_depth is not None and (
            self.aligned_depth.shape != (h, w) or self.aligned_depth.dtype != np.uint16
        ):
            raise ValueError("invalid aligned depth")
        for name in ("host_received_monotonic_s", "color_timestamp_ms", "depth_timestamp_ms", "depth_scale_m"):
            v = self.metadata.get(name)
            if type(v) not in (int, float) or not math.isfinite(v) or v < 0:
                raise ValueError(f"invalid frame {name}")
        if self.metadata["depth_scale_m"] <= 0 or type(self.metadata.get("synthetic")) is not bool:
            raise ValueError("explicit depth scale and source required")
        for name in ("color_frame_number", "depth_frame_number"):
            if type(self.metadata.get(name)) is not int or self.metadata[name] < 0:
                raise ValueError("explicit device frame numbers required")
        return self


class MockCamera:
    def __init__(self, config):
        self.config = config
        self.sequence = 0
        self.metadata = {
            "source": "mock",
            "synthetic": True,
            "config": asdict(config),
            "serial": "synthetic-d435i",
            "calibration": None,
            "firmware": None,
        }

    def start(self):
        return self

    def read(self):
        self.sequence += 1
        c = self.config
        color = np.zeros((c.height, c.width, 3), dtype=np.uint8)
        color[:, :, 0] = self.sequence % 256
        depth = np.full((c.height, c.width), 1000 + self.sequence, dtype=np.uint16)
        metadata = {
            "synthetic": True,
            "host_received_monotonic_s": time.monotonic(),
            "clock_identity": clock_identity(),
            "timestamp_domain": {"color": "synthetic", "depth": "synthetic"},
            "color_timestamp_ms": self.sequence * 1000 / c.fps,
            "depth_timestamp_ms": self.sequence * 1000 / c.fps,
            "color_frame_number": self.sequence,
            "depth_frame_number": self.sequence,
            "depth_scale_m": 0.001,
            "synchronization": "unsynchronized",
            "exposure_metadata": {},
        }
        return RGBDFrame(color, depth, metadata, depth.copy() if c.align_depth_to_color else None)

    def close(self):
        pass


def enumerate_devices(rs=None):
    if rs is None:
        import pyrealsense2 as rs
    result = []
    for device in rs.context().query_devices():
        info = {}
        for key in ("name", "serial_number", "firmware_version", "usb_type_descriptor"):
            field = getattr(rs.camera_info, key)
            info[key] = device.get_info(field) if device.supports(field) else None
        info["profiles"] = [str(p) for sensor in device.query_sensors() for p in sensor.get_stream_profiles()]
        result.append(info)
    return result


class RealSenseCamera:
    """Lazy SDK import. No reset, firmware update, calibration or option writes."""

    def __init__(self, config: CameraConfig, *, rs=None):
        if not config.serial:
            raise ValueError("explicit D435i serial required; auto-select is forbidden")
        self.config, self.rs = config, rs
        self.pipeline = None
        self.started = False

    def start(self):
        if self.pipeline is not None:
            raise RuntimeError("camera cannot auto-restart after failure")
        if self.rs is None:
            import pyrealsense2 as rs

            self.rs = rs
        rs, c = self.rs, self.config
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_device(c.serial)
        config.enable_stream(rs.stream.color, c.width, c.height, rs.format.rgb8, c.fps)
        config.enable_stream(rs.stream.depth, c.width, c.height, rs.format.z16, c.fps)
        try:
            profile = self.pipeline.start(config)  # Unsupported combinations fail, never fall back.
            self.started = True
            device = profile.get_device()
            name = device.get_info(rs.camera_info.name)
            if "D435I" not in name.upper() or device.get_info(rs.camera_info.serial_number) != c.serial:
                raise ValueError("selected device is not the explicitly selected D435i")
            color = profile.get_stream(rs.stream.color).as_video_stream_profile()
            depth = profile.get_stream(rs.stream.depth).as_video_stream_profile()
            extrinsics = depth.get_extrinsics_to(color)
            self.scale = float(device.first_depth_sensor().get_depth_scale())
            if not math.isfinite(self.scale) or self.scale <= 0:
                raise ValueError("invalid device depth scale")

            def intrinsic(stream):
                i = stream.get_intrinsics()
                return {
                    "width": i.width,
                    "height": i.height,
                    "fx": i.fx,
                    "fy": i.fy,
                    "ppx": i.ppx,
                    "ppy": i.ppy,
                    "model": str(i.model),
                    "coeffs": list(i.coeffs),
                }

            self.metadata = {
                "source": "realsense",
                "synthetic": False,
                "config": asdict(c),
                "serial": c.serial,
                "device_name": name,
                "firmware": device.get_info(rs.camera_info.firmware_version),
                "usb_type": (
                    device.get_info(rs.camera_info.usb_type_descriptor)
                    if device.supports(rs.camera_info.usb_type_descriptor)
                    else None
                ),
                "intrinsics": {"color": intrinsic(color), "depth": intrinsic(depth)},
                "depth_to_color": {
                    "rotation_column_major": list(extrinsics.rotation),
                    "translation_m": list(extrinsics.translation),
                },
                "depth_scale_m": self.scale,
                "hand_eye": None,
                "synchronization": "unsynchronized",
                "imu_recorded": False,
                "sdk_version": (
                    importlib.metadata.version("pyrealsense2")
                    if rs.__class__.__name__ == "module"
                    else "injected-test-sdk"
                ),
            }
            for intrinsics in self.metadata["intrinsics"].values():
                if (intrinsics["width"], intrinsics["height"]) != (c.width, c.height):
                    raise ValueError("device returned an unexpected stream profile")
            self.align = rs.align(rs.stream.color) if c.align_depth_to_color else None
            return self
        except Exception:
            self.close()
            raise

    def read(self):
        frames = self.pipeline.wait_for_frames(self.config.timeout_ms)
        received = time.monotonic()  # Before copying or alignment.
        color, depth = frames.get_color_frame(), frames.get_depth_frame()
        if not color or not depth:
            raise RuntimeError("missing RGB/depth frame")
        exposure = {}
        for name, frame in (("color", color), ("depth", depth)):
            exposure[name] = {}
            for key in ("actual_exposure", "sensor_timestamp", "frame_timestamp", "backend_timestamp"):
                field = getattr(self.rs.frame_metadata_value, key, None)
                exposure[name][key] = (
                    frame.get_frame_metadata(field)
                    if field is not None and frame.supports_frame_metadata(field)
                    else None
                )
        metadata = {
            "synthetic": False,
            "host_received_monotonic_s": received,
            "clock_identity": clock_identity(),
            "color_timestamp_ms": float(color.get_timestamp()),
            "depth_timestamp_ms": float(depth.get_timestamp()),
            "color_frame_number": int(color.get_frame_number()),
            "depth_frame_number": int(depth.get_frame_number()),
            "depth_scale_m": self.scale,
            "timestamp_domain": {
                "color": str(color.get_frame_timestamp_domain()),
                "depth": str(depth.get_frame_timestamp_domain()),
            },
            "exposure_metadata": exposure,
            "synchronization": "unsynchronized",
        }
        # Own the buffers beyond the SDK frame lifetime; native depth is always retained.
        rgb = np.asanyarray(color.get_data()).copy()
        z16 = np.asanyarray(depth.get_data()).copy()
        aligned = (
            None
            if self.align is None
            else np.asanyarray(self.align.process(frames).get_depth_frame().get_data()).copy()
        )
        return RGBDFrame(rgb, z16, metadata, aligned).validate()

    def close(self):
        if self.started:
            self.started = False
            self.pipeline.stop()
