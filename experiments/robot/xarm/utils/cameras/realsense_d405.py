import pyrealsense2 as rs
import yaml
import numpy as np
import os

config_path = os.path.join(os.path.dirname(__file__), "camera_config.yaml")

def save_config(cfg, path=config_path):
    with open(path, "w") as f:
        yaml.safe_dump(cfg, f)
    print(f"[Config] Saved -> {path}")


def load_or_create_config(path=config_path):
    import os

    if os.path.exists(path):
        print(f"[Config] Loaded from {path}")
        with open(path, "r") as f:
            return yaml.safe_load(f)

    # If not exist -> create
    print(f"[Config] Not found, creating new one at {path}")

    ctx = rs.context()
    devices = ctx.query_devices()
    serials = [d.get_info(rs.camera_info.serial_number) for d in devices]

    cfg = {
        "cameras": [
            {"serial": s, "width": 640, "height": 480, "fps": 30, "exposure": 8000}
            for s in serials
        ]
    }

    save_config(cfg, path)
    return cfg


class RealSenseD405:
    def __init__(self, cfg):
        """
        cfg: dict {serial, width, height, fps, exposure}
        """
        self.cfg = cfg   # <-- THIS IS IMPORTANT

        self.serial = cfg["serial"]
        self.width = cfg["width"]
        self.height = cfg["height"]
        self.fps = cfg["fps"]

        self.pipeline = rs.pipeline()
        self.config = rs.config()

        self.config.enable_device(self.serial)
        self.config.enable_stream(rs.stream.color,
                                  self.width, self.height,
                                  rs.format.bgr8, self.fps)

        self.profile = self.pipeline.start(self.config)

        # D405 uses Stereo Module as "color"
        self.sensor = self.profile.get_device().query_sensors()[0]

        self.set_exposure(cfg["exposure"])

        print(f"[D405] Started {self.serial}, exposure={cfg['exposure']}")

    def set_exposure(self, value):
        """Update camera exposure and update cfg dict."""
        try:
            self.sensor.set_option(rs.option.enable_auto_exposure, 0)
            self.sensor.set_option(rs.option.exposure, float(value))
            self.cfg["exposure"] = int(value)  # update config dict
            print(f"[D405] Exposure -> {value}")
        except Exception as e:
            print("[D405] Exposure update failed:", e)

    def get_color_frame(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            return np.zeros((self.height, self.width, 3), dtype=np.uint8)
        return np.asanyarray(color_frame.get_data())

    def stop(self):
        self.pipeline.stop()
