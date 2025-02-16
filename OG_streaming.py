import omnigibson as og
from omnigibson.macros import gm
from omnigibson import lazy
from omnigibson.macros import gm, macros

gm.REMOTE_STREAMING = "native"
gm.HEADLESS = True

cfg = dict()

cfg["env"] = {
    "device": "cpu",
}

cfg["scene"] = {
    "type": "Scene",
    # "scene_model": "Rs_int",
    "floor_plane_visible": True,
}

cfg["objects"] = [
    {
        "type": "USDObject",
        "name": "ghost_stain",
        "usd_path": f"{gm.ASSET_PATH}/models/stain/stain.usd",
        "category": "stain",
        "visual_only": True,
        "scale": [1.0, 1.0, 1.0],
        "position": [1.0, 2.0, 0.001],
        "orientation": [0, 0, 0, 1.0],
    },
]

cfg["robots"] = [
    {
        "type": "Fetch",
        "name": "skynet_robot",
        "obs_modalities": ["rgb", "depth"],
        "default_arm_pose": "diagonal30",
        "default_reset_mode": "tuck",
    },
]

env = og.Environment(cfg)

og.sim.enable_viewer_camera_teleoperation()

while True:
    og.sim.step()
