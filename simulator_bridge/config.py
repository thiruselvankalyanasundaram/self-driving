"""
Configuration for the CARLA to UniAD Simulator Bridge.
Defines sensor layout (6 surround cameras matching nuScenes), 
controller parameters, and CARLA server settings.
"""

from dataclasses import dataclass, field
from typing import Dict, List, Tuple

@dataclass
class CameraConfig:
    name: str
    x: float       # Forward relative to vehicle center (meters)
    y: float       # Right relative to vehicle center (meters)
    z: float       # Up relative to vehicle center (meters)
    pitch: float   # Degrees
    yaw: float     # Degrees
    roll: float    # Degrees
    fov: float = 70.0
    width: int = 1600
    height: int = 900

# 6 Surround-view cameras calibrated to nuScenes sensor setup
NU_SCENES_CAMERAS: Dict[str, CameraConfig] = {
    'CAM_FRONT': CameraConfig(
        name='CAM_FRONT',
        x=1.5, y=0.0, z=1.5,
        pitch=0.0, yaw=0.0, roll=0.0,
        fov=70.0, width=1600, height=900
    ),
    'CAM_FRONT_LEFT': CameraConfig(
        name='CAM_FRONT_LEFT',
        x=1.3, y=-0.5, z=1.5,
        pitch=0.0, yaw=-55.0, roll=0.0,
        fov=70.0, width=1600, height=900
    ),
    'CAM_FRONT_RIGHT': CameraConfig(
        name='CAM_FRONT_RIGHT',
        x=1.3, y=0.5, z=1.5,
        pitch=0.0, yaw=55.0, roll=0.0,
        fov=70.0, width=1600, height=900
    ),
    'CAM_BACK': CameraConfig(
        name='CAM_BACK',
        x=-1.5, y=0.0, z=1.5,
        pitch=0.0, yaw=180.0, roll=0.0,
        fov=110.0, width=1600, height=900
    ),
    'CAM_BACK_LEFT': CameraConfig(
        name='CAM_BACK_LEFT',
        x=-1.3, y=-0.5, z=1.5,
        pitch=0.0, yaw=-110.0, roll=0.0,
        fov=70.0, width=1600, height=900
    ),
    'CAM_BACK_RIGHT': CameraConfig(
        name='CAM_BACK_RIGHT',
        x=-1.3, y=0.5, z=1.5,
        pitch=0.0, yaw=110.0, roll=0.0,
        fov=70.0, width=1600, height=900
    ),
}

@dataclass
class BridgeConfig:
    # CARLA connection
    host: str = "127.0.0.1"
    port: int = 2000
    timeout: float = 10.0
    sync_mode: bool = True
    fixed_delta_seconds: float = 0.05  # 20 FPS simulation
    
    # Ego vehicle
    vehicle_filter: str = "vehicle.tesla.model3"
    vehicle_role_name: str = "hero"
    
    # UniAD Model settings
    uniad_config: str = "UniAD/projects/configs/stage2_e2e/base_e2e.py"
    uniad_checkpoint: str = "UniAD/ckpts/uniad_base_e2e.pth"
    device: str = "cuda"  # Fallback to cpu/mps if testing
    
    # Controller gains
    target_speed: float = 25.0  # km/h
    # Longitudinal PID (throttle / brake)
    kp_lon: float = 0.5
    ki_lon: float = 0.05
    kd_lon: float = 0.1
    # Lateral steering uses Pure Pursuit (geometric), not PID — no gain tuning needed.
    wheelbase: float = 2.875  # Tesla Model 3 wheelbase (meters)
    lookahead_distance: float = 5.0  # meters
