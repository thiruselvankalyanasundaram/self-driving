"""
Sensor Management and Data Synchronization for CARLA.
Spawns the 6 surround cameras on the ego vehicle and provides
synchronized multi-view frames for the UniAD model.
"""

import queue
import numpy as np
from typing import Dict, Any, Optional
from simulator_bridge.config import NU_SCENES_CAMERAS, CameraConfig

try:
    import carla
except ImportError:
    carla = None


class SensorManager:
    """Manages the 6 surround cameras and synchronizes their output frames."""
    
    def __init__(self, world, ego_vehicle, config):
        if carla is None:
            raise RuntimeError("CARLA Python API is not installed. Please install carla or run with a mock simulator.")
        
        self.world = world
        self.ego_vehicle = ego_vehicle
        self.config = config
        self.sensor_dict: Dict[str, Any] = {}
        self.queues: Dict[str, queue.Queue] = {}
        self._spawn_cameras()

    def _spawn_cameras(self):
        """Spawns the 6 nuScenes cameras attached to the ego vehicle."""
        bp_library = self.world.get_blueprint_library()

        for cam_name, cam_cfg in NU_SCENES_CAMERAS.items():
            # Configure camera blueprint
            bp = bp_library.find('sensor.camera.rgb')
            bp.set_attribute('image_size_x', str(cam_cfg.width))
            bp.set_attribute('image_size_y', str(cam_cfg.height))
            bp.set_attribute('fov', str(cam_cfg.fov))
            bp.set_attribute('sensor_tick', str(self.config.fixed_delta_seconds))

            # Set transform relative to vehicle
            transform = carla.Transform(
                carla.Location(x=cam_cfg.x, y=cam_cfg.y, z=cam_cfg.z),
                carla.Rotation(pitch=cam_cfg.pitch, yaw=cam_cfg.yaw, roll=cam_cfg.roll)
            )

            # Spawn actor
            sensor = self.world.spawn_actor(bp, transform, attach_to=self.ego_vehicle)
            sensor_queue = queue.Queue()
            
            # Setup callback with camera name tag
            def make_callback(q, name):
                return lambda data: q.put((data.frame, name, data))

            sensor.listen(make_callback(sensor_queue, cam_name))
            self.sensor_dict[cam_name] = sensor
            self.queues[cam_name] = sensor_queue

    def get_synchronized_data(self, timeout: float = 2.0) -> Optional[Dict[str, np.ndarray]]:
        """
        Retrieves synchronized RGB frames from all 6 cameras for the current simulation tick.
        Validates that all 6 cameras share the same frame_id to prevent stale/mixed frames.

        Returns:
            Dict mapping camera name ('CAM_FRONT', etc.) to (H, W, 3) RGB uint8 image array,
            or None on timeout or frame ID mismatch.
        """
        images = {}
        frame_ids = {}
        for cam_name, q in self.queues.items():
            try:
                frame_id, name, raw_img = q.get(timeout=timeout)
                # Convert raw BGRA carla.Image to RGB numpy array
                img_array = np.frombuffer(raw_img.raw_data, dtype=np.uint8)
                img_array = img_array.reshape((raw_img.height, raw_img.width, 4))
                img_rgb = img_array[:, :, :3][:, :, ::-1]  # BGRA to RGB
                images[name] = img_rgb
                frame_ids[cam_name] = frame_id
            except queue.Empty:
                print(f"[Warning] Timeout waiting for frame from {cam_name}")
                return None

        # Validate all cameras produced frames from the same simulation tick
        unique_frames = set(frame_ids.values())
        if len(unique_frames) > 1:
            print(f"[Warning] Frame ID mismatch across cameras: {frame_ids}. Dropping frame.")
            return None

        return images

    def destroy(self):
        """Cleans up all spawned camera sensors."""
        for name, sensor in self.sensor_dict.items():
            if sensor is not None and sensor.is_alive:
                sensor.stop()
                sensor.destroy()
        self.sensor_dict.clear()
        self.queues.clear()
