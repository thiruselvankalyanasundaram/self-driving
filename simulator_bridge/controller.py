"""
Vehicle Controller for Autonomous Driving in CARLA.
Converts UniAD predicted waypoints into low-level steering, throttle, and brake inputs.
"""

import math
import numpy as np
from typing import List, Tuple

class LongitudinalPID:
    """PID controller for vehicle speed regulation."""
    def __init__(self, kp: float = 0.5, ki: float = 0.05, kd: float = 0.1, dt: float = 0.05):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.integral = 0.0
        self.prev_error = 0.0

    def step(self, target_speed: float, current_speed: float) -> float:
        """
        Compute throttle/brake control value.
        Positive value -> Throttle [0, 1]
        Negative value -> Brake [0, 1]
        """
        error = target_speed - current_speed
        self.integral += error * self.dt
        # Anti-windup clamping
        self.integral = max(-10.0, min(10.0, self.integral))
        derivative = (error - self.prev_error) / self.dt if self.dt > 0 else 0.0
        self.prev_error = error

        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        return output

    def reset(self):
        """Reset integrator and derivative state. Must be called between CARLA episodes."""
        self.integral = 0.0
        self.prev_error = 0.0


class LateralPurePursuit:
    """Pure Pursuit + Stanley controller to track UniAD waypoints."""
    def __init__(self, wheelbase: float = 2.875, min_lookahead: float = 3.0, max_lookahead: float = 12.0):
        self.wheelbase = wheelbase
        self.min_lookahead = min_lookahead
        self.max_lookahead = max_lookahead

    def step(self, waypoints: np.ndarray, current_speed_kmh: float) -> float:
        """
        Compute steering angle in range [-1.0, 1.0].
        
        Args:
            waypoints: 2D array of (x, y) coordinates relative to the vehicle frame.
                       x is forward, y is left/lateral. Shape: (N, 2).
            current_speed_kmh: Current vehicle speed in km/h.
            
        Returns:
            steer: Steering command in [-1.0, 1.0] (negative = left, positive = right in CARLA)
        """
        if waypoints is None or len(waypoints) == 0:
            return 0.0

        # Target waypoint selection
        # Automatic axis detection: In autonomous driving, forward motion is monotonically increasing
        # over the planning horizon (e.g. 0m to 20m), while lateral lane offset is small (e.g. -2m to +2m).
        col0_range = np.ptp(waypoints[:, 0])
        col1_range = np.ptp(waypoints[:, 1])

        if col1_range > col0_range:
            # nuScenes / UniAD default: col 1 is forward (Y), col 0 is lateral (X)
            forward_col = 1
            lateral_col = 0
            # nuScenes: positive X is RIGHT. In CARLA, positive steer turns RIGHT.
            steer_sign = 1.0
        else:
            # Standard vehicle frame: col 0 is forward (X), col 1 is lateral (Y)
            forward_col = 0
            lateral_col = 1
            # Standard: positive Y is LEFT. In CARLA, positive steer turns RIGHT, so negate.
            steer_sign = -1.0

        forward_dists = waypoints[:, forward_col]
        lateral_offsets = waypoints[:, lateral_col]

        # Dynamic lookahead based on speed
        speed_ms = current_speed_kmh / 3.6
        lookahead = np.clip(speed_ms * 0.8, self.min_lookahead, self.max_lookahead)

        # Find target waypoint closest to lookahead distance along forward path
        target_idx = np.argmin(np.abs(forward_dists - lookahead))
        tx = float(forward_dists[target_idx])
        ty = float(lateral_offsets[target_idx])

        if tx <= 0.2:
            return 0.0

        # Pure Pursuit curvature: kappa = 2 * y / L^2
        dist_sq = tx**2 + ty**2
        curvature = 2.0 * ty / max(dist_sq, 1.0)
        
        # Steering angle = atan(curvature * wheelbase)
        steering_angle = math.atan(curvature * self.wheelbase)

        # Normalize to CARLA steer range [-1.0, 1.0]
        max_steer_angle_rad = math.radians(35.0)  # ~35 degrees max steering
        normalized_steer = float(np.clip(steer_sign * (steering_angle / max_steer_angle_rad), -1.0, 1.0))
        return normalized_steer


class VehicleController:
    """Combined Longitudinal + Lateral vehicle controller."""
    def __init__(self, config):
        self.lon_controller = LongitudinalPID(
            kp=config.kp_lon, ki=config.ki_lon, kd=config.kd_lon, dt=config.fixed_delta_seconds
        )
        self.lat_controller = LateralPurePursuit(
            wheelbase=config.wheelbase,
            min_lookahead=config.lookahead_distance
        )
        self.default_target_speed = config.target_speed

    def get_control(self, waypoints: np.ndarray, current_speed_kmh: float, target_speed_kmh: float = None):
        """
        Calculate CARLA throttle, steer, and brake controls.
        """
        if target_speed_kmh is None:
            target_speed_kmh = self.default_target_speed

        # Lateral control
        steer = self.lat_controller.step(waypoints, current_speed_kmh)

        # Slow down on sharp turns
        if abs(steer) > 0.3:
            target_speed_kmh *= max(0.4, (1.0 - abs(steer)))

        # Longitudinal control
        accel = self.lon_controller.step(target_speed_kmh, current_speed_kmh)

        if accel >= 0.0:
            throttle = float(np.clip(accel, 0.0, 1.0))
            brake = 0.0
        else:
            throttle = 0.0
            brake = float(np.clip(-accel, 0.0, 1.0))

        return {
            'throttle': throttle,
            'steer': steer,
            'brake': brake,
            'hand_brake': False,
            'reverse': False
        }

    def reset(self):
        """Reset controller state between CARLA episodes to avoid stale integral lurch."""
        self.lon_controller.reset()
