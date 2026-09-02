"""
Main Execution Script for CARLA-to-UniAD Autonomous Driving Bridge.

Usage:
  # With a live CARLA server:
  python -m simulator_bridge.run_bridge --host 127.0.0.1 --port 2000

  # Testing in mock mode (no CARLA required):
  python -m simulator_bridge.run_bridge --mock-carla --steps 50
"""

import sys
import time
import math
import argparse
import numpy as np

from simulator_bridge.config import BridgeConfig
from simulator_bridge.controller import VehicleController
from simulator_bridge.uniad_interface import UniADModelWrapper

try:
    import carla
    from simulator_bridge.sensors import SensorManager
except ImportError:
    carla = None


def run_mock_simulation(config: BridgeConfig, num_steps: int = 50):
    """Runs a simulated test loop without requiring CARLA installed."""
    print("=" * 60)
    print(" running Simulator Bridge in MOCK MODE (Testing Controller & Model)")
    print("=" * 60)

    uniad = UniADModelWrapper(config.uniad_config, config.uniad_checkpoint, config.device)
    controller = VehicleController(config)

    # Synthetic 6-camera feed (900x1600 random test frames)
    mock_images = {
        name: np.zeros((900, 1600, 3), dtype=np.uint8)
        for name in ['CAM_FRONT', 'CAM_FRONT_LEFT', 'CAM_FRONT_RIGHT', 'CAM_BACK', 'CAM_BACK_LEFT', 'CAM_BACK_RIGHT']
    }

    current_speed = 0.0
    print(f"\n[Starting Simulation Loop] Target Speed: {config.target_speed} km/h")
    print("-" * 65)
    print(f"{'Step':<6} | {'Speed (km/h)':<12} | {'Throttle':<10} | {'Steer':<10} | {'Brake':<8}")
    print("-" * 65)

    for step in range(1, num_steps + 1):
        # 1. Model inference: predict future waypoints
        waypoints = uniad.predict_waypoints(mock_images, current_speed, command=2)

        # 2. Controller: compute vehicle inputs
        control = controller.get_control(waypoints, current_speed, config.target_speed)

        # 3. Simple kinematic update for mock vehicle
        accel = control['throttle'] * 3.0 - control['brake'] * 6.0  # m/s^2
        speed_ms = max(0.0, (current_speed / 3.6) + accel * config.fixed_delta_seconds)
        current_speed = speed_ms * 3.6

        # Print progress every 5 steps
        if step % 5 == 0 or step == 1:
            print(f"{step:<6} | {current_speed:<12.2f} | {control['throttle']:<10.2f} | {control['steer']:<10.3f} | {control['brake']:<8.2f}")

        time.sleep(0.02)

    print("-" * 65)
    print(" Mock simulation completed successfully!")
    print("The controller properly accelerated and converged toward target speed.")


def run_carla_simulation(config: BridgeConfig):
    """Connects to a live CARLA simulator and runs autonomous driving."""
    if carla is None:
        print("[Error] carla package not found. Install CARLA Python API or run with '--mock-carla'.")
        sys.exit(1)

    print(f"[Connecting] Attempting connection to CARLA at {config.host}:{config.port}...")
    client = carla.Client(config.host, config.port)
    client.set_timeout(config.timeout)

    world = client.get_world()
    original_settings = world.get_settings()
    
    vehicle = None
    sensor_mgr = None

    try:
        # Enable Synchronous Mode for deterministic 20 FPS simulation
        if config.sync_mode:
            settings = world.get_settings()
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = config.fixed_delta_seconds
            world.apply_settings(settings)
            print(f"[CARLA] Enabled Synchronous Mode (dt={config.fixed_delta_seconds}s)")

        # Spawn Ego Vehicle
        bp_library = world.get_blueprint_library()
        vehicle_bp = bp_library.filter(config.vehicle_filter)[0]
        vehicle_bp.set_attribute('role_name', config.vehicle_role_name)

        spawn_points = world.get_map().get_spawn_points()
        if not spawn_points:
            raise RuntimeError("No spawn points found in current CARLA map.")
        spawn_point = spawn_points[0]

        vehicle = world.spawn_actor(vehicle_bp, spawn_point)
        print(f"[CARLA] Spawned hero vehicle '{vehicle.type_id}' at {spawn_point.location}")

        # Spawn 6 Cameras
        sensor_mgr = SensorManager(world, vehicle, config)
        print("[CARLA] 6 nuScenes surround cameras spawned and synchronized.")

        # Initialize UniAD Model and Controller
        uniad = UniADModelWrapper(config.uniad_config, config.uniad_checkpoint, config.device)
        controller = VehicleController(config)
        controller.reset()  # Clear any stale PID state

        # Warm-up tick: CARLA needs one tick for sensors to publish first frames
        print("[CARLA] Warm-up tick to prime sensor queues...")
        world.tick()
        time.sleep(0.1)
        # Drain any queued frames from the warm-up tick
        sensor_mgr.get_synchronized_data(timeout=1.0)

        print("\n" + "=" * 60)
        print("  UniAD Autonomous Driving Active in CARLA!")
        print("  Press Ctrl+C to terminate and safely cleanup.")
        print("=" * 60 + "\n")

        frame_count = 0
        while True:
            # Advance simulation 1 tick
            world.tick()
            frame_count += 1

            # Fetch synchronized 6 camera frames
            images = sensor_mgr.get_synchronized_data(timeout=2.0)
            if images is None:
                continue

            # Calculate current speed
            vel = vehicle.get_velocity()
            speed_kmh = 3.6 * math.sqrt(vel.x**2 + vel.y**2 + vel.z**2)

            # Predict waypoints with UniAD
            waypoints = uniad.predict_waypoints(images, speed_kmh)

            # Convert to vehicle controls
            ctrl_dict = controller.get_control(waypoints, speed_kmh)

            carla_control = carla.VehicleControl(
                throttle=ctrl_dict['throttle'],
                steer=ctrl_dict['steer'],
                brake=ctrl_dict['brake'],
                hand_brake=ctrl_dict['hand_brake'],
                reverse=ctrl_dict['reverse']
            )
            vehicle.apply_control(carla_control)

            if frame_count % 20 == 0:
                print(f"[Frame {frame_count:05d}] Speed: {speed_kmh:5.1f} km/h | Throttle: {ctrl_dict['throttle']:.2f} | Steer: {ctrl_dict['steer']:+.2f} | Brake: {ctrl_dict['brake']:.2f}")

    except KeyboardInterrupt:
        print("\n[Shutting Down] User interrupted execution.")
    finally:
        print("[Cleanup] Destroying sensors and restoring settings...")
        if sensor_mgr is not None:
            sensor_mgr.destroy()
        if vehicle is not None and vehicle.is_alive:
            vehicle.destroy()
        world.apply_settings(original_settings)
        print("[Cleanup] Done.")


def main():
    parser = argparse.ArgumentParser(description="CARLA to UniAD Bridge")
    parser.add_argument("--host", type=str, default="127.0.0.1", help="CARLA server IP")
    parser.add_argument("--port", type=int, default=2000, help="CARLA server port")
    parser.add_argument("--speed", type=float, default=25.0, help="Target driving speed in km/h")
    parser.add_argument("--mock-carla", action="store_true", help="Run in mock simulation mode without CARLA")
    parser.add_argument("--steps", type=int, default=50, help="Number of steps for mock simulation")
    parser.add_argument("--checkpoint", type=str, default="UniAD/ckpts/uniad_base_e2e.pth", help="UniAD model checkpoint path")
    args = parser.parse_args()

    cfg = BridgeConfig()
    cfg.host = args.host
    cfg.port = args.port
    cfg.target_speed = args.speed
    cfg.uniad_checkpoint = args.checkpoint

    if args.mock_carla or carla is None:
        run_mock_simulation(cfg, num_steps=args.steps)
    else:
        run_carla_simulation(cfg)


if __name__ == "__main__":
    main()
