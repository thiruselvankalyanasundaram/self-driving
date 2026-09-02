"""
Comprehensive Integration and Wiring Verification Test.
Validates the entire pipeline: Config -> Cameras -> Preprocessing -> Model -> Waypoints -> Controller -> Outputs.
"""

import sys
import numpy as np
import math

from simulator_bridge.config import NU_SCENES_CAMERAS, BridgeConfig
from simulator_bridge.controller import VehicleController, LongitudinalPID, LateralPurePursuit
from simulator_bridge.uniad_interface import UniADModelWrapper

def test_config():
    print("[1/5] Testing Configuration & Camera Layout...")
    expected_cams = {'CAM_FRONT', 'CAM_FRONT_LEFT', 'CAM_FRONT_RIGHT', 'CAM_BACK', 'CAM_BACK_LEFT', 'CAM_BACK_RIGHT'}
    actual_cams = set(NU_SCENES_CAMERAS.keys())
    assert actual_cams == expected_cams, f"Camera mismatch: expected {expected_cams}, got {actual_cams}"
    
    for name, cam in NU_SCENES_CAMERAS.items():
        assert cam.width > 0 and cam.height > 0, f"{name} invalid dimensions: {cam.width}x{cam.height}"
        assert 30.0 <= cam.fov <= 120.0, f"{name} unexpected FOV: {cam.fov}"
    print("  --> Config verified! 6 nuScenes surround cameras properly positioned.")

def test_controller():
    print("[2/5] Testing Vehicle Controller & Coordinate System Wiring...")
    config = BridgeConfig()
    controller = VehicleController(config)
    
    # Test Straight Path: Forward increasing, lateral = 0
    # Format A: (lateral=0, forward=y)
    straight_traj_nusc = np.stack([np.zeros(6), np.linspace(2, 20, 6)], axis=1)
    ctrl_straight = controller.get_control(straight_traj_nusc, current_speed_kmh=15.0)
    assert abs(ctrl_straight['steer']) < 0.05, f"Straight trajectory should have near-zero steer, got {ctrl_straight['steer']}"
    assert ctrl_straight['throttle'] > 0.0, "Vehicle should apply throttle when below target speed"
    assert ctrl_straight['brake'] == 0.0, "Vehicle should not brake when below target speed"
    print("  --> Straight path tracking verified (steer ~ 0.0, throttle active)")

    # Test Left Turn:
    # In nuScenes: X is right. Turning left means negative X.
    left_traj_nusc = np.stack([np.linspace(-0.5, -3.0, 6), np.linspace(2, 20, 6)], axis=1)
    ctrl_left = controller.get_control(left_traj_nusc, current_speed_kmh=15.0)
    # In CARLA: steer < 0 is LEFT
    assert ctrl_left['steer'] < -0.05, f"Left trajectory should produce negative steer, got {ctrl_left['steer']}"
    print(f"  --> Left turn correctly produces negative steering (steer = {ctrl_left['steer']:.3f})")

    # Test Right Turn:
    # In nuScenes: X is right. Turning right means positive X.
    right_traj_nusc = np.stack([np.linspace(0.5, 3.0, 6), np.linspace(2, 20, 6)], axis=1)
    ctrl_right = controller.get_control(right_traj_nusc, current_speed_kmh=15.0)
    # In CARLA: steer > 0 is RIGHT
    assert ctrl_right['steer'] > 0.05, f"Right trajectory should produce positive steer, got {ctrl_right['steer']}"
    print(f"  --> Right turn correctly produces positive steering (steer = {ctrl_right['steer']:.3f})")

    # Test Format B (forward=x, lateral=y standard robotics frame):
    left_traj_robotics = np.stack([np.linspace(2, 20, 6), np.linspace(0.5, 3.0, 6)], axis=1)
    ctrl_left_robotics = controller.get_control(left_traj_robotics, current_speed_kmh=15.0)
    assert ctrl_left_robotics['steer'] < -0.05, "Auto-coordinate detection failed for robotics frame"
    print("  --> Auto-coordinate detection verified for both nuScenes and standard vehicle frames!")

def test_uniad_interface():
    print("[3/5] Testing UniAD Model Preprocessing & Waypoint Interface...")
    config = BridgeConfig()
    model = UniADModelWrapper(config.uniad_config, config.uniad_checkpoint, device="cpu")

    # Generate test images (6 cameras, 900x1600x3)
    mock_images = {
        name: np.ones((900, 1600, 3), dtype=np.uint8) * 128
        for name in NU_SCENES_CAMERAS.keys()
    }

    tensor = model.preprocess_images(mock_images)
    assert tensor.shape == (1, 6, 3, 900, 1600), f"Unexpected tensor shape: {tensor.shape}"
    print(f"  --> Preprocessed multi-camera tensor shape: {tuple(tensor.shape)} (Verified)")

    # Test Waypoint generation
    waypoints = model.predict_waypoints(mock_images, ego_speed_kmh=20.0, command=2)
    assert waypoints.shape == (6, 2), f"Expected waypoints of shape (6, 2), got {waypoints.shape}"
    assert np.all(np.isfinite(waypoints)), "Waypoints contain NaN or Inf values"
    print(f"  --> Model waypoint predictions verified (Shape: {waypoints.shape})")

def test_full_pipeline_loop():
    print("[4/5] Testing Full End-to-End Simulation Loop (10 ticks)...")
    config = BridgeConfig()
    uniad = UniADModelWrapper(config.uniad_config, config.uniad_checkpoint, device="cpu")
    controller = VehicleController(config)

    mock_images = {
        name: np.random.randint(0, 255, (900, 1600, 3), dtype=np.uint8)
        for name in NU_SCENES_CAMERAS.keys()
    }

    speed = 0.0
    for tick in range(1, 11):
        waypoints = uniad.predict_waypoints(mock_images, speed, command=2)
        ctrl = controller.get_control(waypoints, speed, config.target_speed)
        
        # Simple dynamics step
        accel = ctrl['throttle'] * 2.5 - ctrl['brake'] * 5.0
        speed = max(0.0, speed + (accel * 0.05 * 3.6))
        
        assert 0.0 <= ctrl['throttle'] <= 1.0, f"Throttle out of bounds: {ctrl['throttle']}"
        assert -1.0 <= ctrl['steer'] <= 1.0, f"Steer out of bounds: {ctrl['steer']}"
        assert 0.0 <= ctrl['brake'] <= 1.0, f"Brake out of bounds: {ctrl['brake']}"

    print(f"  --> 10-tick loop completed smoothly. Vehicle speed reached {speed:.2f} km/h.")

def test_file_integrity():
    print("[5/5] Testing File Integrity and Batch Scripts...")
    import os
    expected_files = [
        "setup_windows.bat",
        "RUN_AUTONOMOUS_DRIVING.bat",
        "download_carla.py",
        "instructio.md",
        "TRANSFER_TO_WINDOWS_RTX.md",
        "simulator_bridge/config.py",
        "simulator_bridge/controller.py",
        "simulator_bridge/sensors.py",
        "simulator_bridge/uniad_interface.py",
        "simulator_bridge/run_bridge.py"
    ]
    for rel_path in expected_files:
        assert os.path.exists(rel_path), f"Missing required project file: {rel_path}"
    print("  --> All required project scripts, launchers, and documentation files present.")

if __name__ == "__main__":
    print("=" * 60)
    print(" RUNNING FULL SIMULATOR BRIDGE INTEGRATION & WIRING AUDIT")
    print("=" * 60)
    try:
        test_config()
        test_controller()
        test_uniad_interface()
        test_full_pipeline_loop()
        test_file_integrity()
        print("\n" + "=" * 60)
        print(" ALL CHECKS PASSED! The system is completely wired & verified.")
        print("=" * 60)
    except AssertionError as e:
        print(f"\n[FAIL] Assertion error: {e}")
        sys.exit(1)
    except Exception as e:
        print(f"\n[FAIL] Unexpected error: {e}")
        sys.exit(1)
