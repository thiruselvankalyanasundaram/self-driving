# CARLA to UniAD Simulator Bridge

This bridge links the **CARLA Autonomous Driving Simulator** (Unreal Engine) to the **UniAD Planning-Oriented End-to-End Model**.

---

## 🏗 Architecture

```
┌────────────────────────────────────────────────────────┐
│                   CARLA SIMULATOR                      │
│                                                        │
│  [Hero Vehicle] ──► 6 Calibrated Surround Cameras      │
│                     (nuScenes Rig: Front, Sides, Back) │
└───────────────────────────┬────────────────────────────┘
                            │  Synchronized RGB Frames (20 FPS)
                            ▼
┌────────────────────────────────────────────────────────┐
│                   SIMULATOR BRIDGE                     │
│                                                        │
│  1. SensorManager      : Synchronizes multi-camera data│
│  2. UniADModelWrapper  : Batches images & predicts (x,y)│
│  3. VehicleController  : Pure Pursuit + PID Controller  │
└───────────────────────────┬────────────────────────────┘
                            │
                            │  VehicleControl(throttle, steer, brake)
                            ▼
┌────────────────────────────────────────────────────────┐
│                   CARLA ACTUATION                      │
│  Ego car steers, accelerates, and navigates the city.   │
└────────────────────────────────────────────────────────┘
```

---

## 📁 File Structure

* **`config.py`**: Defines the 6 camera transforms (positions, orientations, FOVs) matching nuScenes, plus PID controller gains and CARLA settings.
* **`sensors.py`**: Manages CARLA camera actors and synchronized image queues so frames across all 6 angles arrive in lockstep.
* **`controller.py`**:
  * **Lateral (Steering)**: Pure Pursuit + Stanley controller to track predicted waypoints.
  * **Longitudinal (Speed)**: Anti-windup PID controller computing throttle and brake.
* **`uniad_interface.py`**: Preprocesses 6 camera frames, runs UniAD inference, and extracts future $(x, y)$ waypoints.
* **`run_bridge.py`**: Main entrypoint supporting both live CARLA connection and standalone mock testing.

---

## 🚀 Running on Your RTX Laptop with CARLA

### 1. Launch CARLA Server
In a terminal on the RTX machine, launch CARLA in low/medium quality to preserve VRAM:

```bash
cd /path/to/CARLA_0.9.15
# Linux:
./CarlaUE4.sh -quality-level=Low
# Windows:
CarlaUE4.exe -quality-level=Low
```

> **Tip**: If you have 8GB VRAM (e.g. RTX 3070/4060), running `-quality-level=Low` limits CARLA to ~1.5GB VRAM, leaving ample room for UniAD inference.

### 2. Run the Simulator Bridge
In a second terminal (in your python environment with CARLA and UniAD dependencies):

```bash
python3 -m simulator_bridge.run_bridge --host 127.0.0.1 --port 2000 --speed 25.0
```

### 3. Verification & Mock Testing (No CARLA required)
You can test the entire control and inference pipeline anytime without a running CARLA server:

```bash
python3 -m simulator_bridge.run_bridge --mock-carla --steps 50
```
