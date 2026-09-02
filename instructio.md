# Instructions: Running UniAD with CARLA on your Windows RTX Laptop

This guide explains how to transfer this project folder to your Windows RTX laptop and launch the autonomous driving simulation.

---

## 1. Overview of the Folder Contents

* **`carla_simulator/`**: Contains the CARLA 0.9.15 Unreal Engine Simulator (downloaded and extracted).
* **`UniAD/`**: The complete UniAD framework repository.
* **`simulator_bridge/`**: The Python bridge that connects CARLA to UniAD:
  * Sets up the 6 surround cameras (`CAM_FRONT`, `CAM_FRONT_LEFT`, `CAM_FRONT_RIGHT`, `CAM_BACK`, `CAM_BACK_LEFT`, `CAM_BACK_RIGHT`).
  * Feeds synchronized camera frames to UniAD.
  * Uses Pure Pursuit (steering) and PID (throttle/brake) to drive the car along predicted waypoints.
* **`setup_windows.bat`**: One-click script to install CUDA PyTorch, CARLA client, and dependencies.
* **`RUN_AUTONOMOUS_DRIVING.bat`**: One-click launcher that starts CARLA and begins autonomous driving.
* **`download_carla.py`**: Multi-threaded downloader used to fetch the simulator archive.

---

## 2. Transferring to Your RTX Laptop

1. Once the CARLA download finishes on this computer, copy this entire directory (`yolo/`) onto a USB drive, portable SSD, or transfer it over your local network.
2. Paste the folder anywhere on your Windows RTX laptop (e.g., `C:\UniAD_CARLA\` or on your Desktop).

---

## 3. First-Time Setup (On Windows RTX Laptop)

Before running the simulation for the first time:

1. Make sure **Python 3.8 or 3.9 (64-bit)** is installed and added to your system PATH.
2. Open the project folder on your Windows laptop.
3. **Double-click `setup_windows.bat`**.

This script will automatically:
- Detect your NVIDIA RTX GPU.
- Install PyTorch with **CUDA 11.8** support.
- Install the CARLA 0.9.15 Python client API.
- Install all required helper libraries (`pyquaternion`, `opencv-python`, `matplotlib`).

---

## 4. Launching the Simulation (One-Click)

To run everything automatically:

* **Double-click `RUN_AUTONOMOUS_DRIVING.bat`**.

What happens automatically:
1. It launches CARLA (`CarlaUE4.exe`) in low-VRAM mode (`-quality-level=Low -fps=20 -windowed`) so it leaves 5–6 GB of VRAM free for the AI model.
2. It waits 8 seconds for the Unreal Engine city map to load.
3. It spawns the car, attaches all 6 surround cameras, connects to UniAD, and the AI starts driving autonomously.

---

## 5. Manual / Advanced Launch (Two Terminals)

If you prefer to run CARLA and the AI agent in separate terminal windows:

### Terminal 1: Start the CARLA Simulator
```cmd
carla_simulator\CarlaUE4.exe -quality-level=Low -fps=20 -windowed -ResX=1280 -ResY=720
```

### Terminal 2: Start the Autonomous Driving Agent
```cmd
python -m simulator_bridge.run_bridge --host 127.0.0.1 --port 2000 --speed 25.0
```

---

## 6. VRAM & Performance Tips for RTX Laptops

* **If your laptop has 6 GB or 8 GB VRAM (RTX 3060 / 3070 / 4060)**:
  * Keep the `-quality-level=Low` flag enabled. This restricts CARLA to ~1.5 GB VRAM, leaving plenty of memory for UniAD inference.
* **If your laptop has 12 GB or 16 GB VRAM (RTX 3080 / 4080 / 4090)**:
  * You can remove `-quality-level=Low` in `RUN_AUTONOMOUS_DRIVING.bat` to run CARLA on Epic graphics settings.
* **Stopping the Simulation**:
  * Press `Ctrl + C` in the agent terminal window. The script will automatically clean up the cameras and vehicle from CARLA.
