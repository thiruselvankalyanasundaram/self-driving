# Complete Guide: Running UniAD + CARLA on your Windows RTX Laptop

This directory contains the complete, self-contained project package designed to be copied to your **Windows 10 / 11 RTX Laptop**.

---

## 📦 What's Inside This Directory

| Folder / File | Description |
| :--- | :--- |
| `carla_simulator/` | The CARLA 0.9.15 Unreal Engine Simulator (downloaded & unzipped). |
| `UniAD/` | Full UniAD repository (perception, prediction, mapping, planning). |
| `simulator_bridge/` | The bridge synchronizing the 6 nuScenes cameras & vehicle controller. |
| `setup_windows.bat` | One-click setup script to install CUDA PyTorch, CARLA client, and dependencies. |
| `RUN_AUTONOMOUS_DRIVING.bat` | One-click master launcher to start CARLA and the UniAD driving agent. |
| `download_carla.py` | Multi-connection downloader & extractor for the CARLA simulator. |

---

## 🚀 How to Transfer & Run on Your Windows RTX Laptop

### Step 1: Copy this entire folder to your RTX laptop
Copy the entire `yolo/` folder onto a USB drive, portable SSD, or transfer via local network to your Windows RTX laptop.

---

### Step 2: Run Setup (First time only)
On the Windows laptop, open this folder and **double-click**:
```cmd
setup_windows.bat
```
This will automatically:
1. Verify Python (3.8 or 3.9 recommended) and your NVIDIA GPU (`nvidia-smi`).
2. Install PyTorch with **CUDA 11.8** support.
3. Install the CARLA Python client API and required libraries (`pyquaternion`, `opencv-python`, etc.).

---

### Step 3: Run Autonomous Driving (One-Click)
Double-click:
```cmd
RUN_AUTONOMOUS_DRIVING.bat
```
What this does automatically:
1. **Launches CARLA** (`CarlaUE4.exe`) in low-VRAM mode (`-quality-level=Low -fps=20 -windowed`) so it leaves 5–6 GB of GPU VRAM for the AI model.
2. **Waits 8 seconds** for Unreal Engine to load the map and city.
3. **Spawns the Tesla / Audi**, mounts the **6 nuScenes surround cameras**, starts the **UniAD planner**, and begins autonomous driving!

---

### 🎮 Manual / Advanced Launching

If you prefer to run CARLA and the agent in separate command prompts:

**Terminal 1 (Start Simulator):**
```cmd
carla_simulator\CarlaUE4.exe -quality-level=Low -fps=20 -windowed -ResX=1280 -ResY=720
```

**Terminal 2 (Start AI Agent):**
```cmd
python -m simulator_bridge.run_bridge --host 127.0.0.1 --port 2000 --speed 25.0
```
