@echo off
title UniAD Autonomous Driving Launcher (CARLA)
echo ============================================================
echo      UniAD + CARLA Autonomous Driving Launcher
echo ============================================================
echo.

:: 1. Check if CARLA executable exists
if not exist "carla_simulator\CarlaUE4.exe" (
    echo [NOTICE] CarlaUE4.exe was not found in carla_simulator\
    echo If CARLA has not been downloaded/extracted yet, downloading now...
    python download_carla.py
    if not exist "carla_simulator\CarlaUE4.exe" (
        echo [ERROR] CarlaUE4.exe is still missing. Please extract CARLA into carla_simulator\
        pause
        exit /b 1
    )
)

:: 2. Launch CARLA Server in a separate window (Low quality to optimize VRAM on RTX laptop)
echo [1/2] Starting CARLA Simulator Server (Low VRAM mode, 20 FPS)...
start "CARLA Simulator Server" "carla_simulator\CarlaUE4.exe" -quality-level=Low -fps=20 -windowed -ResX=1280 -ResY=720

echo [Waiting] Giving Unreal Engine 8 seconds to initialize...
timeout /t 8 /nobreak >nul

:: 3. Launch the Simulator Bridge
echo.
echo [2/2] Starting UniAD Autonomous Driving Agent...
echo Spawning hero vehicle, 6 surround cameras, and connecting controller...
python -m simulator_bridge.run_bridge --host 127.0.0.1 --port 2000 --speed 25.0

pause
