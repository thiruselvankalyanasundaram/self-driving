@echo off
title Setup Environment - UniAD + CARLA (Windows)
echo ============================================================
echo   Setting up Python Environment for UniAD and CARLA
echo ============================================================

:: Check Python
python --version >nul 2>&1
if %errorlevel% neq 0 (
    echo [ERROR] Python is not installed or not in PATH!
    echo Please install Python 3.8 or 3.9 (64-bit) and check "Add to PATH".
    pause
    exit /b 1
)

:: Check NVIDIA GPU
nvidia-smi >nul 2>&1
if %errorlevel% neq 0 (
    echo [WARNING] 'nvidia-smi' not found! Make sure NVIDIA drivers are installed.
) else (
    echo [OK] NVIDIA GPU detected!
)

echo.
echo [1/3] Installing PyTorch with CUDA 11.8...
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu118

echo.
echo [2/3] Installing CARLA 0.9.15 Python client...
pip install carla==0.9.15
if %errorlevel% neq 0 (
    echo Attempting installation from local wheel if available...
    for %%f in (carla_simulator\PythonAPI\carla\dist\*.whl) do pip install "%%f"
)

echo.
echo [3/3] Installing UniAD dependencies...
pip install -r UniAD\requirements.txt
pip install pyquaternion opencv-python matplotlib

echo.
echo ============================================================
echo   Setup Complete! You can now run RUN_AUTONOMOUS_DRIVING.bat
echo ============================================================
pause
