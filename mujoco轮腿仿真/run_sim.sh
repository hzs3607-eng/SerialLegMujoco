#!/bin/bash
# Launch script for MuJoCo wheel-leg robot simulation

# Change to script directory
cd "$(dirname "$0")"

# Compile first
echo "Compiling..."
g++ -fdiagnostics-color=always -g taskfinal.cpp SM_io.cpp chassiscontrolsoftware/Src/*.cpp -o taskfinal \
    -I./mujoco-3.4.0-linux-x86_64/mujoco-3.4.0/include \
    -I./chassiscontrolsoftware/Inc \
    -L./mujoco-3.4.0-linux-x86_64/mujoco-3.4.0/lib \
    -lmujoco -ldl -lglfw -lm \
    -Wl,-rpath,./mujoco-3.4.0-linux-x86_64/mujoco-3.4.0/lib

if [ $? -eq 0 ]; then
    echo "Compilation successful. Starting simulation..."
    # Set MuJoCo library path and run simulation
    LD_LIBRARY_PATH=./mujoco-3.4.0-linux-x86_64/mujoco-3.4.0/lib:$LD_LIBRARY_PATH ./taskfinal
else
    echo "Compilation failed!"
    exit 1
fi
