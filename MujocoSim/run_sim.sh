#!/bin/bash
cd "$(dirname "$0")"

# 直接写死你的绝对路径，杜绝任何找不到文件的情况
MUJOCO_DIR="/home/hzs/mujoco-3.4.0"

echo "Compiling using Global MuJoCo 3.4.0 SDK..."
g++ -fdiagnostics-color=always -g taskfinal.cpp SM_io.cpp ./chassiscontrolsoftware/Src/*.cpp -o taskfinal \
    -I"${MUJOCO_DIR}/include" \
    -I./chassiscontrolsoftware/Inc \
    -L"${MUJOCO_DIR}/lib" \
    -lmujoco -ldl -lglfw -lm \
    -Wl,-rpath,"${MUJOCO_DIR}/lib"

if [ $? -eq 0 ]; then
    echo -e "\n✅ Compilation successful! Starting simulation..."
    export LD_LIBRARY_PATH="${MUJOCO_DIR}/lib":$LD_LIBRARY_PATH
    ./taskfinal
else
    echo -e "\n❌ Compilation failed!"
    exit 1
fi