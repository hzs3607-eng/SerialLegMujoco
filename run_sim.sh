#!/bin/bash
# 确保在脚本所在目录执行
cd "$(dirname "$0")"

# 依然指向你家目录下的全局 SDK
MUJOCO_DIR="${HOME}/mujoco-3.4.0"

echo "Compiling Hero Robot project..."
# 注意：这里移除了 MujocoSim/ 前缀，直接编译当前目录下的文件
g++ -fdiagnostics-color=always -g taskfinal.cpp SM_io.cpp ./chassiscontrolsoftware/Src/*.cpp -o taskfinal \
    -I"${MUJOCO_DIR}/include" \
    -I./chassiscontrolsoftware/Inc \
    -L"${MUJOCO_DIR}/lib" \
    -lmujoco -ldl -lglfw -lm \
    -Wl,-rpath,"${MUJOCO_DIR}/lib"

if [ $? -eq 0 ]; then
    echo -e "\n✅ Compilation successful! Running hero.xml simulation..."
    export LD_LIBRARY_PATH="${MUJOCO_DIR}/lib":$LD_LIBRARY_PATH
    # 记得在你的 C++ 代码里把 mj_loadXML 的文件名改为 "hero.xml"
    ./taskfinal
else
    echo -e "\n❌ Compilation failed!"
    exit 1
fi