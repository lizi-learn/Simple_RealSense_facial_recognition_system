#!/bin/bash

set -e

echo "==============================================================="
echo "   Intel RealSense SDK 2.57.4 一键自动安装脚本（完整版）"
echo "   支持: Ubuntu 20.04 / 22.04 / 24.04, D435i / D455 / L515"
echo "==============================================================="

echo "[1/7] 更新系统依赖..."
sudo apt update || true
sudo apt install -y \
    git cmake build-essential pkg-config \
    libusb-1.0-0-dev libudev-dev \
    libgtk-3-dev libglfw3-dev libgl1-mesa-dev \
    libssl-dev libcurl4-openssl-dev \
    python3 python3-dev python3-pip

echo
echo "[2/7] 下载 librealsense 源码 2.57.4..."
cd ~
if [ -d "librealsense" ]; then
    echo "librealsense 已存在，跳过克隆。"
else
    git clone https://github.com/IntelRealSense/librealsense.git
fi

cd librealsense
git fetch --all
git checkout v2.57.4

echo
echo "[3/7] 创建构建目录..."
rm -rf build
mkdir build && cd build

echo
echo "[4/7] CMake 配置..."
PYTHON_EXEC=$(which python3)
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_EXAMPLES=true \
    -DBUILD_GRAPHICAL_EXAMPLES=true \
    -DFORCE_RSUSB_BACKEND=true \
    -DBUILD_PYTHON_BINDINGS=true \
    -DPYTHON_EXECUTABLE="$PYTHON_EXEC"

echo
echo "[5/7] 开始编译（可能需要 10~30 分钟）..."
make -j$(nproc)

echo
echo "[6/7] 安装 SDK 到系统..."
sudo make install

echo
echo "[7/7] 安装 udev 规则..."
sudo cp ../config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger

echo
echo "==============================================================="
echo "   ✔ 安装完成！"
echo "   你现在可以运行：  realsense-viewer"
echo "   来验证 D435i 深度 和 IMU 是否正常工作"
echo "==============================================================="

echo
echo "可选: 安装 Python 模块..."
cd ~/librealsense/build
sudo make install-py || true

echo
echo "==============================================================="
echo "   RealSense SDK 2.57.4 已完全安装。"
echo "   如需 ROS，请安装："
echo "   sudo apt install ros-noetic-realsense2-camera"
echo "==============================================================="
