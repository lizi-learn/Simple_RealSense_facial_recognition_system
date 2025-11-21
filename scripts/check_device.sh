#!/bin/bash

# RealSense设备检测脚本

echo "==============================================================="
echo "   RealSense 设备检测"
echo "==============================================================="

# 加载环境
if [ -z "$ROS_DISTRO" ]; then
    source /opt/ros/noetic/setup.bash
fi

# 添加pyrealsense2路径
export PYTHONPATH="/usr/lib/python3.8/site-packages:$PYTHONPATH"

echo ""
echo "1. 检查USB设备..."
echo "----------------------------------------"
if lsusb | grep -i intel > /dev/null; then
    echo "✓ 检测到Intel USB设备:"
    lsusb | grep -i intel
else
    echo "✗ 未检测到Intel USB设备"
    echo "  请检查RealSense相机是否已连接"
fi

echo ""
echo "2. 检查RealSense设备（使用pyrealsense2）..."
echo "----------------------------------------"
python3 << EOF
import sys
sys.path.insert(0, '/usr/lib/python3.8/site-packages')

try:
    import pyrealsense2 as rs
    ctx = rs.context()
    devices = ctx.query_devices()
    
    if len(devices) == 0:
        print("✗ 未检测到RealSense设备")
        print("")
        print("可能的原因:")
        print("  1. 相机未连接或连接不稳定")
        print("  2. USB端口不是USB 3.0")
        print("  3. 需要重新加载udev规则:")
        print("     sudo udevadm control --reload-rules")
        print("     sudo udevadm trigger")
        print("  4. 需要将用户添加到plugdev组:")
        print("     sudo usermod -a -G plugdev \$USER")
        print("     (然后重新登录)")
    else:
        print(f"✓ 检测到 {len(devices)} 个RealSense设备:")
        for i, dev in enumerate(devices):
            name = dev.get_info(rs.camera_info.name)
            serial = dev.get_info(rs.camera_info.serial_number)
            firmware = dev.get_info(rs.camera_info.firmware_version) if dev.supports(rs.camera_info.firmware_version) else "未知"
            print(f"  设备 {i+1}:")
            print(f"    名称: {name}")
            print(f"    序列号: {serial}")
            print(f"    固件版本: {firmware}")
            
except ImportError:
    print("✗ 无法导入pyrealsense2")
    print("  请确保RealSense SDK已正确安装")
except Exception as e:
    print(f"✗ 错误: {e}")
EOF

echo ""
echo "3. 检查RealSense命令行工具..."
echo "----------------------------------------"
if command -v rs-enumerate-devices &> /dev/null; then
    echo "✓ rs-enumerate-devices 可用"
    echo "运行设备枚举..."
    rs-enumerate-devices 2>&1 | head -20
else
    echo "✗ rs-enumerate-devices 不可用"
    echo "  请确保RealSense SDK已正确安装"
fi

echo ""
echo "==============================================================="
echo "   检测完成"
echo "==============================================================="

