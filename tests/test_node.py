#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
简单的测试脚本，检查节点是否可以正常导入和初始化（不连接实际设备）
"""

import sys
import os

# 添加ROS路径
sys.path.insert(0, '/opt/ros/noetic/lib/python3/dist-packages')

# 添加pyrealsense2路径
sys.path.insert(0, '/usr/lib/python3.8/site-packages')

try:
    print("1. 测试导入ROS模块...")
    import rospy
    print("   ✓ rospy 导入成功")
    
    print("2. 测试导入pyrealsense2...")
    import pyrealsense2 as rs
    print("   ✓ pyrealsense2 导入成功")
    
    print("3. 测试导入其他依赖...")
    import numpy as np
    import cv2
    from sensor_msgs.msg import Image, CameraInfo
    from cv_bridge import CvBridge
    print("   ✓ 所有依赖导入成功")
    
    print("4. 测试RealSense上下文...")
    ctx = rs.context()
    devices = ctx.query_devices()
    print(f"   ✓ 检测到 {len(devices)} 个RealSense设备")
    
    if len(devices) > 0:
        print("   ✓ 设备信息:")
        for dev in devices:
            print(f"      - {dev.get_info(rs.camera_info.name)}")
    else:
        print("   ⚠ 未检测到RealSense设备（这是正常的，如果没有连接设备）")
    
    print("\n✓ 所有测试通过！节点应该可以正常运行。")
    print("\n要启动节点，请运行:")
    print("  source /home/pc/realsense/devel/setup.bash")
    print("  roslaunch realsense_camera realsense_camera.launch")
    
except ImportError as e:
    print(f"   ✗ 导入错误: {e}")
    sys.exit(1)
except Exception as e:
    print(f"   ✗ 错误: {e}")
    sys.exit(1)

