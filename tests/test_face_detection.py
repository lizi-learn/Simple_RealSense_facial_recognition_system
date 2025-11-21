#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试人脸检测功能
"""

import sys
import os

# 添加ROS路径
sys.path.insert(0, '/opt/ros/noetic/lib/python3/dist-packages')
sys.path.insert(0, '/usr/lib/python3.8/site-packages')

try:
    print("=" * 60)
    print("RealSense人脸检测节点测试")
    print("=" * 60)
    
    print("\n1. 检查模型文件...")
    import rospkg
    rospack = rospkg.RosPack()
    package_path = rospack.get_path('realsense_camera')
    models_dir = os.path.join(package_path, 'models')
    
    prototxt = os.path.join(models_dir, 'deploy.prototxt')
    model = os.path.join(models_dir, 'res10_300x300_ssd_iter_140000.caffemodel')
    
    if os.path.exists(prototxt):
        print(f"   ✓ deploy.prototxt 存在: {prototxt}")
    else:
        print(f"   ✗ deploy.prototxt 不存在: {prototxt}")
        sys.exit(1)
    
    if os.path.exists(model):
        size = os.path.getsize(model) / (1024 * 1024)
        print(f"   ✓ 模型文件存在: {model} ({size:.1f} MB)")
    else:
        print(f"   ✗ 模型文件不存在: {model}")
        sys.exit(1)
    
    print("\n2. 测试OpenCV DNN模块...")
    import cv2
    print(f"   ✓ OpenCV 版本: {cv2.__version__}")
    
    print("\n3. 测试加载人脸检测模型...")
    try:
        net = cv2.dnn.readNetFromCaffe(prototxt, model)
        print("   ✓ 人脸检测模型加载成功")
    except Exception as e:
        print(f"   ✗ 模型加载失败: {e}")
        sys.exit(1)
    
    print("\n4. 测试导入ROS模块...")
    import rospy
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    print("   ✓ ROS模块导入成功")
    
    print("\n5. 测试导入pyrealsense2...")
    import pyrealsense2 as rs
    print("   ✓ pyrealsense2 导入成功")
    
    print("\n" + "=" * 60)
    print("✓ 所有测试通过！")
    print("=" * 60)
    print("\n启动节点:")
    print("  source /home/pc/realsense/devel/setup.bash")
    print("  roslaunch realsense_camera realsense_camera.launch")
    print("\n查看人脸检测结果:")
    print("  rosrun image_view image_view image:=/camera/face_detection/image_raw")
    print("=" * 60)
    
except ImportError as e:
    print(f"✗ 导入错误: {e}")
    sys.exit(1)
except Exception as e:
    print(f"✗ 错误: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)

