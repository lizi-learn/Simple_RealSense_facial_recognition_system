#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试语音播报功能
"""

import subprocess
import sys

def test_voice():
    print("=" * 60)
    print("测试语音播报功能")
    print("=" * 60)
    
    # 测试音频设备
    print("\n1. 检查音频输出设备...")
    try:
        result = subprocess.run(['pactl', 'list', 'short', 'sinks'], 
                              capture_output=True, text=True, check=True)
        print("可用音频设备:")
        print(result.stdout)
    except Exception as e:
        print(f"   ✗ 无法列出音频设备: {e}")
        return False
    
    # 测试espeak
    print("\n2. 测试espeak...")
    try:
        result = subprocess.run(['espeak', '--version'], 
                              capture_output=True, text=True, check=True)
        print(f"   ✓ espeak已安装")
        print(f"   版本信息: {result.stdout.split()[0] if result.stdout else '未知'}")
    except Exception as e:
        print(f"   ✗ espeak未安装或不可用: {e}")
        print("   请运行: sudo apt install espeak espeak-data")
        return False
    
    # 测试语音播报
    print("\n3. 测试语音播报...")
    test_texts = [
        "Face detected, distance 1.5 meters",
        "2 faces detected, nearest distance 80 centimeters"
    ]
    
    for i, text in enumerate(test_texts, 1):
        print(f"\n   测试 {i}: {text}")
        try:
            # 设置默认音频输出
            sink = "alsa_output.pci-0000_00_1f.3.analog-stereo"
            subprocess.run(['pactl', 'set-default-sink', sink], 
                         capture_output=True, check=False)
            
            # 播报
            subprocess.run(['espeak', '-s', '150', '-a', '100', text], 
                         check=True)
            print(f"   ✓ 播报成功")
        except Exception as e:
            print(f"   ✗ 播报失败: {e}")
            return False
    
    print("\n" + "=" * 60)
    print("✓ 所有测试通过！")
    print("=" * 60)
    print("\n现在可以启动节点测试:")
    print("  source /home/pc/realsense/devel/setup.bash")
    print("  roslaunch realsense_camera realsense_camera.launch")
    print("=" * 60)
    
    return True

if __name__ == '__main__':
    success = test_voice()
    sys.exit(0 if success else 1)

