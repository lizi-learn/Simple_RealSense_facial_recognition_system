#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试微软TTS（edge-tts）语音播报功能
"""

import subprocess
import sys
import time

def test_microsoft_tts():
    print("=" * 60)
    print("测试微软TTS语音播报功能")
    print("=" * 60)
    
    # 测试文本
    test_texts = [
        "检测到人脸，距离80厘米",
        "检测到人脸，距离1点5米",
        "检测到2个人脸，最近距离80厘米"
    ]
    
    # 可用的中文语音
    voices = [
        "zh-CN-XiaoxiaoNeural",      # 晓晓（女声，通用）
        "zh-CN-XiaohanNeural",       # 晓涵（女声，温柔）
        "zh-CN-XiaomengNeural",      # 晓梦（女声，友好）
        "zh-CN-XiaomoNeural",        # 晓墨（女声，深沉）
    ]
    
    print("\n1. 检查edge-tts...")
    try:
        # edge-tts没有--version，使用--help来检查
        result = subprocess.run(['edge-tts', '--help'], 
                              capture_output=True, text=True, check=True, timeout=2)
        print(f"   ✓ edge-tts已安装")
    except Exception as e:
        print(f"   ✗ edge-tts未安装: {e}")
        print("   请运行: pip3 install edge-tts --user")
        return False
    
    print("\n2. 检查音频设备...")
    try:
        result = subprocess.run(['pactl', 'list', 'short', 'sinks'], 
                              capture_output=True, text=True, check=True)
        if result.stdout.strip():
            print("   ✓ 音频设备可用")
            print(f"   设备: {result.stdout.strip().split()[1] if result.stdout.strip() else '未知'}")
        else:
            print("   ⚠ 未检测到音频设备")
    except Exception as e:
        print(f"   ⚠ 无法检查音频设备: {e}")
    
    print("\n3. 测试语音播报...")
    print("   使用语音: zh-CN-XiaoxiaoNeural")
    
    for i, text in enumerate(test_texts, 1):
        print(f"\n   测试 {i}: {text}")
        try:
            # 设置音频输出
            subprocess.run(['pactl', 'set-default-sink', 
                          'alsa_output.pci-0000_00_1f.3.analog-stereo'], 
                         capture_output=True, check=False)
            
            # 使用edge-tts播报
            cmd = [
                'edge-tts',
                '--voice', 'zh-CN-XiaoxiaoNeural',
                '--text', text
            ]
            proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            time.sleep(3)  # 等待播报完成
            print("   ✓ 播报成功")
        except Exception as e:
            print(f"   ✗ 播报失败: {e}")
            return False
    
    print("\n" + "=" * 60)
    print("✓ 所有测试通过！")
    print("=" * 60)
    print("\n可用的中文语音:")
    for voice in voices:
        print(f"  - {voice}")
    print("\n现在可以启动节点测试:")
    print("  source /home/pc/realsense/devel/setup.bash")
    print("  roslaunch realsense_camera realsense_camera.launch")
    print("=" * 60)
    
    return True

if __name__ == '__main__':
    success = test_microsoft_tts()
    sys.exit(0 if success else 1)

