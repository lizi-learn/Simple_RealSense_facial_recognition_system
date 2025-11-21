#!/bin/bash
# 语音播报诊断脚本

echo "==============================================================="
echo "  语音播报功能诊断"
echo "==============================================================="
echo ""

# 1. 检查 edge-tts
echo "[1/6] 检查 edge-tts..."
if command -v edge-tts &> /dev/null; then
    echo "  ✓ edge-tts 已安装: $(which edge-tts)"
else
    echo "  ✗ edge-tts 未安装"
    echo "    安装命令: pip3 install edge-tts --user"
    exit 1
fi

# 2. 检查 ffplay
echo "[2/6] 检查 ffplay..."
if command -v ffplay &> /dev/null; then
    echo "  ✓ ffplay 已安装: $(which ffplay)"
else
    echo "  ✗ ffplay 未安装"
    echo "    安装命令: sudo apt install ffmpeg"
    exit 1
fi

# 3. 检查音频设备
echo "[3/6] 检查音频设备..."
if command -v pactl &> /dev/null; then
    sinks=$(pactl list short sinks 2>/dev/null | wc -l)
    if [ "$sinks" -gt 0 ]; then
        echo "  ✓ 找到 $sinks 个音频输出设备:"
        pactl list short sinks 2>/dev/null | head -3 | while read line; do
            echo "    - $line"
        done
    else
        echo "  ✗ 未找到音频输出设备"
    fi
else
    echo "  ⚠ pactl 未安装，无法检查音频设备"
fi

# 4. 检查网络连接
echo "[4/6] 检查网络连接..."
if curl -s --max-time 5 https://api.msedgeservices.com > /dev/null 2>&1; then
    echo "  ✓ 可以访问微软TTS服务"
else
    echo "  ✗ 无法访问微软TTS服务"
    echo "    提示: 可能需要配置代理"
fi

# 5. 检查代理设置
echo "[5/6] 检查代理设置..."
if [ -n "$HTTP_PROXY" ] || [ -n "$HTTPS_PROXY" ]; then
    echo "  ✓ 检测到代理设置:"
    [ -n "$HTTP_PROXY" ] && echo "    HTTP_PROXY=$HTTP_PROXY"
    [ -n "$HTTPS_PROXY" ] && echo "    HTTPS_PROXY=$HTTPS_PROXY"
else
    echo "  ⚠ 未检测到代理设置"
    echo "    如果需要，可以设置:"
    echo "    export HTTP_PROXY=http://127.0.0.1:7890"
    echo "    export HTTPS_PROXY=http://127.0.0.1:7890"
fi

# 6. 测试语音生成
echo "[6/6] 测试语音生成..."
TMP_FILE=$(mktemp /tmp/test_tts_XXXXXX.mp3)
if edge-tts --voice zh-CN-XiaoxiaoNeural --text "测试" --write-media "$TMP_FILE" 2>/dev/null; then
    if [ -f "$TMP_FILE" ] && [ -s "$TMP_FILE" ]; then
        SIZE=$(stat -c%s "$TMP_FILE" 2>/dev/null || stat -f%z "$TMP_FILE" 2>/dev/null)
        echo "  ✓ 语音生成成功 (文件大小: $SIZE 字节)"
        
        # 测试播放
        echo "  正在测试播放..."
        if ffplay -nodisp -autoexit -loglevel quiet "$TMP_FILE" 2>/dev/null; then
            echo "  ✓ 播放测试成功"
        else
            echo "  ✗ 播放测试失败"
        fi
        rm -f "$TMP_FILE"
    else
        echo "  ✗ 语音文件生成失败或为空"
        rm -f "$TMP_FILE"
    fi
else
    echo "  ✗ 语音生成失败"
    echo "    可能原因:"
    echo "    1. 网络连接问题"
    echo "    2. 需要配置代理"
    echo "    3. 微软服务暂时不可用"
    rm -f "$TMP_FILE"
fi

echo ""
echo "==============================================================="
echo "  诊断完成"
echo "==============================================================="
echo ""
echo "如果所有检查都通过但仍无声音，请检查:"
echo "  1. ROS 节点日志: rosnode info /realsense_camera_node"
echo "  2. 是否识别成功（识别成功才会播报）"
echo "  3. 音频设备音量是否调高"
echo "  4. 系统音量是否静音"

