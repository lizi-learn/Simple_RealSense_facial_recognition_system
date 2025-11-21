# 微软TTS语音播报功能说明

## 📢 功能概述

使用微软TTS（edge-tts）进行中文语音播报，发音质量更好，支持自然的中文语音。

## ✨ 功能特点

- ✅ **高质量中文语音**：使用微软Edge的TTS引擎，发音自然流畅
- ✅ **多种语音选择**：支持多种中文语音（女声）
- ✅ **自动音频播放**：自动生成音频并播放
- ✅ **5秒循环播报**：每5秒播报一次，避免过于频繁

## 🎯 播报内容

### 单人脸场景
- **距离 < 1米**：`"检测到人脸，距离80厘米"`
- **距离 ≥ 1米**：`"检测到人脸，距离1点5米"`

### 多人脸场景
- **距离 < 1米**：`"检测到2个人脸，最近距离80厘米"`
- **距离 ≥ 1米**：`"检测到2个人脸，最近距离1点5米"`

## 🚀 使用方法

### 1. 安装依赖

```bash
# 安装edge-tts
pip3 install edge-tts --user

# 确保有音频播放器（ffplay通常已安装）
which ffplay
```

### 2. 测试语音播报

```bash
cd /home/pc/realsense
python3 test_microsoft_tts.py
```

### 3. 启动节点

```bash
source devel/setup.bash
roslaunch realsense_camera realsense_camera.launch
```

## 🎤 可用的中文语音

在 `realsense_camera.launch` 中可以配置以下语音：

- `zh-CN-XiaoxiaoNeural` - 晓晓（女声，通用，推荐）
- `zh-CN-XiaohanNeural` - 晓涵（女声，温柔）
- `zh-CN-XiaomengNeural` - 晓梦（女声，友好）
- `zh-CN-XiaomoNeural` - 晓墨（女声，深沉）
- `zh-CN-XiaochenNeural` - 晓辰（女声，友好）
- `zh-CN-XiaoruiNeural` - 晓睿（女声，自信）

查看所有可用语音：
```bash
edge-tts --list-voices | grep "zh-CN"
```

## ⚙️ 参数配置

在 `realsense_camera.launch` 文件中：

```xml
<!-- 语音播报参数 -->
<param name="enable_voice_announcement" value="true" />
<param name="voice_announcement_interval" value="5.0" />
<param name="audio_sink" value="alsa_output.pci-0000_00_1f.3.analog-stereo" />
<param name="tts_voice" value="zh-CN-XiaoxiaoNeural" />
```

### 参数说明

- `enable_voice_announcement`: 启用/禁用语音播报（true/false）
- `voice_announcement_interval`: 播报间隔（秒，默认5.0）
- `audio_sink`: 音频输出设备（使用 `pactl list short sinks` 查看）
- `tts_voice`: TTS语音选择（见上面的可用语音列表）

## 🔧 故障排除

### 问题1：没有声音

**检查步骤：**

1. **检查edge-tts是否安装**
   ```bash
   edge-tts --help
   ```

2. **检查ffplay是否可用**
   ```bash
   which ffplay
   ```
   如果没有，安装ffmpeg：
   ```bash
   sudo apt install ffmpeg
   ```

3. **检查音频设备**
   ```bash
   pactl list short sinks
   ```

4. **测试直接播放**
   ```bash
   edge-tts --voice zh-CN-XiaoxiaoNeural --text "测试" --write-media /tmp/test.mp3
   ffplay -nodisp -autoexit /tmp/test.mp3
   ```

### 问题2：播报延迟

**原因：** edge-tts需要从网络获取音频（首次使用）

**解决方案：**
- 首次使用会有延迟，后续会更快
- 确保网络连接正常

### 问题3：语音选择无效

**检查：**
```bash
edge-tts --list-voices | grep "zh-CN"
```
确保使用的语音名称正确。

## 📊 性能说明

- **首次播报**：可能需要2-3秒（从网络获取）
- **后续播报**：通常1-2秒
- **CPU使用**：较低，主要是网络和音频播放
- **网络要求**：需要互联网连接（edge-tts使用在线服务）

## 🆚 与espeak对比

| 特性 | espeak | 微软TTS (edge-tts) |
|------|--------|-------------------|
| 中文发音质量 | ⭐⭐ | ⭐⭐⭐⭐⭐ |
| 需要网络 | ❌ | ✅ |
| 安装难度 | 简单 | 简单 |
| 语音选择 | 有限 | 丰富 |
| 响应速度 | 快 | 中等 |

## 📚 相关文件

- **节点代码**：`src/realsense_camera/scripts/realsense_camera_node.py`
- **Launch文件**：`src/realsense_camera/launch/realsense_camera.launch`
- **测试脚本**：`test_microsoft_tts.py`

---

**享受高质量的中文语音播报！** 🎉

