# 快速开始 - 语音播报功能

## 🚀 三步启动

### 1. 测试语音功能（可选）
```bash
cd /home/pc/realsense
python3 test_voice_announcement.py
```

### 2. 启动节点
```bash
source devel/setup.bash
roslaunch realsense_camera realsense_camera.launch
```

### 3. 等待播报
- 站在相机前
- 系统检测到人脸后，每5秒会播报一次距离
- 例如：`"Face detected, distance 1.5 meters"`

## 📢 播报示例（中文）

- **单人，距离80厘米**：`"检测到人脸，距离80厘米"`
- **单人，距离1.5米**：`"检测到人脸，距离1点5米"`
- **多人，最近80厘米**：`"检测到2个人脸，最近距离80厘米"`

**注意**：实际播报使用拼音发音，但表达的是中文内容。

## ⚙️ 调整参数

编辑 `src/realsense_camera/launch/realsense_camera.launch`：

```xml
<!-- 播报间隔（秒） -->
<param name="voice_announcement_interval" value="5.0" />

<!-- 音频设备 -->
<param name="audio_sink" value="alsa_output.pci-0000_00_1f.3.analog-stereo" />
```

## ❓ 问题？

- **没声音？** 运行 `pactl list short sinks` 检查音频设备
- **播报太快？** 增加 `voice_announcement_interval` 值
- **需要帮助？** 查看 `VOICE_ANNOUNCEMENT.md`

