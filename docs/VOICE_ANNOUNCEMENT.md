# 人脸距离语音播报功能说明

## 📢 功能概述

当检测到人脸时，系统会通过语音播报人脸的距离信息，每5秒循环播报一次。

## ✨ 功能特点

- ✅ **自动语音播报**：检测到人脸时自动播报距离
- ✅ **5秒循环播报**：每5秒播报一次，避免过于频繁
- ✅ **多语言支持**：使用英文播报（更可靠）
- ✅ **多设备支持**：可配置音频输出设备
- ✅ **距离单位智能转换**：小于1米显示厘米，大于1米显示米

## 🎯 播报内容

### 单人脸场景
- **距离 < 1米**：`"Face detected, distance 80 centimeters"`
- **距离 ≥ 1米**：`"Face detected, distance 1.5 meters"`

### 多人脸场景
- **距离 < 1米**：`"2 faces detected, nearest distance 80 centimeters"`
- **距离 ≥ 1米**：`"2 faces detected, nearest distance 1.5 meters"`

## 🚀 使用方法

### 1. 测试语音播报功能

```bash
cd /home/pc/realsense
python3 test_voice_announcement.py
```

这会测试：
- 音频设备是否可用
- espeak是否正常工作
- 语音播报是否正常

### 2. 启动节点（带语音播报）

```bash
cd /home/pc/realsense
source devel/setup.bash
roslaunch realsense_camera realsense_camera.launch
```

节点启动后，当检测到人脸时，每5秒会自动播报一次距离。

### 3. 查看音频设备

如果需要更改音频输出设备，先查看可用设备：

```bash
pactl list short sinks
```

输出示例：
```
5657	alsa_output.pci-0000_00_1f.3.analog-stereo	module-alsa-card.c	s16le 2ch 48000Hz	SUSPENDED
```

## ⚙️ 参数配置

在 `realsense_camera.launch` 文件中可以配置以下参数：

### 启用/禁用语音播报

```xml
<param name="enable_voice_announcement" value="true" />
```

- `true`：启用语音播报（默认）
- `false`：禁用语音播报

### 播报间隔

```xml
<param name="voice_announcement_interval" value="5.0" />
```

- 单位：秒
- 默认值：5.0秒
- 建议范围：3.0 - 10.0秒

### 音频输出设备

```xml
<param name="audio_sink" value="alsa_output.pci-0000_00_1f.3.analog-stereo" />
```

- 使用 `pactl list short sinks` 查看可用设备
- 将设备名称填入 `value` 中

## 📝 代码说明

### 语音播报函数

```python
def speak_distance(self, distance_meters, face_count):
    """使用espeak播报人脸距离"""
    # 格式化距离文本
    # 设置音频输出设备
    # 使用espeak播报
```

### 播报触发逻辑

```python
# 每5秒检查一次是否需要播报
current_time = time.time()
if len(faces) > 0 and (current_time - self.last_announcement_time) >= 5.0:
    # 计算最近的人脸距离
    # 调用speak_distance播报
    self.last_announcement_time = current_time
```

## 🔧 故障排除

### 问题1：没有声音

**检查步骤：**

1. **检查音频设备是否激活**
   ```bash
   pactl list short sinks
   ```
   确保设备状态不是 `SUSPENDED`

2. **激活音频设备**
   ```bash
   pactl set-default-sink alsa_output.pci-0000_00_1f.3.analog-stereo
   ```

3. **测试espeak**
   ```bash
   espeak "Test voice announcement"
   ```

4. **检查系统音量**
   ```bash
   alsamixer
   ```
   或使用系统音量控制

### 问题2：播报太频繁或太慢

**解决方案：**

修改 `voice_announcement_interval` 参数：
- 更频繁：设置为 `3.0`
- 更慢：设置为 `10.0`

### 问题3：espeak未安装

**安装命令：**

```bash
sudo apt install espeak espeak-data
```

### 问题4：音频设备名称错误

**解决方法：**

1. 查看可用设备：
   ```bash
   pactl list short sinks
   ```

2. 更新launch文件中的 `audio_sink` 参数

## 🎛️ 高级配置

### 调整语音速度

在代码中修改 `-s` 参数：
- 默认：150（适中）
- 更快：200
- 更慢：100

### 调整音量

在代码中修改 `-a` 参数：
- 默认：100（正常音量）
- 更大：150
- 更小：50

### 使用其他TTS引擎

可以替换espeak为其他TTS引擎：
- **festival**：`sudo apt install festival`
- **pico2wave**：`sudo apt install libttspico-utils`
- **gTTS**：Python库，需要网络连接

## 📊 性能影响

- **CPU使用率**：语音播报会增加约1-2%的CPU使用率
- **延迟**：播报过程是异步的，不会阻塞主循环
- **资源占用**：espeak进程会在播报时短暂运行

## 🎯 使用场景

1. **无障碍应用**：帮助视障人士了解人脸距离
2. **安全监控**：距离提醒
3. **社交距离**：提醒保持安全距离
4. **演示展示**：实时展示深度相机功能

## 📚 相关文件

- **节点代码**：`src/realsense_camera/scripts/realsense_camera_node.py`
- **Launch文件**：`src/realsense_camera/launch/realsense_camera.launch`
- **测试脚本**：`test_voice_announcement.py`

## 🔄 更新日志

### v1.0 (当前版本)
- ✅ 基础语音播报功能
- ✅ 5秒循环播报
- ✅ 多设备音频支持
- ✅ 距离单位智能转换

---

**享受语音播报功能！** 🎉

