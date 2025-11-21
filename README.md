# RealSense 人脸识别系统

基于 Intel RealSense 深度相机的人脸识别系统，集成了 RetinaFace + ArcFace 工业级识别方案，支持实时人脸检测、识别和语音问候功能。

## ✨ 功能特性

- 🎯 **工业级人脸识别**: RetinaFace (检测) + ArcFace (识别)
- 🔊 **智能语音问候**: 根据时间自动播报（早上好/中午好/下午好/晚上好）
- 📷 **实时深度信息**: 获取人脸距离和深度数据
- 🖼️ **中文显示支持**: 完美显示中文姓名和身份
- 🚀 **一键启动**: 自动化启动脚本，支持进程清理
- 📦 **ROS 集成**: 完整的 ROS1 Noetic 节点和话题发布

## 📋 目录

- [功能特性](#功能特性)
- [系统要求](#系统要求)
- [快速开始](#快速开始)
- [使用说明](#使用说明)
- [人脸识别](#人脸识别)
- [语音播报](#语音播报)
- [故障排除](#故障排除)
- [常见问题](#常见问题)
- [参考文档](#参考文档)

## 🔧 系统要求

### 操作系统
- Ubuntu 20.04 (Focal Fossa)
- Ubuntu 22.04 (Jammy Jellyfish)
- Ubuntu 24.04 (Noble Numbat)

### 硬件要求
- 至少 4GB RAM（推荐 8GB 或更多）
- 至少 5GB 可用磁盘空间
- USB 3.0 端口（用于连接 RealSense 相机）

### 软件依赖
脚本会自动安装以下依赖：
- Git
- CMake
- Build tools (gcc, g++, make)
- USB 开发库
- OpenGL 和图形库
- Python 3 开发环境

## 📷 支持的设备

本脚本支持以下 Intel RealSense 相机：
- **D435i** - 深度相机 + IMU
- **D455** - 深度相机
- **L515** - LiDAR 深度相机
- 以及其他 RealSense D400 系列设备

## 🚀 快速开始

### 1. 克隆或下载项目

```bash
cd ~
git clone <your-repo-url> realsense
cd realsense
```

### 2. 安装 RealSense SDK

```bash
chmod +x scripts/build_realsense_in_ubuntu_2004.sh
./scripts/build_realsense_in_ubuntu_2004.sh
```

**注意：** 安装过程可能需要 10-30 分钟，具体取决于您的系统性能。

### 3. 安装系统依赖

```bash
# ROS 依赖
sudo apt install ros-noetic-cv-bridge ros-noetic-image-transport

# Python 依赖
pip3 install -i https://mirrors.tuna.tsinghua.edu.cn/pypi/web/simple \
    opencv-python numpy pyrealsense2 insightface edge-tts pillow

# 中文字体
sudo apt install fonts-wqy-microhei fonts-wqy-zenhei

# 音频工具
sudo apt install ffmpeg pulseaudio-utils
```

### 4. 编译工作空间

```bash
cd /home/pc/realsense
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

### 5. 添加人物到数据库

```bash
python3 src/realsense_camera/scripts/add_person.py \
    person001 张三 学生 \
    data/picture/face1.jpg data/picture/face2.jpg
```

### 6. 启动系统

```bash
./start.sh
```

详细使用说明请查看 `docs/` 目录下的文档。

## 💻 使用说明

### 一键启动

```bash
cd /home/pc/realsense
./start.sh
```

脚本会自动：
1. 清理之前的进程
2. 启动 roscore（如果未运行）
3. 启动 RealSense 相机节点
4. 打开人脸识别图像查看器

### 手动启动

```bash
# 终端1：启动 roscore
roscore

# 终端2：启动相机节点
cd /home/pc/realsense
source devel/setup.bash
roslaunch realsense_camera realsense_camera.launch

# 终端3：查看识别结果（可选）
rosrun image_view image_view image:=/camera/face_detection/image_raw
```

### ROS 话题

系统会发布以下话题：

- `/camera/color/image_raw` - 原始彩色图像
- `/camera/depth/image_raw` - 深度图像
- `/camera/face_detection/image_raw` - 带识别结果图像
- `/camera/color/camera_info` - 彩色相机内参
- `/camera/depth/camera_info` - 深度相机内参

### 查看识别结果

```bash
# 使用 image_view 查看
rosrun image_view image_view image:=/camera/face_detection/image_raw

# 使用 rviz 可视化
rosrun rviz rviz
# 添加 Image 显示，话题选择 /camera/face_detection/image_raw
```

### 测试工具

```bash
# 测试识别功能
python3 tests/test_recognition.py data/picture/曾福明-2.jpg

# 测试语音播报
python3 tests/test_microsoft_tts.py

# 检查设备连接
./scripts/check_device.sh

# 诊断语音功能
./scripts/diagnose_voice.sh
```

## 👤 人脸识别

### 添加人物

```bash
python3 src/realsense_camera/scripts/add_person.py \
    <person_id> <姓名> <身份> <图片1> [图片2] [图片3] ...

# 示例
python3 src/realsense_camera/scripts/add_person.py \
    person001 张三 学生 \
    data/picture/face1.jpg data/picture/face2.jpg
```

### 识别功能

- **检测**: 使用 RetinaFace 进行高精度人脸检测
- **识别**: 使用 ArcFace 提取 512 维特征向量进行比对
- **阈值**: 默认 0.5（可在 launch 文件中调整）
- **防重复**: 10 秒内不重复识别同一人

### 配置参数

在 `src/realsense_camera/launch/realsense_camera.launch` 中：

```xml
<param name="enable_face_recognition" value="true" />
<param name="recognition_threshold" value="0.5" />  <!-- 0.3-0.7，越低越容易识别 -->
<param name="recognition_cooldown" value="10.0" />  <!-- 防重复播报时间（秒） -->
```

## 🔊 语音播报

### 功能说明

- **自动问候**: 识别成功后根据时间自动播报
  - 5:00-10:59: 早上好
  - 11:00-12:59: 中午好
  - 13:00-17:59: 下午好
  - 18:00-4:59: 晚上好
- **语音引擎**: Microsoft Edge TTS（中文自然语音）
- **防重复**: 防止重复播报，支持多人在线

### 诊断工具

```bash
./scripts/diagnose_voice.sh
```

### 配置

在 `src/realsense_camera/launch/realsense_camera.launch` 中：

```xml
<param name="enable_voice_announcement" value="true" />
<param name="tts_voice" value="zh-CN-XiaoxiaoNeural" />  <!-- 中文语音 -->
<param name="audio_sink" value="alsa_output.pci-0000_00_1f.3.analog-stereo" />
```

## 🔧 故障排除

### 问题 1: 设备未检测到

**症状：** 无法检测到 RealSense 相机

**解决方案：**

1. 运行设备检查脚本：
   ```bash
   ./scripts/check_device.sh
   ```

2. 检查 USB 连接（确保使用 USB 3.0 端口）

3. 检查 udev 规则：
   ```bash
   ls -l /etc/udev/rules.d/99-realsense-libusb.rules
   sudo udevadm control --reload-rules
   sudo udevadm trigger
   ```

### 问题 2: 识别失败

**症状：** 一直显示"未知"

**解决方案：**

1. 检查数据库：
   ```bash
   cat data/face_database/database.json
   ```

2. 降低识别阈值（在 launch 文件中）：
   ```xml
   <param name="recognition_threshold" value="0.4" />
   ```

3. 添加更多样本图片（不同角度、光照）

### 问题 3: 语音播报失败

**症状：** 识别成功但没有声音

**解决方案：**

1. 运行诊断脚本：
   ```bash
   ./scripts/diagnose_voice.sh
   ```

2. 检查网络连接（edge-tts 需要访问微软服务）

3. 配置代理（如果需要）：
   ```bash
   export HTTP_PROXY=http://127.0.0.1:7890
   export HTTPS_PROXY=http://127.0.0.1:7890
   ```

4. 检查音频设备：
   ```bash
   pactl list short sinks
   ```

### 问题 4: 模块导入错误

**症状：** `ModuleNotFoundError: No module named 'face_recognition_manager'`

**解决方案：**

代码已自动处理路径问题。如果仍有问题：
```bash
# 重新编译工作空间
cd /home/pc/realsense
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

### 问题 5: 中文显示为问号

**症状：** 图像上中文显示为 `???`

**解决方案：**

安装中文字体：
```bash
sudo apt install fonts-wqy-microhei fonts-wqy-zenhei
```

### 问题 6: 重复播报

**症状：** 同一人重复播报

**解决方案：**

已自动处理，系统会：
- 10 秒内不重复识别同一人
- 使用锁机制防止并发播报
- 2 秒内相同文本不重复

### 问题 7: 启动脚本清理失败

**症状：** 再次运行 `./start.sh` 时进程未清理

**解决方案：**

手动清理：
```bash
# 查找并终止进程
pkill -f "realsense_camera_node"
pkill -f "roslaunch.*realsense_camera"
```

## ❓ 常见问题

### Q: 如何添加新人物？

A: 使用 `add_person.py` 脚本：
```bash
python3 src/realsense_camera/scripts/add_person.py \
    person001 姓名 身份 \
    data/picture/face1.jpg data/picture/face2.jpg
```

### Q: 识别准确度如何提高？

A: 
1. 添加多张不同角度的照片（3-5 张）
2. 确保照片清晰，光照良好
3. 适当降低识别阈值（0.4-0.5）
4. 定期更新数据库

### Q: 支持多少人同时识别？

A: 理论上支持多人，但建议同时识别不超过 5 人以保证性能。

### Q: 可以离线使用吗？

A: 
- 人脸识别：可以（模型已下载）
- 语音播报：需要网络（edge-tts 需要连接微软服务）

### Q: 如何修改问候语时间？

A: 编辑 `src/realsense_camera/scripts/face_recognition_manager.py` 中的 `get_greeting()` 方法。

### Q: 如何查看识别日志？

A: 
```bash
# ROS 日志
rosnode info /realsense_camera_node

# 后台运行日志
tail -f /tmp/realsense_camera.log
```

## 📚 参考文档

项目文档位于 `docs/` 目录：

- **完整教程**: `docs/learn.md` - 从零开始的技术教程
- **快速开始**: `docs/QUICK_START.md` - 快速上手指南
- **人脸识别**: `docs/FACE_RECOGNITION_README.md` - 识别功能说明
- **语音播报**: `docs/MICROSOFT_TTS.md` - TTS 使用说明
- **目录结构**: `docs/DIRECTORY_STRUCTURE.md` - 项目结构说明

### 外部链接

- [Intel RealSense SDK 官方文档](https://dev.intelrealsense.com/)
- [InsightFace 项目](https://github.com/deepinsight/insightface)
- [ROS1 Noetic 文档](http://wiki.ros.org/noetic)
- [Microsoft Edge TTS](https://github.com/rany2/edge-tts)

## 📄 许可证

本脚本遵循 Intel RealSense SDK 的许可证。Intel RealSense SDK 使用 Apache 2.0 许可证。

## 🤝 贡献

欢迎提交 Issue 和 Pull Request！

## 📝 更新日志

### v1.0.0 (当前版本)
- ✅ 完整的人脸识别系统
- ✅ RetinaFace + ArcFace 工业级识别
- ✅ 智能语音问候（时间自适应）
- ✅ 中文显示支持
- ✅ 一键启动脚本（自动清理进程）
- ✅ 完整的 ROS 节点集成
- ✅ 深度信息获取
- ✅ 防重复播报机制

### v0.1.0
- RealSense SDK 2.57.4 安装脚本
- 支持 Ubuntu 20.04/22.04/24.04
- 自动安装 Python 绑定

## ⚠️ 注意事项

1. **权限要求：** SDK 安装脚本需要 sudo 权限
2. **网络连接：** 
   - SDK 安装需要下载源码
   - 语音播报需要访问微软 TTS 服务
   - InsightFace 模型首次使用会自动下载
3. **磁盘空间：** 确保有足够的磁盘空间（至少 10GB）
4. **USB 端口：** 建议使用 USB 3.0 端口以获得最佳性能
5. **系统要求：** Ubuntu 20.04 + ROS1 Noetic
6. **数据备份：** 定期备份 `data/face_database/` 目录

## 📧 支持

如果遇到问题，请：
1. 查看 [故障排除](#故障排除) 部分
2. 检查 [常见问题](#常见问题)
3. 访问 [Intel RealSense 社区论坛](https://support.intelrealsense.com/)

---

**祝您使用愉快！** 🎉

