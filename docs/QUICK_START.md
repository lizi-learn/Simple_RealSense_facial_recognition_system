# 快速开始指南

## 🚀 一键启动

```bash
cd /home/pc/realsense
./start.sh
```

## 📋 使用步骤

### 1. 添加人物到数据库

```bash
cd /home/pc/realsense
python3 src/realsense_camera/scripts/add_person.py \
    person001 张三 学生 \
    data/picture/face1.jpg data/picture/face2.jpg
```

### 2. 测试识别

```bash
# 测试单张图片识别
python3 tests/test_recognition.py data/picture/曾福明-2.jpg

# 测试语音播报
python3 tests/test_microsoft_tts.py

# 测试设备连接
./scripts/check_device.sh

# 诊断语音功能
./scripts/diagnose_voice.sh
```

### 3. 查看文档

- **完整教程**: `docs/learn.md` - 从零开始的技术教程
- **人脸识别**: `docs/FACE_RECOGNITION_README.md` - 识别功能说明
- **语音播报**: `docs/MICROSOFT_TTS.md` - TTS 使用说明
- **目录结构**: `docs/DIRECTORY_STRUCTURE.md` - 项目结构说明

## 📁 目录结构

```
realsense/
├── README.md              # 主文档
├── start.sh               # 启动脚本
├── scripts/               # 工具脚本
├── tests/                 # 测试脚本
├── docs/                  # 文档
├── data/                  # 数据（数据库、图片）
└── src/                   # ROS 源代码
```

详细说明请查看 `docs/DIRECTORY_STRUCTURE.md`

