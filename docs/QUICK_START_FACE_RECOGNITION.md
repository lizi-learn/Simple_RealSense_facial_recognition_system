# 人脸识别快速开始指南

## 🚀 三步使用

### 1. 添加人物到数据库

```bash
cd /home/pc/realsense
python3 src/realsense_camera/scripts/add_person.py <person_id> <name> <role> <image1> [image2] [image3] ...
```

**示例：**
```bash
# 添加一个学生
python3 src/realsense_camera/scripts/add_person.py student001 张三 学生 \
    ~/photos/face1.jpg ~/photos/face2.jpg ~/photos/face3.jpg

# 添加一个老师
python3 src/realsense_camera/scripts/add_person.py teacher001 李老师 老师 \
    ~/photos/teacher1.jpg ~/photos/teacher2.jpg
```

### 2. 启动节点

```bash
source devel/setup.bash
roslaunch realsense_camera realsense_camera.launch
```

### 3. 测试识别

站在相机前，系统会自动：
- 检测人脸（RetinaFace）
- 识别人脸（ArcFace）
- 播报问候语（如："早上好，张三，学生"）

## 📢 问候语规则

| 时间 | 问候语 |
|------|--------|
| 5:00 - 11:59 | 早上好 |
| 12:00 - 13:59 | 中午好 |
| 14:00 - 17:59 | 下午好 |
| 18:00 - 21:59 | 晚上好 |
| 22:00 - 4:59 | 晚上好 |

## ⚙️ 参数调整

编辑 `src/realsense_camera/launch/realsense_camera.launch`：

```xml
<!-- 识别阈值（0.0-1.0，越大越严格） -->
<param name="recognition_threshold" value="0.6" />

<!-- 重复识别冷却时间（秒） -->
<param name="recognition_cooldown" value="10.0" />
```

## 📝 图片要求

- **格式**：JPG、PNG等
- **数量**：建议3-5张
- **角度**：包含正面、侧面等
- **光照**：不同光照条件
- **清晰度**：人脸清晰可见

## ❓ 常见问题

- **识别不准确？** 增加训练图片数量，使用不同角度
- **识别速度慢？** 使用GPU（设置 `gpu_id=0`）
- **需要帮助？** 查看 `FACE_RECOGNITION_README.md`

