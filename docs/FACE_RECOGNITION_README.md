# 人脸识别功能说明（RetinaFace + ArcFace）

## 🎯 功能概述

使用工业级的人脸识别方案：
- **RetinaFace**：高精度人脸检测
- **ArcFace (InsightFace)**：高精度人脸识别

识别到已知人物后，会根据时间自动播报问候语（早上好/中午好/下午好/晚上好）。

## ✨ 功能特点

- ✅ **工业级精度**：使用RetinaFace + ArcFace组合
- ✅ **自动问候**：根据时间自动播报（早上好/中午好/下午好/晚上好）
- ✅ **支持多人**：可同时识别多个人脸
- ✅ **中文语音**：使用微软TTS播报
- ✅ **防重复播报**：10秒内不重复识别同一人

## 📋 使用步骤

### 1. 添加人物到数据库

使用 `add_person.py` 脚本添加人物：

```bash
cd /home/pc/realsense
python3 src/realsense_camera/scripts/add_person.py <person_id> <name> <role> <image1> [image2] [image3] ...
```

**示例：**

```bash
# 添加一个学生
python3 src/realsense_camera/scripts/add_person.py student001 张三 学生 \
    /path/to/face1.jpg /path/to/face2.jpg /path/to/face3.jpg

# 添加一个老师
python3 src/realsense_camera/scripts/add_person.py teacher001 李老师 老师 \
    /path/to/teacher1.jpg /path/to/teacher2.jpg
```

**参数说明：**
- `person_id`: 人物唯一ID（如：student001, teacher001）
- `name`: 姓名
- `role`: 身份/角色（如：学生、老师、员工）
- `image1, image2, ...`: 人脸图片路径（建议3-5张不同角度的照片）

**图片要求：**
- 格式：JPG、PNG等常见格式
- 内容：清晰的人脸照片
- 数量：建议3-5张，不同角度和光照条件
- 质量：人脸清晰可见，正面或侧面均可

### 2. 启动节点

```bash
cd /home/pc/realsense
source devel/setup.bash
roslaunch realsense_camera realsense_camera.launch
```

### 3. 测试识别

站在相机前，系统会：
1. 检测人脸（RetinaFace）
2. 识别人脸（ArcFace）
3. 如果识别成功，播报问候语（如："早上好，张三，学生"）

## 🎤 问候语规则

系统会根据当前时间自动选择问候语：

| 时间 | 问候语 |
|------|--------|
| 5:00 - 11:59 | 早上好 |
| 12:00 - 13:59 | 中午好 |
| 14:00 - 17:59 | 下午好 |
| 18:00 - 21:59 | 晚上好 |
| 22:00 - 4:59 | 晚上好 |

**播报格式：**
- 有身份：`"早上好，张三，学生"`
- 无身份：`"早上好，张三"`

## ⚙️ 参数配置

在 `realsense_camera.launch` 中可以配置：

```xml
<!-- 人脸识别参数 -->
<param name="enable_face_recognition" value="true" />
<param name="recognition_threshold" value="0.6" />
<param name="recognition_cooldown" value="10.0" />
<param name="gpu_id" value="-1" />
```

**参数说明：**

- `enable_face_recognition`: 启用/禁用人脸识别（true/false）
- `recognition_threshold`: 识别阈值（0.0-1.0，越大越严格）
  - 推荐值：0.6（平衡精度和召回率）
  - 更严格：0.7（减少误识别）
  - 更宽松：0.5（识别更多人）
- `recognition_cooldown`: 重复识别冷却时间（秒）
  - 默认10秒，避免重复播报
- `gpu_id`: GPU设备ID
  - `-1`: 使用CPU（默认）
  - `0+`: 使用GPU（如果有CUDA）

## 📁 数据库结构

人脸数据库存储在 `face_database/` 目录：

```
face_database/
├── database.json          # 人物信息数据库
└── faces/                 # 人脸图像
    ├── student001/
    │   ├── face_0.jpg
    │   ├── face_1.jpg
    │   └── face_2.jpg
    └── teacher001/
        ├── face_0.jpg
        └── face_1.jpg
```

## 🔧 管理数据库

### 查看数据库

```bash
cat face_database/database.json
```

### 删除人物

直接编辑 `database.json` 文件，删除对应的人物条目，或删除 `faces/<person_id>/` 目录。

### 备份数据库

```bash
cp -r face_database face_database_backup
```

## 🎨 识别结果可视化

识别结果会在图像上显示：

- **蓝色框**：已识别的人脸（显示姓名、身份、相似度）
- **绿色框**：未识别的人脸（显示"未知"）

## 📊 性能说明

### RetinaFace + ArcFace 优势

- **高精度**：工业级识别准确率
- **鲁棒性**：对光照、角度变化不敏感
- **实时性**：CPU模式下约30-50ms/帧
- **多尺度**：可检测不同大小的人脸

### 性能优化建议

1. **使用GPU**：如果有NVIDIA GPU，设置 `gpu_id=0` 可大幅提升速度
2. **降低分辨率**：如果CPU性能不足，可降低相机分辨率
3. **调整阈值**：根据实际需求调整识别阈值

## 🔍 故障排除

### 问题1：模型下载失败

**症状：** 启动时提示模型加载失败

**解决方案：**
```bash
# 手动下载模型（使用代理）
export http_proxy=http://127.0.0.1:7890
export https_proxy=http://127.0.0.1:7890
python3 -c "import insightface; app = insightface.app.FaceAnalysis(name='buffalo_l')"
```

### 问题2：识别不准确

**解决方案：**
1. 增加训练图片数量（建议5-10张）
2. 使用不同角度和光照条件的照片
3. 提高识别阈值（如0.7）
4. 确保人脸清晰可见

### 问题3：识别速度慢

**解决方案：**
1. 使用GPU（设置 `gpu_id=0`）
2. 降低相机分辨率
3. 降低帧率

### 问题4：无法识别已知人物

**检查：**
1. 确认人物已添加到数据库
2. 检查识别阈值是否过高
3. 确认人脸清晰可见
4. 尝试重新添加人物（使用更多照片）

## 📚 技术细节

### RetinaFace
- 单阶段人脸检测器
- 支持多尺度检测
- 高精度边界框和关键点

### ArcFace
- 深度人脸识别模型
- 512维特征向量
- 使用余弦相似度比对

### 识别流程
1. RetinaFace检测人脸 → 获取边界框和关键点
2. ArcFace提取特征 → 512维特征向量
3. 特征比对 → 与数据库中的特征计算余弦相似度
4. 阈值判断 → 相似度 >= 阈值则识别成功

## 🚀 快速开始示例

```bash
# 1. 添加人物
python3 src/realsense_camera/scripts/add_person.py student001 张三 学生 \
    ~/photos/face1.jpg ~/photos/face2.jpg ~/photos/face3.jpg

# 2. 启动节点
source devel/setup.bash
roslaunch realsense_camera realsense_camera.launch

# 3. 站在相机前，等待识别和问候
```

## 📝 注意事项

1. **首次使用**：模型会自动下载（buffalo_l），需要网络连接
2. **图片质量**：使用清晰的人脸照片，避免模糊、遮挡
3. **光照条件**：建议在不同光照条件下拍摄训练照片
4. **角度变化**：包含正面、侧面等不同角度的照片
5. **重复播报**：10秒内不会重复识别同一人，避免频繁播报

---

**享受精准的人脸识别体验！** 🎉

