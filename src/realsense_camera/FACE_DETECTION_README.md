# 人脸检测功能说明

## 功能特性

- 实时人脸检测：使用OpenCV DNN模块和ResNet-SSD模型
- 深度信息显示：在检测到的人脸框上显示距离信息
- 置信度显示：显示每个检测到的人脸的置信度百分比
- 多话题发布：同时发布原始图像和带检测框的图像

## 发布的话题

- `/camera/color/image_raw` - 原始彩色图像
- `/camera/depth/image_raw` - 深度图像
- `/camera/face_detection/image_raw` - 带有人脸检测框的图像
- `/camera/color/camera_info` - 彩色相机内参
- `/camera/depth/camera_info` - 深度相机内参

## 使用方法

### 1. 启动节点

```bash
cd /home/pc/realsense
source devel/setup.bash
roslaunch realsense_camera realsense_camera.launch
```

### 2. 查看人脸检测结果

在另一个终端中：

```bash
source /home/pc/realsense/devel/setup.bash
rosrun image_view image_view image:=/camera/face_detection/image_raw
```

### 3. 查看原始图像

```bash
rosrun image_view image_view image:=/camera/color/image_raw
```

### 4. 使用rviz可视化

```bash
rosrun rviz rviz
```

在rviz中添加：
- Image: `/camera/face_detection/image_raw` - 查看人脸检测结果
- Image: `/camera/color/image_raw` - 查看原始彩色图像
- Image: `/camera/depth/image_raw` - 查看深度图像

## 参数配置

在launch文件中可以调整以下参数：

- `width`: 图像宽度 (默认: 640)
- `height`: 图像高度 (默认: 480)
- `fps`: 帧率 (默认: 30)
- `face_confidence_threshold`: 人脸检测置信度阈值 (默认: 0.5, 范围: 0.0-1.0)

示例：

```xml
<param name="face_confidence_threshold" value="0.7" />
```

较高的阈值（如0.7）会减少误检，但可能漏检一些人脸。
较低的阈值（如0.3）会检测到更多人脸，但可能增加误检。

## 模型文件

人脸检测模型文件位于：
- `models/deploy.prototxt` - 网络结构定义
- `models/res10_300x300_ssd_iter_140000.caffemodel` - 训练好的模型权重

## 故障排除

### 问题：检测不到人脸

1. 检查光照条件是否充足
2. 降低置信度阈值：`face_confidence_threshold` 设为 0.3-0.4
3. 确保人脸正对摄像头
4. 检查模型文件是否存在

### 问题：误检太多

1. 提高置信度阈值：`face_confidence_threshold` 设为 0.6-0.7
2. 改善光照条件

### 问题：模型加载失败

检查模型文件路径：
```bash
rospack find realsense_camera
ls -lh $(rospack find realsense_camera)/models/
```

确保两个文件都存在且大小正确：
- `deploy.prototxt` 约 28KB
- `res10_300x300_ssd_iter_140000.caffemodel` 约 10MB

