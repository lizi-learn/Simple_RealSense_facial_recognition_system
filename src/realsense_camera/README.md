# RealSense Camera ROS1 Noetic 包

这是一个用于在ROS1 Noetic中调用Intel RealSense摄像头的Python包。

## 功能特性

- 发布彩色图像到 `/camera/color/image_raw`
- 发布深度图像到 `/camera/depth/image_raw`
- 发布相机内参信息到 `/camera/color/camera_info` 和 `/camera/depth/camera_info`
- 支持可配置的分辨率和帧率
- 自动对齐深度图到彩色图

## 依赖

### 系统依赖

- ROS1 Noetic
- Intel RealSense SDK 2.57.4 (已通过 `build_realsense_in_ubuntu_2004.sh` 安装)
- Python 3
- pyrealsense2 (RealSense Python绑定)
- OpenCV (Python)
- NumPy

### ROS依赖

```bash
sudo apt install ros-noetic-cv-bridge ros-noetic-image-transport
```

## 安装

1. 将包复制到你的catkin工作空间：

```bash
cd ~/catkin_ws/src
cp -r /home/pc/realsense/realsense_camera .
```

2. 编译工作空间：

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## 使用方法

### 启动节点

```bash
roslaunch realsense_camera realsense_camera.launch
```

### 查看话题

```bash
# 列出所有话题
rostopic list

# 查看彩色图像
rostopic echo /camera/color/image_raw

# 查看深度图像
rostopic echo /camera/depth/image_raw
```

### 使用rviz可视化

```bash
rosrun rviz rviz
```

在rviz中添加以下显示：
- Image: `/camera/color/image_raw`
- Image: `/camera/depth/image_raw`

### 使用image_view查看图像

```bash
# 查看彩色图像
rosrun image_view image_view image:=/camera/color/image_raw

# 查看深度图像
rosrun image_view image_view image:=/camera/depth/image_raw
```

## 参数配置

在launch文件中可以修改以下参数：

- `width`: 图像宽度 (默认: 640)
- `height`: 图像高度 (默认: 480)
- `fps`: 帧率 (默认: 30)

示例：

```xml
<param name="width" value="1280" />
<param name="height" value="720" />
<param name="fps" value="60" />
```

## 发布的话题

- `/camera/color/image_raw` (sensor_msgs/Image) - 彩色图像
- `/camera/depth/image_raw` (sensor_msgs/Image) - 深度图像
- `/camera/color/camera_info` (sensor_msgs/CameraInfo) - 彩色相机内参
- `/camera/depth/camera_info` (sensor_msgs/CameraInfo) - 深度相机内参

## 故障排除

### 问题：找不到pyrealsense2模块

确保RealSense SDK已正确安装，并且Python可以找到模块：

```bash
python3 -c "import sys; sys.path.insert(0, '/usr/lib/python3.8/site-packages'); import pyrealsense2 as rs; print('OK')"
```

### 问题：无法检测到设备

1. 检查USB连接（确保使用USB 3.0端口）
2. 检查设备权限：
   ```bash
   lsusb | grep Intel
   ```
3. 重新加载udev规则：
   ```bash
   sudo udevadm control --reload-rules
   sudo udevadm trigger
   ```

### 问题：ROS找不到包

确保已source工作空间：

```bash
source ~/catkin_ws/devel/setup.bash
```

或者添加到 `~/.bashrc`：

```bash
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

## 许可证

Apache 2.0

