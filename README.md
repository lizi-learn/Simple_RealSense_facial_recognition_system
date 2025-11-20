# Intel RealSense SDK 2.57.4 安装脚本

这是一个用于在 Ubuntu 系统上自动安装 Intel RealSense SDK 2.57.4 的一键安装脚本。

## 📋 目录

- [系统要求](#系统要求)
- [支持的设备](#支持的设备)
- [快速开始](#快速开始)
- [安装步骤](#安装步骤)
- [验证安装](#验证安装)
- [使用说明](#使用说明)
- [Python 使用示例](#python-使用示例)
- [故障排除](#故障排除)
- [常见问题](#常见问题)
- [参考链接](#参考链接)

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

### 2. 运行安装脚本

```bash
chmod +x build_realsense_in_ubuntu_2004.sh
./build_realsense_in_ubuntu_2004.sh
```

**注意：** 安装过程可能需要 10-30 分钟，具体取决于您的系统性能。

## 📝 安装步骤

脚本会自动执行以下步骤：

1. **更新系统依赖** - 更新 apt 包管理器并安装必要的开发工具
2. **下载源码** - 从 GitHub 克隆 librealsense 仓库（v2.57.4）
3. **创建构建目录** - 准备编译环境
4. **CMake 配置** - 配置编译选项（包括 Python 绑定）
5. **编译 SDK** - 使用多核编译加速构建过程
6. **安装到系统** - 将库文件和工具安装到系统目录
7. **配置 udev 规则** - 设置 USB 设备权限

## ✅ 验证安装

### 检查命令行工具

```bash
# 检查 realsense-viewer 版本
realsense-viewer --version

# 检查设备枚举工具
rs-enumerate-devices --version

# 列出所有已安装的工具
ls /usr/local/bin/rs-*
```

### 检查库文件

```bash
# 检查 C++ 库
ls -lh /usr/local/lib/librealsense2.so*

# 检查 Python 模块
ls -lh /usr/lib/python3.8/site-packages/pyrealsense2/
```

### 测试 Python 模块

```bash
python3 -c "
import sys
sys.path.insert(0, '/usr/lib/python3.8/site-packages')
import pyrealsense2 as rs
ctx = rs.context()
devices = ctx.query_devices()
print(f'✓ Python 模块工作正常！检测到 {len(devices)} 个设备')
"
```

### 测试设备连接

```bash
# 连接 RealSense 相机后运行
rs-enumerate-devices

# 或启动图形界面
realsense-viewer
```

## 💻 使用说明

### 命令行工具

安装完成后，您可以使用以下命令行工具：

- `realsense-viewer` - 图形化相机查看器
- `rs-enumerate-devices` - 列出所有连接的设备
- `rs-capture` - 捕获图像和深度数据
- `rs-pointcloud` - 点云可视化
- `rs-align` - 对齐彩色和深度流
- `rs-record` - 录制数据流
- `rs-fw-update` - 固件更新工具

### 查看所有可用工具

```bash
ls /usr/local/bin/rs-*
```

### 在 C++ 项目中使用

```cpp
#include <librealsense2/rs.hpp>

int main() {
    rs2::pipeline p;
    p.start();
    
    // 获取深度帧
    auto frames = p.wait_for_frames();
    auto depth = frames.get_depth_frame();
    
    // 处理深度数据...
    
    return 0;
}
```

编译时链接库：

```bash
g++ your_code.cpp -lrealsense2 -o your_program
```

## 🐍 Python 使用示例

### 基本示例

```python
import sys
sys.path.insert(0, '/usr/lib/python3.8/site-packages')
import pyrealsense2 as rs
import numpy as np
import cv2

# 创建管道
pipeline = rs.pipeline()
config = rs.config()

# 配置流
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# 启动流
pipeline.start(config)

try:
    while True:
        # 等待帧
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        
        if not depth_frame or not color_frame:
            continue
        
        # 转换为 numpy 数组
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        
        # 应用颜色映射到深度图
        depth_colormap = cv2.applyColorMap(
            cv2.convertScaleAbs(depth_image, alpha=0.03), 
            cv2.COLORMAP_JET
        )
        
        # 显示图像
        images = np.hstack((color_image, depth_colormap))
        cv2.imshow('RealSense', images)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
```

### 点云示例

```python
import sys
sys.path.insert(0, '/usr/lib/python3.8/site-packages')
import pyrealsense2 as rs
import numpy as np

# 创建点云对象
pc = rs.pointcloud()
points = rs.points()

# 创建管道
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)

pipeline.start(config)

try:
    frames = pipeline.wait_for_frames()
    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()
    
    # 生成点云
    pc.map_to(color_frame)
    points = pc.calculate(depth_frame)
    
    # 获取顶点和纹理坐标
    vertices = np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1, 3)
    tex_coords = np.asanyarray(points.get_texture_coordinates()).view(np.float32).reshape(-1, 2)
    
    print(f"点云包含 {len(vertices)} 个点")
    
finally:
    pipeline.stop()
```

## 🔧 故障排除

### 问题 1: Python 模块导入失败

**症状：**
```python
ModuleNotFoundError: No module named 'pyrealsense2'
```

**解决方案：**
```python
import sys
sys.path.insert(0, '/usr/lib/python3.8/site-packages')
import pyrealsense2 as rs
```

或者创建符号链接：
```bash
sudo ln -s /usr/lib/python3.8/site-packages/pyrealsense2 /usr/local/lib/python3.8/dist-packages/
```

### 问题 2: 设备未检测到

**症状：**
```
No device detected. Is it plugged in?
```

**解决方案：**

1. 检查 USB 连接（确保使用 USB 3.0 端口）
2. 检查 udev 规则：
   ```bash
   ls -l /etc/udev/rules.d/99-realsense-libusb.rules
   ```
3. 重新加载 udev 规则：
   ```bash
   sudo udevadm control --reload-rules
   sudo udevadm trigger
   ```
4. 检查设备权限：
   ```bash
   lsusb | grep Intel
   ```

### 问题 3: 编译错误

**症状：** CMake 配置或编译失败

**解决方案：**

1. 确保所有依赖已安装：
   ```bash
   sudo apt update
   sudo apt install -y git cmake build-essential libusb-1.0-0-dev
   ```

2. 清理构建目录并重新编译：
   ```bash
   cd ~/librealsense
   rm -rf build
   mkdir build && cd build
   cmake .. -DFORCE_RSUSB_BACKEND=true
   make -j$(nproc)
   ```

### 问题 4: 权限错误

**症状：** 无法访问 USB 设备

**解决方案：**

1. 将用户添加到 `plugdev` 组：
   ```bash
   sudo usermod -a -G plugdev $USER
   ```
   然后重新登录。

2. 检查 udev 规则文件：
   ```bash
   cat /etc/udev/rules.d/99-realsense-libusb.rules
   ```

### 问题 5: 库文件未找到

**症状：** 运行时找不到 `librealsense2.so`

**解决方案：**

```bash
# 更新库缓存
sudo ldconfig

# 检查库路径
ldconfig -p | grep realsense
```

## ❓ 常见问题

### Q: 安装需要多长时间？

A: 通常需要 10-30 分钟，取决于您的 CPU 性能和网络速度。

### Q: 可以卸载吗？

A: 可以，运行以下命令：
```bash
cd ~/librealsense/build
sudo make uninstall
sudo rm -rf ~/librealsense
sudo rm /etc/udev/rules.d/99-realsense-libusb.rules
```

### Q: 支持哪些 Python 版本？

A: 支持 Python 3.7 及以上版本。脚本会自动检测并使用系统的 Python 3。

### Q: 如何更新固件？

A: 使用 `rs-fw-update` 工具：
```bash
rs-fw-update -l  # 列出设备
rs-fw-update -f <firmware_file.bin>  # 更新固件
```

### Q: 可以在 ROS 中使用吗？

A: 可以！安装 ROS wrapper：
```bash
# 对于 ROS Noetic (Ubuntu 20.04)
sudo apt install ros-noetic-realsense2-camera

# 对于 ROS 2
sudo apt install ros-humble-realsense2-camera
```

### Q: 如何录制和回放数据？

A: 使用 `rs-record` 录制：
```bash
rs-record -a  # 录制所有流
```

使用 Python API 回放：
```python
config.enable_device_from_file("recording.bag")
```

## 📚 参考链接

- [Intel RealSense SDK 官方文档](https://dev.intelrealsense.com/)
- [librealsense GitHub 仓库](https://github.com/IntelRealSense/librealsense)
- [API 参考文档](https://intelrealsense.github.io/librealsense/doxygen/annotated.html)
- [Python API 文档](https://intelrealsense.github.io/librealsense/python_docs/_generated/pyrealsense2.html)
- [社区论坛](https://support.intelrealsense.com/)

## 📄 许可证

本脚本遵循 Intel RealSense SDK 的许可证。Intel RealSense SDK 使用 Apache 2.0 许可证。

## 🤝 贡献

欢迎提交 Issue 和 Pull Request！

## 📝 更新日志

### v2.57.4
- 初始版本
- 支持 Ubuntu 20.04/22.04/24.04
- 自动安装 Python 绑定
- 修复 apt update 错误处理
- 修复 Python 路径配置问题

## ⚠️ 注意事项

1. **权限要求：** 脚本需要 sudo 权限来安装系统库和配置 udev 规则
2. **网络连接：** 需要稳定的网络连接来下载源码和依赖
3. **磁盘空间：** 确保有足够的磁盘空间（至少 5GB）
4. **编译时间：** 首次编译可能需要较长时间，请耐心等待
5. **USB 端口：** 建议使用 USB 3.0 端口以获得最佳性能

## 📧 支持

如果遇到问题，请：
1. 查看 [故障排除](#故障排除) 部分
2. 检查 [常见问题](#常见问题)
3. 访问 [Intel RealSense 社区论坛](https://support.intelrealsense.com/)

---

**祝您使用愉快！** 🎉

