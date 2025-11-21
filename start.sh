#!/bin/bash

# RealSense人脸识别系统一键启动脚本
# 功能：人脸检测 + 人脸识别 + 语音问候
# 使用方法: ./start.sh

# 不设置 set -e，允许某些命令失败
set +e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "==============================================================="
echo "   RealSense 人脸识别系统 - 一键启动"
echo "==============================================================="

# 清理之前的进程
echo "[0/4] 清理之前的进程..."
cleanup_previous_processes() {
    local killed_count=0
    
    # 查找并终止 realsense_camera_node
    local camera_pids=$(pgrep -f "realsense_camera_node.py" 2>/dev/null)
    if [ -n "$camera_pids" ]; then
        echo "  正在终止 realsense_camera_node 进程..."
        echo "$camera_pids" | xargs kill -TERM 2>/dev/null
        sleep 1
        # 如果还在运行，强制终止
        local remaining=$(pgrep -f "realsense_camera_node.py" 2>/dev/null)
        if [ -n "$remaining" ]; then
            echo "$remaining" | xargs kill -KILL 2>/dev/null
        fi
        killed_count=$((killed_count + 1))
    fi
    
    # 查找并终止 roslaunch 进程（realsense_camera）
    local launch_pids=$(pgrep -f "roslaunch.*realsense_camera" 2>/dev/null)
    if [ -n "$launch_pids" ]; then
        echo "  正在终止 roslaunch 进程..."
        echo "$launch_pids" | xargs kill -TERM 2>/dev/null
        sleep 1
        local remaining=$(pgrep -f "roslaunch.*realsense_camera" 2>/dev/null)
        if [ -n "$remaining" ]; then
            echo "$remaining" | xargs kill -KILL 2>/dev/null
        fi
        killed_count=$((killed_count + 1))
    fi
    
    # 查找并终止 roscore（可选，如果用户想重启）
    # 注意：roscore 可能被其他节点使用，所以默认不终止
    # 如果需要终止 roscore，取消下面的注释
    # local roscore_pids=$(pgrep -f "roscore" 2>/dev/null)
    # if [ -n "$roscore_pids" ]; then
    #     echo "  正在终止 roscore 进程..."
    #     echo "$roscore_pids" | xargs kill -TERM 2>/dev/null
    #     sleep 2
    #     local remaining=$(pgrep -f "roscore" 2>/dev/null)
    #     if [ -n "$remaining" ]; then
    #         echo "$remaining" | xargs kill -KILL 2>/dev/null
    #     fi
    #     killed_count=$((killed_count + 1))
    # fi
    
    # 关闭相关的 gnome-terminal 窗口（如果使用 GUI）
    # 使用 pkill 查找并终止相关的 gnome-terminal 进程
    if [ -n "$DISPLAY" ]; then
        # 查找包含特定标题的 gnome-terminal 进程
        pkill -f "gnome-terminal.*roscore" 2>/dev/null
        pkill -f "gnome-terminal.*RealSense Camera" 2>/dev/null
        pkill -f "gnome-terminal.*Face Detection" 2>/dev/null
        sleep 0.5
    fi
    
    if [ $killed_count -gt 0 ]; then
        echo "  ✓ 已清理 $killed_count 个进程，等待 2 秒..."
        sleep 2
    else
        echo "  ✓ 未发现运行中的进程"
    fi
    
    # 额外清理：确保所有相关进程都已终止
    local final_check=$(pgrep -f "realsense_camera_node\|roslaunch.*realsense_camera" 2>/dev/null)
    if [ -n "$final_check" ]; then
        echo "  警告: 仍有进程在运行，强制清理..."
        echo "$final_check" | xargs kill -KILL 2>/dev/null
        sleep 1
    fi
}

cleanup_previous_processes

# 检测是否有图形环境
if [ -n "$DISPLAY" ] && command -v gnome-terminal &> /dev/null; then
    USE_GUI=true
else
    USE_GUI=false
    echo "提示: 未检测到图形环境，将在后台运行"
fi

# 加载ROS环境
if [ -z "$ROS_DISTRO" ]; then
    source /opt/ros/noetic/setup.bash
fi

# 加载工作空间
if [ -f "$SCRIPT_DIR/devel/setup.bash" ]; then
    source "$SCRIPT_DIR/devel/setup.bash"
else
    echo "错误: 工作空间未编译，请先运行: catkin_make"
    exit 1
fi

# 检查roscore
if ! rostopic list &>/dev/null 2>&1; then
    echo "[1/4] 启动roscore..."
    if [ "$USE_GUI" = true ]; then
        gnome-terminal --tab --title="roscore" -- bash -c "
            cd $SCRIPT_DIR
            source /opt/ros/noetic/setup.bash
            roscore
            exec bash
        " 2>/dev/null &
    else
        # 后台运行
        nohup bash -c "
            cd $SCRIPT_DIR
            source /opt/ros/noetic/setup.bash
            roscore > /tmp/roscore.log 2>&1
        " > /dev/null 2>&1 &
        echo "  roscore PID: $!"
    fi
    sleep 3
    echo "✓ roscore已启动"
else
    echo "[1/4] ✓ ROS Master正在运行"
fi

echo "[2/4] 启动RealSense人脸识别节点..."
if [ "$USE_GUI" = true ]; then
    gnome-terminal --tab --title="RealSense Camera" -- bash -c "
        cd $SCRIPT_DIR
        source /opt/ros/noetic/setup.bash
        source devel/setup.bash
        roslaunch realsense_camera realsense_camera.launch
        exec bash
    " 2>/dev/null &
    CAMERA_PID=$!
else
    # 后台运行
    nohup bash -c "
        cd $SCRIPT_DIR
        source /opt/ros/noetic/setup.bash
        source devel/setup.bash
        roslaunch realsense_camera realsense_camera.launch
    " > /tmp/realsense_camera.log 2>&1 &
    CAMERA_PID=$!
    echo "  RealSense节点 PID: $CAMERA_PID"
fi

sleep 3

echo "[3/4] 启动人脸识别图像查看器..."
if [ "$USE_GUI" = true ]; then
    gnome-terminal --tab --title="Face Detection" -- bash -c "
        cd $SCRIPT_DIR
        source /opt/ros/noetic/setup.bash
        source devel/setup.bash
        sleep 2
        echo '正在打开人脸识别图像查看器...'
        rosrun image_view image_view image:=/camera/face_detection/image_raw
        exec bash
    " 2>/dev/null &
    VIEWER_PID=$!
else
    echo "  提示: 无图形环境，跳过图像查看器"
    echo "  可以使用以下命令查看图像:"
    echo "    rosrun image_view image_view image:=/camera/face_detection/image_raw"
    VIEWER_PID=""
fi

sleep 1

echo "[4/4] ✓ 所有组件已启动"
echo ""
echo "==============================================================="
echo "   ✓ 启动完成！"
echo "==============================================================="
echo ""

if [ "$USE_GUI" = true ]; then
    echo "已打开的窗口:"
    echo "  1. roscore - ROS主节点"
    echo "  2. RealSense Camera - 相机节点（人脸识别+语音问候）"
    echo "  3. Face Recognition - 人脸识别图像查看器"
    echo ""
    echo "提示:"
    echo "  - 关闭终端窗口即可停止对应的节点"
else
    echo "后台运行的进程:"
    echo "  1. roscore - ROS主节点"
    echo "  2. RealSense Camera - 相机节点（人脸识别+语音问候）(PID: $CAMERA_PID)"
    if [ -n "$VIEWER_PID" ]; then
        echo "  3. Face Recognition - 图像查看器 (PID: $VIEWER_PID)"
    fi
    echo ""
    echo "提示:"
    echo "  - 查看日志: tail -f /tmp/realsense_camera.log"
    echo "  - 停止节点: kill $CAMERA_PID"
    echo "  - 查看话题列表: rostopic list"
fi

echo ""
echo "常用命令:"
echo "  - 查看人脸识别结果: rosrun image_view image_view image:=/camera/face_detection/image_raw"
echo "  - 查看原始图像: rosrun image_view image_view image:=/camera/color/image_raw"
echo "  - 查看深度图像: rosrun image_view image_view image:=/camera/depth/image_raw"
echo "  - 查看话题列表: rostopic list"
echo "  - 查看节点状态: rosnode list"
echo ""
echo "人脸识别功能:"
echo "  - 自动识别已添加的人物（曾福明、袁逸萧、冯国祥）"
echo "  - 识别成功后会根据时间自动播报问候语（早上好/中午好/下午好/晚上好）"
echo "  - 添加新人物: python3 src/realsense_camera/scripts/add_person.py <ID> <姓名> <身份> <图片路径>"
echo "==============================================================="
