#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
实时显示深度图像和彩色图像（两个独立窗口）
按 'q' 键退出
"""

import sys
sys.path.insert(0, '/usr/lib/python3.8/site-packages')

import numpy as np
import cv2
import pyrealsense2 as rs

def main():
    print("=" * 60)
    print("RealSense 深度图像查看器")
    print("=" * 60)
    print("操作说明:")
    print("  - 彩色图像窗口: 显示RGB彩色图像")
    print("  - 深度图像窗口: 显示深度图（彩色映射）")
    print("  - 按 'q' 键退出")
    print("=" * 60)
    
    # 创建管道
    pipeline = rs.pipeline()
    config = rs.config()
    
    # 配置流
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    
    # 启动流
    try:
        profile = pipeline.start(config)
        print("\n✓ RealSense相机启动成功")
    except Exception as e:
        print(f"\n✗ 无法启动相机: {e}")
        print("请检查:")
        print("  1. 相机是否已连接")
        print("  2. 运行 'rs-enumerate-devices' 检查设备")
        sys.exit(1)
    
    # 创建对齐对象（将深度图对齐到彩色图）
    align_to = rs.align(rs.stream.color)
    
    try:
        while True:
            # 等待帧
            frames = pipeline.wait_for_frames(timeout_ms=1000)
            
            # 对齐深度帧到彩色帧
            aligned_frames = align_to.process(frames)
            
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            
            if not depth_frame or not color_frame:
                continue
            
            # 转换为numpy数组
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            
            # 将深度图转换为彩色图（用于可视化）
            # 深度值范围通常在0-65535（16位），单位是毫米
            # 使用颜色映射来可视化深度
            depth_colormap = cv2.applyColorMap(
                cv2.convertScaleAbs(depth_image, alpha=0.03), 
                cv2.COLORMAP_JET
            )
            
            # 在深度图上添加距离信息
            # 获取中心点的深度值
            h, w = depth_image.shape
            center_x, center_y = w // 2, h // 2
            center_depth = depth_image[center_y, center_x]
            
            # 在深度图上绘制中心点和距离信息
            cv2.circle(depth_colormap, (center_x, center_y), 5, (255, 255, 255), 2)
            if center_depth > 0:
                depth_text = f"Center: {center_depth/1000:.2f}m"
                cv2.putText(depth_colormap, depth_text, (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # 在彩色图上也添加中心点标记
            cv2.circle(color_image, (center_x, center_y), 5, (0, 255, 0), 2)
            if center_depth > 0:
                cv2.putText(color_image, depth_text, (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # 显示图像（两个独立窗口）
            cv2.imshow('彩色图像 (Color)', color_image)
            cv2.imshow('深度图像 (Depth)', depth_colormap)
            
            # 按 'q' 键退出
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
    except KeyboardInterrupt:
        print("\n\n用户中断")
    except Exception as e:
        print(f"\n✗ 错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # 清理
        pipeline.stop()
        cv2.destroyAllWindows()
        print("\n✓ 已关闭相机和窗口")

if __name__ == '__main__':
    main()

