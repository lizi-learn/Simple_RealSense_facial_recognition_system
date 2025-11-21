#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
添加人物到人脸识别数据库（使用RetinaFace + ArcFace）
使用方法: python3 add_person.py <person_id> <name> <role> <image1> [image2] [image3] ...
"""

import sys
import os

# 添加路径
sys.path.insert(0, '/usr/lib/python3.8/site-packages')
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from face_recognition_manager import FaceRecognitionManager

def main():
    if len(sys.argv) < 5:
        print("=" * 60)
        print("添加人物到人脸识别数据库")
        print("=" * 60)
        print("\n使用方法:")
        print("  python3 add_person.py <person_id> <name> <role> <image1> [image2] [image3] ...")
        print("\n示例:")
        print("  python3 add_person.py person001 张三 学生 face1.jpg face2.jpg face3.jpg")
        print("\n参数说明:")
        print("  person_id: 人物唯一ID（如：person001, teacher001）")
        print("  name: 姓名")
        print("  role: 身份/角色（如：学生、老师、员工）")
        print("  image1, image2, ...: 人脸图片路径（至少1张，建议3-5张）")
        print("=" * 60)
        sys.exit(1)
    
    person_id = sys.argv[1]
    name = sys.argv[2]
    role = sys.argv[3]
    image_files = sys.argv[4:]
    
    # 检查图像文件
    for img_file in image_files:
        if not os.path.exists(img_file):
            print(f"✗ 错误: 图像文件不存在: {img_file}")
            sys.exit(1)
    
    print("=" * 60)
    print("添加人物到人脸识别数据库")
    print("=" * 60)
    print(f"人物ID: {person_id}")
    print(f"姓名: {name}")
    print(f"身份: {role}")
    print(f"图像数量: {len(image_files)}")
    print("=" * 60)
    
    # 创建管理器
    script_dir = os.path.dirname(os.path.abspath(__file__))
    database_dir = os.path.join(script_dir, '..', '..', '..', 'data', 'face_database')
    database_dir = os.path.abspath(database_dir)
    
    print(f"\n数据库目录: {database_dir}")
    print("\n正在初始化InsightFace模型（RetinaFace + ArcFace）...")
    
    manager = FaceRecognitionManager(database_dir, ctx_id=-1)  # -1表示使用CPU
    
    if manager.app is None:
        print("✗ 错误: 无法加载InsightFace模型")
        sys.exit(1)
    
    # 添加人物
    print(f"\n正在处理 {len(image_files)} 张图像...")
    success = manager.add_person(person_id, name, role, image_files)
    
    if success:
        print("\n" + "=" * 60)
        print(f"✓ 成功添加人物: {name} ({role})")
        print("=" * 60)
    else:
        print("\n" + "=" * 60)
        print("✗ 添加失败")
        print("=" * 60)
        sys.exit(1)

if __name__ == '__main__':
    main()
