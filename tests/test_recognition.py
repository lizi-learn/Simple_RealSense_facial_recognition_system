#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
测试人脸识别功能
"""

import sys
import os
sys.path.insert(0, 'src/realsense_camera/scripts')

import cv2
import numpy as np
from face_recognition_manager import FaceRecognitionManager

def test_recognition(image_path, database_dir='data/face_database'):
    """测试识别功能"""
    print("=" * 60)
    print("人脸识别测试")
    print("=" * 60)
    
    # 加载图像
    if not os.path.exists(image_path):
        print(f"错误: 图像文件不存在: {image_path}")
        return
    
    image = cv2.imread(image_path)
    if image is None:
        print(f"错误: 无法读取图像: {image_path}")
        return
    
    print(f"✓ 成功加载图像: {image_path}")
    print(f"  图像尺寸: {image.shape}")
    
    # 初始化识别管理器
    print("\n正在初始化识别管理器...")
    manager = FaceRecognitionManager(database_dir, ctx_id=-1, recognition_threshold=0.5)
    
    if manager.app is None:
        print("✗ 识别模型加载失败")
        return
    
    print("✓ 识别模型加载成功")
    print(f"✓ 数据库中有 {len(manager.known_faces)} 个已知人物")
    
    # 显示数据库信息
    for person_id, person_data in manager.known_faces.items():
        print(f"  - {person_id}: {person_data['name']} ({person_data['role']}), {len(person_data['embeddings'])} 个特征")
    
    # 先提取特征看看
    print("\n正在提取人脸特征...")
    faces, embeddings = manager.extract_face_embedding(image)
    print(f"检测到 {len(faces)} 个人脸，提取了 {len(embeddings)} 个特征向量")
    
    if len(embeddings) > 0:
        print(f"特征向量维度: {embeddings[0].shape}")
        print(f"特征向量示例（前10个值）: {embeddings[0][:10]}")
    
    # 进行识别
    print("\n正在识别...")
    results = manager.detect_and_recognize(image)
    
    print(f"\n检测到 {len(results)} 个人脸:")
    print("-" * 60)
    
    for i, result in enumerate(results):
        person_id = result['person_id']
        name = result['name']
        role = result['role']
        confidence = result['confidence']
        det_confidence = result['detection_confidence']
        bbox = result['bbox']
        
        print(f"\n人脸 {i+1}:")
        print(f"  检测置信度: {det_confidence:.4f}")
        print(f"  边界框: {bbox}")
        
        if person_id is not None:
            print(f"  ✓ 识别成功!")
            print(f"    人物ID: {person_id}")
            print(f"    姓名: {name}")
            print(f"    身份: {role}")
            print(f"    相似度: {confidence:.4f} (阈值: {manager.recognition_threshold})")
        else:
            print(f"  ✗ 未识别")
            if confidence > 0:
                print(f"    最高相似度: {confidence:.4f} (阈值: {manager.recognition_threshold})")
                print(f"    相似度低于阈值 {manager.recognition_threshold}, 未匹配")
            else:
                print(f"    未找到匹配的人脸")
    
    # 手动测试识别
    if len(results) > 0 and results[0]['confidence'] == 0.0:
        print("\n" + "=" * 60)
        print("详细诊断: 为什么识别失败?")
        print("=" * 60)
        
        # 提取人脸区域
        bbox = results[0]['bbox']
        face_roi = image[bbox[1]:bbox[3], bbox[0]:bbox[2]]
        
        # 手动调用 recognize_face，并添加详细调试
        print("\n手动调用 recognize_face...")
        
        # 先提取特征
        faces2, embeddings2 = manager.extract_face_embedding(face_roi)
        print(f"从人脸区域提取: {len(faces2)} 个人脸, {len(embeddings2)} 个特征")
        
        if len(embeddings2) > 0:
            query_emb = np.array(embeddings2[0])
            query_emb = query_emb / np.linalg.norm(query_emb)
            print(f"查询特征向量: 维度={query_emb.shape}, 范数={np.linalg.norm(query_emb):.4f}")
            
            # 手动计算相似度
            print("\n与数据库中的特征比对:")
            for person_id, person_data in manager.known_faces.items():
                for idx, emb in enumerate(person_data['embeddings']):
                    db_emb = np.array(emb)
                    db_emb = db_emb / np.linalg.norm(db_emb)
                    similarity = np.dot(query_emb, db_emb)
                    print(f"  {person_id} ({person_data['name']}) 特征{idx}: 相似度={similarity:.4f}")
        
        person_id, name, role, confidence = manager.recognize_face(face_roi, threshold=0.3)
        print(f"\nrecognize_face 结果: person_id={person_id}, name={name}, confidence={confidence:.4f}")
        
        # 检查数据库中的特征
        print("\n数据库中的特征向量:")
        for person_id, person_data in manager.known_faces.items():
            print(f"  {person_id} ({person_data['name']}):")
            for idx, emb in enumerate(person_data['embeddings']):
                emb_array = np.array(emb)
                print(f"    特征 {idx}: 维度={emb_array.shape}, 范数={np.linalg.norm(emb_array):.4f}")
    
    # 测试不同阈值
    print("\n" + "=" * 60)
    print("测试不同阈值下的识别结果:")
    print("=" * 60)
    
    thresholds = [0.3, 0.4, 0.5, 0.6, 0.7]
    for threshold in thresholds:
        manager.recognition_threshold = threshold
        results = manager.detect_and_recognize(image)
        for result in results:
            if result['person_id'] is not None:
                print(f"阈值 {threshold}: ✓ 识别为 {result['name']} (相似度: {result['confidence']:.4f})")
            else:
                if result['confidence'] > 0:
                    print(f"阈值 {threshold}: ✗ 未识别 (最高相似度: {result['confidence']:.4f})")
                else:
                    print(f"阈值 {threshold}: ✗ 未识别 (未找到匹配)")

if __name__ == '__main__':
    import sys
    if len(sys.argv) > 1:
        image_path = sys.argv[1]
    else:
        image_path = '曾福明-2.jpg'
    
    test_recognition(image_path)

