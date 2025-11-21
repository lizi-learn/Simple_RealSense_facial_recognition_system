#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
人脸识别管理器 - 使用RetinaFace + ArcFace（工业级）
"""

import os
import json
import cv2
import numpy as np
from datetime import datetime
import insightface
from insightface.app import FaceAnalysis
from insightface.data import get_image as ins_get_image

class FaceRecognitionManager:
    def __init__(self, database_dir, ctx_id=0, recognition_threshold=0.5):
        """
        初始化人脸识别管理器
        
        Args:
            database_dir: 人脸数据库目录
            ctx_id: GPU设备ID，-1表示CPU
            recognition_threshold: 识别阈值（余弦相似度，默认0.5，范围0-1）
        """
        self.database_dir = database_dir
        self.recognition_threshold = recognition_threshold
        self.known_faces = {}  # {person_id: {'name': str, 'role': str, 'embeddings': list}}
        
        # 创建数据库目录
        os.makedirs(database_dir, exist_ok=True)
        os.makedirs(os.path.join(database_dir, 'faces'), exist_ok=True)
        
        # 初始化InsightFace（RetinaFace检测 + ArcFace识别）
        try:
            # 使用buffalo_l模型（推荐，精度高）
            self.app = FaceAnalysis(providers=['CPUExecutionProvider'], name='buffalo_l')
            self.app.prepare(ctx_id=ctx_id, det_size=(640, 640))
            try:
                import rospy
                rospy.loginfo("✓ InsightFace模型加载成功（RetinaFace + ArcFace）")
            except:
                print("✓ InsightFace模型加载成功（RetinaFace + ArcFace）")
        except Exception as e:
            print(f"✗ InsightFace模型加载失败: {e}")
            print("正在下载模型...")
            # 如果模型不存在，会自动下载
            try:
                self.app = FaceAnalysis(providers=['CPUExecutionProvider'], name='buffalo_l')
                self.app.prepare(ctx_id=ctx_id, det_size=(640, 640))
                print("✓ 模型下载并加载成功")
            except Exception as e2:
                print(f"✗ 模型加载失败: {e2}")
                self.app = None
        
        # 加载已知人脸数据库
        self.load_database()
    
    def extract_face_embedding(self, image):
        """
        提取人脸特征向量（使用ArcFace）
        
        Args:
            image: 图像（BGR格式）
            
        Returns:
            (faces, embeddings) - 检测到的人脸列表和对应的特征向量列表
        """
        if self.app is None:
            return [], []
        
        try:
            # 使用RetinaFace检测人脸并提取ArcFace特征
            faces = self.app.get(image)
            
            embeddings = []
            valid_faces = []
            
            for face in faces:
                # face.embedding 是512维的ArcFace特征向量
                if face.embedding is not None:
                    embeddings.append(face.embedding)
                    valid_faces.append(face)
            
            return valid_faces, embeddings
        except Exception as e:
            print(f"提取人脸特征失败: {e}")
            return [], []
    
    def add_person(self, person_id, name, role, face_images):
        """
        添加新人物到数据库
        
        Args:
            person_id: 人物ID（唯一标识）
            name: 姓名
            role: 身份/角色
            face_images: 人脸图像列表（可以是文件路径或numpy数组）
        """
        if self.app is None:
            print("错误: InsightFace模型未加载")
            return False
        
        embeddings = []
        face_dir = os.path.join(self.database_dir, 'faces', person_id)
        os.makedirs(face_dir, exist_ok=True)
        
        for idx, face_img in enumerate(face_images):
            # 如果是文件路径，读取图像
            if isinstance(face_img, str):
                if not os.path.exists(face_img):
                    print(f"警告: 图像文件不存在: {face_img}")
                    continue
                img = cv2.imread(face_img)
            else:
                img = face_img
            
            if img is None:
                continue
            
            # 提取人脸特征
            faces, face_embeddings = self.extract_face_embedding(img)
            
            if len(face_embeddings) == 0:
                print(f"警告: 在图像中未检测到人脸: {face_img if isinstance(face_img, str) else 'numpy array'}")
                continue
            
            # 使用第一个检测到的人脸（通常是最清晰的）
            embedding = face_embeddings[0]
            embeddings.append(embedding.tolist())
            
            # 保存人脸图像（用于可视化）
            face = faces[0]
            bbox = face.bbox.astype(int)
            face_roi = img[bbox[1]:bbox[3], bbox[0]:bbox[2]]
            
            face_file = os.path.join(face_dir, f"face_{idx}.jpg")
            cv2.imwrite(face_file, face_roi)
        
        if len(embeddings) == 0:
            print(f"错误: 未能从提供的图像中提取任何人脸特征: {person_id}")
            return False
        
        # 保存到数据库
        self.known_faces[person_id] = {
            'name': name,
            'role': role,
            'embeddings': embeddings
        }
        
        # 保存到文件
        self.save_database()
        
        try:
            import rospy
            rospy.loginfo(f"✓ 成功添加人物: {name} ({role}), ID: {person_id}, 人脸数: {len(embeddings)}")
        except:
            print(f"✓ 成功添加人物: {name} ({role}), ID: {person_id}, 人脸数: {len(embeddings)}")
        return True
    
    def recognize_face(self, image, threshold=None):
        """
        识别人脸（使用ArcFace特征比对）
        
        Args:
            image: 图像（BGR格式）
            threshold: 识别阈值（余弦相似度，越大越严格，范围0-1），如果为None则使用初始化时的阈值
            
        Returns:
            (person_id, name, role, confidence) 或 (None, None, None, 0)
        """
        if threshold is None:
            threshold = self.recognition_threshold
        
        if self.app is None or len(self.known_faces) == 0:
            return (None, None, None, 0.0)
        
        # 提取人脸特征
        faces, embeddings = self.extract_face_embedding(image)
        
        if len(embeddings) == 0:
            return (None, None, None, 0.0)
        
        # 使用第一个检测到的人脸
        query_embedding = np.array(embeddings[0])
        query_embedding = query_embedding / np.linalg.norm(query_embedding)  # 归一化
        
        best_match = None
        best_similarity = -1.0
        
        # 与所有已知人脸比对（使用余弦相似度）
        for person_id, person_data in self.known_faces.items():
            for embedding in person_data['embeddings']:
                embedding = np.array(embedding)
                embedding = embedding / np.linalg.norm(embedding)  # 归一化
                
                # 计算余弦相似度
                similarity = np.dot(query_embedding, embedding)
                
                if similarity > best_similarity:
                    best_similarity = similarity
                    best_match = {
                        'person_id': person_id,
                        'name': person_data['name'],
                        'role': person_data['role'],
                        'similarity': similarity
                    }
        
        # 判断是否匹配（余弦相似度越大越相似）
        if best_match and best_similarity >= threshold:
            return (best_match['person_id'], best_match['name'], best_match['role'], best_similarity)
        else:
            # 即使未匹配，也返回最高相似度值，用于调试显示
            if best_match:
                return (None, None, None, best_similarity)
            else:
                return (None, None, None, 0.0)
    
    def detect_and_recognize(self, image):
        """
        检测并识别人脸（返回所有检测到的人脸及其识别结果）
        
        Args:
            image: 图像（BGR格式）
            
        Returns:
            list of (bbox, person_id, name, role, confidence)
        """
        if self.app is None:
            return []
        
        results = []
        faces, embeddings = self.extract_face_embedding(image)
        
        for i, (face, embedding) in enumerate(zip(faces, embeddings)):
            bbox = face.bbox.astype(int)
            
            # 直接使用已提取的特征向量进行识别，避免重复检测
            # 这样可以避免从裁剪区域重新检测时可能失败的问题
            person_id, name, role, confidence = self.recognize_face_from_embedding(
                embedding, threshold=self.recognition_threshold
            )
            
            results.append({
                'bbox': bbox,
                'person_id': person_id,
                'name': name,
                'role': role,
                'confidence': confidence,
                'detection_confidence': face.det_score
            })
        
        return results
    
    def recognize_face_from_embedding(self, embedding, threshold=None):
        """
        从特征向量识别人脸（避免重复提取特征）
        
        Args:
            embedding: 人脸特征向量（512维）
            threshold: 识别阈值（余弦相似度，越大越严格，范围0-1），如果为None则使用初始化时的阈值
            
        Returns:
            (person_id, name, role, confidence) 或 (None, None, None, 0)
        """
        if threshold is None:
            threshold = self.recognition_threshold
        
        if self.app is None or len(self.known_faces) == 0:
            return (None, None, None, 0.0)
        
        # 归一化查询特征向量
        query_embedding = np.array(embedding)
        query_embedding = query_embedding / np.linalg.norm(query_embedding)
        
        best_match = None
        best_similarity = -1.0
        
        # 与所有已知人脸比对（使用余弦相似度）
        for person_id, person_data in self.known_faces.items():
            for db_embedding in person_data['embeddings']:
                db_emb = np.array(db_embedding)
                db_emb = db_emb / np.linalg.norm(db_emb)  # 归一化
                
                # 计算余弦相似度
                similarity = np.dot(query_embedding, db_emb)
                
                if similarity > best_similarity:
                    best_similarity = similarity
                    best_match = {
                        'person_id': person_id,
                        'name': person_data['name'],
                        'role': person_data['role'],
                        'similarity': similarity
                    }
        
        # 判断是否匹配（余弦相似度越大越相似）
        if best_match and best_similarity >= threshold:
            return (best_match['person_id'], best_match['name'], best_match['role'], best_similarity)
        else:
            # 即使未匹配，也返回最高相似度值，用于调试显示
            if best_match:
                return (None, None, None, best_similarity)
            else:
                return (None, None, None, 0.0)
    
    def save_database(self):
        """保存数据库到文件"""
        db_file = os.path.join(self.database_dir, 'database.json')
        with open(db_file, 'w', encoding='utf-8') as f:
            json.dump(self.known_faces, f, ensure_ascii=False, indent=2)
    
    def load_database(self):
        """从文件加载数据库"""
        db_file = os.path.join(self.database_dir, 'database.json')
        if os.path.exists(db_file):
            try:
                with open(db_file, 'r', encoding='utf-8') as f:
                    self.known_faces = json.load(f)
                print(f"✓ 加载了 {len(self.known_faces)} 个已知人物")
            except Exception as e:
                print(f"警告: 加载数据库失败: {e}")
                self.known_faces = {}
        else:
            self.known_faces = {}
    
    def get_greeting(self):
        """根据当前时间获取问候语"""
        hour = datetime.now().hour
        
        if 5 <= hour < 11:
            return "早上好"
        elif 11 <= hour < 13:
            return "中午好"
        elif 13 <= hour < 18:
            return "下午好"
        elif 18 <= hour < 22:
            return "晚上好"
        else:
            return "晚上好"  # 深夜也用晚上好
