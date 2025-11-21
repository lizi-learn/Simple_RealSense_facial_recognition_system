#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import sys
import os

# 添加脚本所在目录到路径（必须在导入face_recognition_manager之前）
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, script_dir)

# 添加pyrealsense2路径（必须在导入之前）
sys.path.insert(0, '/usr/lib/python3.8/site-packages')

import numpy as np
import cv2
import subprocess
import time
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import pyrealsense2 as rs
import rospkg
from face_recognition_manager import FaceRecognitionManager
from PIL import Image as PILImage, ImageDraw, ImageFont

class RealSenseCameraNode:
    def __init__(self):
        rospy.init_node('realsense_camera_node', anonymous=True)
        
        self.bridge = CvBridge()
        
        # 创建发布者
        self.color_pub = rospy.Publisher('/camera/color/image_raw', Image, queue_size=1)
        self.depth_pub = rospy.Publisher('/camera/depth/image_raw', Image, queue_size=1)
        self.color_info_pub = rospy.Publisher('/camera/color/camera_info', CameraInfo, queue_size=1)
        self.depth_info_pub = rospy.Publisher('/camera/depth/camera_info', CameraInfo, queue_size=1)
        self.face_detection_pub = rospy.Publisher('/camera/face_detection/image_raw', Image, queue_size=1)
        
        # 从参数服务器获取配置
        self.width = rospy.get_param('~width', 640)
        self.height = rospy.get_param('~height', 480)
        self.fps = rospy.get_param('~fps', 30)  # 提高帧率以减少延迟
        self.face_confidence_threshold = rospy.get_param('~face_confidence_threshold', 0.5)
        self.face_detection_skip = rospy.get_param('~face_detection_skip', 1)  # 每帧都检测，减少延迟
        
        # 加载人脸检测模型（用于快速检测）
        self.load_face_detector()
        
        # 加载人脸识别管理器（RetinaFace + ArcFace）
        self.enable_face_recognition = rospy.get_param('~enable_face_recognition', True)
        if self.enable_face_recognition:
            try:
                database_dir = rospy.get_param('~face_database_dir', 
                                              os.path.join(rospkg.RosPack().get_path('realsense_camera'), '..', '..', 'data', 'face_database'))
                database_dir = os.path.abspath(database_dir)
                ctx_id = rospy.get_param('~gpu_id', -1)  # -1表示CPU，0+表示GPU
                recognition_threshold = rospy.get_param('~recognition_threshold', 0.5)  # 降低默认阈值以提高识别率
                self.face_recognition_manager = FaceRecognitionManager(
                    database_dir, 
                    ctx_id=ctx_id,
                    recognition_threshold=recognition_threshold
                )
                if self.face_recognition_manager.app is not None:
                    rospy.loginfo("✓ 人脸识别功能已启用（RetinaFace + ArcFace）")
                else:
                    rospy.logwarn("人脸识别模型加载失败，功能已禁用")
                    self.enable_face_recognition = False
            except Exception as e:
                rospy.logwarn(f"初始化人脸识别失败: {e}")
                self.enable_face_recognition = False
        else:
            self.face_recognition_manager = None
        
        # 人脸识别相关
        self.last_recognized_person = None
        self.last_recognition_time = 0
        self.recognition_cooldown = rospy.get_param('~recognition_cooldown', 10.0)  # 10秒内不重复识别同一人
        self.recognition_threshold = rospy.get_param('~recognition_threshold', 0.6)  # 识别阈值（余弦相似度）
        
        # 防止重复播报的锁和标志
        import threading
        self.speaking_lock = threading.Lock()  # 播报锁
        self.is_speaking = False  # 是否正在播报
        self.last_spoken_text = None  # 上次播报的文本
        self.last_spoken_time = 0  # 上次播报的时间
        
        # 检查设备是否存在
        self.check_device()
        
        # 初始化RealSense管道
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        
        # 配置流
        self.config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)
        self.config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
        
        # 启动管道
        try:
            profile = self.pipeline.start(self.config)
            rospy.loginfo("RealSense相机启动成功")
            
            # 获取相机内参
            depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
            color_profile = rs.video_stream_profile(profile.get_stream(rs.stream.color))
            
            depth_intrinsics = depth_profile.get_intrinsics()
            color_intrinsics = color_profile.get_intrinsics()
            
            # 创建相机信息消息
            self.depth_camera_info = self.create_camera_info(depth_intrinsics, self.width, self.height)
            self.color_camera_info = self.create_camera_info(color_intrinsics, self.width, self.height)
            
        except Exception as e:
            rospy.logerr(f"无法启动RealSense相机: {e}")
            sys.exit(1)
        
        # 创建对齐对象（将深度图对齐到彩色图）
        self.align_to = rs.align(rs.stream.color)
        
        # 帧超时计数器
        self.frame_timeout_count = 0
        self.max_timeout_count = 10  # 连续超时10次后警告
        
        # 性能优化：帧计数器（用于跳帧检测）
        self.frame_counter = 0
        
        # 语音播报相关
        self.enable_voice_announcement = rospy.get_param('~enable_voice_announcement', True)
        self.voice_announcement_interval = rospy.get_param('~voice_announcement_interval', 5.0)  # 5秒
        self.last_announcement_time = 0
        self.audio_sink = rospy.get_param('~audio_sink', 'alsa_output.pci-0000_00_1f.3.analog-stereo')
        self.tts_voice = rospy.get_param('~tts_voice', 'zh-CN-XiaoxiaoNeural')  # 微软TTS中文语音
        
        # 检查edge-tts是否可用
        if self.enable_voice_announcement:
            try:
                # edge-tts没有--version，使用--help来检查
                result = subprocess.run(['edge-tts', '--help'], 
                                      capture_output=True, check=True, timeout=2)
                rospy.loginfo("微软TTS语音播报功能已启用")
            except (subprocess.CalledProcessError, FileNotFoundError, subprocess.TimeoutExpired):
                rospy.logwarn("edge-tts未安装，语音播报功能已禁用")
                rospy.logwarn("请运行: pip3 install edge-tts --user")
                self.enable_voice_announcement = False
        
        rospy.on_shutdown(self.shutdown)
    
    def check_device(self):
        """检查RealSense设备是否存在"""
        ctx = rs.context()
        devices = ctx.query_devices()
        
        if len(devices) == 0:
            rospy.logerr("=" * 60)
            rospy.logerr("错误: 未检测到RealSense设备！")
            rospy.logerr("=" * 60)
            rospy.logerr("请检查:")
            rospy.logerr("  1. RealSense相机是否已连接到USB 3.0端口")
            rospy.logerr("  2. USB连接是否稳定")
            rospy.logerr("  3. 运行 'rs-enumerate-devices' 检查设备")
            rospy.logerr("  4. 检查设备权限: lsusb | grep Intel")
            rospy.logerr("=" * 60)
            sys.exit(1)
        
        rospy.loginfo(f"检测到 {len(devices)} 个RealSense设备:")
        for dev in devices:
            name = dev.get_info(rs.camera_info.name)
            serial = dev.get_info(rs.camera_info.serial_number)
            
            # 检查USB类型
            try:
                usb_type = dev.get_info(rs.camera_info.usb_type_descriptor)
                if usb_type and "2." in str(usb_type):
                    rospy.logwarn("=" * 60)
                    rospy.logwarn("⚠ 性能警告: 检测到USB 2.x连接！")
                    rospy.logwarn("=" * 60)
                    rospy.logwarn("RealSense D435I需要USB 3.0以获得最佳性能")
                    rospy.logwarn("当前连接: USB " + str(usb_type))
                    rospy.logwarn("")
                    rospy.logwarn("建议:")
                    rospy.logwarn("  1. 将相机连接到USB 3.0端口（通常是蓝色的）")
                    rospy.logwarn("  2. 检查USB线缆是否支持USB 3.0")
                    rospy.logwarn("  3. 已自动降低帧率到15fps以提升性能")
                    rospy.logwarn("=" * 60)
            except:
                pass
            
            rospy.loginfo(f"  - {name} (序列号: {serial})")
    
    def load_face_detector(self):
        """加载OpenCV DNN人脸检测模型"""
        # 获取包路径
        rospack = rospkg.RosPack()
        try:
            package_path = rospack.get_path('realsense_camera')
        except rospkg.ResourceNotFound:
            # 如果找不到包，使用脚本相对路径
            package_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..')
        
        models_dir = os.path.join(package_path, 'models')
        
        prototxt_path = os.path.join(models_dir, 'deploy.prototxt')
        model_path = os.path.join(models_dir, 'res10_300x300_ssd_iter_140000.caffemodel')
        
        if not os.path.exists(prototxt_path):
            rospy.logerr(f"找不到prototxt文件: {prototxt_path}")
            self.face_net = None
            return
        
        if not os.path.exists(model_path):
            rospy.logerr(f"找不到模型文件: {model_path}")
            self.face_net = None
            return
        
        try:
            self.face_net = cv2.dnn.readNetFromCaffe(prototxt_path, model_path)
            rospy.loginfo("人脸检测模型加载成功")
        except Exception as e:
            rospy.logerr(f"加载人脸检测模型失败: {e}")
            self.face_net = None
    
    def detect_faces(self, image):
        """检测图像中的人脸（优化版本，减少延迟）"""
        if self.face_net is None:
            return []
        
        (h, w) = image.shape[:2]
        
        # 优化：如果图像较大，先缩小再检测，然后按比例放大结果
        # 这样可以加快检测速度
        scale_factor = 1.0
        if w > 640 or h > 480:
            # 如果图像较大，缩小到640宽度
            scale_factor = 640.0 / w
            new_w = 640
            new_h = int(h * scale_factor)
            resized_image = cv2.resize(image, (new_w, new_h))
            detect_h, detect_w = resized_image.shape[:2]
        else:
            resized_image = image
            detect_h, detect_w = h, w
        
        # 创建blob（300x300是模型输入尺寸）
        blob = cv2.dnn.blobFromImage(cv2.resize(resized_image, (300, 300)), 1.0,
                                    (300, 300), (104.0, 177.0, 123.0))
        
        # 通过网络进行前向传播
        self.face_net.setInput(blob)
        detections = self.face_net.forward()
        
        faces = []
        # 遍历检测结果
        for i in range(0, detections.shape[2]):
            confidence = detections[0, 0, i, 2]
            
            # 过滤低置信度的检测
            if confidence > self.face_confidence_threshold:
                # 计算边界框坐标（基于检测图像尺寸）
                box = detections[0, 0, i, 3:7] * np.array([detect_w, detect_h, detect_w, detect_h])
                (startX, startY, endX, endY) = box.astype("int")
                
                # 如果进行了缩放，需要将坐标转换回原始图像尺寸
                if scale_factor != 1.0:
                    startX = int(startX / scale_factor)
                    startY = int(startY / scale_factor)
                    endX = int(endX / scale_factor)
                    endY = int(endY / scale_factor)
                
                # 确保坐标在图像范围内
                startX = max(0, startX)
                startY = max(0, startY)
                endX = min(w, endX)
                endY = min(h, endY)
                
                faces.append({
                    'bbox': (startX, startY, endX, endY),
                    'confidence': confidence
                })
        
        return faces
    
    def draw_chinese_text(self, image, text, position, font_size=20, color=(255, 255, 255)):
        """在图像上绘制中文文本（使用PIL）"""
        try:
            # 转换为PIL图像
            pil_image = PILImage.fromarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
            draw = ImageDraw.Draw(pil_image)
            
            # 尝试加载中文字体
            font_paths = [
                '/usr/share/fonts/truetype/wqy/wqy-microhei.ttc',  # 文泉驿微米黑
                '/usr/share/fonts/truetype/wqy/wqy-zenhei.ttc',     # 文泉驿正黑
                '/usr/share/fonts/truetype/arphic/uming.ttc',       # AR PL UMing
                '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', # 备用字体
            ]
            
            font = None
            for font_path in font_paths:
                if os.path.exists(font_path):
                    try:
                        font = ImageFont.truetype(font_path, font_size)
                        break
                    except:
                        continue
            
            if font is None:
                # 使用默认字体
                font = ImageFont.load_default()
            
            # 绘制文本
            x, y = position
            draw.text((x, y), text, font=font, fill=color)
            
            # 转换回OpenCV格式
            result_image = cv2.cvtColor(np.array(pil_image), cv2.COLOR_RGB2BGR)
            return result_image
        except Exception as e:
            rospy.logwarn(f"绘制中文文本失败，使用英文: {e}")
            # 如果失败，使用OpenCV的putText（只显示ASCII字符）
            cv2.putText(image, text.encode('ascii', 'ignore').decode('ascii'), position,
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
            return image
    
    def draw_faces(self, image, faces, depth_image=None):
        """在图像上绘制检测到的人脸框，并使用RetinaFace+ArcFace进行识别"""
        result_image = image.copy()
        current_time = time.time()
        
        # 如果启用了人脸识别，使用RetinaFace+ArcFace进行检测和识别
        if self.enable_face_recognition and self.face_recognition_manager and self.face_recognition_manager.app is not None:
            # 使用RetinaFace检测并识别（更准确）
            recognition_results = self.face_recognition_manager.detect_and_recognize(image)
            
            # 绘制识别结果
            for result in recognition_results:
                bbox = result['bbox']
                startX, startY, endX, endY = bbox[0], bbox[1], bbox[2], bbox[3]
                
                person_id = result['person_id']
                name = result['name']
                role = result['role']
                confidence = result['confidence']
                det_confidence = result['detection_confidence']
                
                # 检查是否需要播报（避免重复播报）
                if person_id is not None:
                    # 判断是否需要播报：
                    # 1. 不同的人，或者
                    # 2. 同一个人但超过冷却时间
                    should_speak = False
                    if person_id != self.last_recognized_person:
                        # 不同的人，可以播报
                        should_speak = True
                    elif (current_time - self.last_recognition_time) >= self.recognition_cooldown:
                        # 同一个人，但超过冷却时间，可以播报
                        should_speak = True
                    
                    if should_speak:
                        # 获取问候语
                        greeting = self.face_recognition_manager.get_greeting()
                        
                        # 构建播报文本
                        if role:
                            text = f"{greeting}，{name}，{role}"
                        else:
                            text = f"{greeting}，{name}"
                        
                        # 语音播报（带锁，防止重复）
                        rospy.loginfo(f"识别成功: {name} ({role}), 相似度: {confidence:.4f}, 准备播报")
                        if self.speak_greeting(text):
                            # 只有成功播报才更新记录
                            self.last_recognized_person = person_id
                            self.last_recognition_time = current_time
                
                # 选择颜色：识别成功用蓝色，未识别用绿色
                if person_id is not None:
                    color = (255, 0, 0)  # 蓝色（BGR）
                    text_color = (255, 255, 255)  # 白色文字
                    label = f"{name}"
                    if role:
                        label += f" ({role})"
                    label += f" {confidence:.2f}"
                else:
                    color = (0, 255, 0)  # 绿色
                    text_color = (255, 255, 255)  # 白色文字
                    # 显示相似度值以便调试（即使未识别也显示最高相似度）
                    if confidence > 0:
                        label = f"未知 (相似度:{confidence:.2f}) {det_confidence:.2f}"
                    else:
                        label = f"未知 {det_confidence:.2f}"
                
                # 绘制边界框
                cv2.rectangle(result_image, (startX, startY), (endX, endY), color, 2)
                
                # 计算深度（如果提供了深度图）
                if depth_image is not None:
                    center_x = (startX + endX) // 2
                    center_y = (startY + endY) // 2
                    if 0 <= center_x < depth_image.shape[1] and 0 <= center_y < depth_image.shape[0]:
                        depth_value = depth_image[center_y, center_x]
                        if depth_value > 0:
                            depth_text = f" {depth_value/1000:.2f}m"
                            label += depth_text
                
                # 绘制标签文本（使用中文绘制函数）
                y = startY - 10 if startY - 10 > 10 else startY + 10
                result_image = self.draw_chinese_text(result_image, label, (startX, y), 
                                                     font_size=16, color=text_color)
        else:
            # 使用原始的人脸检测结果（不进行识别）
            for face in faces:
                (startX, startY, endX, endY) = face['bbox']
                confidence = face['confidence']
                
                # 绘制边界框
                color = (0, 255, 0)  # 绿色
                label = f"{confidence * 100:.1f}%"
                
                cv2.rectangle(result_image, (startX, startY), (endX, endY), color, 2)
                
                # 计算深度
                if depth_image is not None:
                    center_x = (startX + endX) // 2
                    center_y = (startY + endY) // 2
                    if 0 <= center_x < depth_image.shape[1] and 0 <= center_y < depth_image.shape[0]:
                        depth_value = depth_image[center_y, center_x]
                        if depth_value > 0:
                            depth_text = f" {depth_value/1000:.2f}m"
                            label += depth_text
                
                # 绘制标签
                y = startY - 10 if startY - 10 > 10 else startY + 10
                cv2.putText(result_image, label, (startX, y),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        return result_image
    
    def speak_greeting(self, text):
        """使用微软TTS播报问候语
        
        Returns:
            bool: 是否成功启动播报
        """
        if not self.enable_voice_announcement:
            rospy.logdebug("语音播报已禁用")
            return False
        
        # 使用锁防止并发播报
        with self.speaking_lock:
            current_time = time.time()
            
            # 检查是否正在播报
            if self.is_speaking:
                rospy.logdebug(f"正在播报中，跳过: {text}")
                return False
            
            # 检查是否与上次播报相同且时间间隔太短（防止同一帧多次触发）
            if (text == self.last_spoken_text and 
                (current_time - self.last_spoken_time) < 2.0):  # 2秒内相同文本不重复
                rospy.logdebug(f"与上次播报相同且间隔太短，跳过: {text}")
                return False
            
            # 设置播报标志
            self.is_speaking = True
            self.last_spoken_text = text
            self.last_spoken_time = current_time
        
        rospy.loginfo(f"准备播报: {text}")
        
        # 使用pactl设置音频输出设备，然后使用edge-tts播报
        try:
            # 设置音频输出设备
            subprocess.run(['pactl', 'set-default-sink', self.audio_sink], 
                         capture_output=True, check=False)
            rospy.logdebug(f"音频输出设备已设置为: {self.audio_sink}")
            
            # 使用edge-tts生成音频并通过ffplay播放
            import tempfile
            import os
            import threading
            
            # 创建临时文件保存音频
            with tempfile.NamedTemporaryFile(suffix='.mp3', delete=False) as tmp_file:
                tmp_audio_file = tmp_file.name
            
            try:
                # 生成音频文件
                cmd_generate = [
                    'edge-tts',
                    '--voice', self.tts_voice,
                    '--text', text,
                    '--write-media', tmp_audio_file
                ]
                
                # 如果设置了代理，添加到命令中
                proxy = os.environ.get('HTTP_PROXY') or os.environ.get('HTTPS_PROXY')
                if proxy:
                    # 移除末尾的斜杠（如果有）
                    proxy = proxy.rstrip('/')
                    cmd_generate.extend(['--proxy', proxy])
                    rospy.logdebug(f"使用代理: {proxy}")
                
                # 传递环境变量
                env = os.environ.copy()
                result = subprocess.run(cmd_generate, capture_output=True, check=True, timeout=10, env=env)
                
                # 检查文件是否生成成功
                if not os.path.exists(tmp_audio_file) or os.path.getsize(tmp_audio_file) == 0:
                    rospy.logwarn(f"音频文件生成失败或为空: {tmp_audio_file}")
                    with self.speaking_lock:
                        self.is_speaking = False
                    if os.path.exists(tmp_audio_file):
                        os.unlink(tmp_audio_file)
                    return False
                
                # 使用ffplay播放音频（静默模式）
                play_cmd = ['ffplay', '-nodisp', '-autoexit', '-loglevel', 'quiet', tmp_audio_file]
                process = subprocess.Popen(play_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                rospy.loginfo(f"语音播报: {text}")
                
                # 延迟删除临时文件和重置播报标志（给播放器时间读取）
                def cleanup():
                    try:
                        # 等待播放完成（最多5秒）
                        process.wait(timeout=5)
                    except subprocess.TimeoutExpired:
                        process.kill()
                    
                    # 重置播报标志
                    with self.speaking_lock:
                        self.is_speaking = False
                    
                    # 删除临时文件
                    time.sleep(1)  # 再等1秒确保文件不再被使用
                    try:
                        if os.path.exists(tmp_audio_file):
                            os.unlink(tmp_audio_file)
                    except:
                        pass
                
                threading.Thread(target=cleanup, daemon=True).start()
                return True  # 成功启动播报
                
            except subprocess.TimeoutExpired:
                rospy.logwarn("生成音频超时")
                with self.speaking_lock:
                    self.is_speaking = False
                try:
                    if os.path.exists(tmp_audio_file):
                        os.unlink(tmp_audio_file)
                except:
                    pass
                return False
            except Exception as e:
                rospy.logwarn(f"播放音频失败: {e}")
                with self.speaking_lock:
                    self.is_speaking = False
                try:
                    if os.path.exists(tmp_audio_file):
                        os.unlink(tmp_audio_file)
                except:
                    pass
                return False
                
        except Exception as e:
            rospy.logwarn(f"语音播报失败: {e}")
            with self.speaking_lock:
                self.is_speaking = False
            return False
    
    def speak_distance(self, distance_meters, face_count):
        """使用微软TTS（edge-tts）播报人脸距离（中文）"""
        if not self.enable_voice_announcement:
            return
        
        # 格式化距离文本（纯中文，不需要转换）
        if distance_meters < 1.0:
            distance_cm = int(distance_meters * 100)
            if face_count == 1:
                text = f"检测到人脸，距离{distance_cm}厘米"
            else:
                text = f"检测到{face_count}个人脸，最近距离{distance_cm}厘米"
        else:
            # 将小数转换为中文表达（例如1.5米）
            distance_int = int(distance_meters)
            distance_decimal = int((distance_meters - distance_int) * 10)
            if distance_decimal == 0:
                distance_text = f"{distance_int}米"
            else:
                distance_text = f"{distance_int}点{distance_decimal}米"
            
            if face_count == 1:
                text = f"检测到人脸，距离{distance_text}"
            else:
                text = f"检测到{face_count}个人脸，最近距离{distance_text}"
        
        # 使用pactl设置音频输出设备，然后使用edge-tts播报
        try:
            # 设置音频输出设备
            subprocess.run(['pactl', 'set-default-sink', self.audio_sink], 
                         capture_output=True, check=False)
            
            # 使用edge-tts生成音频并通过ffplay播放
            import tempfile
            import os
            import threading
            
            # 创建临时文件保存音频
            with tempfile.NamedTemporaryFile(suffix='.mp3', delete=False) as tmp_file:
                tmp_audio_file = tmp_file.name
            
            try:
                # 生成音频文件
                cmd_generate = [
                    'edge-tts',
                    '--voice', self.tts_voice,
                    '--text', text,
                    '--write-media', tmp_audio_file
                ]
                subprocess.run(cmd_generate, capture_output=True, check=True, timeout=10)
                
                # 使用ffplay播放音频（静默模式）
                play_cmd = ['ffplay', '-nodisp', '-autoexit', '-loglevel', 'quiet', tmp_audio_file]
                subprocess.Popen(play_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                rospy.loginfo(f"语音播报（中文）: {text}")
                
                # 延迟删除临时文件（给播放器时间读取）
                def cleanup():
                    time.sleep(5)
                    try:
                        if os.path.exists(tmp_audio_file):
                            os.unlink(tmp_audio_file)
                    except:
                        pass
                threading.Thread(target=cleanup, daemon=True).start()
                
            except subprocess.TimeoutExpired:
                rospy.logwarn("生成音频超时")
                try:
                    if os.path.exists(tmp_audio_file):
                        os.unlink(tmp_audio_file)
                except:
                    pass
            except Exception as e:
                rospy.logwarn(f"播放音频失败: {e}")
                try:
                    if os.path.exists(tmp_audio_file):
                        os.unlink(tmp_audio_file)
                except:
                    pass
                
        except Exception as e:
            rospy.logwarn(f"语音播报失败: {e}")
        
    def create_camera_info(self, intrinsics, width, height):
        """创建CameraInfo消息"""
        camera_info = CameraInfo()
        camera_info.width = width
        camera_info.height = height
        camera_info.distortion_model = "plumb_bob"
        
        # 内参矩阵 (3x3)
        camera_info.K = [
            intrinsics.fx, 0.0, intrinsics.ppx,
            0.0, intrinsics.fy, intrinsics.ppy,
            0.0, 0.0, 1.0
        ]
        
        # 旋转矩阵 (3x3) - 单位矩阵
        camera_info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        
        # 投影矩阵 (3x4)
        camera_info.P = [
            intrinsics.fx, 0.0, intrinsics.ppx, 0.0,
            0.0, intrinsics.fy, intrinsics.ppy, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]
        
        # 畸变系数
        camera_info.D = list(intrinsics.coeffs)
        
        return camera_info
    
    def run(self):
        rate = rospy.Rate(self.fps)
        
        while not rospy.is_shutdown():
            try:
                # 等待帧（减少超时时间以更快响应）
                frames = self.pipeline.wait_for_frames(timeout_ms=500)
                
                # 重置超时计数器
                self.frame_timeout_count = 0
                
                # 对齐深度帧到彩色帧
                aligned_frames = self.align_to.process(frames)
                
                depth_frame = aligned_frames.get_depth_frame()
                color_frame = aligned_frames.get_color_frame()
                
                if not depth_frame or not color_frame:
                    continue
                
                # 转换为numpy数组
                depth_image = np.asanyarray(depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())
                
                # 每帧都进行人脸检测（减少延迟）
                # 注意：如果启用了RetinaFace识别，draw_faces会重新检测，这里主要用于兼容
                faces = self.detect_faces(color_image)
                
                # 保存检测结果
                self.last_faces = faces
                
                # 语音播报（每5秒一次）
                # 注意：如果启用了人脸识别，识别成功时会自动播报问候语，这里不再播报"检测到人脸"
                current_time = time.time()
                if (not self.enable_face_recognition and len(faces) > 0 and 
                    (current_time - self.last_announcement_time) >= self.voice_announcement_interval):
                    # 计算最近的人脸距离
                    min_distance = float('inf')
                    for face in faces:
                        (startX, startY, endX, endY) = face['bbox']
                        center_x = (startX + endX) // 2
                        center_y = (startY + endY) // 2
                        if 0 <= center_x < depth_image.shape[1] and 0 <= center_y < depth_image.shape[0]:
                            depth_value = depth_image[center_y, center_x]
                            if depth_value > 0:
                                distance_m = depth_value / 1000.0
                                if distance_m < min_distance:
                                    min_distance = distance_m
                    
                    # 如果有有效距离，进行播报
                    if min_distance != float('inf'):
                        self.speak_distance(min_distance, len(faces))
                        self.last_announcement_time = current_time
                
                # 在图像上绘制人脸框
                face_detection_image = self.draw_faces(color_image, faces, depth_image)
                
                # 转换为ROS消息
                color_msg = self.bridge.cv2_to_imgmsg(color_image, "bgr8")
                color_msg.header.stamp = rospy.Time.now()
                color_msg.header.frame_id = "camera_color_optical_frame"
                
                depth_msg = self.bridge.cv2_to_imgmsg(depth_image, "16UC1")
                depth_msg.header.stamp = rospy.Time.now()
                depth_msg.header.frame_id = "camera_depth_optical_frame"
                
                face_detection_msg = self.bridge.cv2_to_imgmsg(face_detection_image, "bgr8")
                face_detection_msg.header.stamp = rospy.Time.now()
                face_detection_msg.header.frame_id = "camera_color_optical_frame"
                
                # 设置相机信息时间戳
                self.color_camera_info.header.stamp = color_msg.header.stamp
                self.color_camera_info.header.frame_id = color_msg.header.frame_id
                
                self.depth_camera_info.header.stamp = depth_msg.header.stamp
                self.depth_camera_info.header.frame_id = depth_msg.header.frame_id
                
                # 发布消息
                self.color_pub.publish(color_msg)
                self.depth_pub.publish(depth_msg)
                self.color_info_pub.publish(self.color_camera_info)
                self.depth_info_pub.publish(self.depth_camera_info)
                self.face_detection_pub.publish(face_detection_msg)
                
                # 打印检测到的人脸数量（可选，避免日志过多）
                if len(faces) > 0:
                    rospy.loginfo(f"检测到 {len(faces)} 个人脸")
                
            except RuntimeError as e:
                error_msg = str(e)
                if "Frame didn't arrive" in error_msg:
                    self.frame_timeout_count += 1
                    if self.frame_timeout_count >= self.max_timeout_count:
                        rospy.logwarn("=" * 60)
                        rospy.logwarn("警告: 连续多次无法获取帧数据")
                        rospy.logwarn("=" * 60)
                        rospy.logwarn("可能的原因:")
                        rospy.logwarn("  1. 相机连接不稳定，请检查USB连接")
                        rospy.logwarn("  2. USB端口不是USB 3.0")
                        rospy.logwarn("  3. 相机被其他程序占用")
                        rospy.logwarn("  4. 相机固件需要更新")
                        rospy.logwarn("=" * 60)
                        rospy.logwarn("尝试重新连接...")
                        self.frame_timeout_count = 0
                        # 尝试重新启动管道
                        try:
                            self.pipeline.stop()
                            rospy.sleep(1)
                            self.pipeline.start(self.config)
                            rospy.loginfo("相机重新连接成功")
                        except Exception as restart_error:
                            rospy.logerr(f"重新连接失败: {restart_error}")
                else:
                    rospy.logerr(f"处理帧时出错: {e}")
                continue
            except Exception as e:
                rospy.logerr(f"处理帧时出错: {e}")
                continue
            
            rate.sleep()
    
    def shutdown(self):
        rospy.loginfo("正在关闭RealSense相机...")
        self.pipeline.stop()
        rospy.loginfo("RealSense相机已关闭")

if __name__ == '__main__':
    try:
        node = RealSenseCameraNode()
        node.run()
    except rospy.ROSInterruptException:
        pass

