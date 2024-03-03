import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

import torch
#from PIL import Image
from tqdm import tqdm 
import numpy as np
import os
import glob 
from zoedepth.models.builder import build_model
from zoedepth.utils.config import get_config
from zoedepth.utils.misc import pil_to_batched_tensor
from zoedepth.utils.misc import save_raw_16bit
from zoedepth.utils.misc import to_raw_16bit

def image_callback(msg):
    try:
        # 使用cv_bridge将ROS图像消息转换为OpenCV图像格式
        bridge = CvBridge()
        cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        cv_image = cv2.resize(cv_image, (640, 480))

        depth = zoe.infer_pil(cv_image)
        depth16bit = depth
        to_raw_16bit(depth, depth16bit)

        # 使用cv_bridge将深度图像转换为ROS图像消息
        depth_ros = bridge.cv2_to_imgmsg(depth16bit, encoding="passthrough")
        
        # 使用cv_bridge将深度图像对应的rgb图转换为ROS图像消息
        color_ros = bridge.cv2_to_imgmsg(cv_image, encoding="passthrough")

        # 发布黑白图像到/usb_cam/image_depth话题
        depth_image_pub.publish(depth_ros)
        # 发布黑白图对应的彩色图到/usb_cam/image_color话题
        color_image_pub.publish(color_ros)
        
        # 将彩色图像转换为灰度图像
        #gray_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # 使用cv_bridge将灰度图像转换为ROS图像消息
        #depth_image_msg = bridge.cv2_to_imgmsg(gray_image, encoding="mono8")

        # 发布黑白图像到/usb_cam/image_depth话题
        #depth_image_pub.publish(depth_image_msg)
    except Exception as e:
        rospy.logerr(e)

if __name__ == '__main__':
    # 初始化ROS节点
    rospy.init_node('image_processing_node')

    conf = get_config("zoedepth_nk", "infer")
    conf['pretrained_resource'] = 'local::/home/gold/depth/ZoeDepth-main/ZoeD_M12_NK.pt'
    model_zoe_n = build_model(conf)
    print("load finished")

    DEVICE = "cuda" if torch.cuda.is_available() else "cpu"
    #DEVICE = "cpu" if torch.cuda.is_available() else "cuda"
    zoe = model_zoe_n.to(DEVICE)
    # 创建图像订阅者，订阅/usb_cam/image_raw话题
    image_sub = rospy.Subscriber('/usb_cam/image_raw', Image, image_callback)

    # 创建图像发布者，发布/usb_cam/image_depth话题
    depth_image_pub = rospy.Publisher('/usb_cam/image_depth', Image, queue_size=10)
    
    # 创建图像发布者，发布/usb_cam/image_color话题
    color_image_pub = rospy.Publisher('/usb_cam/image_color', Image, queue_size=10)

    # 循环等待回调函数
    rospy.spin()
