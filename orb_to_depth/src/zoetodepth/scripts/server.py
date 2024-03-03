'''
This is the start function for monocular depth estimation. First, load the fine-tuning model, then accept the keyframe conversion depth map request sent by the implementation reconstruction module, and process it as a depth map in the callback function before returning the result
'''


#!/home/gold/anaconda3/envs/zoe/bin/python3.9


import os
import sys
#print(torch.__file__)
sys.path.append('/home/zjd/anaconda3/envs/zoe/lib/python3.9/site-packages')
sys.path.append('/home/zjd/orb_to_depth/src/zoetodepth')
# for p in sys.path:
#     print(p)

import rospy
from zoetodepth.srv import depthimage,depthimageRequest,depthimageResponse

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


def doReq(req):
    bridge = CvBridge()

    cv_image = bridge.imgmsg_to_cv2(req.rgbimage, desired_encoding="bgr8")

    depth = zoe.infer_pil(cv_image)
    depth16bit = depth
    to_raw_16bit(depth, depth16bit)

    depth_ros = bridge.cv2_to_imgmsg(depth16bit, encoding="passthrough")

    # resp = AddIntsResponse()
    # resp.sum = sum
    resp = depthimageResponse(depth_ros)
    return resp


if __name__ == "__main__":

    rospy.init_node("addints_server_p")

    conf = get_config("zoedepth_nk", "infer")
    conf['pretrained_resource'] = 'local::/home/zjd/depth/ZoeDepth-main/ZoeD_M12_NK.pt'
    model_zoe_n = build_model(conf)
    print("load finished")

    DEVICE = "cuda" if torch.cuda.is_available() else "cpu"
    #DEVICE = "cpu" if torch.cuda.is_available() else "cuda"
    zoe = model_zoe_n.to(DEVICE)


    server = rospy.Service("depthimage",depthimage,doReq)

    rospy.spin()
