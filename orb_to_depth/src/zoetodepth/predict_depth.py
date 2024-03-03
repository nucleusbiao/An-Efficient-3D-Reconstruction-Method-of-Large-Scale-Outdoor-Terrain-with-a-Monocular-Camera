import torch
from PIL import Image
from tqdm import tqdm 
import numpy as np
import os
import glob 
import cv2 
from zoedepth.models.builder import build_model
from zoedepth.utils.config import get_config
from zoedepth.utils.misc import pil_to_batched_tensor
from zoedepth.utils.misc import save_raw_16bit

input_path = '/home/gold/shujuji/ip24/rgb'
output_path = '/home/gold/shujuji/ip24/depth'

conf = get_config("zoedepth_nk", "infer")
conf['pretrained_resource'] = 'local::/home/gold/depth/ZoeDepth-main/ZoeD_M12_NK.pt'
model_zoe_n = build_model(conf)
#model_zoe_n = torch.hub.load(".", "ZoeD_N", source="local", pretrained=True)

print("load finished")

DEVICE = "cuda" if torch.cuda.is_available() else "cpu"
zoe = model_zoe_n.to(DEVICE)

image_files = glob.glob(os.path.join(input_path, '*.jpg')) + glob.glob(os.path.join(input_path, '*.png'))
for image_file in tqdm(image_files, desc='Processing'):
    image = Image.open(image_file).convert("RGB")
    depth = zoe.infer_pil(image)

    file_name = os.path.basename(image_file)
    output_file = os.path.join(output_path, file_name)

    save_raw_16bit(depth, output_file)

