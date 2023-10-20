import os
import torch.nn as nn
import torch
import torchvision
import torchvision.transforms as transforms
from glob import glob
import os
from torch.utils.data import Dataset, DataLoader
from torchvision.datasets.folder import default_loader
from tqdm import tqdm
import numpy as np
import cv2


# tmp import path #
ROOT = '/home/detection/'
import sys
sys.path.append(ROOT)

from utils.adet.modeling.backbone.rgbdfpn import build_resnet_rgbd_latefusion_fpn_backbone
from utils.adet.modeling.rcnn.rcnn_heads import ORCNNROIHeads
from utils.adet.config import get_cfg
from utils.adet.utils.post_process import detector_postprocess, DefaultPredictor

    
def load_model(base_folder, api_name, 
               device='cpu',
               confidence_threshold=0.7,
               cfg_name='R50_rgbdconcat_mlc_occatmask_hom_concat.yaml',
               ckp_name='model.pt'):
    model = ''
    config_file = os.path.join('/home/detection', 'utils', 'configs', cfg_name)
    ckp_file = os.path.join(base_folder, api_name, ckp_name)
    # UOAIS-Net
    cfg = get_cfg()
    cfg.merge_from_file(config_file)
    cfg.defrost()
    cfg.MODEL.DEVICE = device
    cfg.MODEL.WEIGHTS = ckp_file
    cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = confidence_threshold
    cfg.MODEL.ROI_HEADS.NMS_THRESH_TEST = 0.3
    model = DefaultPredictor(cfg)
    model.POST_PROC = detector_postprocess
    return model


def load_pretrained_model(base_folder, api_name, DEVICE):
    ckp_name = 'model_rgbd.pth'
    model = load_model(base_folder, api_name, ckp_name=ckp_name, device=DEVICE)
    # model.net.load_state_dict(torch.load(os.path.join(base_folder, api_name, 'model.pt'), map_location='cpu'))
    return model


def normalize_depth(depth, min_val=250.0, max_val=1500.0):
    """ normalize the input depth (mm) and return depth image (0 ~ 255)
    Args:
        depth ([np.float]): depth array [H, W] (mm) 
        min_val (float, optional): [min depth]. Defaults to 250 mm
        max_val (float, optional): [max depth]. Defaults to 1500 mm.

    Returns:
        [np.uint8]: normalized depth array [H, W, 3] (0 ~ 255)
    """
    depth[depth < min_val] = min_val
    depth[depth > max_val] = max_val
    depth = (depth - min_val) / (max_val - min_val) * 255
    depth = np.expand_dims(depth, -1)
    depth = np.uint8(np.repeat(depth, 3, -1))
    return depth

def unnormalize_depth(depth, min_val=250.0, max_val=1500.0):
    """ unnormalize the input depth (0 ~ 255) and return depth image (mm)
    Args:
        depth([np.uint8]): normalized depth array [H, W, 3] (0 ~ 255)
        min_val (float, optional): [min depth]. Defaults to 250 mm
        max_val (float, optional): [max depth]. Defaults to 1500 mm.
    Returns:
        [np.float]: depth array [H, W] (mm) 
    """
    depth = np.float32(depth) / 255
    depth = depth * (max_val - min_val) + min_val
    return depth


def inpaint_depth(depth, factor=1, kernel_size=3, dilate=False):
    """ inpaint the input depth where the value is equal to zero
    Args:
        depth ([np.uint8]): normalized depth array [H, W, 3] (0 ~ 255)
        factor (int, optional): resize factor in depth inpainting. Defaults to 4.
        kernel_size (int, optional): kernel size in depth inpainting. Defaults to 5.
    Returns:
        [np.uint8]: inpainted depth array [H, W, 3] (0 ~ 255)
    """
    H, W, _ = depth.shape
    resized_depth = cv2.resize(depth, (W//factor, H//factor))
    mask = np.all(resized_depth == 0, axis=2).astype(np.uint8)
    if dilate:
        mask = cv2.dilate(mask, np.ones((kernel_size, kernel_size), np.uint8), iterations=1)
    inpainted_data = cv2.inpaint(resized_depth, mask, kernel_size, cv2.INPAINT_TELEA)
    inpainted_data = cv2.resize(inpainted_data, (W, H))
    depth = np.where(depth == 0, inpainted_data, depth)
    return depth


def load_transform(W, H):
    # image: (H, W, 3), np.uint8
    # depth: (H, W), np.uint16
    # input: (H, W, 6), np.uint8
    def transform(image, depth):
        # resize 
        image = cv2.resize(image, (W, H))
        depth = normalize_depth(depth)
        depth = cv2.resize(depth, (W, H), interpolation=cv2.INTER_NEAREST)
        depth = inpaint_depth(depth)
        input = np.concatenate([image, depth], -1)  
        return input
    return transform
