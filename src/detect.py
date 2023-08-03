import argparse
import os, sys
import shutil
import time
from pathlib import Path

# This adds the subdirectories of YOLOP to be accessed
BASE_DIR = os.path.dirname(os.path.abspath(__file__)) + "/YOLOP"
sys.path.append(BASE_DIR)
print(BASE_DIR)

print(sys.path)
import cv2
import torch
import torch.backends.cudnn as cudnn
from numpy import random
import scipy.special
import numpy as np
import torchvision.transforms as transforms
import PIL.Image as image

from lib.config import cfg
from lib.config import update_config
from lib.utils.utils import create_logger, select_device, time_synchronized
from lib.models import get_net
from lib.dataset import LoadImages, LoadStreams
from lib.core.general import non_max_suppression, scale_coords
from lib.utils import plot_one_box,show_seg_result, letterbox_for_img
from lib.core.function import AverageMeter
# from lib.core.postprocess import morphological_process, connect_lane
from tqdm import tqdm
normalize = transforms.Normalize(
        mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]
    )

transform=transforms.Compose([
            transforms.ToTensor(),
            normalize,
        ])


def detect(model, image0, DEVICE, IMG_SIZE, CONF_THRES, IOU_THRES):
    '''
    Parameters
    ----------
    model - The model initialized somewhere else
    image0 - torch.tensor - PyTorch tensor conatining image info TODO: size?
    DEVICE - PyTorch device object that needs to be passed in TODO: I honestly have no idea to much
    IMG_SIZE - int? - Input image size TODO: Flexibility for tuple?
    CONF_THRES - float - Confidence threshold
    IOU_THRES - float - IOU  threshold

    Return
    ------
    det - List of vehicle detections
    da_seg_mask - Mask of the driveable area
    ll_seg_mask - Mask of the detected lanes
    '''

    half = False
    # Get names and colors
    names = model.module.names if hasattr(model, 'module') else model.names
    colors = [[random.randint(0, 255) for _ in range(3)] for _ in range(len(names))]


    # Run inference
    t0 = time.time()

    # # TODO: Do I really need to init this?
    # vid_path, vid_writer = None, None

    inf_time = AverageMeter()
    nms_time = AverageMeter()

    h0, w0 = image0.shape[:2]

    # Padded resize
    image, ratio, pad = letterbox_for_img(image0, new_shape=IMG_SIZE, auto=True)
    h, w = image.shape[:2]
    shapes = (h0, w0), ((h / h0, w / w0), pad)

    # Convert
    image = np.ascontiguousarray(image)
    
    image = transform(image).to(DEVICE)
    image = image.half() if half else image.float()  # uint8 to fp16/32
    if image.ndimension() == 3:
        image = image.unsqueeze(0)
    # Inference
    t1 = time_synchronized()
    det_out, da_seg_out,ll_seg_out= model(image)
    # det_out, ll_seg_out = model(image)
    t2 = time_synchronized()
    # if i == 0:
    #     print(det_out)
    inf_out, _ = det_out
    inf_time.update(t2-t1,image.size(0))

    # Apply NMS
    t3 = time_synchronized()
    det_pred = non_max_suppression(inf_out, conf_thres=CONF_THRES, iou_thres=IOU_THRES, classes=None, agnostic=False)
    t4 = time_synchronized()

    nms_time.update(t4-t3,image.size(0))
    det=det_pred[0]

    _, _, height, width = image.shape
    h,w,_= image0.shape
    pad_w, pad_h = shapes[1][1]
    pad_w = int(pad_w)
    pad_h = int(pad_h)
    ratio = shapes[1][0][1]

    da_predict = da_seg_out[:, :, pad_h:(height-pad_h),pad_w:(width-pad_w)]
    da_seg_mask = torch.nn.functional.interpolate(da_predict, scale_factor=(1/ratio), mode='bilinear')
    _, da_seg_mask = torch.max(da_seg_mask, 1)
    da_seg_mask = da_seg_mask.int().squeeze().cpu().numpy()
    # da_seg_mask = morphological_process(da_seg_mask, kernel_size=7)

    
    ll_predict = ll_seg_out[:, :,pad_h:(height-pad_h),pad_w:(width-pad_w)]
    ll_seg_mask = torch.nn.functional.interpolate(ll_predict, scale_factor=(1/ratio), mode='bilinear')
    _, ll_seg_mask = torch.max(ll_seg_mask, 1)
    ll_seg_mask = ll_seg_mask.int().squeeze().cpu().numpy()
    # Lane line post-processing
    #ll_seg_mask = morphological_process(ll_seg_mask, kernel_size=7, func_type=cv2.MORPH_OPEN)
    #ll_seg_mask = connect_lane(ll_seg_mask)

    # TODO: This can test to see if desired
    # image0 = show_seg_result(image0, (da_seg_mask, ll_seg_mask), _, _, is_demo=True)

    if len(det):
            det[:,:4] = scale_coords(image.shape[2:],det[:,:4],image0.shape).round()
    #         for *xyxy,conf,cls in reversed(det):
    #             label_det_pred = f'{names[int(cls)]} {conf:.2f}'
    #             plot_one_box(xyxy, image0 , label=label_det_pred, color=colors[int(cls)], line_thickness=2)
    
    # cv2.imshow('image', image0)
    # cv2.waitKey(1)  # 1 millisecond

    return det, da_seg_mask, ll_seg_mask
    # return det, ll_seg_mask