#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
IMX219 ???????????
"""
import time
import numpy as np
import cv2
from hobot_vio import libsrcampy as srcampy

def take_photo(output_path="/home/sunrise/RBE201TC/test_camera.jpg"):
    print("????? IMX219 ??? (MIPI-CSI ??)...")
    cam = srcampy.Camera()
    
    # ?????: pipe_id=0, 30fps, 1080P
    ret = cam.open_cam(0, -1, 30, [1920], [1088], 1080, 1920)
    if ret != 0:
        print(f"? ???????????: {ret}")
        return False
    
    print("? ????????????????...")
    time.sleep(1.0)
    
    print("???????...")
    img_data = cam.get_img(2, 1920, 1088)
    if img_data is None:
        print("? ?????????")
        cam.close_cam()
        return False
    
    # ?? NV12 ??? BGR ????
    raw_array = np.frombuffer(img_data, dtype=np.uint8).reshape((1088 * 3 // 2, 1920))
    bgr_img = cv2.cvtColor(raw_array, cv2.COLOR_YUV2BGR_NV12)
    
    # ????? 1920x1080 ??????? JPEG
    final_img = bgr_img[:1080, :1920]
    cv2.imwrite(output_path, final_img, [cv2.IMWRITE_JPEG_QUALITY, 95])
    
    cam.close_cam()
    print(f"?? ??????????????: {output_path}")
    return True

if __name__ == "__main__":
    take_photo()
