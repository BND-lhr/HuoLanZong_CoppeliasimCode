import numpy as np
import cv2
import time
import sim


def encode_visionsensorImage(raw_img, resolution):
    img = np.array(raw_img, dtype=np.uint8)
    img.resize([resolution[1], resolution[0], 3])
    img = cv2.flip(img, 0)
    # print(img.shape)
    # img = cv2.transpose(img, 1)
    return img

