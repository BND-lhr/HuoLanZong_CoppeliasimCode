import time

import torch
from utils import *
from data.models.unet_res_atten import UNet_res_atten
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from skimage.morphology import skeletonize
from scipy.spatial.distance import cdist
import matplotlib.pyplot as plt
import os

client = RemoteAPIClient()
sim = client.require('sim')

WEIGHT_PATH = r'data/models/unet_r.pth'
NUM_CLASSES = 2
DEVICE = torch.device('cuda')
PER_PIEXL_length_M = 0.09/77  # m/pixels

def load_model(weight_path):
    model = UNet_res_atten(NUM_CLASSES).to(DEVICE)
    if os.path.exists(weight_path):
        model.load_state_dict(torch.load(weight_path, map_location=DEVICE))
        print("Weight load successfully!")
    else:
        print('No weight!')
    return model

def inference(model, raw_rgb):
    img = raw_img_pre_process(raw_rgb)

    img_tensor = transform(img)
    img_tensor = torch.unsqueeze(img_tensor, dim=0)

    img_tensor = img_tensor.to(DEVICE)

    model.eval()
    out = model(img_tensor)
    out = torch.argmax(out, dim=1)
    # print(set((out).reshape(-1).tolist()))
    out = (out).permute((1, 2, 0)).cpu().detach().numpy()
    res = np.array(img.copy())
    res[out[:, :, -1] == 1] = (0, 255, 0)
    res_img = pred_img_post_process(res)

    return out, res_img

def get_vs_rgb(visionSensor_name):
    visionSensorHandle = sim.getObject(visionSensor_name)
    img, res = sim.getVisionSensorImg(visionSensorHandle)
    img = np.frombuffer(img, dtype=np.uint8).reshape(res[1], res[0], 3)
    img = cv2.flip(cv2.cvtColor(img, cv2.COLOR_BGR2RGB), 0)
    return img

def get_pts_by_mask(res_img):
    mask = np.zeros((res_img.shape[0], res_img.shape[1], 1))
    mask[res_img[:, :, 1]==255] = 1
    skeleton = skeletonize(mask > 0)
    img_sk = res_img.copy()
    img_sk[skeleton[:, :, 0] == True] = (255, 0, 0)

    points = np.column_stack(np.where(skeleton))  # (N, 2) -> (y, x)）
    ordered = [points[0]]
    remaining = list(points[1:])
    while remaining:
        last = ordered[-1]
        dists = cdist([last], remaining)[0]
        idx = np.argmin(dists)
        ordered.append(remaining.pop(idx))
    ordered = np.array(ordered)
    sampled_points = sample_points(ordered)

    return sampled_points, img_sk

def sample_points(points, num=50):
    idx = np.linspace(0, len(points) - 1, num).astype(int)
    return points[idx]

def get_target_position(target_object):
    tar_handle = sim.getObject(target_object)
    pos = sim.getObjectPosition(tar_handle)
    return pos

def generate_path(initial_pos, sample_pts):
    '''
    pts: [x y z qx qy qz qw]
    e.g. pts = [0,0,0, 0,0,0,1,
           0.5,0,0.5, 0,0,0,1,
           0.5,0.5,1, 0,0,0,1]
    '''
    print('init : ', initial_pos)
    x, y, z = initial_pos[0], initial_pos[1], initial_pos[2]
    x_a, y_a, z_a = -1 * PER_PIEXL_length_M, 1 * PER_PIEXL_length_M, -1 * PER_PIEXL_length_M
    first_pts = sample_pts[0]
    pts = []
    for pi in sample_pts:
        pi_x, pi_z = pi[1], pi[0]
        pts_l = [x + x_a * (pi_x - first_pts[1]), y - 0.3, z + z_a * abs(pi_z - first_pts[0]), 0, 0, 0, 1]
        pts.extend(pts_l)
    print('pts : ', pts)
    path = sim.createPath(pts[:-1], 0, 50, 0.8)

    # return path

def main():
    cam = '/UR5/kinect/rgb'
    target_sphere = '/target'
    script_sphere = '/Sphere/Script'
    model = load_model(WEIGHT_PATH)
    sim.startSimulation()
    time.sleep(.1)

    # initial Sphere position
    pos = get_target_position(target_sphere)

    # get RGB from Vision Sensor
    raw_img = get_vs_rgb(cam)
    # get mask from Model
    output_img, res_img = inference(model, raw_img)
    # get points from result
    sample_pts, img_sk = get_pts_by_mask(res_img)
    # generate path from points
    generate_path(pos, sample_pts)
    # init script & start
    script_handle = sim.getObject(script_sphere)
    sim.initScript(script_handle)

    # visualization
    plt.subplot(221)
    plt.imshow(raw_img)
    plt.subplot(222)
    plt.imshow(output_img)
    plt.subplot(223)
    plt.imshow(res_img)
    plt.subplot(224)
    plt.imshow(img_sk)
    plt.show()

    sim.stopSimulation()

if __name__ == '__main__':
    main()