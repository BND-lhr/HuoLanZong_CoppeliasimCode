import numpy as np
import csv
import math
import sim
import cv2

def encode_vs_img(resolution, raw_img):
    img = np.array(raw_img, dtype=np.uint8)
    img.resize([resolution[1], resolution[0], 3])
    img = cv2.flip(img, 0)
    # print(img.shape)
    return img

def get_csvInfo():
    jointPath = r'./data/JointDegreeInfo.csv'
    posePath = r'./data/TipPoseInfo.csv'
    targetjoinPos1 = []
    targetPos = []
    joint_info = csv.reader(open(jointPath))
    pose_info = csv.reader(open(posePath))
    for row in joint_info:
        targetjoinPos1 = [float(i) * math.pi / 180 for i in row]

    for row in pose_info:
        row = [float(i) for i in row]
        targetPos.append(row)

    # targetjoinPos1 = [90 * math.pi / 180, -20 * math.pi / 180, 45 *
    #                   math.pi / 180, -25 * math.pi / 180, 0 * math.pi / 180, 0 * math.pi / 180]
    #
    # # Pose is relative to world
    # targetPos = []
    #
    # targetPos1 = [0.757, -0.567, 0.757,
    #               -0.469, -0.428, -0.522, -0.571]
    # targetPos2 = [0.76, -0.539, 0.757,
    #               0.63, -0.079, 0.77, 0.053]
    # targetPos3 = [0.774, -0.587, 0.641,
    #               -0.541, -0.249, -0.744, -0.305]
    # targetPos4 = [0.708, -0.715, 0.678,
    #               -0.524, -0.482, -0.483, -0.512]
    # targetPos5 = [0.447, -0.519, 0.636,
    #               -0.427, -0.498, -0.582, -0.482]
    # targetPos6 = [0.595, -0.46, 0.854,
    #               -0.348, -0.445, -0.649, -0.511]
    # targetPos7 = [0.45, -0.492, 0.637,
    #               -0.393, -0.498, -0.603, -0.486]
    # # targetPos8 = [0.575, -0.34, 0.571,
    # #               0.128, -0.657, -0.3, -0.68]
    # # targetPos9 = [0.552, -0.438, 0.838,
    # #               -0.366, -0.429, -0.651, -0.51]
    # # targetPos10 = [0.64, -0.571, 0.827,
    # #                -0.482, -0.372, -0.584, -0.539]
    # targetPos8 = [0.714, -0.46, 0.857,
    #               -0.088, -0.525, -0.585, -0.613]
    # targetPos9 = [0.754, -0.415, 0.851,
    #               0.012, -0.572, -0.373, -0.732]
    # targetPos10 = [0.618, -0.516, 0.591,
    #                -0.646, -0.438, -0.483, -0.399]
    #
    # targetPos.append(targetPos1)
    # targetPos.append(targetPos2)
    # targetPos.append(targetPos3)
    # targetPos.append(targetPos4)
    # targetPos.append(targetPos5)
    # targetPos.append(targetPos6)
    # targetPos.append(targetPos7)
    # targetPos.append(targetPos8)
    # targetPos.append(targetPos9)
    # targetPos.append(targetPos10)

    return targetjoinPos1, targetPos

def save_np(mtx, dist, R_cam2gripper, t_cam2gripper, path=r'./data/cmodel.npz'):
    # mtx = np.array([[589.28755183, 0., 318.41181377],
    #                 [0., 589.18754742, 241.54789642],
    #                 [0., 0., 1.]])
    # dist = np.array([[-2.08685922e-02, 2.28233380e-01, 8.45098011e-04, -5.19213751e-04, -6.73892679e-01]])
    # R_cam2gripper = np.array([[0.99997287, 0.00267643, -0.00686203],
    #                           [0.00255335, -0.99983683, -0.01788258],
    #                           [-0.00690877, 0.01786458, -0.99981655]])
    # t_cam2gripper = np.array([[-0.01215071],
    #                           [0.08191152],
    #                           [0.16138325]])
    np.savez(path, mtx=mtx, dist=dist, R_cam2gripper=R_cam2gripper, t_cam2gripper=t_cam2gripper)

def load_cmodel(path=r'./data/cmodel.npz'):
    data = np.load(path)
    mtx = data['mtx']
    dist = data['dist']
    R_cam2gripper = data['R_cam2gripper']
    t_cam2gripper = data['t_cam2gripper']

    return mtx, dist, R_cam2gripper, t_cam2gripper

def buildMatrix(R_board_to_world, t_board_to_world):
    # ubuntu下，正常np数组可以传入
    cal_chessboard_matrix = np.array(
        [R_board_to_world[0][0], R_board_to_world[0][1], R_board_to_world[0][2], t_board_to_world[0],
         R_board_to_world[1][0], R_board_to_world[1][1], R_board_to_world[1][2], t_board_to_world[1],
         R_board_to_world[2][0], R_board_to_world[2][1], R_board_to_world[2][2], t_board_to_world[2]])


    # windows下，可能会遇到np数值解析失败时，最稳妥的方式，使用list传入，包不会错的
    # cal_chessboard_matrix = \
    #     [R_board_to_world[0][0], R_board_to_world[0][1], R_board_to_world[0][2], t_board_to_world[0],
    #      R_board_to_world[1][0], R_board_to_world[1][1], R_board_to_world[1][2], t_board_to_world[1],
    #      R_board_to_world[2][0], R_board_to_world[2][1], R_board_to_world[2][2], t_board_to_world[2]]
    return cal_chessboard_matrix
