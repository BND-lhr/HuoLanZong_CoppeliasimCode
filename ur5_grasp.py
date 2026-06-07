import matplotlib.pyplot as plt

from calibrate_hand_eye import *

print('Program started')

client = RemoteAPIClient()
sim = client.require('sim')

def grasp_setting(signal=0):
    sim.setInt32Signal('RG2_open', signal)
    time.sleep(.5)

def get_object_coordinate_byHSV(rgb ,hsv_img, is_visual=False):
    # e.g.Green
    lower_green = np.array([50, 100, 100])
    upper_green = np.array([70, 255, 255])
    mask = cv2.inRange(hsv_img, lower_green, upper_green)
    x, y, w, h = cv2.boundingRect(mask)
    center_coord = [int(x + 1/2 * w), int(y + 1/2 * h)]
    # print(center_coord)
    if is_visual:
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        res = cv2.drawContours(rgb.copy(), contours, -1, (255, 0, 0), 3)
        return res, center_coord, mask
    return center_coord, mask

def test(mtx, dist, R_cam2gripper, t_cam2gripper, list_param, list_handle, list_vaj, display=None):
    """
    Test camera model
    """
    chessboard_size, objpoints, imgpoints, robot_poses_R, robot_poses_t, camera_poses_R, camera_poses_t, objp, targetjoinPos1, targetPos = list_param
    targetHandle, tipHandle, robotHandle, visionSensorHandle, deepSensorHandle, chessboardHandle, jointHandles = list_handle
    jmaxVel, jmaxAccel, jmaxJerk, maxVel, maxAccel, maxJerk = list_vaj
    moveToConfig(jointHandles, jmaxVel, jmaxAccel, jmaxJerk, targetjoinPos1)
    # open grasp
    grasp_setting(1)

    for i in range(1):
        # while True:
        moveToPose(targetPos[i], tipHandle, targetHandle, maxVel, maxAccel, maxJerk)
        img, gray = get_vs_rgb(visionSensorHandle)
        depth_image = get_vs_depth(deepSensorHandle)

        hsv_img = rgb2hsv(img)
        res_img, center_xy, mask = get_object_coordinate_byHSV(img, hsv_img, True)
        plt.subplot(221)
        plt.imshow(img)
        plt.subplot(222)
        plt.imshow(hsv_img)
        plt.subplot(223)
        plt.imshow(res_img)
        plt.subplot(224)
        plt.imshow(depth_image)
        plt.show()

        # save images
        # mask_ = np.ones([480, 640])
        # plt.xticks([])
        # plt.yticks([])
        # plt.imshow(depth_image)
        # save_path = r'/home/sun/lhr_projects/graspnet-baseline/doc/test2/depth.png'
        # plt.imsave(save_path, depth_image)
        # plt.show()
        # depth_image = (depth_image * 1000).astype(np.uint16)
        # cv2.imwrite(save_path, depth_image)

        # 找到棋盘格的角点
        # ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)
        # img = cv2.drawChessboardCorners(img, chessboard_size, corners, ret)
        # ret, rvec, tvec = cv2.solvePnP(objp, corners, mtx, dist)
        rvec = np.array([[-0.14744289],
                         [-0.31215],
                         [3.11476837]])
        tvec = np.array([[0.15440214],
                         [0.13234639],
                         [0.80824459]])


        u = center_xy[0]
        v = center_xy[1]
        Z = np.mean(depth_image[mask>250]) + 0.01


        # 计算相机坐标系下的三维点
        P_cam = pixel_to_camera_coordinates(u, v, Z, mtx)

        print("相机坐标系下的三维点 P_cam:", P_cam)
        # t_cam2gripper=t_cam2gripper.reshape(-1)
        # 计算物体在手爪坐标系中的位置

        # 计算点在末端坐标系下的坐标 P_end
        P_end = np.dot(R_cam2gripper, P_cam) + t_cam2gripper.reshape(-1)
        # sim.setObjectPosition(targetHandle, P_end,tipHandle)

        # 计算点在基座坐标系下的坐标 P_base
        tip_matrix = sim.getObjectMatrix(tipHandle)
        # 提取旋转矩阵 R_end_to_base (3x3)
        R_end_to_base = np.array([
            [tip_matrix[0], tip_matrix[1], tip_matrix[2]],
            [tip_matrix[4], tip_matrix[5], tip_matrix[6]],
            [tip_matrix[8], tip_matrix[9], tip_matrix[10]]
        ])

        # 提取平移向量 t_end_to_base (3x1)
        t_end_to_base = np.array([
            [tip_matrix[3]],
            [tip_matrix[7]],
            [tip_matrix[11]]
        ])

        P_base = np.dot(R_end_to_base, P_end) + t_end_to_base.reshape(-1)
        # sim.setObjectPosition(targetHandle, P_base)
        Tip_pose = sim.getObjectPose(tipHandle)

        # 将旋转向量转换为旋转矩阵
        R_board_to_camera, _ = cv2.Rodrigues(rvec)

        # 计算标定板相对于末端的旋转矩阵和平移向量
        R_board_to_end = R_cam2gripper @ R_board_to_camera
        t_board_to_end = R_cam2gripper @ tvec.flatten() + t_cam2gripper.flatten()

        # 计算标定板相对于世界坐标系的旋转矩阵和平移向量
        R_board_to_world = R_end_to_base @ R_board_to_end
        t_board_to_world = R_end_to_base @ t_board_to_end + t_end_to_base.flatten()
        chessboard_matrix = sim.getObjectMatrix(targetHandle)

        cal_chessboard_matrix = buildMatrix(R_board_to_world, t_board_to_world)
        sim.setObjectMatrix(targetHandle, cal_chessboard_matrix)

        goalTr = Tip_pose.copy()
        goalTr[0] = P_base[0]
        goalTr[1] = P_base[1]
        goalTr[2] = P_base[2]
        print('Target T in world = ', goalTr)
        moveToPose(goalTr, tipHandle, targetHandle, maxVel, maxAccel, maxJerk)

    # grasp inital
    grasp_setting(0)
    moveToPose(targetPos[i+1], tipHandle, targetHandle, maxVel, maxAccel, maxJerk)

def main():
    list_param = init_param()
    list_handle = init_handle()
    list_vaj = init_VAJ()
    model_path = r'data/cmodel.npz'
    # 实例化显示图像
    # display = ImageStreamDisplay([640, 480])

    sim.startSimulation()

    mtx, dist, R_cam2gripper, t_cam2gripper = load_cmodel(path=model_path)

    test(mtx, dist, R_cam2gripper, t_cam2gripper, list_param, list_handle, list_vaj)
    time.sleep(3)

    sim.stopSimulation()

if __name__ == '__main__':
    main()