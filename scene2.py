import sim
import time
import matplotlib.pyplot as plt
import random
from utils import *

IP = '127.0.0.1'
PORT = 19997


def IK_robot_arm(clientId, handle_name, target_info, move_per_distance=0.001, relative_handle=-1):
    sim.simxGetPingTime(clientID=clientId)
    # the tip of robot
    # ret为0时表示正确获取，为8时表示发生错误
    ret0, manipSphere = sim.simxGetObjectHandle(clientId, handle_name, sim.simx_opmode_blocking)
    if ret0 == 0:
        plt.ion()
        fig = plt.figure('vs1_img')
        targ_position = target_info['position']
        # targ_orientation = target_info['orientation']
        while 1:
            _, ms_position = sim.simxGetObjectPosition(clientId, manipSphere, relative_handle, sim.simx_opmode_blocking)
            _, ms_orientation = sim.simxGetObjectOrientation(clientId, manipSphere, relative_handle,
                                                             sim.simx_opmode_blocking)

            ret_p = move_position(clientId, manipSphere, targ_position, ms_position, move_per_distance=0.01)
            # ret_o = move_orientation(clientId, manipSphere, targ_orientation, ms_orientation, move_per_distance=0.08)

            ax = fig.add_subplot(111)
            ax.axis('off')
            img = get_vs_img(clientId, '/vs1')
            ax.imshow(img)
            plt.pause(.05)
            fig.clf()

            # if ret_p != 1 and ret_o != 1:
            if ret_p != 1:
                break

        plt.ioff()
        print("tip position = {}\ntip orientation = {}".format(ms_position, ms_orientation))
    else:
        print('Fail to get the object handle!')


def move_position(clientId, handle, targ_p, tip_p, move_per_distance, relative_handle=-1):
    # 默认距离阈值与单位移动距离一致
    dist_shrehold = move_per_distance
    d1 = targ_p[0] - tip_p[0]
    d2 = targ_p[1] - tip_p[1]
    d3 = targ_p[2] - tip_p[2]
    if abs(d1) > dist_shrehold or abs(d2) > dist_shrehold or abs(d3) > dist_shrehold:
        a1 = 1 if d1 > 0 else -1
        a2 = 1 if d2 > 0 else -1
        a3 = 1 if d3 > 0 else -1
        a1 = 0 if abs(d1) <= dist_shrehold else a1
        a2 = 0 if abs(d2) <= dist_shrehold else a2
        a3 = 0 if abs(d3) <= dist_shrehold else a3

        sim.simxSetObjectPosition(clientId, handle, relative_handle,
                                  (tip_p[0] + a1 * move_per_distance,
                                   tip_p[1] + a2 * move_per_distance,
                                   tip_p[2] + a3 * move_per_distance),
                                  sim.simx_opmode_oneshot)
        return 1
    else:
        return 0


def move_orientation(clientId, handle, targ_o, tip_o, move_per_distance, relative_handle=-1):
    angle_shrehold = move_per_distance
    o1 = targ_o[0] - tip_o[0]
    o2 = targ_o[1] - tip_o[1]
    o3 = targ_o[2] - tip_o[2]
    if abs(o1) > angle_shrehold or abs(o2) > angle_shrehold or abs(o3) > angle_shrehold:
        a1 = 1 if o1 > 0 else -1
        a2 = 1 if o2 > 0 else -1
        a3 = 1 if o3 > 0 else -1
        a1 = 0 if abs(o1) <= angle_shrehold else a1
        a2 = 0 if abs(o2) <= angle_shrehold else a2
        a3 = 0 if abs(o3) <= angle_shrehold else a3
        # 旋转角度在两端不同，python的o数组是弧度制，仿真那端是角度制，本质一样，但数值不同
        # 弧度 = （角度*pi）/180
        sim.simxSetObjectOrientation(clientId, handle, relative_handle,
                                     (tip_o[0] + a1 * move_per_distance,
                                      tip_o[1] + a2 * move_per_distance,
                                      tip_o[2] + a3 * move_per_distance),
                                     sim.simx_opmode_oneshot
                                     )
        return 1
    else:
        return 0


def get_target_info(clientId, handle_name, relative_handle=-1):
    # 可以存储目标物体的各类信息，包括但不限于位置，方向等
    target_info = {
        'position': [],
        'orientation': []
    }
    ret0, target = sim.simxGetObjectHandle(clientId, handle_name, sim.simx_opmode_blocking)
    if ret0 == 0:
        _, target_position = sim.simxGetObjectPosition(clientId, target, relative_handle, sim.simx_opmode_blocking)
        _, target_orientation = sim.simxGetObjectOrientation(clientId, target, relative_handle,
                                                             sim.simx_opmode_blocking)
        # print('target_position = ', target_position)
        target_info['position'] = target_position
        target_info['orientation'] = target_orientation
    else:
        print("Fail to get Target info!")

    return target_info


def tagert_move(clientId, handle_name, move_range, relative_handle=-1, mode=0):
    """
    move_range : the move range of position and orientation respectively
    mode set 0: random move object
    """
    _, target = sim.simxGetObjectHandle(clientId, handle_name, sim.simx_opmode_blocking)
    _, targ_position = sim.simxGetObjectPosition(clientId, target, relative_handle, sim.simx_opmode_blocking)
    _, targ_orientation = sim.simxGetObjectOrientation(clientId, target, relative_handle,
                                                       sim.simx_opmode_blocking)
    if mode == 0:
        # move_range ∈ (min_, max_)
        max_ = 1
        min_ = -1
        for i in range(len(targ_position)):
            a = random.random()
            b = random.random()
            targ_position[i] += ((max_ - min_) * a + min_) * move_range[0]
            targ_orientation[i] += ((max_ - min_) * b + min_) * move_range[1]
        sim.simxSetObjectPosition(clientId, target, relative_handle,
                                  targ_position,
                                  sim.simx_opmode_oneshot)
        sim.simxSetObjectOrientation(clientId, target, relative_handle,
                                     targ_orientation,
                                     sim.simx_opmode_oneshot)
    else:
        print('Other modes aren\'t yet available!')


def get_vs_img(clientId, visionsensor_name, mode=1):
    """
    mode set 0: initial vision sensor, no return!
    mode set 1: get vision sensor's results
    First call init func(getvisionsensorimage), opmode should be set "simx_opmode_streaming"
    After init, opmode should be set "simx_opmode_buffer"
    """
    if mode == 0:
        _, vs_handle = sim.simxGetObjectHandle(clientId, visionsensor_name, sim.simx_opmode_blocking)
        sim.simxGetVisionSensorImage(clientId, vs_handle, 0, sim.simx_opmode_streaming)
        time.sleep(.5)
    elif mode == 1:
        _, vs = sim.simxGetObjectHandle(clientId, visionsensor_name, sim.simx_opmode_blocking)
        ret, resolution, raw_img = sim.simxGetVisionSensorImage(clientId, vs, 0, sim.simx_opmode_buffer)
        img = encode_visionsensorImage(raw_img, resolution)
        return img
    else:
        print("Other modes aren\'t yet available!")


def main():
    sim.simxFinish(-1)
    cid = sim.simxStart(IP, PORT, True, True, 3000, 5)
    if cid != -1:
        # strat simulation
        print("Connect to remote API sever successfully!")
        sim.simxStartSimulation(cid, sim.simx_opmode_blocking)
        print("Simulation Start!")

        get_vs_img(cid, '/vs1', mode=0)
        time.sleep(.5)

        for _ in range(5):
            targ_info = get_target_info(cid, '/dummy_green')
            IK_robot_arm(cid, '/UR5/Sphere1', targ_info)
            tagert_move(cid, '/sphere_green', move_range=[0.1, 0.3])

        # stop simulation
        sim.simxStopSimulation(cid, sim.simx_opmode_blocking)
        print("Simulation Stop!")
        sim.simxFinish(cid)
    else:
        print("Failed to connect to remote API sever!")


if __name__ == '__main__':
    main()
