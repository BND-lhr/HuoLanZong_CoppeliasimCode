import sim
import time
import matplotlib.pyplot as plt
from utils import *

IP = '127.0.0.1'
PORT = 19995


def IK_robot_arm(clientId, handle_name, target_info, move_per_distance=0.005, relative_handle=-1):
    sim.simxGetPingTime(clientID=clientId)
    ret0, manipSphere = sim.simxGetObjectHandle(clientId, handle_name, sim.simx_opmode_blocking)
    if ret0 == 0:
        plt.ion()
        fig = plt.figure("vs_img")
        targ_pos = target_info['position']
        targ_ori = target_info['orientation']
        while 1:
            _, ms_pos = sim.simxGetObjectPosition(clientId, manipSphere, relative_handle, sim.simx_opmode_blocking)
            _, ms_ori = sim.simxGetObjectOrientation(clientId, manipSphere, relative_handle, sim.simx_opmode_blocking)

            ret_p = move_positon(clientId, manipSphere, targ_pos, ms_pos, move_per_distance)

            ax = fig.add_subplot(111)
            ax.axis('off')
            img = get_vs_img(clientId, '/vs')
            ax.imshow(img)
            plt.pause(.1)
            fig.clf()

            if ret_p != 1:
                break

        plt.ioff()
        print("tip_positon = {}".format(ms_pos))

    else:
        print("Fail to get the object handle!")


def move_positon(cid, handle, targ_p, tip_p, move_per_distance):
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

        sim.simxSetObjectPosition(cid, handle, -1,
                                  (tip_p[0] + a1 * move_per_distance,
                                   tip_p[1] + a2 * move_per_distance,
                                   tip_p[2] + a3 * move_per_distance),
                                  sim.simx_opmode_oneshot)
        return 1
    else:
        return 0


def move_orientiation():
    pass


def get_teget_info(cid, handle_name):
    target_info = {
        'position': [],
        'orientation': [],
    }
    ret0, target = sim.simxGetObjectHandle(cid, handle_name, sim.simx_opmode_blocking)
    if ret0 == 0:
        _, target_position = sim.simxGetObjectPosition(cid, target, -1, sim.simx_opmode_blocking)
        _, target_orientation = sim.simxGetObjectOrientation(cid, target, -1, sim.simx_opmode_blocking)
        target_info['position'] = target_position
        target_info['orientation'] = target_orientation
    else:
        print('Fail to get Target handle')

    return target_info


def get_vs_img(cid, visionsensor_name, mode=1):
    """
    mode set 0: initial vision sensor, no return!
    mode set 1: get vision sensor's results
    First call init func(getvisionsensorimage), opmode should be set "sim.simx_opmode_streaming"
    After init, opmode should be set "sim.simx_opmode_buffer"
    """
    if mode == 0:
        _, vs_handle = sim.simxGetObjectHandle(cid, visionsensor_name, sim.simx_opmode_blocking)
        sim.simxGetVisionSensorImage(cid, vs_handle, 0, sim.simx_opmode_streaming)
        time.sleep(.5)
    elif mode == 1:
        _, vs_handle = sim.simxGetObjectHandle(cid, visionsensor_name, sim.simx_opmode_blocking)
        _, resolution, raw_img = sim.simxGetVisionSensorImage(cid, vs_handle, 0, sim.simx_opmode_buffer)
        img = encode_vs_img(resolution, raw_img)
        return img
    else:
        print("Other mode aren't available!")


def main():
    sim.simxFinish(-1)
    cid = sim.simxStart(IP, PORT, True, True, 3000, 5)
    if cid != -1:
        # strat simulation
        print("Connected to remote API sever successfully!")
        sim.simxStartSimulation(cid, sim.simx_opmode_blocking)
        print("Simulation Start!")

        # init
        get_vs_img(cid, '/vs', mode=0)

        targ_info = get_teget_info(cid, '/dummy_green')
        IK_robot_arm(cid, '/manipSphere', targ_info)

        # stop simulation
        sim.simxStopSimulation(cid, sim.simx_opmode_blocking)
        print("Simulation Stop!")
        sim.simxFinish(cid)
    else:
        print("Failed to connecting to remote API sever!")


if __name__ == '__main__':
    main()
