import sys
sys.path.append('..')
import logging

import rtde.rtde as rtde
import rtde.rtde_config as rtde_config

import numpy as np
from threading import Thread
import time

class RTDE_urx(object):
    def __init__(self, ROBOT_HOST='127.0.0.1',
                 ROBOT_PORT=30004,
                 config_filename='control_loop_configuration.xml'):

        keep_running = True
        logging.getLogger().setLevel(logging.INFO)
        conf = rtde_config.ConfigFile(config_filename)

        state_names, state_types = conf.get_recipe('state')
        setp_names, setp_types = conf.get_recipe('setp')
        watchdog_names, watchdog_types = conf.get_recipe('watchdog')
        # gripper_names, gripper_types = conf.get_recipe("gripper")

        self.con = rtde.RTDE(ROBOT_HOST, ROBOT_PORT)
        self.con.connect()

        # get controller version
        self.con.get_controller_version()

        # setup recipes
        self.con.send_output_setup(state_names, state_types)
        self.setp = self.con.send_input_setup(setp_names, setp_types)
        # self.watchdog = self.con.send_input_setup(watchdog_names, watchdog_types)
        # self.gripper = self.con.send_input_setup(gripper_names, gripper_types)
        ### Initialize variables of the register
        self.initialize()

        # start data synchronization
        if not self.con.send_start():
            sys.exit()

    def initialize(self):
        self.setp.input_double_register_0 = 0
        self.setp.input_double_register_1 = 0
        self.setp.input_double_register_2 = 0
        self.setp.input_double_register_3 = 0
        self.setp.input_double_register_4 = 0
        self.setp.input_double_register_5 = 0
        # self.gripper.input_int_register_1 = 0
        # self.watchdog.input_int_register_0 = 0

    def setp_to_list(self):
        list = []
        for i in range(0, 6):
            list.append(self.setp.__dict__["input_double_register_%i" % i])
        return list

    def list_to_setp(self, list):
        for i in range(0, 6):
            self.setp.__dict__["input_double_register_%i" % i] = list[i]
        return self.setp

    def wait(self, target):
        '''
        Keep program in movel while movement is not finished.
        :param target: target in cartsian coordinates
        :return:
        '''
        state = self.con.receive()
        actual_tcp_pose = np.array(state.actual_TCP_pose)
        actual_tcp_pose[3:], target[3:] = np.abs(actual_tcp_pose[3:]), np.abs(target[3:])   # Angles are taken positves
        deltapose = actual_tcp_pose - np.array(target)
        distance = (deltapose[:]*deltapose[:]).sum()
        if distance < 0.02*2:
            return True
        else:
            return False


    def moveJoint(self, joint_states):
        """Move the robot based on the given joint states. This function call is non-blocking.

        Args:
            joint_states (list[float, float, float, float, float, float]): Requested joint states
        """
        print("Target joint states", joint_states)
        self.list_to_setp(joint_states)
        self.con.send(self.setp)

    def movel(self, pos):
        '''
        Perform a linear move to cartesian position pos
        :param pos: cartesian position (x, y, z, Rx, Ry, Rz)
        :return: None, break when the actual position is <2mm from the target position
        '''
        print('target position', pos)
        while True:
            self.list_to_setp(pos)
            self.con.send(self.setp)
            if self.wait(pos):
                break

    # def open_gripper(self):
    #     self.gripper.input_int_register_1 = 2
    #     self.con.send(self.gripper)
    #     time.sleep(2)

    # def close_gripper(self):
    #     self.gripper.input_int_register_1 = 1
    #     self.con.send(self.gripper)
    #     time.sleep(2)

    def angle_antipodal_grasp(self, ang):
        '''
        :param ang: angle in degree to perform an antipodal grasp (Rz=0) in cartesian coordinates (x, y, z, Rx, Ry, Rz)
        :return: Rx, Ry
        '''
        grasp_orientation = [0.0, 1.0]
        ang = ang*np.pi/180
        if ang * np.pi / 180 > np.pi:
            ang = ang - 2 * np.pi
        tool_rotation_angle = ang/2
        tool_orientation = np.asarray(
            [grasp_orientation[0] * np.cos(tool_rotation_angle) - grasp_orientation[1] * np.sin(tool_rotation_angle),
             grasp_orientation[0] * np.sin(tool_rotation_angle) + grasp_orientation[1] * np.cos(tool_rotation_angle),
             0.0]) * np.pi
        return tool_orientation

class KeepAlive(Thread):
    def __init__(self, con):
        super().__init__()
        self.con = con
        self.daemon = True

    def run(self):
        while True:
            state = self.con.receive()
            time.sleep(0.1)

if __name__=="__main__":
    robot = RTDE_urx()
    test = KeepAlive(robot.con)
    test.start()
    # robot.open_gripper()
    ang = 100
    angle = robot.angle_antipodal_grasp(ang)
    while True:
        robot.moveJoint([-1.5, -1.5, 0.0, 0, 0, 0])
        time.sleep(2)
        robot.moveJoint([1.5, 0.0, 0.0, 0, 0, 0])
        # robot.close_gripper()
        # robot.movel([-0.957, -0.089, -0.037, angle[0], angle[1], 0])
        # robot.movel([-0.657, -0.089, -0.037, angle[0], angle[1], 0])
        # robot.movel([-0.657, -0.089, -0.220, angle[0], angle[1], 0])
        # robot.open_gripper()
        # robot.close_gripper()
        # robot.movel([-0.657, -0.089, -0.037, angle[0], angle[1], 0])
        # robot.movel([-0.957, -0.089, -0.037, angle[0], angle[1], 0])
        # robot.movel([-0.957, -0.089, -0.220, angle[0], angle[1], 0])
        # robot.open_gripper()






