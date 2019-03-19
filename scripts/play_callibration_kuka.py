#!/usr/bin/env python2

'''
Movement of the wheel fro the r5

@author lukashuber
@date 2019-03-01
'''

import rospy
import rospkg
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import TwistStamped, Twist, Pose, PoseStamped
from std_msgs.msg import Float64MultiArray

import numpy as np
import numpy.linalg as LA
import warnings

from os.path import join

# import matplotlib.pyplot as plt

import json

import warnings
import sys
import signal

# import os
# os.chdir("/home/lukas/catkin_ws/src/wheel_polishing/scripts/")

# from testLines import calculate_cuver

from multiprocessing import Lock

N_JOINTS = 7 # == DIM
# JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               # 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

# DT = 1./25 # fixed DT
DT_INPUT = 0.04 # make sure this is bigger than self.dt_pub

# TODO -- regression algorithm

class ReplayCallibration_KUKA():
    def __init__(self, setUp="", n_loops=0):
        # n_loops: negative number account result in infinite loops
        self.mutex = Lock()
        rospack = rospkg.RosPack()
        rospy.init_node("talker_playBack", anonymous=True)
        
        self.pub_jointPos = rospy.Publisher("/lwr/joint_controllers/command_joint_pos", Float64MultiArray, queue_size=5)

        self.recieved_jointState_msg = False

        # Initialize topics
        self.sub_joint_pos = rospy.Subscriber("/lwr/joint_states", JointState, self.callback_jointState)

        self.freq = 50 # Warning - too high frequence leads to jerky behavior
        self.dt_pub = 1./self.freq
        self.rate = rospy.Rate(self.freq)    # Frequency 10Hz

        self.n_loops = n_loops # number times the points are looped (-1==inf)

        self.data_points = []
        self.pos_next = []
        # with open(join(rospack.get_path("wheel_polishing"), "data", "kuka_lwr_calibration.json")) as json_data:
        # with open(join(rospack.get_path("robot_calibration"), "data", "ridgeback_calibration_v2.json")) as json_data:
        with open(join(rospack.get_path("robot_calibration"), "data", "lwr_calibration.json")) as json_data:

            self.data_points = json.load(json_data)
        # import pdb; pdb.set_trace()

        self.n_points=len(self.data_points) # maximum number of points

        self.dt_loopPrint = 100

        while not (self.recieved_jointState_msg):
            rospy.sleep(0.5)
            print("Waiting for first callbacks...")
            if rospy.is_shutdown():
                print('Shuting down')
                break

        self.it_loop = 0
        self.it_attr = 0

        start_margin = 0.1
        # Move to initial point
        self.update_boundary_conditions()
        print('Going for initial point')
        while not rospy.is_shutdown():
            shutdown = self.update_position()

            dist_attr = self.get_distance_from_attractor()
            if dist_attr < start_margin:
                print('Initial point reached')
                break
            self.it_loop += 1
            if not(self.it_loop%self.dt_loopPrint):
                print('Loop #', self.it_loop)

            if shutdown:
                self.shutdown_command()

            self.rate.sleep()

        print('Initial point reached')
    

    def run(self):
        # TODO more general, to include more points in spline
        print('Starting loop')
        while not rospy.is_shutdown():
            # TODO update spline each loop to have flexible DS
            # self.update_spline()
            # self.update_velocity(vel_limit=0.2)
            shutdown = self.update_boundary_conditions()
            self.update_position()

            if shutdown:
                self.shutdown_command()

            self.it_attr += 1
            self.it_loop += 1

            if not(self.it_loop%self.dt_loopPrint):
                print('Loop #', self.it_loop)
            if not(self.it_attr%self.dt_loopPrint):
                print('Going to attractor #{} of {}'.format(self.it_attr, self.n_points) )
            self.rate.sleep()
        
    def shutdown_command(self, signal, frame):
        # TODO activate after CTRL-c
        print('Shuting down....')

        msg_jointVel = JointTrajectory()
        msg_jointVel.header.stamp = rospy.Time.now()

        msg_jointVel.joint_names = JOINT_NAMES
        # msg_jointVel.points.velocities = des_joint_vel.tolist()

        newPoint = JointTrajectoryPoint()
        newPoint.newPoint.velocities = np.zeros(self.joint_pos).tolist()
        newPoint.newPoint.velocities = np.zeros(N_JOINTS).tolist()
        for ii in range(3): # repeat several times
            msg_jointVel.points.append(newPoint)

        self.pub_jointVel.publish(msg_jointVel)
        
        rospy.signal_shutdown('User shutdown.')
        print('See you next time')

    def update_position(self):
        msg_jointPos = Float64MultiArray()
        # msg_jointPos.layout.dim = N_JOINTS
        msg_jointPos.data = self.pos_next

        self.pub_jointPos.publish(msg_jointPos)

    def get_distance_from_attractor(self):
        delta_pos =  self.pos_next - self.joint_pos
        return LA.norm((delta_pos))
        # TODO Get distance only points which are on the 'wrong' side
        # ind_wrong = (self.pos_boundary[:, 2]-self.joint_pos)*delta_pos < 0
        # return LA.norm((delta_pos[ind_wrong]))

    def update_boundary_conditions(self):
        state_shutdown = False

        if (self.it_attr+1 >= self.n_points):
            if self.n_loops == 0:
                print('Loop limit reached.')
                # self.pos_next = self.
                state_shutdown = True  # shutdown 
            else:
                if self.n_loops > 0:
                    self.n_loops -= 1
                self.it_attr = 0
                
        if not state_shutdown:
            self.pos_next = self.data_points[self.it_attr]['position'][:N_JOINTS]

        return state_shutdown # continue loops


    def callback_jointState(self, msg):
        with self.mutex:
            # TODO apply transform to np.array only when used
            self.joint_pos = np.array(msg.position[:N_JOINTS])
            self.joint_vel = np.array(msg.velocity[:N_JOINTS])
            self.joint_torque = msg.effort[:N_JOINTS]

            if not self.recieved_jointState_msg:
                self.recieved_jointState_msg = True
                print("Recieved first joint state")


if __name__ == '__main__':
    try:
        print('Input argmunt', sys.argv)
        
        ReplayCallibration_instance = ReplayCallibration_KUKA()
        signal.signal(signal.SIGINT, ReplayCallibration_instance.shutdown_command)

        if not rospy.is_shutdown():
            ReplayCallibration_instance.run()
        
    except rospy.ROSInterruptException:
        pass
