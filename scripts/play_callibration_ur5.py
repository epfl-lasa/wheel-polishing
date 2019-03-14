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

N_JOINTS = 6 # == DIM
JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']


# DT = 1./25 # fixed DT
DT_INPUT = 0.04 # make sure this is bigger than self.dt_pub

# TODO -- regression algorithm

class ReplayCallibration():
    def __init__(self, setUp="", n_loops=0):
        # n_loops: negative number account result in infinite loops
        self.mutex = Lock()
        rospack = rospkg.RosPack()
        rospy.init_node("talker_playBack", anonymous=True)
        
        self.pub_jointVel = rospy.Publisher("/ur5/ur_driver/joint_speed", JointTrajectory, queue_size=5)

        # First subscriber call back variable
        self.recieved_eeVel_msg = False
        self.recieved_jointState_msg = False

        # Initialize topics
        self.sub_joint_pos = rospy.Subscriber("/ur5/joint_states", JointState, self.callback_jointState)

        self.freq = 50 # Warning - too high frequence leads to jerky behavior
        self.dt_pub = 1./self.freq
        self.rate = rospy.Rate(self.freq)    # Frequency 10Hz

        self.n_loops = n_loops # number times the points are looped (-1==inf)

        self.data_points = []
        # with open(join(rospack.get_path("robot_calibration"), "data", "ridgeback_calibration_v2.json")) as json_data:
        with open(join(rospack.get_path("robot_calibration"), "data", "ur5_calibration.json")) as json_data:
            self.data_points = json.load(json_data)

        self.n_points=len(self.data_points) # maximum number of points

        print('Node initialized')

    
    def run(self):
        self.attr_margin = 0.1 # set margin
        self.time_attr = rospy.get_time()
        goal_attr_reached = False
        self.spline_factors = np.zeros((N_JOINTS, 4)) # spline 4 order

        while not (self.recieved_jointState_msg):
            rospy.sleep(0.5)
            print("Waiting for first callbacks...")
            if rospy.is_shutdown():
                print('Shuting down')
                break
        # TODO more general, to include more points in spline
        self.pos_boundary = np.tile(self.joint_pos, (2,1)).T
        self.vel_boundary = np.zeros((N_JOINTS, 2))
        
        self.it_attr = 0
        self.it_loop = 0

        while not rospy.is_shutdown():
            # TODO update spline each loop to have flexible DS
            self.update_spline()
            self.update_velocity(vel_limit=0.2)

            if self.check_if_attractor_reached:
                print('Reached attractor #{} of {}'.format(self.it_attr, self.n_points))
                if goal_attr_reached:
                    print('Movement is finished.')
                    break
                
                self.it_attr += 1
                self.time_attr = rospy.get_time()

                goal_attr_reached = self.update_boundary_conditions()
                
            self.it_loop += 1

            if not(self.it_loop%100):
                print('Loop #', self.it_loop)
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
        
    def update_spline(self):
        # Create a curve  of the form spline approximation ORDER == 4
        # x(t) = c_0 + c_1*t + + c_2*t^2 + + c_3*t^3
        self.pos_boundary[:, 0] = self.joint_pos
        self.vel_boundary[:, 0] = self.joint_vel

        self.spline_factors[:, 0] = self.pos_boundary[:, 0]
        self.spline_factors[:, 1] = self.vel_boundary[:, 0]

        # Time to next goal
        dt = np.max([self.time_attr+DT_INPUT-rospy.get_time(), self.dt_pub*1.0])
        c_matr = np.array(([[dt**2, dt**3],
                            [2*dt, 3*dt**2]]))
        c_matr_inv = LA.inv(c_matr)
        
        for ii in range(N_JOINTS):
            # TODO parallel caclulation for speed
            c_vect = np.array(([self.pos_boundary[ii, 1]-self.spline_factors[ii,0]-self.spline_factors[ii,1]*dt, self.vel_boundary[ii, 1]-self.spline_factors[ii,1]]))

            c_vect = c_matr_inv.dot(c_vect)
            self.spline_factors[ii,2] = c_vect[0]
            self.spline_factors[ii,3] = c_vect[1]

    def update_velocity(self, vel_limit=0.1):
        des_joint_vel = self.get_interpolated_velocity()

        max_des_vel = np.max(np.abs(des_joint_vel)) 
        des_joint_vel = np.min([vel_limit/max_des_vel, max_des_vel])*des_joint_vel

        # Create message
        msg_jointVel = JointTrajectory()
        msg_jointVel.header.stamp = rospy.Time.now()
        msg_jointVel.joint_names = JOINT_NAMES
        newPoint = JointTrajectoryPoint()
        newPoint.positions = (self.joint_pos + self.dt_pub*des_joint_vel).tolist()
        newPoint.velocities = des_joint_vel.tolist()

        # if not(self.it_loop%5):
            # print('des_joint', des_joint_vel)
            # print('actual_joint', self.joint_vel)
            # print('next via point', self.pos_boundary[:, 1])

        # !!! WARNING !!! The robot input message and control message have a different mapping. 
        # I.e. a mapping needs to be applied before sending commands as follows (0<->2):
        # joint mapping [ur5/joint_states -> ur5/ur_driver/joint_speed]
        # ur5_arm_shoulder_pan_joint=2 -> 0
        # 1 -> 1
        # ur5_arm_shoulder_lift_shoulder=0 -> 2
        # 3 -> 3
        # 4 -> 4
        # 5 -> 5
        # input_map  = np.array([0,1,2,3,4,5])
        # output_map = np.array([2,1,0,3,4,5])
        # msg_jointVel.joint_names[output_map] = msg_jointVel.joint_names[input_map]
        # newPoint.positions[output_map] = newPoint.positions[input_map]
        # newPoint.velocities[output_map] = newPoint.velocities[input_map]
        
        msg_jointVel.joint_names[0], msg_jointVel.joint_names[2] = msg_jointVel.joint_names[2], msg_jointVel.joint_names[0]
        newPoint.positions[0], newPoint.positions[2] = newPoint.positions[2], newPoint.positions[0]
        newPoint.velocities[0], newPoint.velocities[2] = newPoint.velocities[2], newPoint.velocities[0]

        msg_jointVel.points.append(newPoint)
        
        self.pub_jointVel.publish(msg_jointVel)
            
    def get_interpolated_velocity(self):
        dt = self.dt_pub
        pos_next = (self.spline_factors[:,0] + self.spline_factors[:,1]*dt
                    + self.spline_factors[:,2]*dt**2+ self.spline_factors[:,3]*dt**3)

        vel = (pos_next - self.pos_boundary[:, 0]) / dt

        # TODO RK4 interpolation > not really, since in position.
        # TODO LIMIT maximum speed
        return vel

    def check_if_attractor_reached(self):
        if get_distance_from_attractor(self) < self.dist_attr:
            return False
        else:
            return True

        # Alternatively
        self.started_at_lower_pos = np.copysign(np.ones(N_JOINTS), (self.pos_boundary[:,1]-self.joint_pos))         


    def get_distance_from_attractor(self):
        delta_pos =  self.pos_boundary[:, 1] - self.joint_pos
        return LA.norm((delta_pos))
        # TODO Get distance only points which are on the 'wrong' side
        # ind_wrong = (self.pos_boundary[:, 2]-self.joint_pos)*delta_pos < 0
        # return LA.norm((delta_pos[ind_wrong]))

    def update_boundary_conditions(self):
        # return  (normal=0) -- (shutdown=1)
        state_shutdown = False
        if (self.it_attr+1 >= self.n_points):
            if self.n_loops == 0:
                print('Shutdown')
                self.vel_boundary[:, 1] = np.zeros(N_JOINTS)
                self.pos_boundary[:, 1] = self.pos_boundary[:, 0] 
                state_shutdown = True  # shutdown 
            else:
                if self.n_loops > 0:
                    self.n_loops -= 1
                self.it_attr = 0

        if not state_shutdown:
            if type(self.data_points[self.it_attr]) == dict:
                self.pos_boundary[:, 1] = self.data_points[self.it_attr]['position'][:N_JOINTS]
                self.vel_boundary[:, 1] = self.data_points[self.it_attr]['velocity'][:N_JOINTS]

        # Check on which side of the new attractor the robot started
        self.started_at_lower_pos = np.copysign(np.ones(N_JOINTS), (self.pos_boundary[:,1]-self.joint_pos)) 
        # self.ind_non
        
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
        
        ReplayCallibration_instance = ReplayCallibration('conveyerBelt_basket')
        signal.signal(signal.SIGINT, ReplayCallibration_instance.shutdown_command)

        if not rospy.is_shutdown():
            ReplayCallibration_instance.run()
        
    except rospy.ROSInterruptException:
        pass
