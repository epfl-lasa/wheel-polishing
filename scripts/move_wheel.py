#!/usr/bin/env python

'''
Movement of the wheel

@author lukashuber
@date 2019-03-01
'''

import sys

import rospy

from geometry_msgs.msg import Twist, Vector3, Quaternion, Pose

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

# from std_msgs.msg import Float64MultiArray
import std_msgs.msg as std_msg

from sensor_msgs.msg import JointState

import numpy as np
import numpy.linalg as LA
from numpy import pi

# from cgkit.cgtypes import quat

import warnings
from class_quaternion import *

# TODO - replace with other data structure
controlModes = {"joint_pos":0,
                "passive_ds":1,
                "gravity_comp":2}

n_joints = 7

class MoveWheel():
    def __init__(self, setUp=""):
        # Initialize variables -- currently constant obstacle geometry
        rospy.init_node("talker_robotWheel", anonymous=True)

        # obs_pub = rospy.Publisher("chatter_robotWheel", Twist, queue_size=10)
        self.pub_vel = rospy.Publisher("/lwr/joint_controllers/passive_ds_command_vel", Twist, queue_size=10)
        self.pub_orient = rospy.Publisher("/lwr/joint_controllers/passive_ds_command_orient", Quaternion, queue_size=10)

        self.pub_debug_ds = rospy.Publisher("/debug_ds", Path, queue_size=10)
        self.pub_debug_pose = rospy.Publisher("/debug_pose", PoseStamped, queue_size=10)
        self.pub_joint_pos = rospy.Publisher("/lwr/joint_controllers/command_joint_pos", std_msg.Float64MultiArray, queue_size=10)

        self.pub_gravity_comp = rospy.Publisher("/lwr/joint_controllers/command_grav", std_msg.Bool)

        # First subscriber call back variable
        self.recieved_pose_msg = False
        self.recieved_jointState_msg = False

        # self.active_controlMode = "passive_ds"
        self.active_controlMode = "joint_pos"

        # Initialize topics
        sub_ee_pose = rospy.Subscriber("/lwr/ee_pose", Pose, self.callback_pose)
        sub_jointState = rospy.Subscriber("/lwr/joint_states", JointState, self.callback_jointState)
        sub_controlMode = rospy.Subscriber("/robot_wheel/control_mode", std_msg.String, self.callback_controlMode)

        self.freq = 200
        self.dt = 1./self.freq
        rate = rospy.Rate(self.freq)    # Frequency 10Hz

        self.position_robo = 0
        self.quat_robo = 0

        self.goal_point = np.array([0.95, 0, 0.70])
        self.goal_jointPose = np.array([0, 0.5, 0, 1.9, 0, 0, 0]) # Canlde position

        
        while not (self.recieved_pose_msg and
                   self.recieved_jointState_msg):
            rospy.sleep(0.1)
            print("Waiting for first callback...")
        print("Got all messages")
        
        # Create publisher
        self.it_attr = 0     # only for linear 
        itCount = 0

        print('Start wheel movement.')
        # Entering main loop
        while not rospy.is_shutdown():

            if not (self.active_controlMode in controlModes):
                warnings.warn("Wrong control mode")

            if controlModes[self.active_controlMode] == controlModes["passive_ds"]:
                self.update_passive_ds()

            if controlModes[self.active_controlMode] == controlModes["joint_pos"]:
                self.update_joint_pos()

            if controlModes[self.active_controlMode] == controlModes["gravity_comp"]:
                self.pub_gravity_comp.publish(std_msg.Bool(True))
                
            # obs_poly.header.stamp = rospy.Time.now()
            # obs_poly.header.frame_id = obs[nn].frame_id

            # Publish new frame to new frame

            rate.sleep()  # Wait zzzz*

            if not(itCount % 100):
                print('Loop #{}'.format(itCount))

                if controlModes[self.active_controlMode] == controlModes["passive_ds"]:
                    print('vel des:', self.vel_des)
                    # print('angvel des:', ang_vel)
                
                    print('norm vel des:', LA.norm(self.vel_des) )
                    print('pos:', self.position_robo)
                
                # print('Distance to attactor:', self.get_distance_from_attractor())
                # print('attractor:', self.attractors[:, self.it_attr])
            itCount += 1

        self.shutdown_command()
        

    def shutdown_command(self):
        # Publish several time // TODO closed loop
        for ii in range(10):
            zeroVel = Twist()
            # print(tuple(self.vel_des))
            # newVel.angular = Vector3(tuple(self.vel_des))
            zeroVel.linear = Vector3(0, 0, 0)
            zeroVel.angular = Vector3(0, 0, 0)
        
            self.pub_vel.publish(zeroVel)
            rospy.sleep(0.1)

    def update_passive_ds(self):
        # vel_des = self.compute_ds_linearAttractor_const()
        self.vel_des = self.compute_ds_ellipsoid_motion()

        newVel = Twist()
        # print(tuple(self.vel_des))
        # newVel.angular = Vector3(tuple(self.vel_des))
        newVel.linear = Vector3(self.vel_des[0], self.vel_des[1], self.vel_des[2])
        # newVel.linear = Vector3(0,0,0)

        # quat_goal = get_quaternion_one_vect(self.goal_point-self.position_robo)
        # ang_vel = get_angVel_from_quat(quat_goal, self.quat_robo, dt=1.0/freq)
        # newVel.angular = Vector3(ang_vel[0], ang_vel[1], ang_vel[2])
        newVel.angular = Vector3(0, 0, 0)

        self.pub_vel.publish(newVel)

        self.newOrient = Quaternion(1,0,0,0)
        self.newOrient = get_quaternion_one_vect(self.goal_point-self.position_robo,
                                                 vec0 = [0,0,1],
                                                 return_quaternion=True)

        # print('new orient', newOrient)
        self.pub_orient.publish(self.newOrient)


        newPath = Path()
        newPath.header.frame_id = "world"
        newPath.header.stamp = rospy.Time.now()

        newPose = PoseStamped()
        newPose.header = newPath.header
        newPose.pose.position = Vector3(self.position_robo[0],
                                        self.position_robo[1],
                                        self.position_robo[2])
        newPath.poses.append(newPose)

        # newPose = PoseStamped()
        # newPose.header = newPath.header
        # dt = 1.0
        # x2 = self.position_robo + vel_des*dt
        # newPose.pose.position = Vector3(x2[0], x2[1], x2[2])
        # newPath.poses.append(newPose)

                        

        newPose = PoseStamped()
        newPose.header = newPath.header
        # x2 = self.position_robo + vel_des*dt
        newPose.pose.position = Vector3(self.goal_point[0],
                                        self.goal_point[1],
                                        self.goal_point[2])
        newPath.poses.append(newPose)

        self.pub_debug_ds.publish(newPath)

        debugPose = PoseStamped()
        debugPose.header = newPath.header
        debugPose.pose.position = Vector3(self.position_robo[0],
                                          self.position_robo[1],
                                          self.position_robo[2])

        debugPose.pose.orientation = self.newOrient

        self.pub_debug_pose.publish(debugPose)

    
    def update_joint_pos(self, vel_limit=1.0, P=10, I=0, D=0, time_fac=100):
        # TODO - monitor forces
        des_joint_vel = (self.goal_jointPose - self.joint_pos)*P

        max_des_vel = np.max(np.abs(des_joint_vel))

        des_joint_vel = np.min([vel_limit/max_des_vel, max_des_vel])*des_joint_vel

        # TODO -- directly publish to robot velocity 
        des_new_pos = self.joint_pos + des_joint_vel*(self.dt*time_fac)

        msg_jointPos = std_msg.Float64MultiArray()
        msg_jointPos.data = des_new_pos.tolist()

        self.pub_joint_pos.publish(msg_jointPos)

    
    def get_distance_from_attractor(self):
        return LA.norm((self.attractors[:, self.it_attr] - self.position_robo))
    
    
    def compute_ds_linearAttractor_const(self, velConst=0.3, distSlow=0.2):
        self.attractors = np.array([[0, 0.0, 1.16],
                                    [0, 0.5, 0.5],
                                    [0.5, 0., 0.5],
                                    [0.0, -0.5, 0.5]]).T
        
        if (self.get_distance_from_attractor() < 0.20): # Adapting attractor
            self.it_attr = (self.it_attr + 1) % self.attractors.shape[1]
        
        vel_des = (self.attractors[:, self.it_attr] - self.position_robo)

        dist_attr = LA.norm(vel_des)

        vel_des = np.min([dist_attr, distSlow])*velConst/distSlow*vel_des/dist_attr

        return vel_des

    def compute_ds_circular_motion(self, rad_circle=0.2, **kwargs):
        self.compute_ds_circular_motion(axis_ellipse=np.ones(2)*rad_circle, **kwargs)

    def compute_ds_ellipsoid_motion(self, center_pos=[0.2, 0.0, 0.9], beta=1, sigma=10, zeta=1, vel_const=0.2, axis_ellipse=[0.2,0.1], vel_mag=0.4, pow_elli=2):
        # TODO rotation?
        pos_rel = self.position_robo - np.array(center_pos)
        pos_rel[1:] = 1./np.array(axis_ellipse) * pos_rel[1:]
        
        normPos_yz = LA.norm(pos_rel[1:])
        # normPos_yz = np.sqrt(np.sum((pos_rel[1:]/np.array(axis_ellipse))**pow_elli))
        if normPos_yz:
            delta_rad = (normPos_yz - 1)/normPos_yz

        vel_des = np.zeros(3)
        vel_des[0] = - pos_rel[0]
        vel_des[1] = - pos_rel[2] - sigma*pos_rel[1]*delta_rad
        vel_des[2] =  pos_rel[1] - sigma*pos_rel[2]*delta_rad

        vel_des[1:] = np.array(axis_ellipse)*vel_des[1:]

        vel_norm = LA.norm(vel_des)
        if vel_norm:
            vel_des = vel_des/vel_norm

        return vel_des * vel_mag

    # def compute_des_quat(self, )
        # print('not implemented')
        
    def callback_pose(self, msg):
        self.position_robo =  np.array([msg.position.x,
                                        msg.position.y,
                                        msg.position.z])
        
        self.quat_robo = np.array([msg.orientation.w,
                                   msg.orientation.x,
                                   msg.orientation.y,
                                   msg.orientation.z])
        
        self.recieved_pose_msg = True

    def callback_jointState(self, msg):
        self.joint_pos = msg.position[:n_joints]
        self.joint_vel = msg.velocity[:n_joints]
        self.joint_torque = msg.effort[:n_joints]

        self.recieved_jointState_msg = True


    def callback_controlMode(self, msg):
        if msg.data in controlModes:
            print("Switched from control mode <<{}>> to <<{}>>".format(self.active_controlMode, msg.data))
            self.active_controlMode = msg.data
        else:
            print("Suggested control <<{}>> does not exist.".format(msg.data))
            print("Possible control modes are:")
            print(controlModes.keys())
            print("\nWe stay in <<{}>>-mode.".format(self.active_controlMode))
    
if __name__ == '__main__':
    try:
        print('Input argmunt', sys.argv)
        
        MoveWheel('conveyerBelt_basket')
        
    except rospy.ROSInterruptException:
        pass
