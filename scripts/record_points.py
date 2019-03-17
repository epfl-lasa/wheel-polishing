#!/usr/bin/env python2

'''
Record points of the wheel from

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

from os.path import join, isfile

# import matplotlib.pyplot as plt

import json

import warnings
import sys
import signal

import datetime

# from robot_calbration.src.robot_calbration.recorder import Recorder
# src.robot_calbration.recorder import Recorder
from recorder import Recorder


# import os
# os.chdir("/home/lukas/catkin_ws/src/wheel_polishing/scripts/")

# from testLines import calculate_cuver

from multiprocessing import Lock

N_JOINTS = 6 # == DIM
# MODES_PROGRAM = {"c":0,
                # "passive_ds":1,
                # "gravity_comp":2}

JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']

# DT = 1./25 # fixed DT
DT_INPUT = 0.04 # make sure this is bigger than self.dt_pub


class RecordJointsStates():
    def __init__(self, topicName="ur5/joint_states", topicType=JointState, n_loops=0):
        print("starting Recording")
        self.start_time = datetime.datetime.now().strftime('%Y%m%d-%H%M%S')
        rospy.init_node("listener_positionMeasure", anonymous=True)

        self.rospack = rospkg.RosPack()
        # n_loops: negative number account result in infinite loops
        self.mutex = Lock()

        self.recording_state = 0
        
        self.data_recorder = Recorder()
        # self.data_recorder = Recorder(joint_state_channel ="/)
        rospy.Subscriber("ur5/joint_states", JointState, self.callback_jointState)
        
        # Initialize topics
        self.recorded_data = []

        self.freq = 10 # Warning - too high frequence leads to jerky behavior
        self.rate = rospy.Rate(self.freq)    # Frequency 10Hz

        print('Node initialized')


    def run(self):
        print("Hello Master. We are ready to record positiosns.")
        print("Please enter a filename without specification: ")
        print("default name is <<recording[DATE-TIME].txt>>)")
        self.file_name = raw_input("")
        if not len(self.file_name):
            self.file_name = "recoring"+self.start_time+".txt"

        self.data_recorder.start_recording()

        while not rospy.is_shutdown():
            print("Enter command to continue: r(ecording) [==default] - q(uit)")

            self.recording_state = raw_input("")

            if len(self.recording_state)>1:
                self.recording_state = self.recording_state[:1]
            elif len(self.recording_state)==0:
                self.recording_state = "r" # default record

            while (self.recording_state=="r" 
                   and not rospy.is_shutdown()):
                print("Waiting for message.")
                rospy.sleep(0.2)

            if self.recording_state == "q":
                break
            elif (self.recording_state == 0):
                print("Recording  succesful.")
            else:
                print("The command was not recognized.")

            self.rate.sleep()

        print("Saving to file")
        self.data_recorder.stop_recording(savefile=self.file_name)
        print("Script finished")

    def callback_jointState(self, msg):
        if self.recording_state=="r":
            self.data_recorder.joint_state_callback(msg)
            print("Point saved")

            self.recording_state = 0


if __name__ == '__main__':
    try:
        print('Input argmunt', sys.argv)
        
        RecordJointsStates_instance = RecordJointsStates('conveyerBelt_basket')
        # signal.signal(signal.SIGINT, ReplayCallibration_instance.shutdown_command)
        if not rospy.is_shutdown():
            RecordJointsStates_instance.run()
        
    except rospy.ROSInterruptException:
        pass
