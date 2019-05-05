#!/usr/bin/env python
import rospy
import rospkg
from sensor_msgs.msg import JointState
from multiprocessing import Lock
from os.path import join
from os.path import exists
from os import makedirs
import json


class Recorder(object):
    def __init__(self):
        rospack = rospkg.RosPack()
        self.data_dir = join(rospack.get_path("wheel_polishing"), "data")
        if not exists(self.data_dir):
            makedirs(self.data_dir)

        self.recording = False
        self.recorded_data = []
        self.lock = Lock()
        
    def start_recording(self):
        with self.lock:
            self.recording = True
            self.recorde_data = []

    def stop_recording(self, savefile):
        savefile = join(self.data_dir, savefile + ".json")
        with self.lock:
            self.recording = False
            with open(savefile, 'w') as f:
                json.dump(self.recorded_data, f, indent=4)
        return savefile

    def joint_state_callback(self, msg):
        if self.recording:
            data = {}
            data["name"] = msg.name
            data["position"] = msg.position
            data["velocity"] = msg.velocity
            data["effort"] = msg.effort

            with self.lock:
                self.recorded_data.append(data)

        
