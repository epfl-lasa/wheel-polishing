#!/usr/bin/env python2

'''
Class to visualize additional tools based on simple geometrical objtects.
TODO: Create more sophisticated models.

@author Lukas Huber
@date 2019-03-22

'''

import rospy
import rospkg

import numpy as np
import numpy.linalg as LA

from geometry_msgs.msg import Point, Quaternion, Vector3
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA

import numpy as np
import numpy.linalg as LA

import copy

# print("I will define everything for you")

class ShowTools():
    def __init__(self, nodeInit=True):

        print("Let us start this adventure together")
        # if nodeInit:
        rospy.init_node("talker_toolVisualizer", anonymous=True)
        print("I prepared a node")
        self.pub_toolMarker_ur5 = rospy.Publisher("/ur5/tool_marker", MarkerArray, queue_size=5)
        self.pub_toolMarker_kuka = rospy.Publisher("/lwr/tool_marker", MarkerArray, queue_size=5)
        self.pub_marker_table = rospy.Publisher("/lwr/table", MarkerArray, queue_size=5)

        self.set_up_wheel()
        self.set_up_polisher()
        self.set_up_table()

    def run(self):
        self.freq = 10
        self.rate = rospy.Rate(self.freq)    

        self.it_count = 0
        print("Starting run")
        while not rospy.is_shutdown():
            self.update()
            
            if not (self.it_count % 10):
                print("Iteration #{}".format(self.it_count))
            self.it_count += 1
            self.rate.sleep()


    def update(self):
        timeNow = rospy.Time.now()
        for mm in range(len(self.markers_wheel.markers)):
            self.markers_wheel.markers[mm].header.stamp = timeNow
        self.pub_toolMarker_ur5.publish(self.markers_wheel)

        for mm in range(len(self.markers_polisher.markers)):
            self.markers_polisher.markers[mm].header.stamp = timeNow
        self.pub_toolMarker_kuka.publish(self.markers_polisher)

        for mm in range(len(self.markers_table.markers)):
            self.markers_table.markers[mm].header.stamp = timeNow
        self.pub_marker_table.publish(self.markers_table)


        
    def set_up_polisher(self):
        self.markers_polisher = MarkerArray()
        
        id_it = 0
        dz = 0

        polisher_part = Marker()
        polisher_part.header.frame_id = "/lwr_7_link"
        polisher_part.ns = "lwr"
        polisher_part.id = id_it
        polisher_part.type = Marker().CYLINDER
        polisher_part.action = Marker().ADD
        polisher_part.scale = Vector3(0.085, 0.085, 0.08)
        polisher_part.color = ColorRGBA(100./265, 100./265, 100./265, 1)
        polisher_part.pose.orientation = Quaternion(0,0,0,1)
        polisher_part.lifetime = rospy.Duration.from_sec(0)
        polisher_part.frame_locked = True

        dz = polisher_part.scale.z/2
        polisher_part.pose.position = Point(0, 0, dz)
        self.markers_polisher.markers.append(polisher_part)
        dz += polisher_part.scale.z/2
        
        # id_it += 1
        # polisher_part = copy.deepcopy(polisher_part)
        # polisher_part.id = id_it
        # polisher_part.type = Marker().CUBE
        # polisher_part.scale = Vector3(0.0275, 0.05, 0.08)
        # dz += polisher_part.scale.z/2
        # polisher_part.pose.position = Point(0,0,dz)
        # self.markers_polisher.markers.append(polisher_part)
        # dz += polisher_part.scale.z/2

        # id_it += 1
        # polisher_part = copy.deepcopy(polisher_part)
        # polisher_part.id = id_it
        # polisher_part.type = Marker().CUBE
        # polisher_part.scale = Vector3(0.019, 0.0275, 0.040)
        # dz += polisher_part.scale.z/2
        # polisher_part.pose.position = Point(0,0,dz)
        # self.markers_polisher.markers.append(polisher_part)
        # dz += polisher_part.scale.z/2

        id_it += 1
        polisher_part = copy.deepcopy(polisher_part)
        polisher_part.id = id_it
        polisher_part.type = Marker().CYLINDER
        polisher_part.scale = Vector3(0.03, 0.03, 0.03)
        dz += polisher_part.scale.z/2
        polisher_part.pose.position = Point(0,0,dz)
        self.markers_polisher.markers.append(polisher_part)
        dz += polisher_part.scale.z/2

        id_it += 1
        polisher_part = copy.deepcopy(polisher_part)
        polisher_part.id = id_it
        polisher_part.type = Marker().CYLINDER
        polisher_part.scale = Vector3(0.01, 0.01, 0.05)
        dz += polisher_part.scale.z/2
        polisher_part.pose.position = Point(0,0,dz)
        self.markers_polisher.markers.append(polisher_part)
        dz += polisher_part.scale.z/2

        id_it += 1
        polisher_part = copy.deepcopy(polisher_part)
        polisher_part.id = id_it
        polisher_part.type = Marker().CYLINDER
        polisher_part.scale = Vector3(0.15, 0.15, 0.01)
        dz += polisher_part.scale.z/2
        polisher_part.pose.position = Point(0,0,dz)
        polisher_part.color = ColorRGBA(192./265, 192./265, 192./265, 1)
        self.markers_polisher.markers.append(polisher_part)
        dz += polisher_part.scale.z/2

        # import pdb; pdb.set_trace()
        
        
    def set_up_table(self):
        self.markers_table = MarkerArray()
        
        it_id = 0

        table_part = Marker()
        table_part.header.frame_id = "/world"
        table_part.ns = "table"
        table_part.id = it_id
        table_part.type = Marker().CUBE
        table_part.action = Marker().ADD
        table_part.scale = Vector3(0.88, 0.88, 0.03)
        colors = np.array([112,118,130])/256.
        table_part.color = ColorRGBA(colors[0], colors[1], colors[2], 1)
        table_part.pose.orientation = Quaternion(0,0,0,1)
        table_part.lifetime = rospy.Duration.from_sec(0)
        table_part.frame_locked = True

        dz =  -table_part.scale.z/2
        table_part.pose.position = Point(0, 0, dz)
        self.markers_polisher.markers.append(table_part)
        dz -= table_part.scale.z/2

        self.markers_table.markers.append(table_part)

        table_part = copy.deepcopy(table_part)
        table_part.scale = Vector3(0.08, 0.08, 0.73)
        
        d_xy = 0.88/2-0.10 
        dz -= table_part.scale.z/2.0
        
        for ii in range(4):
            it_id += 1
            table_part = copy.deepcopy(table_part)
            dx = ((ii<2)-0.5)*2 * d_xy
            dy = ((ii%2)-0.5)*2 * d_xy
            # print('ii', ii)
            # print("ix:{}, iy:{}\n".format(dx, dy))
            table_part.id = it_id
            table_part.pose.position = Vector3(dx, dy, dz)
            self.markers_table.markers.append(table_part)
            # print('ii', ii)
            

    def set_up_wheel (self):
        self.markers_wheel = MarkerArray()
        
        it_id = 0

        wheel_part = Marker()
        wheel_part.header.frame_id = "/robotiq_force_torque_frame_id"
        wheel_part.ns = "ur5"
        wheel_part.id = it_id
        wheel_part.type = Marker().CYLINDER
        wheel_part.action = Marker().ADD
        # wheel_part.scale = Vector3(0.08, 0.08, 0.035)
        wheel_part.scale = Vector3(0.08, 0.08, 0.06)
        wheel_part.color = ColorRGBA(100./265, 100./265, 100./265, 1)
        wheel_part.pose.orientation = Quaternion(0,0,0,1)
        wheel_part.lifetime = rospy.Duration.from_sec(0)
        wheel_part.frame_locked = True
        self.markers_wheel.markers.append(wheel_part)

        dz = wheel_part.scale.z/2
        wheel_part.pose.position = Point(0, 0, dz)
        dz += wheel_part.scale.z/2
        
        it_id += 1
        wheel_part = copy.deepcopy(wheel_part)
        wheel_part.id = it_id
        wheel_part.scale = Vector3(0.40, 0.40, 0.001)
        wheel_part.color = ColorRGBA(70./265, 70./265, 70./265, 1.)
        dz += wheel_part.scale.z/2
        wheel_part.pose.position = Point(0, 0, dz)
        dz += wheel_part.scale.z/2
        self.markers_wheel.markers.append(wheel_part)

        it_id += 1
        wheel_part = copy.deepcopy(wheel_part)
        wheel_part.id = it_id
        wheel_part.scale = Vector3(0.40, 0.40, 0.014)
        wheel_part.color = ColorRGBA(130./265, 130./265, 130./265, 1.)
        dz += wheel_part.scale.z/2
        wheel_part.pose.position = Point(0, 0, dz)
        dz += wheel_part.scale.z/2
        self.markers_wheel.markers.append(wheel_part)

if __name__ == '__main__':
    try:
        # if len(sys.argv == 1)
        # print('Input argmunts {}'.format(sys.argv[1:]))
        # if 
        print("Hello master. I will be publishing the tools for you.")
        ShowTools_instance = ShowTools()

        # signal.signal(signal.SIGINT, MoveWheelTrajectory_instance.shutdown_command)
        
        print("Let me get this running.")
        if not rospy.is_shutdown():
            ShowTools_instance.run()
        
    except rospy.ROSInterruptException:
        print("Could not be run.")
        pass
