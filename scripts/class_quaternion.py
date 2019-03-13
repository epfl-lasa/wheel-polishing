#!/usr/bin/env python

'''
Quaternion class for calculation

@author lukashuber
@date 2019-03-01
'''
import sys

import rospy

from geometry_msgs.msg import Twist, Vector3, Quaternion, Pose

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

import numpy as np
import numpy.linalg as LA
from numpy import pi

# from cgkit.cgtypes import quat

import warnings

class quat():
    # quaternion library with q=[q_w, q_x, q_y, q_z]
    def __init__(self, q):
        if LA.norm(q):
            warnings.warn('Zero value quaternion')
        self.q = np.array(q)

    def normalize(self):
        self.q = self.q / LA.norm(self.q)
        
    def inverse(self):
        q_inv = np.array([self.q[0], -self.q[1], -self.q[2], -self.q[3]])
        q_norm = LA.norm(self.q)
        if q_norm:
            return q_inv/q_norm

    def dot(self, r):
        r = np.array(r)
        q_mult = np.zeros(4)
        q_mult[0] = r[0]*self.q[0] - r[1]*self.q[1] - r[2]*self.q[2] - r[3]*self.q[3]
        q_mult[1] = r[0]*self.q[1] + r[1]*self.q[0] - r[2]*self.q[3] + r[3]*self.q[2]
        q_mult[2] = r[0]*self.q[2] + r[1]*self.q[3] + r[2]*self.q[0] - r[3]*self.q[1]
        q_mult[3] = r[0]*self.q[3] - r[1]*self.q[2] + r[2]*self.q[1] + r[3]*self.q[0]
        return q_mult


def get_quaternion_one_vect(vec1 , vec0=[1,0,0], return_quaternion=False):
    normVec1 = LA.norm(vec1)
    if normVec1:
        vec1 = vec1 / normVec1
        
    vec0 = np.array(vec0)
    phi = np.arccos(vec0.dot(vec1))
    vec_perp = np.cross(vec0, vec1)
    
    norm_vecPerp = LA.norm(vec_perp)
    if norm_vecPerp:
        vec_perp = vec_perp/norm_vecPerp

    quat0 = np.zeros(4)
    quat0[0] = np.cos(phi/2.0) # w
    quat0[1:] = vec_perp*np.sin(phi/2.0) # x - y - z

    if return_quaternion:
        return Quaternion(quat0[1], quat0[2], quat0[3], quat0[0])

    return quat0


def get_angVel_from_quat(quat0, quat1, dt, angVel_max=3.0, ang_small=0.2):
    quat0 = quat(quat0)
    quat1 = quat(quat1)
    
    rot_quat = quat1.dot(quat0.inverse())
    Theta = 2*np.arccos(rot_quat[0])
    omega = Theta/dt*rot_quat[1:]/LA.norm(rot_quat[1:])

    norm_omega= LA.norm(omega)
    if norm_omega:
        omega = omega/norm_omega*angVel_max
        vel_des = np.min([norm_omega, angVel_max])*angVel_max/ang_small*omega/norm_omega
    return omega
