#!/usr/bin/env python

import rospy
import math as m
import numpy as np
import rotmat

#if rospy.has_param('/earth/mag_vec_ned'):
#    mag_vec=rospy.get_param('/earth/mag_vec_ned')
#    mag_vec=np.asmatrix(mag_vec).T
#else:
mag_vec=np.matrix('18886.3; -2349.7; 50389.2')

mag_vec_hat=rotmat.unit(mag_vec)

#if rospy.has_param('/earth/grav_vec_ned'):
#    grav_vec=rospy.get_param('/earth/grav_vec_ned')
#    grav_vec=np.asmatrix(grav_vec).T
#else:
grav_vec=np.matrix('0.0; 0.0; 9.81')

grav_vec_hat=rotmat.unit(grav_vec)

# just so they're in one place
rad2deg=np.pi/180.0
deg2rad=1.0/rad2deg


