#!/usr/bin/env python

import rospy

if rospy.has_param('/sensors/maxIter'):
    maxIt=rospy.get_param('/sensors/maxIter')
else:
    maxIt=1

def imu_measurement(data,args):

    args[0][0]=data.linear_acceleration.x
    args[0][1]=data.linear_acceleration.y
    args[0][2]=data.linear_acceleration.z

    args[1][0]=data.angular_velocity.x
    args[1][1]=data.angular_velocity.y
    args[1][2]=data.angular_velocity.z

    args[2]=args[2]+1

    i=args[2]

    args[3][i%maxIt]=args[0]
    args[4][i%maxIt]=args[1]
    args[5]=data.header.stamp.secs+data.header.stamp.nsecs/1e9

def mag_measurement(data,args):

    args[0][0]=data.magnetic_field.x
    args[0][1]=data.magnetic_field.y
    args[0][2]=data.magnetic_field.z

    args[1]=args[1]+1

    i=args[1]

    args[2][i%maxIt]=args[0]

