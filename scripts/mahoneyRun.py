#!/usr/bin/env python

import rospy
import sys
import math as m
import numpy as np
import rotmat as rm
from sensor_msgs.msg import Imu, MagneticField
from estimate_this.msg import rpy

global maxIt

def accel_measurement(data,args):
    #rospy.loginfo("%f %f %f",data.angular_velocity.x,data.angular_velocity.y,data.angular_velocity.z)

    args[0][0]=data.linear_acceleration.x
    args[0][1]=data.linear_acceleration.y
    args[0][2]=data.linear_acceleration.z

    args[1]=args[1]+1

    i=args[1]

    args[2][i%maxIt]=args[0]

    if i%maxIt == 0:
        xavg=args[2].mean(axis=0)

def mag_measurement(data,args):
    #rospy.loginfo("%f %f %f",data.angular_velocity.x,data.angular_velocity.y,data.angular_velocity.z)

    args[0][0]=data.magnetic_field.x
    args[0][1]=data.magnetic_field.y
    args[0][2]=data.magnetic_field.z

    args[1]=args[1]+1

    i=args[1]

    args[2][i%maxIt]=args[0]


def triadEstimation():
    global maxIt

    rospy.init_node('triadEstimation', anonymous=True)

    i=0
    j=0
    acc=np.zeros(3)
    mag=np.zeros(3)

    accel_avg=np.zeros((maxIt,3))
    mag_avg=np.zeros((maxIt,3))


    args_accel=[acc,i,accel_avg]
    args_mag=[mag,j,mag_avg]

    rospy.loginfo("Running, with max it: %d", maxIt)

    rospy.Subscriber("/imu/data_raw",Imu,accel_measurement,args_accel)
    rospy.Subscriber("/imu/mag",MagneticField,mag_measurement,args_mag)

    pub=rospy.Publisher('triadAtt',rpy, queue_size=10)

    r=rospy.Rate(1000)

    while not rospy.is_shutdown():
        if args_accel[1]% maxIt ==0:
            acc=args_accel[2].mean(axis=0)
            mag=args_mag[2].mean(axis=0)

            if np.linalg.norm(mag,2) != 0 and np.linalg.norm(acc,2) != 0:

                Cinst=rm.triad(acc,mag)
                rllptchyw=rm.RPYfromC(Cinst)
                for i in range(3):
                    rllptchyw[i]=rllptchyw[i]*180./m.pi

                pub.publish(roll=rllptchyw[0],pitch=rllptchyw[1],yaw=rllptchyw[2])


        r.sleep()

if __name__=='__main__':
    global maxIt
    maxIt=int(sys.argv[1])
    triadEstimation()
