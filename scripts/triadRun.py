#!/usr/bin/env python

import rospy
import sys
import math as m
import numpy as np
import rotmat as rm
from sensor_msgs.msg import Imu, MagneticField

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
#        rospy.loginfo("%f %f %f %d", xavg[0],xavg[1],xavg[2], args[1])

def mag_measurement(data,args):
    #rospy.loginfo("%f %f %f",data.angular_velocity.x,data.angular_velocity.y,data.angular_velocity.z)

    args[0][0]=data.magnetic_field.x
    args[0][1]=data.magnetic_field.y
    args[0][2]=data.magnetic_field.z

    args[1]=args[1]+1

    i=args[1]

    args[2][i%maxIt]=args[0]

#    if i%maxIt == 0:
#        xavg=args[2].mean(axis=0)
#        rospy.loginfo("%f %f %f %d", xavg[0],xavg[1],xavg[2], args[1])


def listener():
    global maxIt

    rospy.init_node('listener', anonymous=True)

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

    r=rospy.Rate(1000)

    Cmag=np.matrix('6.909327e-01,-2.146449e-03,6.848350e-03;-2.146449e-03, 6.751378e-01, 2.575075e-03; 6.848350e-03, 2.575075e-03, 7.385365e-01')
    bvec=np.array([-3.005217e1,1.328892e2,-6.558522e1])/1e9

    while not rospy.is_shutdown():
        if args_accel[1]% maxIt ==0:
            acc=args_accel[2].mean(axis=0)
            mag=args_mag[2].mean(axis=0)

            if np.linalg.norm(mag,2) != 0 and np.linalg.norm(acc,2) != 0:

                Cinst=rm.triad(acc,mag)
                rpy=rm.RPYfromC(Cinst)
                for i in range(3):
                    rpy[i]=rpy[i]*180./m.pi

                rospy.loginfo("%f %f %f", rpy[0], rpy[1], rpy[2])

#        rospy.loginfo("%d",args_accel[1])
        r.sleep()

if __name__=='__main__':
    global maxIt
    maxIt=int(sys.argv[1])
    listener()
