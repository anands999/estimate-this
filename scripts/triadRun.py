#!/usr/bin/env python

import rospy
import sys
import math as m
import numpy as np
import rotmat as rm
import constants
import attitude_estimators
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Vector3

global maxIt

def accel_measurement(data,args):
    #rospy.loginfo("%f %f %f",data.angular_velocity.x,data.angular_velocity.y,data.angular_velocity.z)

    args[0][0]=data.linear_acceleration.x
    args[0][1]=data.linear_acceleration.y
    args[0][2]=data.linear_acceleration.z

    args[1]=args[1]+1
#    if args[1] > maxIt
#        args[1] = 0

    i=args[1]

    args[2][i%maxIt]=args[0]


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

    rospy.Subscriber("/IMU_RotData",Imu,accel_measurement,args_accel)
    rospy.Subscriber("/IMU_MagData",MagneticField,mag_measurement,args_mag)

    pub=rospy.Publisher('triadAtt',Vector3, queue_size=10)

    r=rospy.Rate(100)

    while not rospy.is_shutdown():
        if args_accel[1]%maxIt ==0:
            acc=args_accel[2].mean(axis=0)
            mag=args_mag[2].mean(axis=0)

            if np.linalg.norm(mag,2) != 0 and np.linalg.norm(acc,2) != 0:

                Cinst=attitude_estimators.triad(acc,mag)
                rllptchyw=rm.RPYfromC(Cinst)
                for i in range(3):
                    rllptchyw[i]=rllptchyw[i]*180./m.pi

                triadMsg=Vector3()

                triadMsg.x=rllptchyw[0]
                triadMsg.y=rllptchyw[1]
                triadMsg.z=rllptchyw[2]
                print rllptchyw
                pub.publish(triadMsg)


        r.sleep()

if __name__=='__main__':
    global maxIt
    if len(sys.argv) is 1:
        maxIt=100
    else:
        maxIt=int(sys.argv[1])

    print len(sys.argv)

    triadEstimation()
