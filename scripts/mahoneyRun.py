#!/usr/bin/env python

import rospy
import sys
import math as m
import numpy as np
import rotmat as rm
import attitude_estimators
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Vector3

global maxIt

def imu_measurement(data,args):
    #rospy.loginfo("%f %f %f",data.angular_velocity.x,data.angular_velocity.y,data.angular_velocity.z)

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
    #rospy.loginfo("%f %f %f",data.angular_velocity.x,data.angular_velocity.y,data.angular_velocity.z)

    args[0][0]=data.magnetic_field.x
    args[0][1]=data.magnetic_field.y
    args[0][2]=data.magnetic_field.z

    args[1]=args[1]+1

    i=args[1]

    args[2][i%maxIt]=args[0]

def mahoneyEstimator(rate):
    global maxIt

    rospy.init_node('mahoneyEstimator', anonymous=True)

    i=0
    j=0
    acc=np.zeros(3)
    mag=np.zeros(3)
    ang_vel=np.zeros(3)

    accel_avg=np.zeros((maxIt,3))
    angvel_avg=np.zeros((maxIt,3))
    mag_avg=np.zeros((maxIt,3))
    imuTime=0.
    magTime=0.

    args_imu=[acc,ang_vel,i,accel_avg,angvel_avg, imuTime]
    args_mag=[mag,j,mag_avg,magTime]

    rospy.loginfo("Running, with max it: %d", maxIt)

    rospy.Subscriber("/IMU_RotData",Imu,imu_measurement,args_imu)
    rospy.Subscriber("/IMU_MagData",MagneticField,mag_measurement,args_mag)


    pub=rospy.Publisher('mahoneyAtt',Vector3, queue_size=10)

    r=rospy.Rate(rate)

    Cea=np.eye(3)
    bhat=np.matrix('0;0;0')

    now=rospy.get_time()

    while not rospy.is_shutdown():
        if args_imu[2]% maxIt ==0:
            acc=np.matrix(args_imu[3].mean(axis=0)).T
            w_y_a=np.matrix(args_imu[4].mean(axis=0)).T
            mag=np.matrix(args_mag[2].mean(axis=0)).T

            if np.linalg.norm(mag,2) != 0 and np.linalg.norm(acc,2) != 0:

                Cba=attitude_estimators.triad(acc,mag)
                rllptchyw=rm.RPYfromC(Cba)
                for i in range(3):
                    rllptchyw[i]=rllptchyw[i]*180./m.pi

                derivatives=attitude_estimators.mahoneyPoisson(Cea, Cba, bhat, w_y_a)
                before=now
                now=rospy.get_time()

                dT=now-before
                Cdot=np.matrix(derivatives.Cdot)

                Cea=rm.eulerIntegrateRotMat(Cea,Cdot, dT)

                bhat=bhat+derivatives.bdot*dT

                rllptchyw=rm.RPYfromC(Cea)
                for i in range(3):
                    rllptchyw[i]=rllptchyw[i]*180./m.pi

#                print rllptchyw

                mahoneyMsg=Vector3()

                mahoneyMsg.x=rllptchyw[0]
                mahoneyMsg.y=rllptchyw[1]
                mahoneyMsg.z=rllptchyw[2]
                pub.publish(mahoneyMsg)


        r.sleep()

def mahoney_help():
    print
    print "mahoneyRun.py - Mahoney-style complimentary attitude filter. "
    print " "
    print "Arguments:"
    print "  -r:<rate>            Operating rate of filter. Default: 100 Hz."
    print "  -i:<interations>     Number of IMU measurements to average before processing. Default is 1. Value of 1 assumes meaurements are coming at the same rate as filter frequency."

if __name__=='__main__':
    global maxIt

    rate=100
    maxIt=1

    if len(sys.argv) > 1:
        sys.argv.pop(0)
        for x in sys.argv:
            y=x.split(':')
            if y[0] == "-r":
                rate=dobule(y[1])
            elif y[0] == "-i":
                matIx=integer(y[1])
            elif y[0] == "-h" or  y[0] == "-help":
                mahoney_help()
                sys.exit()
            else:
                print
                print " Bad argument: \""+y[0]+"\""
                mahoney_help()
                sys.exit()
    print
    print "         Operating rate: "+str(rate)+" Hz"
    print "Measurements to average: "+str(maxIt)

    mahoneyEstimator(rate)

