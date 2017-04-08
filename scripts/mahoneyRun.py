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

def mag_measurement(data,args):
    #rospy.loginfo("%f %f %f",data.angular_velocity.x,data.angular_velocity.y,data.angular_velocity.z)

    args[0][0]=data.magnetic_field.x
    args[0][1]=data.magnetic_field.y
    args[0][2]=data.magnetic_field.z

    args[1]=args[1]+1

    i=args[1]

    args[2][i%maxIt]=args[0]

def mahoneyEstimator():
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


    args_imu=[acc,ang_vel,i,accel_avg,angvel_avg]
    args_mag=[mag,j,mag_avg]

    rospy.loginfo("Running, with max it: %d", maxIt)

    rospy.Subscriber("/IMU_RotData",Imu,imu_measurement,args_imu)
    rospy.Subscriber("/IMU_MagData",MagneticField,mag_measurement,args_mag)


    pub=rospy.Publisher('mahoneyAtt',Vector3, queue_size=10)

    r=rospy.Rate(100)

    Cea=np.eye(3)
    bhat=np.matrix('0;0;0')

    now=rospy.get_time()

    while not rospy.is_shutdown():
        if args_imu[2]% maxIt ==0:
            acc=args_imu[3].mean(axis=0)
            w_y_a=args_imu[4].mean(axis=0)
            mag=args_mag[2].mean(axis=0)
            if np.linalg.norm(mag,2) != 0 and np.linalg.norm(acc,2) != 0:

                Cba=attitude_estimators.triad(acc,mag)
                rllptchyw=rm.RPYfromC(Cba)
                for i in range(3):
                    rllptchyw[i]=rllptchyw[i]*180./m.pi

                print rllptchyw

#                Cba=Cba.transpose(1,0)

                derivatives=attitude_estimators.mahoneyPoisson(Cba, Cea, bhat, w_y_a)
                before=now
                now=rospy.get_time()
                dT=now-before
                Cdot=np.matrix(derivatives.Cdot)

#                print Cdot
#                print Cea


                Cea=rm.eulerIntegrateRotMat(Cea,Cdot, dT)
#                sys.exit()
                bhat=bhat+derivatives.bdot*dT

                rllptchyw=rm.RPYfromC(Cea)
                for i in range(3):
                    rllptchyw[i]=rllptchyw[i]*180./m.pi

                mahoneyMsg=Vector3()

                mahoneyMsg.x=rllptchyw[0]
                mahoneyMsg.y=rllptchyw[1]
                mahoneyMsg.z=rllptchyw[2]
                pub.publish(mahoneyMsg)


        r.sleep()

if __name__=='__main__':
    global maxIt
    if len(sys.argv) is 1:
        maxIt=100
    else:
        maxIt=int(sys.argv[1])
    print len(sys.argv)
    mahoneyEstimator()

