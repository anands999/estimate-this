#!/usr/bin/env python

import rospy
import sys
import math as m
import numpy as np
import rotmat as rm
import attitude_estimators
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Vector3
import sensorProc as sP

if rospy.has_param('/sensors/maxIter'):
    maxIt=rospy.get_param('/sensors/maxIter')
else:
    maxIt=1

def SigmaPointFilter(rate, param_namespace):

    # get parameters
#    filter_gains=[]
#
    if rospy.has_param(param_namespace):
        imu_topic=rospy.get_param(param_namespace+"/imu_topic")
        mag_topic=rospy.get_param(param_namespace+"/mag_topic")
        rpy_topic=rospy.get_param(param_namespace+"/rpy_est_topic")
        bias_topic=rospy.get_param(param_namespace+"/bias_est_topic")
    else:
        print "Namespace \""+para_namespace+"\" not found. We're done here."
        sys.exit()

    rospy.init_node('SigmaPointFilter', anonymous=True)

    # set up sensor vectors
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

    #initialize subscriptions to sensor topics
    rospy.Subscriber(imu_topic,Imu,sP.imu_measurement,args_imu)
    rospy.Subscriber(mag_topic,MagneticField,sP.mag_measurement,args_mag)

    #initiliaze filter output publisher
    rpy_pub=rospy.Publisher(rpy_topic,Vector3, queue_size=10)
    bias_pub=rospy.Publisher(bias_topic,Vector3, queue_size=10)

    r=rospy.Rate(rate)

    #Initial state
    xk=np.mat('0.;0.;0.');
    #Initial covariance
    Pk=np.mat(np.eye(3))*1e-4

    Qk=np.mat(np.eye(3))*1e-6
    Rk=np.mat(np.eye(3))*1e-4

    now=rospy.get_time()

    while not rospy.is_shutdown():
        if args_imu[2]% maxIt ==0:
            acc=np.matrix(args_imu[3].mean(axis=0)).T
            w_y_a=np.matrix(args_imu[4].mean(axis=0)).T
            mag=np.matrix(args_mag[2].mean(axis=0)).T

            if np.linalg.norm(mag,2) != 0 and np.linalg.norm(acc,2) != 0:


#                derivatives=attitude_estimators.mahoneyPoisson(Cea, Cba, bhat, w_y_a,filter_gains)


                before=now
                now=rospy.get_time()

                dT=now-before

                ukfOut=attitude_estimators.rotmatUKF(xk, Pk, w_y_a, Qk, Rk, mag, acc, dT, 4.0)
                xk=ukfOut.xk
                Pk=ukfOut.Pk

                Ck=rm.ColToRotMat(xk)
                rllptchyw=rm.RPYfromC(Ck)
                for i in range(3):
                    rllptchyw[i]=rllptchyw[i]*180./m.pi


                print rllptchyw
                rpyMsg=Vector3()

                rpyMsg.x=rllptchyw[0]
                rpyMsg.y=rllptchyw[1]
                rpyMsg.z=rllptchyw[2]
                rpy_pub.publish(rpyMsg)



        r.sleep()

def ukf_help():
    print
    print "ukfRun.py - Sigma point filter for attitude estimation. "
    print " "
    print "Arguments:"
    print "  -r:<DOUBLE:rate>            Operating rate of filter. Default: 100 Hz."
    print "  -i:<INT:interations>        Number of IMU measurements to average before processing. Default is 1. Value of 1 assumes meaurements are coming at the same rate as filter frequency."
    print "  -p:<STR:namespace>          Specify node parameter workspace"

if __name__=='__main__':

    rate=100
    param_namespace='/ukf_filter'

    if len(sys.argv) > 1:
        sys.argv.pop(0)
        for x in sys.argv:
            y=x.split(':')
            # set filter operating rate
            if y[0] == "-r":
                rate=float(y[1])
            # set filter measurement averaging value
            elif y[0] == "-i":
                matIx=int(y[1])
            # set parameter namespace
            elif y[0] == "-p":
                param_namespace=y[1]
            elif y[0] == "-h" or  y[0] == "-help":
                mahoney_help()
                sys.exit()
#            else:
#                print
#                print " Bad argument: \""+y[0]+"\""
#                mahoney_help()
#                sys.exit()
    print
    print "         Operating rate: "+str(rate)+" Hz"
    print "Measurements to average: "+str(maxIt)
    print "    Parameter namespace: "+param_namespace

    SigmaPointFilter(rate,param_namespace)

