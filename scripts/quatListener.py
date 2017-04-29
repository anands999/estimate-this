#!/usr/bin/env python

import rospy
import sys
import math as m
import numpy as np
import rotmat as rm
import sensorProc as sp
import attitude_estimators
import constants as c
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Vector3
from estimate_this.msg import rpy

if rospy.has_param('/sensors/maxIter'):
    maxIt=rospy.get_param('/sensors/maxIter')
else:
    maxIt=1


def quaternionListener(rate, param_namespace):


    if rospy.has_param(param_namespace):
        imu_topic=rospy.get_param(param_namespace+"/imu_topic")
        rpy_topic=rospy.get_param(param_namespace+"/rpy_topic")
    else:
        print "Namespace \""+param_namespace+"\" not found. We're done here."
        sys.exit()

    rospy.init_node('quaternionListener', anonymous=True)

    # set up sensor vectors
    qua=np.mat('1.;0.;0.;0.')

    args_qua=[qua]
    #initialize subscriptions to sensor topics
    rospy.Subscriber(imu_topic,Imu,sp.qua_measurement,args_qua)

    #initiliaze filter output publisher
    rpy_pub=rospy.Publisher(rpy_topic,rpy, queue_size=10)

    r=rospy.Rate(rate)

    while not rospy.is_shutdown():
        qua=args_qua[0]

        if qua.T*qua is not 1.0:

            C=rm.quat2rot(qua,1)

            [rl, p, y] = rm.RPYfromC(C)#
            rpyMsg=Vector3()
            rpyMsg=rpy()

            rpyMsg.roll=rl*c.rad2deg
            rpyMsg.pitch=p*c.rad2deg
            rpyMsg.yaw=y*c.rad2deg
            rpy_pub.publish(rpyMsg)

        r.sleep()

#def mahoney_help():
#    print
#    print "mahoneyRun.py - Mahoney-style complimentary attitude filter. "
#    print " "
#    print "Arguments:"
#    print "  -r:<DOUBLE:rate>            Operating rate of filter. Default: 100 Hz."
#    print "  -i:<INT:interations>        Number of IMU measurements to average before processing. Default is 1. Value of 1 assumes meaurements are coming at the same rate as filter frequency."
#    #print "  -p:<STR:namespace>          Specify node parameter workspace"

if __name__=='__main__':

    rate=100
    maxIt=1
    param_namespace='/quatListener'

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
#                mahoney_help()
                sys.exit()
#            else:
#                print
#                print " Bad argument: \""+y[0]+"\""
#                mahoney_help()
#                sys.exit()
#    print
#    print "         Operating rate: "+str(rate)+" Hz"
#    print "Measurements to average: "+str(maxIt)
#    print "    Parameter namespace: "+param_namespace

    quaternionListener(rate,param_namespace)

