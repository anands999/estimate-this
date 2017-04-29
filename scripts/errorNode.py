#!/usr/bin/env python

import rospy
import sys
import math as m
import numpy as np
import rotmat as rm
import constants
import attitude_estimators
from geometry_msgs.msg import Vector3
from estimate_this.msg import rpy


def GetAttitude(data,args):

    args[0][0] = data.roll
    args[0][1] = data.pitch
    args[0][2] = data.yaw
    args[1] = data.stamp



def errorNode(TrueTopic,EstTopic,rate,param_namespace):


    rospy.init_node('AttErrorNode', anonymous=True)

    if rospy.has_param(param_namespace):
        error_topic=rospy.get_param(param_namespace+"/error_topic")
    else:
        print "ErrorNode: Namespace \""+param_namespace+"\" not found. We're done here."
        sys.exit()

    rpy_true=np.zeros(3)
    rpy_ttime=rospy.get_rostime()
    rpy_est=np.zeros(3)
    rpy_etime=rospy.get_rostime()


    args_TrueAtt = [rpy_true, rpy_ttime]
    args_EstAtt = [rpy_est, rpy_etime]

    #initialize subscriptions to sensor topics
    rospy.Subscriber(TrueTopic,rpy,GetAttitude,args_TrueAtt)
    rospy.Subscriber(EstTopic,rpy,GetAttitude,args_EstAtt)

    #initiliaze filter output publisher
    error_pub=rospy.Publisher(error_topic,Vector3, queue_size=10)

    r=rospy.Rate(rate)

    while not rospy.is_shutdown():
        ErrMsg=Vector3()
        time_tol=rospy.get_param(param_namespace+"/time_tol/")
#        if (rpy_etime.secs == rpy_ttime.secs) and (abs(rpy_etime.nsecs - rpy_ttime.nsecs) < time_tol) :
        ErrMsg.x=(rpy_est[0]-rpy_true[0])
        ErrMsg.y=(rpy_est[1]-rpy_true[1])
        ErrMsg.z=(rpy_est[2]-rpy_true[2])

        error_pub.publish(ErrMsg)

        r.sleep()


if __name__=='__main__':

    truetopic="/imu/EulAngData"
    esttopic="/mahoney_fil/RPY"
    rate=100
    param_namespace='/calcError'

    if len(sys.argv) > 1:
        sys.argv.pop(0)
        for x in sys.argv:
            y=x.split(':')
            # set filter operating rate
            if y[0] == "-r":
                rate=float(y[1])
            # set filter measurement averaging value
            elif y[0] == "-t":
                truetopic=y[1]
            elif y[0] == "-e":
                esttopic=y[1]
            # set parameter namespace
            elif y[0] == "-p":
                param_namespace=y[1]
            elif y[0] == "-h" or  y[0] == "-help":
                mahoney_help()
                sys.exit()
    print
    print "    True Attitude Topic: "+truetopic
    print "    Est. Attitude Topic: "+esttopic
    print "         Operating rate: "+str(rate)+" Hz"
    print "    Parameter namespace: "+param_namespace

    errorNode(truetopic,esttopic,rate,param_namespace)

