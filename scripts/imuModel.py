#!/usr/bin/env python

import rospy
import sys
import math as m
import numpy as np
import rotmat as rm
import constants
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Vector3
from estimate_this.msg import rpy

def imuModel(r0,p0,y0,w0,rate):

    rospy.init_node('imuModel', anonymous=True)

    pub_ImuMeas=rospy.Publisher('/imu/data_raw',Imu, queue_size=10)
    pub_MagField=rospy.Publisher('/imu/mag',MagneticField,queue_size=10)

    pub_EulerAng=rospy.Publisher('/imu/EulAngData',rpy,queue_size=10)

    r=rospy.Rate(rate)
    C=rm.rotRPY(r0,p0,y0)


    g0=constants.grav_vec_hat
    m0=constants.mag_vec_hat

    w=w0

    now=rospy.get_time()

    while not rospy.is_shutdown():
        before=now
        now=rospy.get_time()

        Cdot=rm.poisson(C,w)
        C=rm.eulerIntegrateRotMat(C,Cdot,now-before)

        # Calculate gravity vector in body frame
        graVec=C*g0
        # Calculate magnetic field vector in body frame
        magVec=C*m0


        # Initialise publisher messages, assign values, publish
        imuMeas=Imu()
        magfldMeas=MagneticField()
        imuMeas.header.stamp=rospy.Time.now()
        imuMeas.angular_velocity.x=w[0]
        imuMeas.angular_velocity.y=w[1]
        imuMeas.angular_velocity.z=w[2]

        imuMeas.header.stamp=rospy.Time.now()
        imuMeas.linear_acceleration.x=graVec[0]
        imuMeas.linear_acceleration.y=graVec[1]
        imuMeas.linear_acceleration.z=graVec[2]

        magfldMeas.header.stamp=rospy.Time.now()
        magfldMeas.magnetic_field.x=magVec[0]
        magfldMeas.magnetic_field.y=magVec[1]
        magfldMeas.magnetic_field.z=magVec[2]
#        print C[0]
#        print C[1]
#        print C[2]

        rpyOut=rm.RPYfromC(C)


        rpyMeas=rpy()
#        rpyMeas.x=rpyOut[0]*180/m.pi
#        rpyMeas.y=rpyOut[1]*180/m.pi
#        rpyMeas.z=rpyOut[2]*180/m.pi

        rpyMeas.roll = rpyOut[0]*180/m.pi
        rpyMeas.pitch = rpyOut[1]*180/m.pi
        rpyMeas.yaw = rpyOut[2]*180/m.pi
        rpyMeas.stamp=rospy.get_rostime()


        pub_ImuMeas.publish(imuMeas)
        pub_MagField.publish(magfldMeas)
        pub_EulerAng.publish(rpyMeas)

        r.sleep()

def imuModel_help():
    print
    print "imuModel_help.py - IMU model. Provides grav and mag measurements. "
    print " "
    print "Arguments:"
    print "  -e:<r,p,y>        Initial attitude given in Euler angles [rad]."
    print "  -w:<wx,wy,wz>     Initial angular velocity [rad/s]"

if __name__=='__main__':

    r0 = 0.0
    p0 = 0.0
    y0 = 0.0
    w0 = np.matrix('0.;0.;0.')
    rate=100


    if len(sys.argv) == 1:
        imuModel_help()
        sys.exit()
    elif len(sys.argv) > 1:
        for x in sys.argv:
            y=x.split(':')
            if y[0] == "-e":
                rpyVec = y[1].split(",")
                r0 = float(rpyVec[0])*constants.rad2deg
                p0 = float(rpyVec[1])*constants.rad2deg
                y0 = float(rpyVec[2])*constants.rad2deg

            elif y[0] == "-w":
                w0 = np.matrix(y[1].replace(",",";",2))
            elif y[0] == "-r":
                if y[1]>=1:
                    rate=y[1]
                else:
                    rate=100

            elif y[0] == "-h" or  y[0] == "-help":
                imuModel_help()
                sys.exit()
    print
    print "Initial roll, pitch, yaw: ", r0, p0, y0, " rad"
    print "Initial angular velocity: ", w0.item(0), w0.item(1), w0.item(2), " rad/s"
    print "          Operating rate: ", rate, " Hz"

    imuModel(r0,p0,y0,w0,rate)



