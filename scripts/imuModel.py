#!/usr/bin/env python

import rospy
import sys
import math as m
import numpy as np
import rotmat as rm
import constants
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Vector3

def imuModel(r0,p0,y0,w0):

    rospy.init_node('imuModel', anonymous=True)

    pub_ImuMeas=rospy.Publisher('IMU_RotData',Imu, queue_size=10)
    pub_MagField=rospy.Publisher('IMU_MagData',MagneticField,queue_size=10)
    pub_EulerAng=rospy.Publisher('IMU_EulAngData',Vector3,queue_size=10)
    r=rospy.Rate(100)
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
        rpyMeas=Vector3()
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

        rpyMeas.x=rpyOut[0]*180/m.pi
        rpyMeas.y=rpyOut[1]*180/m.pi
        rpyMeas.z=rpyOut[2]*180/m.pi


        pub_ImuMeas.publish(imuMeas)
        pub_MagField.publish(magfldMeas)
        pub_EulerAng.publish(rpyMeas)

        r.sleep()

if __name__=='__main__':
    if len(sys.argv) == 1:
        r0 = 0.0
        p0 = 0.0
        y0 = 0.0

        w0 = np.matrix('0.;0.;0.')
    elif len(sys.argv) < 8:
        r0 = float(sys.argv[1])
        p0 = float(sys.argv[2])
        y0 = float(sys.argv[3])
        wStr = sys.argv[4]+';'+(sys.argv[5]+';'+sys.argv[6])
        w0 = np.matrix(wStr)

    imuModel(r0,p0,y0,w0)


