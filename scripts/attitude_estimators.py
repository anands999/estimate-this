# Rotation matrix library

import math as m
import numpy as np
import rospy as rp
import constants
from scipy import linalg as la
import collections
import rotmat as rm

estimator_output=collections.namedtuple('Estimator_Output', ['Cdot','bdot'])

def triad(accel,mag):

    x2b=rm.col(rm.unit(mag))
    x1b=rm.col(rm.unit(accel))

    v1b=x1b
    v2b=rm.skew(x1b)*x2b
    v2b=rm.unit(v2b)
    v3b=rm.skew(v1b)*v2b
    v3b=rm.unit(v3b)

    # unitized gravity vector
    x1a=constants.grav_vec_hat

    # magnetic dipole
    x2a=constants.mag_vec_hat


    v1a=x1a
    v2a=rm.skew(x1a)*x2a
    v2a=rm.unit(v2a)
    v3a=rm.skew(v1a)*v2a
    v3a=rm.unit(v3a)

    C1=np.column_stack((v1b,v2b,v3b))
    C2=np.column_stack((v1a,v2a,v3a))

    C=C1*C2.transpose(1,0)

    return C


def mahoneyPoisson(Cea, Cba, bhat, w_y_a, gains):


    kp=gains[0]
    Kbias=gains[1]
    Kbias=np.asmatrix(np.diag(Kbias))


    Cbe=Cba*Cea.T


    CbeTr=np.trace(Cbe)
    PaCbe=rm.skewInv(rm.ProjAnti(Cbe))
    eVec=-1.0/(1.0+CbeTr)*PaCbe

    CeaDot=-1.0*rm.skew(w_y_a-bhat+kp*eVec)*Cea

    bhatDot=-Kbias*eVec

    mahoneyOut=estimator_output(Cdot=CeaDot,bdot=bhatDot)
    return mahoneyOut
