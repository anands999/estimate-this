# Rotation matrix library

import math as m
import numpy as np
import rospy as rp
import constants
from scipy import linalg as la
import collections
import rotmat as rm

estimator_output=collections.namedtuple('Estimator_Output', ['Cdot','bdot'])

StochFilterOutput=collections.namedtuple('StochFilterOut',['xk', 'Pk'])

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

    C=C1*C2.T

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


def rotmatUKF(xkm, Pkm, wk, Qk, Rk, mk, gk, dtk, kappa):
#def rotmatUKF(xkm, Pkm, wk, Qk, dtk, kappa):

    L=3
    KL=float(L)+kappa
    sqrtLK=m.sqrt(KL)

    mu=np.mat(np.zeros([6,1]))

    Sigzz=np.mat(np.zeros([6,6]))
    Sigzz[0:3,0:3]=Pkm
    Sigzz[3:6,3:6]=Qk

    S=la.cholesky(Sigzz)
    S=np.mat(S)

    xk_prop=np.mat(np.zeros([3,1]))
    xks_prop=np.mat(np.zeros([3,2*L+1]))

    mu[0:3]=xkm

    for i in range(2*L+1):
        alpha=0.5/KL
        if i == 2*L:
            z=mu;
            alpha=kappa/KL
        elif i < L:
            z=mu+sqrtLK*S[:,i]
        else:
            z=mu-sqrtLK*S[:,i]

        Ckm = rm.ColToRotMat(z[0:3]+z[3:6])
        Ck_prop = rm.discretePoisson(Ckm, wk,dtk)
        xks_prop[:,i] = rm.RotMatToCol(Ck_prop)
        xk_prop=xk_prop + alpha*xks_prop[:,i]

    Pk_prop=np.mat(np.zeros([L,L]))

    for i in range(2*L+1):
        alpha=0.5/KL
        if i == 2*L:
            alpha=kappa/KL

        err=xks_prop[:,i]-xk_prop
        Pk_prop=Pk_prop+alpha*err*err.T


    # unitized gravity vector
    g0=constants.grav_vec_hat

    # magnetic dipole
    m0=constants.mag_vec_hat

    Cmeas=triad(gk,mk)
    Ckis=np.zeros([3,3,2*L+1])
    Ck_mean=np.mat(np.eye(3))

    mu=np.mat(np.zeros([6,1]))
    mu[0:3]=xk_prop

    for i in range(2*L+1):
        alpha=0.5/KL
        if i == 2*L:
            alpha=kappa/KL
        elif i < L:
            z=mu+sqrtLK*S[:,i]
        else:
            z=mu-sqrtLK*S[:,i]

        Ckm=rm.ColToRotMat(z[0:3])
        mki=Ckm*m0
        gki=Ckm*g0
        Cki=rm.ColToRotMat(z[3:6])*triad(gki,mki)
        Ckis[:,:,i]=np.array(Cki)
        dC=rm.RotMatToCol(Cki)
        rCki=rm.ColToRotMat(alpha*dC)
        Ck_mean=rCki*Ck_mean

    Sigyy=np.mat(np.zeros([3,3]))
    Sigxy=Sigyy

    for i in range(2*L+1):
        alpha=0.5/KL
        if i == 2*L:
            alpha=kappa/KL

        err=xks_prop[:,i]-xk_prop
        Cki=np.mat(Ckis[:,:,i])
        m_err=rm.skewInv(rm.ProjAnti(Cki.T*Ck_mean))

        Sigyy=Sigyy+alpha*(m_err*m_err.T)
        Sigxy=Sigxy+alpha*(err*m_err.T)


    K_gain=Sigxy*la.inv(Sigyy)
    m_err=rm.skewInv(rm.ProjAnti(Cmeas.T*Ck_mean))

    xk=xk_prop+K_gain*m_err
    Pk=Pk_prop-K_gain*Sigxy.T

    ukfOut=StochFilterOutput(xk, Pk)

    return ukfOut
