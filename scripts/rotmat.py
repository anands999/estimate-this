# Rotation matrix library

import math as m
import numpy as np
import rospy as rp
from scipy import linalg as la
import collections

estimator_output=collections.namedtuple('Estimator_Output', ['Cdot','bdot'])

def eye():
    R=np.matrix('1 0 0;0 1 0; 0 0 1')
    return R

def c1(a):
    c=np.cos(a)
    s=np.sin(a)
    c1=np.matrix('{} {} {}; {} {} {}; {} {} {}'.format(1,0,0,0,c,s,0,-s,c))
    return c1

def c2(a):
    c=np.cos(a)
    s=np.sin(a)
    c2=np.matrix('{} {} {}; {} {} {}; {} {} {}'.format(c,0,-s,0,1,0,s,0,c))
    return c2

def c3(a):
    c=np.cos(a)
    s=np.sin(a)
    c3=np.matrix('{} {} {}; {} {} {}; {} {} {}'.format(c,s,0,-s,c,0,0,0,1))
    return c3

def skew(a):
    if type(a) is np.ndarray:
        a1=a[0]
        a2=a[1]
        a3=a[2]
    elif type(a) is np.matrix:
        a1=a.item(0)
        a2=a.item(1)
        a3=a.item(2)

    skew=np.matrix('{} {} {}; {} {} {}; {} {} {}'.format(0.,-a3,a2,a3,0.,-a1,-a2,a1,0.))
    return skew

def skewInv(A):
    vec=np.matrix('0.;0.;0.')

    if type(A) is np.matrix:
        vec[0]=-A.item(5)
        vec[1]=A.item(2)
        vec[2]=-A.item(1)
    elif type(A) is np.ndarray:
        vec[0]=-A[1][2]
        vec[1]=A[0][2]
        vec[2]=-A[0][1]
    return vec

def ProjAnti(A):
    if type(A) is np.matrix:
        return 0.5*(A-A.T)

def rotRPY(r,p,y):
    cr=c1(r)
    cp=c2(p)
    cy=c3(y)
    return cy*cp*cr

def unit(x):
    if type(x) is np.ndarray:
        xproc=np.asmatrix(x)
    else:
        xproc=x

    xT=xproc.T

    if xproc.shape[0] is 3 and xproc.shape[1] is 1:
        dotP=float(np.dot(xT,xproc))
    else:
        dotP=float(np.dot(xproc,xT))

    axis=x/np.sqrt(dotP)
    return axis

def col(x):
    if type(x) is np.matrix:
        if x.shape[0] is 3 and x.shape[1] is 1:
            xout=x
        else:
            xout=x.T
    else:
        xout=np.matrix(x).T
    return xout

def rotEulAx(theta, ax_in):

    axis=unit(ax_in)

    c=np.cos(theta)
    s=np.sin(theta)

    C=eye()*c+(1.-c)*np.outer(axis,axis)-s*skew(axis)
    return C

def poisson(C,w):
    wX=skew(w)


    Cdot=-wX*C

#    print Cdot[0]
#    print Cdot[1]
#    print Cdot[2]

    return Cdot

def discretePoisson(Ckm,w,dt):
    Ck=la.expm(skew(-w*dt))*Ckm
    return Ck

def RPYfromC(C):
    zero_tol=1e-8
    if C is not np.matrix and C.size is not 9:
        print 'Matrix not the right size'
        sys.exit()
    else:
        s1c2=C.item(2,1)#-float(C[2][1])
        c1c2=C.item(2,2) #float(C[2][2])
        if (abs(s1c2) > zero_tol) and (abs(c1c2) > zero_tol):
            roll=np.arctan2(s1c2,c1c2)
            c1=np.cos(roll)
            s2=C.item(2,0) #float(C[2][0])

            if abs(c1) > zero_tol:
                c2=c1c2/c1
                pitch=np.arctan2(s2,c2)
            else:
                pitch=np.arcsin(s2)

            c2c3=C.item(0,0)  #float(C[0][0])
            c2s3=C.item(1,0) #float(C[1][0])
            yaw=np.arctan2(c2s3,c2c3)

        else:
            s2=C.item(2,0) #float(C[2][0])
            pitch=np.arcsin(s2)
            roll=np.arctan(C.item(0,1)/C.item(1,1))
            yaw=0.0


    return [roll, pitch, yaw]

def eulerIntegrateRotMat(C,Cdot,dt):
    C=C+Cdot*dt

    if abs(la.det(C)-1.0) > 1e-4:
        Ctst=np.real(la.inv(la.sqrtm(C*C.transpose(1,0))))
        Cout=np.matrix(Ctst)*C
    else:
        Cout=C
    return Cout

def ColToRotMat(col):
    C=la.expm(skew(col))
    C=np.mat(C)
    return C

def RotMatToCol(C):
    col=skewInv(la.logm(C))
    return col

def matDot(x):
    if x.shape[0] ==1:
        dotP=x*x.T
    else:
        dotP=x.T*x

    return float(dotP)

def matOuter(x,y):
    if (x.shape[0] == 1) and (y.shape[0] == 1):
        out = x.T*y
    elif (x.shape[0] == 1) and (y.shape[0] > 1):
        out = x.T*y.T
    elif (x.shape[0] > 1) and (y.shape[0] > 1):
        out = x*y.T
    elif (x.shape[0] > 1) and (y.shape[0] == 1):
        out = x*y
    else:
        print 'inputs are wrong dimensions'
        out=-1
    return out

def quat2rot(q,sf):
    if sf == 1:
        eta=float(q.item(0))
        eps=q[1:4]
    else:
        eta=float(q.item(3))
        eps=q[0:3]

    C=(eta*eta-matDot(eps))*eye()
    C=C+2.0*matOuter(eps,eps)-2.0*eta*skew(eps)
    return C
