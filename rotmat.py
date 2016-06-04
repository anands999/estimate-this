# Rotation matrix library

import math as m
import numpy as np

def eye():
    R=np.matrix('1 0 0;0 1 0; 0 0 1')
    return R;

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
        a1=float(a[0])
        a2=float(a[1])
        a3=float(a[2])

    skew=np.matrix('{} {} {}; {} {} {}; {} {} {}'.format(0,-a3,a2,a3,0,-a1,-a2,a1,0))
    return skew

def rotRPY(r,p,y):
    cr=c1(r)
    cp=c2(p)
    cy=c3(y)
    return cy*cp*cr

def unit(x):
    if type(x) is np.ndarray or type(x) is np.array:
        if np.dot(x,x) != 1:
            axis=x/np.linalg.norm(x,2)
        else:
            axis=x

    elif type(x) is np.matrix:
        xT=x.T
        if x.shape[0] is 3 and x.shape[1] is 1:
            dotP=float(np.dot(xT,x))
        else:
            dotP=float(p.dot(x,xT))

       if  dotP != 1:
            axis=x/np.linalg.norm(x,2)
        else:
            axis=x

    return axis

def col(x):
    if x is np.matrix:
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

    C=eye()*c+(1-c)*np.outer(axis,axis)-s*skew(axis)
    return C

def poisson(C,w):
    wX=skew(w)
    Cdot=-wX*C
    return Cdot

def triad(mag, accel):
    x1b=col(unit(mag))
    x2b=col(unit(accel))

    v1b=x1b
    v2b=skew(x1b)*x2b
    v2b=unit(v2b)
    v3b=skew(v1b)*v2b
    v3b=unit(v3b)

    x1a=col(np.array([1,0,0]))
    x2a=col(np.array([0,0,1]))

    v1a=x1a
    v2a=skew(x1a)*x2a
    v2a=unit(v2a)
    v3a=skew(v1a)*v2a
    v3a=unit(v3a)

    C1=np.column_stack((v1b,v2b,v3b))
    C2=np.column_stack((v1a,v2a,v3a))

    C=C1*C2.T
    return C

