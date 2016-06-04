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
    a1=a[0]
    a2=a[1]
    a3=a[2]
    skew=np.matrix('{} {} {}; {} {} {}; {} {} {}'.format(0,-a3,a2,a3,0,-a1,-a2,a1,0))
    return skew

def rotRPY(r,p,y):
    cr=c1(r)
    cp=c2(p)
    cy=c3(y)
    return cy*cp*cr

def rotEulAx(theta, ax_in):
    if np.dot(ax_in,ax_in) != 1:
        axis=ax_in/np.linalg.norm(ax_in,2)
    else:
        axis=ax_in

    c=np.cos(theta)
    s=np.sin(theta)

    C=eye()*c+(1-c)*np.outer(axis,axis)-s*skew(axis)
    return C

def poisson(C,w):
    wX=skew(w)
    Cdot=-wX*C
    return Cdot




