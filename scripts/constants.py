#!/usr/bin/env python

import math as m
import numpy as np
import rotmat

mag_vec=np.matrix('18886.3; -2349.7; 50389.2')
mag_vec_hat=rotmat.unit(mag_vec)

grav_vec=np.matrix('0.0; 0.0; 9.81')
grav_vec_hat=rotmat.unit(grav_vec)

rad2deg=np.pi/180.0

deg2rad=1.0/rad2deg


