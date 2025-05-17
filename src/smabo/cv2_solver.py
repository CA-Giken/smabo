#!/usr/bin/env python3

import cv2
import numpy as np
from scipy.spatial.transform import Rotation as Rot
from smabo import tflib 
import copy

def solve(M,P):
  bTg=M[:,:3]
  bRg=Rot.from_quat(M[:,3:7]).as_matrix()
  cTt=P[:,:3]
  cRt=Rot.from_quat(P[:,3:7]).as_matrix()
  R,T=cv2.calibrateHandEye(bRg,bTg,cRt,cTt,method=cv2.CALIB_HAND_EYE_PARK)
  RT=np.vstack((np.hstack((R,T)),np.array([0,0,0,1]).reshape((1,4))))
  return RT
