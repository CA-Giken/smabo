#!/usr/bin/env python3

import numpy as np
import open3d as o3d
import copy
import time

Param={
  "normal_radius":10,
  "icp_threshold":1,
  "eval_threshold":0,
}

score={"transform":np.eye(4),"fitness":0,"rmse":0}

def toNumpy(pcd):
  return np.reshape(np.asarray(pcd.points),(-1,3))

def fromNumpy(dat):
  pc=o3d.geometry.PointCloud()
  pc.points=o3d.utility.Vector3dVector(dat)
  return pc

def add_normal(cloud):
  cloud.estimate_normals(o3d.geometry.KDTreeSearchParamRadius(radius=Param["normal_radius"]))
  viewpoint=np.array([0.0,0.0,0.0],dtype=float)
  cloud.orient_normals_towards_camera_location(camera_location=viewpoint)

def solve(source,target,prm,RT):
  global Param,score
  Param.update(prm)
  radius=Param["normal_radius"]
  if radius>0:
    add_normal(source)
    add_normal(target)
    method=o3d.pipelines.registration.TransformationEstimationPointToPlane()
  else:
    method=o3d.pipelines.registration.TransformationEstimationPointToPoint()
  threshold=Param["icp_threshold"]
  if threshold>0:
    result=o3d.pipelines.registration.registration_icp(source,target,threshold,RT,method)
    score["transform"]=result.transformation
    score["fitness"]=result.fitness
    score["rmse"]=result.inlier_rmse
  elif Param["eval_threshold"]>0:
    result=o3d.pipelines.registration.evaluate_registration(source,target,Param["eval_threshold"],RT)
    score["transform"]=RT
    score["fitness"]=result.fitness
    score["rmse"]=result.inlier_rmse
  return score

