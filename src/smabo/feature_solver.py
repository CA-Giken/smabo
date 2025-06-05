#!/usr/bin/env python3

import numpy as np
import open3d as o3d
import copy
import time

Param={
  "normal_radius":0.01,
  "normal_min_nn":0,
  "feature_mesh":0.002,
  "feature_radius":0.025,
  "feature_fitness":0.5,
  "distance_threshold":0,
  "eval_threshold":0,
  "repeat":1
}

modCloud=o3d.geometry.PointCloud()
modFeat=None
score={"transform":np.eye(4),"fitness":0}

def toNumpy(pcd):
  return np.reshape(np.asarray(pcd.points),(-1,3))

def fromNumpy(dat):
  pc=o3d.geometry.PointCloud()
  pc.points=o3d.utility.Vector3dVector(dat)
  return pc

def _get_features(cloud):
  o3d.geometry.PointCloud.estimate_normals(cloud,o3d.geometry.KDTreeSearchParamRadius(radius=Param["normal_radius"]))
  viewpoint=np.array([0.0,0.0,0.0],dtype=float)
  o3d.geometry.PointCloud.orient_normals_towards_camera_location(cloud, camera_location=viewpoint)
#  nfmin=Param["normal_min_nn"]
#  if nfmin<=0: nfmin=1
#  cl,ind=o3d.geometry.PointCloud.remove_radius_outlier(cloud,nb_points=nfmin,radius=Param["normal_radius"])
#  nfcl=o3d.geometry.PointCloud.select_by_index(cloud,ind)
#  cloud.points=nfcl.points
#  cloud.normals=nfcl.normals
  cds=cloud
  if Param["feature_mesh"]>0:
    cds=o3d.geometry.PointCloud.voxel_down_sample(cloud,voxel_size=Param["feature_mesh"])
  return cds,o3d.pipelines.registration.compute_fpfh_feature(cds,o3d.geometry.KDTreeSearchParamRadius(radius=Param["feature_radius"]))

def learn(cloud,prm):
  global modCloud,modFeat
  modCloud,modFeat=_get_features(cloud)
  return modCloud

def solve(cloud,prm):
  global Param
  print("ransac_solver::solve start")
  Param.update(prm)
  scnCloud,scnFeat=_get_features(cloud)
  t1=time.time()
  score={"transform":np.eye(4),"fitness":0,"rmse":0}
  distance_threshold=Param["feature_threshold"]
  if distance_threshold>0:
    result=o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
      modCloud,scnCloud,modFeat,scnFeat,True,distance_threshold,
      o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
      3, [
        o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
        o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)
      ], o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999))
    score["transform"]=result.transformation
    score["fitness"]=result.fitness
    score["rmse"]=result.inlier_rmse
  print("feature_solver::fitness",score["fitness"],len(modCloud.points),len(scnCloud.points))
  return score

