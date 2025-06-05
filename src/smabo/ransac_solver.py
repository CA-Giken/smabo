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
  "distance_threshold":0,
  "icp_threshold":0.003,
  "eval_threshold":0,
  "repeat":1
}

modFtArray=[]
modPcArray=[]
scnFtArray=[]
scnPcArray=[]
score={"transform":[np.eye(4)],"fitness":[None],"rmse":[None]}

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
#  cds=cloud
  if Param["feature_mesh"]>0:
    cds=o3d.geometry.PointCloud.voxel_down_sample(cloud,voxel_size=Param["feature_mesh"])
  else:
    cds=cloud
  return cds,o3d.pipelines.registration.compute_fpfh_feature(cds,o3d.geometry.KDTreeSearchParamRadius(radius=Param["feature_radius"]))

def learn(datArray,prm):
  global modPcArray
  modPcArray=[]
  for dat in datArray:
    pc=fromNumpy(dat)
    modPcArray.append(pc)
  return modPcArray

def solve(datArray,prm):
  global scnFtArray,scnPcArray,Param,score
  print("ransac_solver::solve start")
  Param.update(prm)
  t1=time.time()
  modFtArray=list(map(_get_features,modPcArray))
  scnPcArray=[]
  for dat in datArray:
    pc=fromNumpy(dat)
    scnPcArray.append(pc)
  scnFtArray=list(map(_get_features,scnPcArray))
  tfeat=time.time()-t1
  print("time for calc feature",tfeat)
  t1=time.time()
  if Param["repeat"]!=len(score["transform"]):
    n=Param["repeat"]
    score={"transform":[np.eye(4)]*n,"fitness":[None]*n,"rmse":[None]*n}
  distance_threshold=Param["distance_threshold"]
  for n in range(Param["repeat"]):
    if distance_threshold>0:
      result=o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        modFtArray[0][0],scnFtArray[0][0],modFtArray[0][1],scnFtArray[0][1],True,distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        3, [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)
        ], o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999))
      score["transform"][n]=result.transformation
      score["fitness"][n]=result.fitness
      score["rmse"][n]=result.inlier_rmse
    if Param["icp_threshold"]>0:
      result=o3d.pipelines.registration.registration_icp(
        modPcArray[0],scnPcArray[0],
        Param["icp_threshold"],
        result.transformation,o3d.pipelines.registration.TransformationEstimationPointToPlane())
      score["transform"][n]=result.transformation
      score["fitness"][n]=result.fitness
      score["rmse"][n]=result.inlier_rmse
    if Param["eval_threshold"]>0:
      result=o3d.pipelines.registration.evaluate_registration(modPcArray[0],scnPcArray[0],Param["eval_threshold"],score["transform"][n])
      score["fitness"].append(result.fitness)
      score["rmse"].append(result.inlier_rmse)
  tmatch=time.time()-t1
  print("ransac_solver::time for feature matching",tmatch)
  score["tfeat"]=tfeat
  score["tmatch"]=tmatch
  return score

