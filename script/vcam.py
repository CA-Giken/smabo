#!/usr/bin/env python3

import numpy as np
import roslib
import rospy
import tf
import tf2_ros
import open3d as o3d
import os
import sys
import subprocess
import copy
from std_msgs.msg import Bool
from geometry_msgs.msg import Transform
from sensor_msgs.msg import PointCloud2
from smabo import open3d_conversions
from smabo import tflib

Config={
  "camera_frame_id":"camera",
  "trim_x":2000,
  "trim_y":2000,
  "trim_far":2000,
  "trim_near":100,
  "view":[[0,0,0]],
  "view_r":50000,
  "hidden":True,
  "ply": ['sma-lab2','mesh/test{:02d}','camera'],
}
Param={
  "streaming":False,
  "mesh":1.0
}
Plocal={
  "frame":0
}

def getRT(base,ref):
  try:
    ts=tfBuffer.lookup_transform(base,ref,rospy.Time())
    RT=tflib.toRT(ts.transform)
  except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    RT=np.eye(4)
  return RT

def pubpcd():
  RT=getRT(Config["camera_frame_id"],Config["ply"][2])
  pcd=copy.deepcopy(Cloud)
  pcd.transform(RT)
  scn=np.array(pcd.points)
  zp=np.ravel(scn.T[2])
  scn=scn[zp<Config["trim_far"]]
  yp=np.ravel(scn.T[1])
  zp=np.ravel(scn.T[2])
  scn=scn[np.abs(yp/zp)<Config["trim_y"]/Config["trim_far"]]
  xp=np.ravel(scn.T[0])
  zp=np.ravel(scn.T[2])
  scn=scn[np.abs(xp/zp)<Config["trim_x"]/Config["trim_far"]]
  print("vcam trimmed",scn.shape)
#  pcd=o3d.geometry.PointCloud()
  pcd.clear()
  if len(scn)<1000:
    print("vcam points too few, abort hidden...",len(scn))
  else:
    pcd.points=o3d.utility.Vector3dVector(scn)
    if Config["hidden"]:
      pset=set([])
      for v in Config["view"]:
        _, pm=pcd.hidden_point_removal(v,Config["view_r"])
        pset=pset.union(set(pm))
      plst=np.array(list(pset))
      pcd=pcd.select_by_index(plst)
  pc2=open3d_conversions.to_msg(pcd,frame_id=Config["camera_frame_id"])
  pub_pc2.publish(pc2)

def loadpcd():
  global Param,Plocal,Cloud
  try:
    Plocal.update(rospy.get_param("/vcam"))
  except Exception as e:
    print("get_param exception:",e.args)
  try:
    Param.update(rospy.get_param("/sensors"))
  except Exception as e:
    print("get_param exception:",e.args)
  pack= subprocess.getoutput("rospack find "+Config["ply"][0])
  frame=int(Plocal['frame'])
  path= pack+'/'+Config["ply"][1].format(frame)+'.ply'
  print("vcam load",path)
  Cloud=o3d.io.read_point_cloud(path)
  print("vcam load ply",path,len(Cloud.points))
  mesh=float(Param["mesh"])
  if mesh>0:
    Cloud=Cloud.voxel_down_sample(mesh)

def cb_capture(msg):
  loadpcd()
  pubpcd()
  pub_done.publish(mTrue)
  rospy.set_param("/vcam/frame",int(Plocal["frame"])+1)

def cb_scan(msg):
  global Param
  try:
    Param.update(rospy.get_param("/sensors"))
  except Exception as e:
    print("get_param exception:",e.args)
  if Param["streaming"]:
    pubpcd()
  rospy.Timer(rospy.Duration(1),cb_scan,oneshot=True)
  return

########################################################
rospy.init_node("vcam",anonymous=True)
###Load params
try:
  Config.update(rospy.get_param("/config/vcam"))
except Exception as e:
  print("get_param exception:",e.args)
###Topics
rospy.Subscriber("/sensors/X1",Bool,cb_capture)
pub_pc2=rospy.Publisher("/sensors/pc2",PointCloud2,queue_size=1)
pub_done=rospy.Publisher("/sensors/Y1",Bool,queue_size=1)
###Globals
mTrue=Bool();mTrue.data=True
mFalse=Bool()
tfBuffer=tf2_ros.Buffer()
listener=tf2_ros.TransformListener(tfBuffer)

#if __name__=="__main__":
#

loadpcd()
rospy.Timer(rospy.Duration(3),cb_scan,oneshot=True)

try:
  rospy.spin()
except KeyboardInterrupt:
  print("Shutting down")
