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
from rovi_utils import tflib
from rovi.msg import Floats
from rospy.numpy_msg import numpy_msg

Config={
  "source_frame_id":"world",
  "target_frame_id":"camera",
  "trim_x":300,
  "trim_y":250,
  "trim_far":600,
  "trim_near":380,
  "view":[[-10,0,0]],
  "view_r":50000,
  "hidden":True,
  "folder":"mesh"
}
Param={
  "mesh":1,
  "frame":0
}

def np2F(d):  #numpy to Floats
  f=Floats()
  f.data=np.ravel(d)
  return f

def getRT(base,ref):
  try:
    ts=tfBuffer.lookup_transform(base,ref,rospy.Time())
    rospy.loginfo("getRT::TF lookup success "+base+"->"+ref)
    RT=tflib.toRT(ts.transform)
  except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    RT=None
  return RT

def cb_ps(msg):
  global Scene
  Scene=open3d_conversions.from_msg(msg)
  print("vcam sub scene",len(Scene.points))
  return

def cb_capture(msg):
  try:
    Config.update(rospy.get_param("/config/vcam"))
  except Exception as e:
    print("get_param exception:",e.args)
  RT=getRT(Config["target_frame_id"],Config["source_frame_id"])
  scn0=np.array(Scene.points)
  scn_1=np.vstack((scn0.T,np.ones(len(scn0))))
  scn_1=RT.dot(scn_1)
  scn=scn_1[:3].T
  scn=scn[np.abs(np.ravel(scn_1[1]))<Config["trim_y"]/2]
  zp=np.ravel(scn.T[2])
  scn=scn[zp<Config["trim_far"]]
  yp=np.ravel(scn.T[1])
  zp=np.ravel(scn.T[2])
  scn=scn[np.abs(yp/zp)<Config["trim_y"]/Config["trim_far"]]
  xp=np.ravel(scn.T[0])
  zp=np.ravel(scn.T[2])
  scn=scn[np.abs(xp/zp)<Config["trim_x"]/Config["trim_far"]]
  print("vcam trimmed",scn.shape)
  if len(scn)<1000:
    print("vcam points too few, abort hidden...",len(scn))
    pub_ps.publish(np2F(scn))
    pub_done.publish(mTrue)
    return  
  pcd=o3d.geometry.PointCloud()
  pcd.points=o3d.utility.Vector3dVector(scn)
  if Config["hidden"]:
    pset=set([])
    for v in Config["view"]:
      _, pm=pcd.hidden_point_removal(v,Config["view_r"])
      pset=pset.union(set(pm))
    plst=np.array(list(pset))
    pcd=pcd.select_by_index(plst)
  pc2=open3d_conversions.to_msg(pcd,frame_id=Config["target_frame_id"])
  pub_pc2.publish(pc2)
  pub_done.publish(mTrue)

def getFolder(datapath,n):
  ls=os.listdir(datapath)
  sel=list(filter(lambda f:f.endswith('.ply'),ls))
  sel.sort()
  print("getFolders",sel)
  if n>=len(sel):
    print("vcam::frame out of folders")
    return None
  return sel[n]

def cb_load(msg):
  try:
    Param.update(rospy.get_param("/sensors"))
  except Exception as e:
    print("get_param exception:",e.args)
  path=thispath+'/'+Config['folder']
  frame=Param["frame"]
  file=getFolder(path,frame)
  if file is None: return
  pcd=o3d.io.read_point_cloud(path+'/'+file)
  print("vcam load ply",file,len(pcd.points))
  dwnpc=pcd.voxel_down_sample(Param["mesh"])
  pc2=open3d_conversions.to_msg(dwnpc,frame_id=Config["target_frame_id"])
  pub_pc2.publish(pc2)
  pub_floats.publish(np2F(dwnpc.points))
  pub_done.publish(mTrue)
  rospy.set_param("/sensors/frame",frame+1)

########################################################
rospy.init_node("vcam",anonymous=True)
thispath= subprocess.getoutput("rospack find smabo")
###Load params
try:
  Config.update(rospy.get_param("/config/vcam"))
except Exception as e:
  print("get_param exception:",e.args)
if "ws" in Config:
  thispath= subprocess.getoutput("rospack find "+Config["ws"])
###Topics
rospy.Subscriber("~scene_pc2",PointCloud2,cb_ps)
#rospy.Subscriber("/sensors/X1",Bool,cb_capture) #emurates capture pcd(:world) and publish
rospy.Subscriber("/sensors/X1",Bool,cb_load)
rospy.Subscriber("/sensors/X10",Bool,cb_load)   #loads pcd(:camera) from file and publish
pub_pc2=rospy.Publisher("/sensors/pc2",PointCloud2,queue_size=1)
pub_floats=rospy.Publisher("/sensors/floats",numpy_msg(Floats),queue_size=1)
pub_done=rospy.Publisher("/sensors/Y1",Bool,queue_size=1)
###Globals
mTrue=Bool();mTrue.data=True
mFalse=Bool()
tfBuffer=tf2_ros.Buffer()
listener=tf2_ros.TransformListener(tfBuffer)

#if __name__=="__main__":
#

try:
  rospy.spin()
except KeyboardInterrupt:
  print("Shutting down")
