#!/usr/bin/env python3

import cv2
import numpy as np
import math
import roslib
import rospy
import tf
import tf2_ros
import open3d as o3d
import copy
import os
import sys
import time

from std_msgs.msg import Bool
from std_msgs.msg import String
from geometry_msgs.msg import Point32
from geometry_msgs.msg import Point
from geometry_msgs.msg import Transform
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2
from smabo import tflib
from smabo import open3d_conversions
from scipy import optimize

Param={"cropZ":0,"cropZup":0,"cropR":0,"mesh":0.001,"ladle":0,"ladC":0,"ladleW":0,"nfrad":0,"nfmin":0,"wd":0}
Config={
  "relay":"/rovi/X1",
  "base_frame_id":"world",
  "source_frame_id":"camera/capture",
  "frame_id":"camera/capture0",
  "capture_frame_id":"camera",
  "streaming_frame_id":"camera"
}

Pcat=None
Tcapt=0
Report={}
Reqcount=0

def P0():
  return np.array([]).reshape((-1,3))

def voxel(pc):
  mesh=Param["mesh"]
  if mesh==0: return pc
  if len(pc.points)<10: return pc
  dwpc=pc.voxel_down_sample(mesh)
  return dwpc

def nf(pc):
  nfmin=Param["nfmin"]
  nfrad=Param["nfrad"]
  if nfmin==0 or nfrad==0: return pc
  nfmin=Param["nfmin"]
  cl,ind = pc.remove_radius_outlier(nb_points=nfmin,radius=Param["nfrad"])
  dwpc=o3d.geometry.PointCloud.select_by_index(pc,ind)
  return dwpc

def getRT(base,ref):
  try:
    ts=tfBuffer.lookup_transform(base,ref,rospy.Time())
    RT=tflib.toRT(ts.transform)
  except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    RT=np.eye(4)
  return RT

def arrange(pc,n):
  capc=Config["source_frame_id"]
  outc=Config["frame_id"]
  RT=getRT(outc+str(n),capc+str(n))
  if RT is None:
    RT=getRT(outc+str(n),capc)
    if RT is None:
      RT=getRT(outc,capc+str(n))
      if RT is None:
        RT=getRT(outc,capc)
        if RT is None:
          RT=np.eye(4)
          rospy.logwarn("cropper::arrange::TF not found")
  pc.transform(RT)

def cb_redraw(msg):
  global Pcat,Pcrop
  print("redraw",len(Pcrop.points))
  pc2=open3d_conversions.to_msg(Pcrop,frame_id=Config["streaming_frame_id"] if Reqcount==0 else Config["frame_id"])
  pub_pc2.publish(pc2)
  return

def merge():
  global Pcat
  Pcat=o3d.geometry.PointCloud()
  for n,pc in enumerate(srcArray):
    arrange(pc,n)
    Pcat=Pcat+pc
#  pc2=open3d_conversions.to_msg(Pcat,frame_id=Config["frame_id"])
#  pub_raw.publish(pc2)
  return Pcat

def zsort(pc,down=False):
  npc=np.array(pc.points)
  nd=np.ravel(npc.T[2].argsort())
  if down: nd=nd[::-1]
  pcc=o3d.geometry.PointCloud()
  pcc.points=o3d.utility.Vector3dVector(npc[nd])
  return pcc

def crop():
  global Pcat,Pcrop
#ladle cropping(camera)
  Pcrop=voxel(Pcat)
#Noise eliminator
  if Param["nfrad"]>Param["mesh"]:
    Pcrop=nf(Pcrop)
#Sort points
  pcc=zsort(Pcrop)
  if Param["ladC"]>0 and len(pcc.points)>Param["ladC"]:
    Pcrop=o3d.geometry.PointCloud()
    Pcrop.points=pcc.points[:Param["ladC"]]
#world z-crop
  if len(Pcrop.points)>0:
    RT=getRT(Config["base_frame_id"],Config["frame_id"])
    if RT is None:
      RT=np.eye(4)
      rospy.logwarn("cropper::crop::TF not found (world)")
    Pcrop.transform(RT)
    pcc=zsort(Pcrop,down=True)
#    if Param["cropZ"]!=0:
#      pcw.points=pcw.points[np.ravel(np.asarray(pcw.points).T[2]>Param["cropZ"])]
#    if Param["cropZup"]!=0:
#      pcw.points=pcw.points[np.ravel(np.asarray(pcw.points).T[2]<Param["cropZup"])]
#ladle cropping(world)
    if Param["ladW"]>0 and len(pcc.points)>Param["ladW"]:
      Pcrop=o3d.geometry.PointCloud()
      Pcrop.points=pcc.points[:Param["ladW"]]
#back to camera coordinate
    Pcrop.transform(np.linalg.inv(RT))
  cb_redraw(True)
  return

def cb_pc2(msg):
  global srcArray,Tcapt,Pcat,Reqcount
  pc=open3d_conversions.from_msg(msg)
  pub_report.publish(str({"pcount":sum(len(pc.points) for pc in srcArray)}))
  if Reqcount>len(srcArray):
    srcArray.append(pc)
    merge()
  else:
    Pcat=pc
    srcArray=[]
    Reqcount=0
  crop()
  rospy.Timer(rospy.Duration(0.1),lambda msg:pub_capture.publish(mTrue),oneshot=True)
  return

def cb_param(msg):
  global Param
  prm=Param.copy()
  try:
    Param.update(rospy.get_param("~param"))
  except Exception as e:
    print("get_param exception:",e.args)
  if prm!=Param:
    print("Param changed",Param)
    crop()
  rospy.Timer(rospy.Duration(1),cb_param,oneshot=True) #Param update itself
  return

def cb_clear(msg):
  global srcArray,tfArray,Pcat,Pcrop,Reqcount
  srcArray=[]
  tfArray=[]
  Reqcount=0
  keeps=Config["capture_frame_id"]
  if type(keeps) is str: keeps=[keeps]
  try:
    tfs=[]
    for keep in keeps:
      keeptf=tfBuffer.lookup_transform(Config["base_frame_id"],keep,rospy.Time())
      keeptf.header.stamp=rospy.Time.now()
      keeptf.header.frame_id=Config["base_frame_id"]
      keeptf.child_frame_id=keep+"/capture0"
      tfs.append(keeptf)
    broadcaster.sendTransform(tfs)
  except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    print("cropper::clear::lookup failure world->"+keep)
  Pcat=o3d.geometry.PointCloud()
  Pcrop=o3d.geometry.PointCloud()
  cb_redraw(True)
  pub_clear.publish(mTrue)

def cb_capture(msg):
  global tfArray,Tcapt,Report,Reqcount
  keeps=Config["capture_frame_id"]
  if type(keeps) is str: keeps=[keeps]
  try:
    if len(srcArray)==0: tfArray=[]
    for keep in keeps:
      keeptf=tfBuffer.lookup_transform(Config["base_frame_id"],keep,rospy.Time())
      keeptf.header.stamp=rospy.Time.now()
      keeptf.header.frame_id=Config["base_frame_id"]
      keeptf.child_frame_id=keep+"/capture"+str(len(srcArray))
      tfArray.append(keeptf)
    broadcaster.sendTransform(tfArray)
  except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    rospy.loginfo("cropper::capture::TF lookup failure world->"+keep)
  if pub_relay is not None:
    pub_relay.publish(mTrue)
    Reqcount=len(srcArray)
  Tcapt=time.time()

def cb_ansback(msg):
  if msg.data is False: pub_capture.publish(mFalse)

def parse_argv(argv):
  args={}
  for arg in argv:
    tokens = arg.split(":=")
    if len(tokens) == 2:
      key = tokens[0]
      args[key]=tokens[1]
  return args

########################################################
rospy.init_node("cropper",anonymous=True)
Config.update(parse_argv(sys.argv))
try:
  Config.update(rospy.get_param("~config"))
except Exception as e:
  print("get_param exception:",e.args)
print("Config",Config)
try:
  Param.update(rospy.get_param("~param"))
except Exception as e:
  print("get_param exception:",e.args)
print("Param",Param)

###Input topics
rospy.Subscriber("~in/pc2",PointCloud2,cb_pc2)
rospy.Subscriber("~clear",Bool,cb_clear)
rospy.Subscriber("~capture",Bool,cb_capture)
rospy.Subscriber("~redraw",Bool,cb_redraw)
if "ansback" in Config:
  rospy.Subscriber(Config["ansback"],Bool,cb_ansback)
###Output topics
pub_pc2=rospy.Publisher("~out/pc2",PointCloud2,queue_size=1)
pub_raw=rospy.Publisher("~out/rawpc2",PointCloud2,queue_size=1)
pub_relay=None
if "relay" in Config:
  pub_relay=rospy.Publisher(Config["relay"],Bool,queue_size=1)
pub_clear=rospy.Publisher("~cleared",Bool,queue_size=1)
pub_capture=rospy.Publisher("~captured",Bool,queue_size=1)
pub_report=rospy.Publisher("/report",String,queue_size=1)

###Globals
mTrue=Bool();mTrue.data=True
mFalse=Bool();mFalse.data=False
tfBuffer=tf2_ros.Buffer()
listener=tf2_ros.TransformListener(tfBuffer)
broadcaster=tf2_ros.StaticTransformBroadcaster()
rospy.sleep(1)
cb_clear(mTrue)

rospy.Timer(rospy.Duration(1),cb_param,oneshot=True) #Param update itself

try:
  rospy.spin()
except KeyboardInterrupt:
  print("Shutting down")

