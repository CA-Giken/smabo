#!/usr/bin/env python3

import numpy as np
import roslib
import rospy
import tf
import tf2_ros
import copy
import os
import sys
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Bool
from std_msgs.msg import String
from geometry_msgs.msg import Transform
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float32MultiArray
from rovi_utils import tflib

Config={
  "multiplex":2,
  "cf":0,
  "solve_frame_id":"camera/capture0",
  "reference_frame_id":["base"],
  "base_frame_id":["base"]}
Param={
  "fitness":{"min":0.8,"max":1},
  "rmse":{"min":0,"max":1000},
  "azimuth":{"min":0,"max":0.3}
}

Stats={}

def cb_judge(dat):
  res=True
  for key in dat:
    val=dat[key]
    dat[key]=(val,0)
    if key in Param:
      minval=Param[key]["min"]
      maxval=Param[key]["max"]
      if minval<maxval:
        if val>maxval:
          dat[key]=(val,1)
          res=False
        elif val<minval:
          dat[key]=(val,-1)
          res=False
      else:
        if val>maxval and val<minval:
          dat[key]=(val,2)
          res=False
  return dat,res

def getcf(frmid):
  tfs=Config[frmid]
  if tfs is not list:
    return tfs
  cf=Config["cf"]
  if cf is str: cf=rospy.getparam(cf)
  try:
    return tfs[cf]
  except Exception:
    return tfs[0]

def cb_tfchk(msg):
  stats={}
#azimuth rotation
  source=getcf("base_frame_id")
  target=getcf("reference_frame_id")
  print("picker::tfchk",source,target);
  try:
    tfs=tfBuffer.lookup_transform(source,target,rospy.Time(0))
  except (tf2_ros.LookupException,tf2_ros.ConnectivityException,tf2_ros.ExtrapolationException):
    print("tf not found",source,target)
  else:
    rTs=tflib.toRT(tfs.transform)
    vz=np.ravel(rTs[:3,2]) #basis vector Z
    vz=vz/np.linalg.norm(vz)
    stats["azimuth"]=np.arccos(np.dot(vz,np.array([0,0,1])))*180/np.pi
    vr=R.from_matrix(rTs[:3,:3]).as_rotvec(degrees=True)
    stats["rotation"]=vr[2]
    stats["norm"]=np.linalg.norm(rTs[:3,3])
    stats["transX"]=rTs[0,3]
    stats["transY"]=rTs[1,3]
    stats["transZ"]=rTs[2,3]
    stats["rotX"]=vr[0]
    stats["rotY"]=vr[1]
    stats["rotZ"]=vr[2]
  stats,judge=cb_judge(stats)
  pub_report.publish(str(stats))
  print("picker::tfchk::report",stats);
  rospy.Timer(rospy.Duration(0.1),lambda ev: pub_checked.publish(mTrue if judge else mFalse),oneshot=True)
  pub_redraw.publish(mTrue)

def cb_stats():
  global Stats
  try:
    Param.update(rospy.get_param("~param"))
  except Exception as e:
    print("get_param exception:",e.args)
  rospy.loginfo("picker::fitness "+str(Stats["fitness"]))
  wfit=np.where(Stats["fitness"]>Param["fitness"]["min"])
  if len(wfit[0])>0:
    amin=np.argmin(Stats["Tz"][wfit])
    pick=wfit[0][amin]
  else:
    pick=np.argmin(Stats["Tz"])
  stats={key:lst[pick] for key,lst in Stats.items()}
  stats,judge=cb_judge(stats)
  tf=TransformStamped()
  tf.header.stamp=rospy.Time.now()
  tf.header.frame_id=Config["solve_frame_id"]
  tf.child_frame_id=Config["solve_frame_id"]+"/solve0"
  tf.transform.translation.x=Stats["Tx"][pick]
  tf.transform.translation.y=Stats["Ty"][pick]
  tf.transform.translation.z=Stats["Tz"][pick]
  tf.transform.rotation.x=Stats["Qx"][pick]
  tf.transform.rotation.y=Stats["Qy"][pick]
  tf.transform.rotation.z=Stats["Qz"][pick]
  tf.transform.rotation.w=Stats["Qw"][pick]
  btf=[tf]
  broadcaster.sendTransform(btf)
  pub_report.publish(str(stats))
  if not judge:
    rospy.Timer(rospy.Duration(0.1),lambda ev: pub_checked.publish(mFalse),oneshot=True)
  else:
    pub_solved.publish(mTrue)
#    rospy.Timer(rospy.Duration(0.1),lambda ev: pub_Y3.publish(mTrue),oneshot=True)
  pub_redraw.publish(mTrue)
  Stats={}

def cb_score(msg):
  global Stats
  dstart=0
  for n,sc in enumerate(msg.layout.dim):
    key=msg.layout.dim[n].label
    size=msg.layout.dim[n].size
    val=np.asarray(msg.data[dstart:dstart+size])
    dstart=dstart+size
    if key in Stats: Stats[key]=np.concatenate((Stats[key],val),axis=None)
    else: Stats[key]=val
  print('picker::cb_score::Stats',Stats)
  if len(set(Stats["proc"]))>=Config["multiplex"]: cb_stats()

def cb_clear(msg):
  global Stats
  Stats={}
  tf=TransformStamped()
  tf.header.stamp=rospy.Time.now()
  tf.header.frame_id=Config["solve_frame_id"]
  tf.child_frame_id=Config["solve_frame_id"]+"/solve0"
  tf.transform.translation.x=0
  tf.transform.translation.y=0
  tf.transform.translation.z=0
  tf.transform.rotation.x=0
  tf.transform.rotation.y=0
  tf.transform.rotation.z=0
  tf.transform.rotation.w=1
  broadcaster.sendTransform([tf])

def cb_solve(msg):
  if msg.data: return
  else:
    pub_checked.publish(mFalse)
    pub_redraw.publish(mTrue)

def parse_argv(argv):
  args={}
  for arg in argv:
    tokens = arg.split(":=")
    if len(tokens) == 2:
      key = tokens[0]
      args[key]=tokens[1]
  return args

def getRT(base,ref):
  try:
    ts=tfBuffer.lookup_transform(base,ref,rospy.Time())
    rospy.loginfo("picker::getRT::TF lookup success "+base+"->"+ref)
    RT=tflib.toRT(ts.transform)
  except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    RT=None
  return RT

########################################################
rospy.init_node("picker",anonymous=True)
Config.update(parse_argv(sys.argv))
try:
  Config.update(rospy.get_param("~config"))
except Exception as e:
  print("get_param exception:",e.args)
try:
  Param.update(rospy.get_param("~param"))
except Exception as e:
  print("get_param exception:",e.args)

###Topics Service
rospy.Subscriber("~clear",Bool,cb_clear)
rospy.Subscriber("~solve",Bool,cb_solve)
rospy.Subscriber("~check",Bool,cb_tfchk)
rospy.Subscriber("~score",Float32MultiArray,cb_score)
pub_redraw=rospy.Publisher("~redraw",Bool,queue_size=1)
pub_solved=rospy.Publisher("~solved",Bool,queue_size=1)
pub_checked=rospy.Publisher("~checked",Bool,queue_size=1)
pub_report=rospy.Publisher("/report",String,queue_size=1)

###Globals
mTrue=Bool();mTrue.data=True
mFalse=Bool();mFalse.data=False
tfBuffer=tf2_ros.Buffer()
listener=tf2_ros.TransformListener(tfBuffer)
broadcaster=tf2_ros.StaticTransformBroadcaster()

try:
  rospy.spin()
except KeyboardInterrupt:
  print("Shutting down")
