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
import yaml
from scipy.spatial.transform import Rotation as R
#from smabo.msg import Floats
#from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Bool
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Transform
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray
from smabo import tflib
#from rovi_utils import sym_solver as rotsym
#from rovi_utils import axis_solver as rotjour
from scipy import optimize
from smabo import open3d_conversions
from smabo import icp_solver as icp

Param={
  "streaming":False,
  "normal_radius":10,
  "feature_radius":15,
  "feature_threshold":10,
  "normal_min_nn":25,
  "icp_threshold":3,
  "rotate":0,
  "repeat":1,
  "cutter":{"base":0,"offset":0,"width":0,"crop":0,"align":"x","trim":0},
  "feature_fitness":0.7,
  "icp_fitness":0.7
}
Config={
  "path":"recipe",
  "solver":"feature_solver",
  "base_frame_id":"base",
  "align_frame_id":"camera/capture0",
  "master_frame_id":"camera/master0",
  "capture_frame_id":"camera/capture0",
  "track_frame_id":"camera/track0",
  "mesh":"/cropper/mesh" }
Score={}

def P0():
  return np.array([]).reshape((-1,3))

def cog(cloud):
  dat=np.array(cloud.points)
  return np.mean(dat,axis=0)

def getRT(base,ref):
  try:
    ts=tfBuffer.lookup_transform(base,ref,rospy.Time())
    RT=tflib.toRT(ts.transform)
  except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    RT=np.eye(4)
  return RT

def learn_feat(pcd,param):
  try:
    mesh=rospy.get_param(Config["mesh"])
  except Exception:
    mesh=0
  if mesh>0:
    pcdn=pcd.voxel_down_sample(mesh)
  else:
    pcdn=pcd
  solver.learn(pcdn,param)
  return pcdn

def learn_rot(pc,num,thres):
  global RotAxis,tfReg
  RotAxis=None
  if num>1:
    RotAxis=rotsym.solve(pc,num,thres)
    if len(RotAxis)>1:
      tf=TransformStamped()
      tf.header.stamp=rospy.Time.now()
      tf.header.frame_id=Config["master_frame_ids"][0]
      tf.child_frame_id=Config["master_frame_ids"][0]+'/axis'
      tf.transform=tflib.fromRT(RotAxis[0])
      tfReg.append(tf)
    else:
      RotAxis=None
      print('No axis')


def learn_journal(pc,base,ofs,wid,align):
  global JourAxis,tfReg
  JourAxis=None
  if wid>0:
    JourAxis=rotjour.solve(pc,base,base+ofs,wid,align)
    if JourAxis is not None:
      tf=TransformStamped()
      tf.header.stamp=rospy.Time.now()
      tf.header.frame_id=Config["master_frame_ids"][0]
      tf.child_frame_id=Config["master_frame_ids"][0]+'/journal'
      tf.transform=tflib.fromRT(JourAxis)
      tfReg.append(tf)
    else:
      print('searcher::learn_journal No journal')

def getBC(key):
  for tf in tfReg:
    if tf.child_frame_id.startswith(key):
      return tf
  return None

def repeat(func,dur):
  tmr=rospy.Timer(rospy.Duration(0.5),func)
  rospy.Timer(rospy.Duration(dur),lambda ev:tmr.shutdown(),oneshot=True)

def cb_master(event):
  trac=getBC(Config["track_frame_id"])
  if trac is None:
    print("No tf found",Config["track_frame_id"])
    return
  if ScenePC2 is not None:
    mTc=getRT(Master_Frame_Id,ScenePC2.header.frame_id)
    trac.transform=tflib.fromRT(mTc.dot(cTs))
  else:
    trac.transform=tflib.fromRT(np.eye(4))
  broadcaster.sendTransform(tfReg)
  if ModelPC2 is not None:
#    rospy.Timer(rospy.Duration(3.0),lambda ev:pub_pc2.publish(ModelPC2),oneshot=True)
    repeat(lambda ev:pub_pc2.publish(ModelPC2),3)

def addtfs(mtf,ofs):
  global tfReg,mTg
  bTm=tflib.toRT(mtf.transform)
  mTb=np.linalg.inv(bTm)
  cog=TransformStamped()
  cog.header=copy.copy(mtf.header)
  cog.header.frame_id=Master_Frame_Id
  cog.child_frame_id=Master_Frame_Id+"/cog"
  mTg=np.eye(4)
  mTg[:3,3]=ofs
  cog.transform=tflib.fromRT(mTg)
  trac=TransformStamped()
  trac.header=copy.copy(mtf.header)
  trac.header.frame_id=Master_Frame_Id
  trac.child_frame_id=Config["track_frame_id"]
  if ScenePC2 is not None:
    bTc=getRT(mtf.header.frame_id,ScenePC2.header.frame_id)
    trac.transform=tflib.fromRT(mTb.dot(bTc))
  else:
    trac.transform=tflib.fromRT(np.eye(4))
  tfReg=[mtf,cog,trac]
  broadcaster.sendTransform(tfReg)

def cb_save(msg):
  global Model,ModelPC2,tfReg,Master_Frame_Id
#save point cloud
  if ScenePC2 is None:
    pub_saved.publish(mFalse)
    print("searcher::cb_save Scene is None")
    return
  if ScenePC2.row_step<1000:
    pub_saved.publish(mFalse)
    print("searcher::cb_save few Scene points")
    return
  modPC2=copy.deepcopy(ScenePC2)
  try:
    tf=tfBuffer.lookup_transform(Config["base_frame_id"],modPC2.header.frame_id,rospy.Time())
  except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    pub_saved.publish(mFalse)
    print("searcher::cb_save No frame found")
    return
  Master_Frame_Id=Config["master_frame_id"]
  tf.child_frame_id=Master_Frame_Id
  path=Config["path"]+"/"+Master_Frame_Id.replace('/','_')+".yaml"
  f=open(path,"w")
  f.write(yaml.dump(tflib.tf2dict(tf.transform)))
  f.close()
  modCloud=open3d_conversions.from_msg(modPC2)
  o3d.io.write_point_cloud(path.replace('.yaml','.ply'),modCloud,True,False)
  Model=learn_feat(modCloud,Param)
#  learn_rot(pcd,Param['rotate'],Param['icp_threshold'])
  addtfs(tf,cog(modCloud))
  pub_saved.publish(mTrue)
  ModelPC2=open3d_conversions.to_msg(Model,frame_id=Config["track_frame_id"])
#  ModelPC2.header.frame_id=trac.child_frame_id
  cb_clear(True)
#  pub_pc2.publish(ModelPC2)

def cb_load(msg):
  global Model,ModelPC2,tfReg,Param,Master_Frame_Id
  tfReg=[]
#load .tf files
  ls=os.listdir(Config["path"])
  dottf=list(filter(lambda f:f.endswith(Config["master_frame_id"].replace('/','_')+'.yaml'),ls))
  print("searcher::cb_load",dottf)
  Master_Frame_Id=dottf[0].replace('_','/').replace('.yaml','')
  path=Config["path"]+"/"+dottf[0]
  try:
    print("searcher::cb_load",path)
    f=open(path, "r+")
  except Exception:
    pub_loaded.publish(mFalse)
    print("searcher::cb_load .tf load error")
    return
  yd=yaml.load(f,Loader=yaml.SafeLoader)
  f.close()

  modCloud=o3d.io.read_point_cloud(path.replace('.yaml','.ply'))
  Model=learn_feat(modCloud,Param)
#  learn_rot(pcd,Param['rotate'],Param['icp_threshold'])
#  learn_journal(pcd,Param["cutter"]["base"],Param["cutter"]["offset"],Param["cutter"]["width"],Param["cutter"]["align"])

  trf=tflib.dict2tf(yd)
  tf=TransformStamped()
  tf.header.stamp=rospy.Time.now()
  tf.header.frame_id=Config["base_frame_id"]
  tf.child_frame_id=Master_Frame_Id
  tf.transform=trf

  addtfs(tf,cog(modCloud))

  pub_loaded.publish(mTrue)
  ModelPC2=open3d_conversions.to_msg(Model,frame_id=Config["track_frame_id"])
  cb_clear(True)
#  pub_pc2.publish(ModelPC2)

def cb_score():
  pass

def pub_stats(result):
  fitness=Float64()
  fitness.data=result["fitness"]
  pub_fitness.publish(fitness)
  rmse=Float64()
  rmse.data=result["rmse"]
  pub_rmse.publish(rmse)
  rep={}
  rep["fitness"]=result["fitness"]
  rep["rmse"]=result["rmse"]
  pub_report.publish(str(rep))

def cb_solve(msg):
  global Param
  rospy.Timer(rospy.Duration(0.01),cb_solve_do,oneshot=True)
  Param.update(rospy.get_param("~param"))

def pub_moving(RT):
  rep={}
  vz=np.ravel(RT[:3,2]) #basis vector Z
  vz=vz/np.linalg.norm(vz)
  rep["azimuth"]=np.arccos(np.dot(vz,np.array([0,0,1])))*180/np.pi
  vr=R.from_matrix(RT[:3,:3]).as_rotvec(degrees=True)
  rep["rotation"]=vr[2]
  rep["Tx"]=RT[0,3]
  rep["Ty"]=RT[1,3]
  rep["Tz"]=RT[2,3]
  rospy.Timer(rospy.Duration(0.5),lambda ev:pub_report.publish(str(rep)),oneshot=True)

def cb_solve_do(msg):
  global cTs
  Scene=open3d_conversions.from_msg(ScenePC2)
  cTs=np.eye(4)
  result=None
  if Param["feature_fitness"]>0.0:
    result=solver.solve(Scene,Param)
    if result["fitness"]<Param["feature_fitness"]:
      print("searcher Feature match fitness is low",result["fitness"])
      pub_stats(result)
      pub_Y2.publish(mFalse)
      return
    cTs=result["transform"]

  if Param["icp_fitness"]>0.0:
    result=icp.solve(Model,Scene,Param,cTs)
    if result["fitness"]<Param["icp_fitness"]:
      print("searcher ICP fitness is low",result["fitness"])
      pub_stats(result)
      pub_Y2.publish(mFalse)
      return
    cTs=result["transform"]

  if result is not None: pub_stats(result)
  cb_master(True)

  mTc=getRT(Master_Frame_Id,Config["capture_frame_id"])
  mTtg=mTc.dot(cTs).dot(mTg)
#  print('searcher::cb_solve_do Solve',np.linalg.inv(mTg).dot(mTtg))

  pub_moving(np.linalg.inv(mTg).dot(mTtg))
  tf=tflib.fromRT(mTtg)
  array=PoseArray()
  array.header.stamp=rospy.Time.now()
  array.header.frame_id=Config["master_frame_id"]
  pose=Pose()
  pose.position.x=tf.translation.x
  pose.position.y=tf.translation.y
  pose.position.z=tf.translation.z
  pose.position.x=tf.translation.x
  pose.orientation.x=tf.rotation.x
  pose.orientation.y=tf.rotation.y
  pose.orientation.z=tf.rotation.z
  pose.orientation.w=tf.rotation.w
  array.poses.append(pose)

#  rospy.Timer(rospy.Duration(5.0),lambda ev:pub_poses.publish(array),oneshot=True)
  repeat(lambda ev:pub_poses.publish(array),3)
  pub_Y2.publish(mTrue)

def cb_pc2(msg):
  global ScenePC2
  print("searcher cb_pc2")
  ScenePC2=msg

def cb_clear(msg):
  global cTs,ScenePC2
  cTs=np.eye(4)
  ScenePC2=None
  cb_master(True)

def cb_scan(msg):
  global Scene,cTs,Param
  try:
    Param.update(rospy.get_param("~param"))
  except Exception as e:
    print("get_param exception:",e.args)
  if Param["streaming"] and ScenePC2 is not None and Model is not None:
    Scene=open3d_conversions.from_msg(ScenePC2)
    result=icp.solve(Model,Scene,Param,cTs)
    print("searcher::scan",result["fitness"])
    if result["fitness"]>Param["icp_fitness"]:
      cTs=result["transform"]
      cb_master(True)
    else:
      cb_clear(None)
    pub_stats(result)
  rospy.Timer(rospy.Duration(0.5),cb_scan,oneshot=True)
  return

def parse_argv(argv):
  args={}
  for arg in argv:
    tokens = arg.split(":=")
    if len(tokens) == 2:
      key = tokens[0]
      args[key] = tokens[1]
  return args

########################################################

rospy.init_node("searcher",anonymous=True)
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

###load solver
exec("from smabo import "+Config["solver"]+" as solver")

###I/O
pub_pc2=rospy.Publisher("~out/pc2",PointCloud2,queue_size=1)
pub_Y2=rospy.Publisher("~solved",Bool,queue_size=1)
pub_saved=rospy.Publisher("~saved",Bool,queue_size=1)
pub_loaded=rospy.Publisher("~loaded",Bool,queue_size=1)
pub_poses=rospy.Publisher("~poses",PoseArray,queue_size=1)
pub_report=rospy.Publisher("/report",String,queue_size=1)
pub_fitness=rospy.Publisher("~fitness",Float64,queue_size=1)
pub_rmse=rospy.Publisher("~rmse",Float64,queue_size=1)
rospy.Subscriber("~in/pc2",PointCloud2,cb_pc2)
rospy.Subscriber("~clear",Bool,cb_clear)
rospy.Subscriber("~solve",Bool,cb_solve)
rospy.Subscriber("~save",Bool,cb_save)
rospy.Subscriber("~load",Bool,cb_load)
rospy.Subscriber("~redraw",Bool,cb_master)

###std_msgs/Bool
mTrue=Bool()
mTrue.data=True
mFalse=Bool()
mFalse.data=False

###TF
tfBuffer=tf2_ros.Buffer()
listener=tf2_ros.TransformListener(tfBuffer)
broadcaster=tf2_ros.StaticTransformBroadcaster()

###Globals
Scene=None     #as O3D
ScenePC2=None  #as ROS
Model=None    #as O3D
ModelPC2=None #as PointCloud2

cTs=np.eye(4) #model to scene conversion in capture cordinate
mTg=np.eye(4) #master to COG
tfReg=[]      #as TransformStamped
Master_Frame_Id=""

#rospy.Timer(rospy.Duration(5),cb_load,oneshot=True)
rospy.Timer(rospy.Duration(1),cb_scan,oneshot=True)
try:
  rospy.spin()
except KeyboardInterrupt:
  print("Shutting down")
