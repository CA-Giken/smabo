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
from smabo.msg import Floats
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Bool
from std_msgs.msg import Int64
from std_msgs.msg import Float64
from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import MultiArrayDimension
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Transform
from geometry_msgs.msg import TransformStamped
from smabo import tflib
from rovi_utils import sym_solver as rotsym
from rovi_utils import axis_solver as rotjour
from scipy import optimize
from smabo import open3d_conversions
from smabo import icp_solver as icp

Param={
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
  "solver":"o3d_solver",
  "base_frame_id":"base",
  "calib_frame_id":"base",
  "align_frame_id":"camera/capture0",
  "master_frame_id":"master0",
  "mesh":"/cropper/mesh" }
Score={
  "Tx":[],
  "Ty":[],
  "Tz":[],
  "Qx":[],
  "Qy":[],
  "Qz":[],
  "Qw":[] }

def P0():
  return np.array([]).reshape((-1,3))

def np2F(d):  #numpy to Floats
  f=Floats()
  f.data=np.ravel(d)
  return f

def getRT(base,ref):
  try:
    ts=tfBuffer.lookup_transform(base,ref,rospy.Time())
    rospy.loginfo("cropper::getRT::TF lookup success "+base+"->"+ref)
    RT=tflib.toRT(ts.transform)
  except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    RT=None
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

def cb_master(event):
  if ModelPC2 is not None: pub_pc2.publish(ModelPC2)

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
  Master_Frame_Id=modPC2.header.frame_id+"/"+Config["master_frame_id"]
  tf.child_frame_id=Master_Frame_Id
  path=Config["path"]+"/"+Master_Frame_Id.replace('/','_')+".yaml"
  f=open(path,"w")
  f.write(yaml.dump(tflib.tf2dict(tf.transform)))
  f.close()
  modCloud=open3d_conversions.from_msg(modPC2)
  o3d.io.write_point_cloud(path.replace('.yaml','.ply'),modCloud,True,False)
  trac=TransformStamped()
  trac.header=copy.copy(tf.header)
  trac.header.frame_id=Master_Frame_Id
  trac.child_frame_id=Master_Frame_Id+"/solve0"
  trac.transform=tflib.fromRT(np.eye(4))
  tfReg=[tf,trac]
  broadcaster.sendTransform(tfReg)
  Model=learn_feat(modCloud,Param)
#  learn_rot(pcd,Param['rotate'],Param['icp_threshold'])
  pub_saved.publish(mTrue)
  ModelPC2=open3d_conversions.to_msg(Model,frame_id=trac.child_frame_id)
#  ModelPC2.header.frame_id=trac.child_frame_id
  pub_pc2.publish(ModelPC2)

def cb_load(msg):
  global Model,ModelPC2,tfReg,Param,Master_Frame_Id
  tfReg=[]
#load .tf files
  ls=os.listdir(Config["path"])
  dottf=list(filter(lambda f:f.endswith(Config["master_frame_id"]+'.yaml'),ls))
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
  trf=tflib.dict2tf(yd)
  tf=TransformStamped()
  tf.header.stamp=rospy.Time.now()
  tf.header.frame_id=Config["base_frame_id"]
  tf.child_frame_id=Master_Frame_Id
  tf.transform=trf

  trac=TransformStamped()
  trac.header=copy.copy(tf.header)
  trac.header.frame_id=tf.child_frame_id
  trac.child_frame_id=tf.child_frame_id+"/solve0"
  trac.transform=tflib.fromRT(np.eye(4))

  tfReg=[tf,trac]
  broadcaster.sendTransform(tfReg)

  modCloud=o3d.io.read_point_cloud(path.replace('.yaml','.ply'))
  Model=learn_feat(modCloud,Param)
#  learn_rot(pcd,Param['rotate'],Param['icp_threshold'])
#  learn_journal(pcd,Param["cutter"]["base"],Param["cutter"]["offset"],Param["cutter"]["width"],Param["cutter"]["align"])
  pub_loaded.publish(mTrue)
  ModelPC2=open3d_conversions.to_msg(Model,frame_id=trac.child_frame_id)
  pub_pc2.publish(ModelPC2)

def cb_score():
  global Score
  score=Float32MultiArray()
  score.layout.data_offset=0
  for n,sc in enumerate(Score):
    score.layout.dim.append(MultiArrayDimension())
    score.layout.dim[n].label=sc
    score.layout.dim[n].size=len(Score[sc])
    score.layout.dim[n].stride=1
    score.data.extend(Score[sc])
  pub_score.publish(score)
  pub_Y2.publish(mTrue)

def cb_solve(msg):
  global Score,Param
  for key in Score: Score[key]=[]
  rospy.Timer(rospy.Duration(0.01),cb_solve_do,oneshot=True)
  Param.update(rospy.get_param("~param"))

def cb_solve_do(msg):
  global Score
  Scene=open3d_conversions.from_msg(ScenePC2)
  if Param["feature_fitness"]>0.0:
    result=solver.solve(Scene,Param)
    if result["fitness"]<Param["feature_fitness"]:
      print("searcher Feature match fitness is low",result["fitness"])
      pub_Y2.publish(mFalse)
      return
    mTs=result["transform"]
  else:
    mTs=np.eye(4)
  result=icp.solve(Model,Scene,Param,mTs)
  if result["fitness"]<Param["icp_fitness"]:
    print("searcher ICP fitness is low",result["fitness"])
    pub_Y2.publish(mFalse)
    return
  mTs=result["transform"]
  tfReg[-1].transform=tflib.fromRT(mTs)    #repub tf ../solve0
  broadcaster.sendTransform(tfReg)
  rospy.Timer(rospy.Duration(0.1),cb_master,oneshot=True)

  tf=tflib.fromRT(mTs)
  print('searcher::cb_solve_do Solve',mTs)

  Score["Tx"].append(tf.translation.x)
  Score["Ty"].append(tf.translation.y)
  Score["Tz"].append(tf.translation.z)
  Score["Qx"].append(tf.rotation.x)
  Score["Qy"].append(tf.rotation.y)
  Score["Qz"].append(tf.rotation.z)
  Score["Qw"].append(tf.rotation.w)

  for key in result:
    if type(result[key]) is float:
      print("searcher add score ",key,result[key])
      if not key in Score:
        Score[key]=[]
      Score[key].append(result[key])
  cb_score()

def cb_pc2(msg):
  global ScenePC2
  print("searcher cb_pc2")
  ScenePC2=msg

def cb_clear(msg):
  global mTs
  mTs=np.eye(4)
  tfReg[-1].transform=tflib.fromRT(mTs)  #.child_frame_id+"/solve0"
  broadcaster.sendTransform(tfReg)
  rospy.Timer(rospy.Duration(0.1),cb_master,oneshot=True)

def cb_scan(msg):
  global Scene,mTs,Param
  try:
    Param.update(rospy.get_param("~param"))
  except Exception as e:
    print("get_param exception:",e.args)
  if Param["streaming"] and ScenePC2 is not None and Model is not None:
    Scene=open3d_conversions.from_msg(ScenePC2)
    result=icp.solve(Model,Scene,Param,mTs)
    if result["fitness"]>Param["feature_fitness"]:
      mTs=result["transform"]
      tfReg[-1].transform=tflib.fromRT(mTs)
      broadcaster.sendTransform(tfReg)
      rospy.Timer(rospy.Duration(0.1),cb_master,oneshot=True)
    else:
      cb_clear(None)
    fitness=Float64()
    fitness.data=result["fitness"]
    pub_fitness.publish(fitness)
    rmse=Float64()
    rmse.data=result["rmse"]
    pub_rmse.publish(rmse)
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
pub_score=rospy.Publisher("~score",Float32MultiArray,queue_size=1)
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

mTs=np.eye(4) #model to scene conversion
tfReg=[]      #as TransformStamped
Master_Frame_Id=""

#rospy.Timer(rospy.Duration(5),cb_load,oneshot=True)
rospy.Timer(rospy.Duration(1),cb_scan,oneshot=True)
try:
  rospy.spin()
except KeyboardInterrupt:
  print("Shutting down")
