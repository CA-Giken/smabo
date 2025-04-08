#!/usr/bin/env python3

import cv2
import numpy as np
import roslib
import rospy
import tf
import tf2_ros
import time
from std_msgs.msg import Bool
from std_msgs.msg import Header
from std_msgs.msg import Float64
from geometry_msgs.msg import Transform
from geometry_msgs.msg import TransformStamped
from smabo import tflib

trArray=[]

def cb_reload(f):
  global trArray
  conf=rospy.get_param("~")
  trArray=[]
  for name in conf:
    attr=conf[name]
    tr=TransformStamped()
    tr.header.stamp=rospy.Time.now()
    try:
      tr.transform=tflib.dict2tf(attr["transform"])
    except Exception:
      tr.transform.rotation.w=1
    tr.header.frame_id=attr["parent_frame_id"]
    tr.child_frame_id=name
    trArray.append(tr)
  broadcaster.sendTransform(trArray)

def cb_update(t):
  for tr in trArray:
    if tr.child_frame_id==t.child_frame_id:
      tr.header.stamp=t.header.stamp
      tr.transform=t.transform
  broadcaster.sendTransform(trArray)

###############################################################
rospy.init_node('config_tf',anonymous=True)

rospy.Subscriber('reload/config_tf',Bool,cb_reload)
rospy.Subscriber('update/config_tf',TransformStamped,cb_update)

broadcaster=tf2_ros.StaticTransformBroadcaster()
cb_reload(Bool())

try:
  rospy.spin()
except KeyboardInterrupt:
  print("Shutting down")
