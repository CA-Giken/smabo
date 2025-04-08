#!/usr/bin/env python3

import rospy
import tf2_ros
import numpy as np
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Pose
from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *
from scipy.spatial.transform import Rotation as R
from smabo import tflib

Config={
  "base_frame_id":"base",
  "frame_id":"tool0_controller",
  "initial_pose":[0,0,1000,180,0,0]
}

tfThis=TransformStamped()

def startMarker(ev):
  global tfThis;
  try:
    ts=tfBuffer.lookup_transform(Config["base_frame_id"],Config["frame_id"],rospy.Time())
  except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    pass
  else:
    tfThis=ts
    int_marker.pose.position=ts.transform.translation
    int_marker.pose.orientation=ts.transform.rotation
    server.insert(int_marker, processFeedback)
    server.applyChanges()
  rospy.Timer(rospy.Duration(1),followMarker,oneshot=True)

def initMarker(ev):
  global tfThis;
  tfThis.header.stamp=rospy.Time.now()
  tfThis.transform.translation.x=Config["initial_pose"][0]
  tfThis.transform.translation.y=Config["initial_pose"][1]
  tfThis.transform.translation.z=Config["initial_pose"][2]
  euler=Config["initial_pose"][3:6]
  quat=R.from_euler('XYZ',euler,degrees=True).as_quat()
  tfThis.transform.rotation.x=quat[0]
  tfThis.transform.rotation.y=quat[1]
  tfThis.transform.rotation.z=quat[2]
  tfThis.transform.rotation.w=quat[3]
  if ev is not None: pub_tf.publish(tfThis)
  return tfThis.transform

def followMarker(ev):
  global tfThis;
  try:
    ts=tfBuffer.lookup_transform(Config["base_frame_id"],Config["frame_id"],rospy.Time())
  except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    pass
  else:
    err=np.array([
      tfThis.transform.translation.x-ts.transform.translation.x,
      tfThis.transform.translation.y-ts.transform.translation.y,
      tfThis.transform.translation.z-ts.transform.translation.z])
    if np.linalg.norm(err)>1:
      tfThis=ts
      header=Header()
      header.stamp=rospy.Time.now()
      pose=Pose()
      pose.position=ts.transform.translation
      pose.orientation=ts.transform.rotation
      server.setPose(int_marker.name,pose,header)
      server.applyChanges()
  rospy.Timer(rospy.Duration(1),followMarker,oneshot=True)

def processFeedback(feedback):
  global tfThis;
  p = feedback.pose.position
  o = feedback.pose.orientation
  tfThis.header.stamp=rospy.Time.now()
  tfThis.transform.translation=p
  tfThis.transform.rotation=o
  tfThis.header.seq=tfThis.header.seq+1
  pub_tf.publish(tfThis)

if __name__=="__main__":
  rospy.init_node("move_marker")
  ###Load Config
  try:
    Config.update(rospy.get_param("~config"))
  except Exception as e:
    print("get_param exception:",e.args)

  tfThis.header.frame_id=Config["base_frame_id"]
  tfThis.child_frame_id=Config["frame_id"]
    
  pub_tf=rospy.Publisher('/update/config_tf',TransformStamped,queue_size=1);

  server = InteractiveMarkerServer("move_marker")
    
  # create an interactive marker for our server
  int_marker = InteractiveMarker()
  int_marker.header.frame_id = Config["base_frame_id"]
  int_marker.name = "move_marker"
  int_marker.description = "Mover"
  int_marker.scale=300

  # create a grey box marker
#  stl_marker = Marker()
#  stl_marker.type = Marker.MESH_RESOURCE
#  stl_marker.mesh_resource = "package://vcam3d/mesh/"+Config["mesh"]+".stl"
#  stl_marker.scale.x = 1
#  stl_marker.scale.y = 1
#  stl_marker.scale.z = 1
#  stl_marker.color.r = 0.5
#  stl_marker.color.g = 0.5
#  stl_marker.color.b = 0.7
#  stl_marker.color.a = 0.5
  # create a non-interactive control which contains the box
#  stl_control = InteractiveMarkerControl()
#  stl_control.always_visible = True
#  stl_control.markers.append( stl_marker )
  # add the control to the interactive marker
#  int_marker.controls.append( stl_control )

  transx_control = InteractiveMarkerControl()
  transx_control.always_visible = True
  transx_control.name = "move_x"
  transx_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
  transx_control.orientation.w = 1
  transx_control.orientation.x = 1
  int_marker.controls.append(transx_control);

  transy_control = InteractiveMarkerControl()
  transy_control.always_visible = True
  transy_control.name = "move_y"
  transy_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
  transy_control.orientation.w = 1
  transy_control.orientation.z = 1
  int_marker.controls.append(transy_control);

  transz_control = InteractiveMarkerControl()
  transz_control.always_visible = True
  transz_control.name = "move_z"
  transz_control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
  transz_control.orientation.w = 1
  transz_control.orientation.y = 1
  int_marker.controls.append(transz_control);

  rotx_control = InteractiveMarkerControl()
  rotx_control.always_visible = True
  rotx_control.name = "rot_x"
  rotx_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
  rotx_control.orientation.w = 1
  rotx_control.orientation.x = 1
  int_marker.controls.append(rotx_control);

  roty_control = InteractiveMarkerControl()
  roty_control.always_visible = True
  roty_control.name = "rot_y"
  roty_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
  roty_control.orientation.w = 1
  roty_control.orientation.z = 1
  int_marker.controls.append(roty_control);

  rotz_control = InteractiveMarkerControl()
  rotz_control.always_visible = True
  rotz_control.name = "rot_z"
  rotz_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
  rotz_control.orientation.w = 1
  rotz_control.orientation.y = 1
  int_marker.controls.append(rotz_control);

  tfBuffer=tf2_ros.Buffer()
  listener=tf2_ros.TransformListener(tfBuffer)
  rospy.Timer(rospy.Duration(3),startMarker,oneshot=True)

  rospy.spin()
