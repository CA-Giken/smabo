#!/usr/bin/env python3

# Python includes
import numpy

# ROS includes
import roslib
import rospy
from geometry_msgs.msg import Vector3,Transform
from tf import transformations
from smabo import tflib
from rviz_tools_py import rviz_tools

Config={
  "mesh": ['smabo','mesh/Sx.stl',1000]
}


# Initialize the ROS Node
rospy.init_node('marker_ycam', anonymous=False, log_level=rospy.INFO, disable_signals=False)
try:
  Config.update(rospy.get_param("/config/sensors"))
except Exception as e:
  print("get_param exception:",e.args)

# Define exit handler
def cleanup_node():
    print("Shutting down node")
    markers.deleteAllMarkers()

rospy.on_shutdown(cleanup_node)

markers = rviz_tools.RvizMarkers('camera', 'ycam_marker')
T1 = transformations.translation_matrix((0,0,0))
try:
  skl=float(Config["mesh"][2])
except Exception:
  skl=1
sc1 = Vector3(skl,skl,skl)
tr2=Transform()
tr2.translation.z=530
tr2.rotation.w=1.
T2=tflib.toRT(tr2)
sc2 = Vector3(400,300,300)

while not rospy.is_shutdown():
  mesh_file1 = "package://"+Config["mesh"][0]+"/"+Config["mesh"][1]
  markers.publishMesh(T1,mesh_file1,'white', sc1, 0.5)
#  markers.publishCube(T2,(0.1,0.1,0.1,0.05), sc2, 0.5)
  rospy.Rate(5).sleep() #5Hz
