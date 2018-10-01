#!/usr/bin/python
import sys
import os
import glob
print(sys.executable)

scale = 10
sdf_file = "/home/yossi/model_editor_models/simple/model.sdf"
sdf_dir = "/home/yossi/model_editor_models/simple/"
# sdf_dir = "/home/yossi/model_editor_models/simple/"



os.chdir(sdf_dir)

files = glob.glob('./*.stl')
print(files)
# Get STL files created by MATLAB


urdf_stub = """
<?xml version="1.0" ?>
<robot name="{name}" xmlns:xacro="http://www.ros.org/wiki/xacro">
  <link name="{name}">
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="model://simple/{filename}" scale="{scaleX} {scaleY} {scaleZ}"/>
      </geometry>
    </collision>
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="model://simple/{filename}" scale="{scaleX} {scaleY} {scaleZ}"/>
      </geometry>
      <material name="grey"/>
    </visual>
    <inertial>
      <origin rpy="0 0 0" xyz="0.0 0.00000 0.0"/>
      <!--<origin xyz="0.0 0.00000 -0.0" rpy="0 0 0"/>-->
      <mass value="1"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>
  </robot>
"""


import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose


rospy.init_node('insert_object',log_level=rospy.INFO)

initial_pose = Pose()
initial_pose.position.x = 0
initial_pose.position.y = 0
initial_pose.position.z = 0.05

rospy.wait_for_service('gazebo/spawn_sdf_model')
spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)

for link in files:
    object_name = link.split("/")[-1].split('.')[0]
    urdf = urdf_stub.format(name = object_name,
    filename = link.split("/")[-1],
    scaleX = scale, scaleY = scale, scaleZ = scale)

    with open(object_name+".urdf",'w') as f:
        f.write(urdf)
    spawn_model_prox(object_name, urdf, "object_name_space", initial_pose, "world")
