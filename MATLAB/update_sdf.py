#!/usr/bin/python
import sys
import os
import glob
print(sys.executable)

scale = 100
sdf_file = "/home/yossi/model_editor_models/simple/model.sdf"
sdf_dir = "/home/yossi/model_editor_models/simple/"
# sdf_dir = "/home/yossi/model_editor_models/simple/"



os.chdir(sdf_dir)

files = glob.glob('./*.stl')
# print(files)

model_prefix = """<?xml version='1.0'?>
<sdf version='1.6'>
  <model name='simple'>
  """


model_suffix = """<static>0</static>
    <allow_auto_disable>1</allow_auto_disable>
  </model>
</sdf>
"""
link_string = """  <link name='{name}'>
  <pose frame=''>0 0 0 0 -0 0</pose>
  <inertial>
    <mass>1</mass>
    <inertia>
      <ixx>0.166667</ixx>
      <ixy>0</ixy>
      <ixz>0</ixz>
      <iyy>0.166667</iyy>
      <iyz>0</iyz>
      <izz>0.166667</izz>
    </inertia>
  </inertial>
  <visual name='visual'>
    <pose frame=''>0 0 0 0 -0 0</pose>
    <geometry>
      <mesh>
        <uri>model://simple/{filename}</uri>          <scale>{scaleX} {scaleY} {scaleZ}</scale>
      </mesh>
    </geometry>
    <material>
      <lighting>1</lighting>
      <script>
        <uri>file://media/materials/scripts/gazebo.material</uri>
        <name>Gazebo/Grey</name>
      </script>
      <ambient>0.3 0.3 0.3 1</ambient>
      <diffuse>0.7 0.7 0.7 1</diffuse>
      <specular>0.01 0.01 0.01 1</specular>
      <emissive>0 0 0 1</emissive>
    </material>
    <cast_shadows>1</cast_shadows>
    <transparency>0</transparency>
  </visual>
  <collision name='collision'>
    <laser_retro>0</laser_retro>
    <max_contacts>10</max_contacts>
    <pose frame=''>0 0 0 0 -0 0</pose>
    <geometry>
      <mesh>
        <uri>model://simple/{filename}</uri>
        <scale>{scaleX} {scaleY} {scaleZ}</scale>
      </mesh>
    </geometry>
    <surface>
      <friction>
        <ode>
          <mu>1</mu>
          <mu2>1</mu2>
          <fdir1>0 0 0</fdir1>
          <slip1>0</slip1>
          <slip2>0</slip2>
        </ode>
        <torsional>
          <coefficient>1</coefficient>
          <patch_radius>0</patch_radius>
          <surface_radius>0</surface_radius>
          <use_patch_radius>1</use_patch_radius>
          <ode>
            <slip>0</slip>
          </ode>
        </torsional>
      </friction>
      <bounce>
        <restitution_coefficient>0</restitution_coefficient>
        <threshold>1e+06</threshold>
      </bounce>
      <contact>
        <collide_without_contact>0</collide_without_contact>
        <collide_without_contact_bitmask>1</collide_without_contact_bitmask>
        <collide_bitmask>1</collide_bitmask>
        <ode>
          <soft_cfm>0</soft_cfm>
          <soft_erp>0.2</soft_erp>
          <kp>1e+13</kp>
          <kd>1</kd>
          <max_vel>0.01</max_vel>
          <min_depth>0</min_depth>
        </ode>
        <bullet>
          <split_impulse>1</split_impulse>
          <split_impulse_penetration_threshold>-0.01</split_impulse_penetration_threshold>
          <soft_cfm>0</soft_cfm>
          <soft_erp>0.2</soft_erp>
          <kp>1e+13</kp>
          <kd>1</kd>
        </bullet>
      </contact>
    </surface>
  </collision>
</link>
"""


f = open(sdf_file, "w")
print("File {} opened successfully.".format(sdf_file))
f.write(model_prefix)

for link in files:
    f.write(link_string.format(name = link.split("/")[-1].split('.')[0],
    filename = link.split("/")[-1],
    scaleX = scale, scaleY = scale, scaleZ = scale))
f.write(model_suffix)

f.close()

print("File {} closed successfully.".format(sdf_file))





import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose


rospy.init_node('insert_object',log_level=rospy.INFO)

initial_pose = Pose()
initial_pose.position.x = 1
initial_pose.position.y = 1
initial_pose.position.z = 1

f = open(sdf_file,'r')
sdff = f.read()

rospy.wait_for_service('gazebo/spawn_sdf_model')
spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
spawn_model_prox("some_robo_name", sdff, "robotos_name_space", initial_pose, "world")
