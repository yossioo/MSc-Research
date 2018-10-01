function [outputArg1,outputArg2] = spawn_box(size, pos)
%SP Summary of this function goes here
%   Detailed explanation goes here
sdf_stub = '<?xml version="1.0"?><sdf version="1.4"><model name="%s">  <pose>0 0 0  0 0 0</pose>  <static>false</static>    <link name="%s">      <inertial><mass>1.0</mass>       <inertia> <!-- inertias are tricky to compute -->          <!-- http://gazebosim.org/tutorials?tut=inertia&cat=build_robot -->          <ixx>0.083</ixx>       <!-- for a box: ixx = 0.083 * mass * (y*y + z*z) -->          <ixy>0.0</ixy>         <!-- for a box: ixy = 0 -->          <ixz>0.0</ixz>         <!-- for a box: ixz = 0 -->          <iyy>0.083</iyy>       <!-- for a box: iyy = 0.083 * mass * (x*x + z*z) -->          <iyz>0.0</iyz>         <!-- for a box: iyz = 0 -->          <izz>0.083</izz>       <!-- for a box: izz = 0.083 * mass * (x*x + y*y) -->        </inertia>      </inertial>      <collision name="collision">        <geometry>          <box>            <size>%f %f %f</size>          </box>        </geometry>      </collision>      <visual name="visual">        <geometry>          <box>            <size>%f %f %f</size>          </box>        </geometry>      </visual>    </link>  </model></sdf>';

a = rosservice('list');
spawn_service = '/gazebo/spawn_sdf_model';
% world_prop_service = '/gazebo/get_world_properties'


% if ismember(world_prop_service, a(:,1))
%     client = rossvcclient(world_prop_service);
%     req = rosmessage(client);
%     resp = call(client,req,'Timeout',5);
% else
%     warning("Gazebo service WorldProperties unavailable")
% end
name = "box_"+randi(5000,1,1);
sdf_xml = sprintf(sdf_stub,"dummy_"+name,"link",...
    size(1),size(2),size(3),...
    size(1),size(2),size(3));
if ismember(spawn_service, a(:,1))
    
    spawn_client = rossvcclient(spawn_service);
    spawn_req = rosmessage(spawn_client);
    spawn_req.ReferenceFrame = 'world';
    spawn_req.ModelName = char(name(1));
    spawn_req.ModelXml = sdf_xml;
    spawn_req.InitialPose.Position.X = pos(1);
    spawn_req.InitialPose.Position.Y = pos(2);
    spawn_req.InitialPose.Position.Z = pos(3);
    resp = call(spawn_client,spawn_req,'Timeout',3)
else
    warning("Gazebo service SPAWN_URDF unavailable")
end
end

