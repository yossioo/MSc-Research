function spawn_sdf_stl(filename, color, scale)
%SPAWN_STL Summary of this function goes here
%   Detailed explanation goes here
% file_stub = fopen('urdf_stub');
sdf_stub = fileread('sdf_stub');

sdf_xml = string(sprintf(sdf_stub,"part_"+filename,"link"+filename,...
    filename,scale,scale,scale,...
    filename,scale,scale,scale));
file_sdf = fopen('temp.sdf','w');
fprintf(file_sdf,sdf_xml);
fclose(file_sdf);
a = rosservice('list');
spawn_service = '/gazebo/spawn_sdf_model';
delete_service = '/gazebo/delete_model';
if ismember(spawn_service, a(:,1))
    name = filename.split('.');
%     Delete model
    delete_client = rossvcclient(delete_service);
    delete_req = rosmessage(delete_client);
    delete_req.ModelName = char(name(1));
    resp = call(delete_client,delete_req,'Timeout',5)
    
    
    
    
    spawn_client = rossvcclient(spawn_service);
    spawn_req = rosmessage(spawn_client);
    spawn_req.ReferenceFrame = 'world';
    spawn_req.ModelName = char(name(1));
    spawn_req.ModelXml = char(sdf_xml);
    spawn_req.InitialPose.Position.Z = 1.01;
    resp = call(spawn_client,spawn_req,'Timeout',3)
else
    warning("Gazebo service SPAWN_URDF unavailable")
end
end

