% Test STLWRITE
clc;
addpath('Classes_Funcs')
clearvars -except PolyList
rosshutdown
rosinit
scale = 1;
% R = 30;
% colors = jet(4);
% P{1} = Polygon_mkII([0 0; R*[cosd(30) sind(30)]; R*1 0],"Tri_A", colors(1,:));
% P{2} = Polygon_mkII([0 0; R*[cosd(30) sind(30)]; R*1 0],"Tri_B", colors(2,:));
% P{3} = Polygon_mkII([0 0; R*[cosd(30) sind(30)]; R*1 0],"Tri_C", colors(3,:));
% P{4} = Polygon_mkII([0 0; 30 30; 25 -5; -25 -10; -30 30],"Concave",  colors(4,:));
% 

stl_folder = "/home/yossi/model_editor_models/simple/";
ff = mkdir(stl_folder);
delete(stl_folder+"*.stl")
h = 50;
for p_cell = PolyList(:)'
    
    p = p_cell{1};
    full_filename =  stl_folder+ p.Name + ".stl";
    createSTL(p.Shape, full_filename)

    spawn_sdf_stl(p.Name + ".stl", 1, 1)
    % spawn_urdf_stl(p.Name + ".stl", 1, 1)
end





% p = P{4};
% h = 10;
% vertices2D = p.Shape.Vertices+[5 0];
% vertices3D = [vertices2D, zeros(size(vertices2D,1),1)];
% vertices3D = [vertices3D+[0 0 h];vertices3D];
% fv.vertices = vertices3D;
% 
% TR = delaunayTriangulation(vertices2D);
% 
% tricenters = incenter(TR);
% good_tri = p.Shape.isinterior(tricenters);
% % TR.ConnectivityList(~good_tri,:) = []
% good_conn_list = TR.ConnectivityList(good_tri,:);
% triplot(good_conn_list,p.Shape.Vertices(:,1),p.Shape.Vertices(:,2))
% for i = 1:size(p.Shape.Vertices)
%     text(p.Shape.Vertices(i,1),p.Shape.Vertices(i,2),num2str(i))
% end
% % The triangles here are in counter clockwise order
% 
% side_tri = zeros(p.N_e*2,3);
% for e = 1:p.N_e
%     temp_ind = [e e+p.N_e e+1;  e+p.N_e e+1+p.N_e e+1];
%     temp_ind = temp_ind -10*(temp_ind-10>0);
%     side_tri([2*e-1 2*e],:) = temp_ind;
% end
% fv.faces = [good_conn_list; side_tri;fliplr(good_conn_list)+p.N_e];
% 
% 
% stlwrite('test.stl',fv)        % Save to binary .stl

%%
