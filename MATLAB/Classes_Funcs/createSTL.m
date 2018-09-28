function createSTL(shape,filename,h,scale_factor)
%CREATESTL Summary of this function goes here
%   Detailed explanation goes here
if nargin < 4
    scale_factor = 1;
end
if nargin < 3
    h = 10;
end
if nargin < 2
    filename = 'output.stl';
end

if strcmp(class(shape),'polyshape')
    vert = shape.Vertices;
else
    error("Input shape has to be a polyshape object.")
end

vertices2D = scale_factor*(vert);
vertices3D = [vertices2D, zeros(size(vertices2D,1),1)];
vertices3D = [vertices3D+[0 0 scale_factor*h];vertices3D];
fv.vertices = vertices3D;



TR = delaunayTriangulation(vertices2D);

tricenters = incenter(TR);
good_tri = shape.isinterior(tricenters/scale_factor);
good_conn_list = TR.ConnectivityList(good_tri,:);
triplot(good_conn_list,shape.Vertices(:,1),shape.Vertices(:,2))
% for i = 1:size(shape.Vertices)
%     text(shape.Vertices(i,1),shape.Vertices(i,2),num2str(i))
% end
% The triangles here are in counter clockwise order
N_e = size(shape.Vertices,1);
side_tri = zeros(N_e*2,3);
for e = 1:N_e
    temp_ind = [e e+N_e e+1;  e+N_e e+1+N_e e+1];
    temp_ind = temp_ind -2*N_e*(temp_ind-2*N_e>0);
    side_tri([2*e-1 2*e],:) = temp_ind;
end
fv.faces = [good_conn_list; side_tri;fliplr(good_conn_list)+N_e];

stlwrite(char(filename),fv)        % Save to binary .stl

end

