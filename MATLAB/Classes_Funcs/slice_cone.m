function cone2d_new = slice_cone(cone3d,cone2d)
%SLICE_CONE Gives the intersection of a 3D cone with 2D cone
%   Both variables dimensions: 3-by-mpts
cone2d_new = [];
N = size(cone3d,2); % Number of vertices of the 3D cone

% n_cone2d = cross(cone2d(:,1), cone2d(:,2) );

% Inward normals of cone3d
n_cone3d = zeros(3,N);
n_cone3d(:,1:N-1) = -cross(cone3d(:,1:N-1), cone3d(:,2:N) );
n_cone3d(:,N) = -cross(cone3d(:,N), cone3d(:,1) );

Vert = [-cone3d cone2d];
try
    XY_CH_ind = convhull(Vert(1,:)',Vert(2,:)');
    XY_CH = polyshape(Vert(1:2,XY_CH_ind)');
    XY_CH_II = Polygon_mkII(Vert(1:2,XY_CH_ind)');
catch
    return
end
dx = 1e-2;
Qx = dx*cosd([0;120;240]);
Qy = dx*sind([0;120;240]);
%% DEBUG
figure(101); clf
quiver3(0*cone3d(1,:),0*cone3d(1,:),0*cone3d(1,:), ...
    cone3d(1,:), cone3d(2,:), cone3d(3,:),'k-','AutoScale','off')
hold on;
% XY_CH.plot()
quiver3(0*cone2d(1,:),0*cone2d(1,:),0*cone2d(1,:), ...
    cone2d(1,:), cone2d(2,:), cone2d(3,:),'r-','AutoScale','off')
quiver3(0*n_cone3d(1,:),0*n_cone3d(1,:),0*n_cone3d(1,:), ...
    n_cone3d(1,:), n_cone3d(2,:), n_cone3d(3,:),'b-','AutoScale','off')
% quiver3(0*n_cone3d(1,:),0*n_cone3d(1,:),0*n_cone3d(1,:), ...
%     -n_cone3d(1,:), -n_cone3d(2,:), -n_cone3d(3,:),'b-','AutoScale','off')
axis equal
grid on
zlabel('\tau_z','FontSize',20)
xlabel('f_x','FontSize',20)
ylabel('f_y','FontSize',20)
% DEBUG END
%%
[in,~] = inpolygon(Qx(:), Qy(:), Vert(1,XY_CH_ind), Vert(2,XY_CH_ind));
if in
    % The normal indeed spans the origin along with other wrenches
    
else
    return
end

w_top = cone2d(:,1);
w_bot = cone2d(:,2);
% Projection of 2D cone vectors on different face normals
proj_top = dot(repmat(w_top,1,N),n_cone3d);
proj_bot = dot(repmat(w_bot,1,N),n_cone3d);

if find(proj_top < 0)
    x = fsolve(@(x) min(dot( repmat((1-x)*w_top+x*w_bot,1,N),n_cone3d)),0.5);
    w_top = (1-x)*w_top+x*w_bot;
end

if find(proj_bot < 0)
    x = fsolve(@(x) min(10*dot( repmat((1-x)*w_bot+x*w_top,1,N),n_cone3d)),0.5);
    w_bot = (1-x)*w_bot+x*w_top;
end

if (proj_top > 1e-7) & (proj_bot > 1e-7 )
    cone2d_new = cone2d;
    return
end

if (proj_top < 1e-4) & (proj_bot < 1e-4 )
    return
end


end

