clc; clear; clf
% % p1 = Polygon_mkII([0 1; 0 0 ; 1 0; 1 1])
% % p1.plot()
% %
% 
% t = 0:360;
% v1 = [1 0];
% tDotCos = t;
% tCrossSin = t;
% v2 = [cosd(t(:)), sind(t(:))];
% for i = 1:numel(t)
%     tDotCos(i) = acosd(dot(v1/norm(v1),v2(i,:)/norm(v2(i,:))));
%     tCrossSin(i) = asind(cross2d(v1/norm(v1),v2(i,:)/norm(v2(i,:))));
% end
% plot(t,tDotCos,'r-x',t,tCrossSin,'b-x')
% legend('tDotCos','tCrossSin')
%% 
[xx,yy] = meshgrid(-1:.1:1);
A = -1/sqrt(2);
B = -0/sqrt(2);
C = 1;
D = 0;
z = -(D +A*xx +B*yy)/C;

surf(xx,yy,z)
axis equal; hold on
contour(xx,yy,z,10)
xlabel('x')
ylabel('y')
zlabel('z')

quiver3(0,0,0,A,B,C,'r','AutoScale','off')

z(xx==1 & yy == 0)
