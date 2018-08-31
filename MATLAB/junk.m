clc; clear; clf
% p1 = Polygon_mkII([0 1; 0 0 ; 1 0; 1 1])
% p1.plot()
%

t = 0:360;
v1 = [1 0];
tDotCos = t;
tCrossSin = t;
v2 = [cosd(t(:)), sind(t(:))];
for i = 1:numel(t)
    tDotCos(i) = acosd(dot(v1/norm(v1),v2(i,:)/norm(v2(i,:))));
    tCrossSin(i) = asind(cross2d(v1/norm(v1),v2(i,:)/norm(v2(i,:))));
end
plot(t,tDotCos,'r-x',t,tCrossSin,'b-x')
legend('tDotCos','tCrossSin')
