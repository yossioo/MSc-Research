%% Demo 1
clc; clear all; %#ok<*CLALL>
warning on verbose
addpath('Classes_Funcs')

finger_d = 10;
%% Create objects
% The script creates a cell array P, which contains the polygonal
% objects.

N = 6; % number of objects
M = 3; % - number of vertices in polygons
Noise = 0;
create_script

% figure(1); clf
% P{1}.plot(); hold on;
% for i = 2:numel(P)
%     P{i}.plot()
% end
% axis equal
% grid on

%% Rearrange the objects

rearrange_script

%% Find inter-object contacts

find_i_contacts_script

%% Find fingers

find_fingers_script

%% Evaluate grasp and Move fingers (if needed)

% evaluate_script

%% Display results
% 
figure(19);
clf;

PolyList{1}.plot(); hold on; axis equal; grid on;
for i = 2:numel(PolyList)
    PolyList{i}.plot()
    text(PolyList{i}.Center(1)-5,PolyList{i}.Center(2),num2str(i))
end
t = linspace(0,2*pi);
x = finger_d/2*cos(t); y = finger_d/2*sin(t);
axis manual


Fingers.ContactVector(4).point_on_the_line = ...
    PolyList{4}.point_from_edgePosition(2,0.9)


p = zeros(numel(Fingers(:,6)),2);
for i = 1:numel(Fingers(:,6))
    f = Fingers{i,6};
    f.plot_contact('b')
    allowed_reg = [PolyList{Fingers.PolygonNum(i)}.point_from_edgePosition(...
        Fingers.EdgeNum(i), Fingers.EdgeRange(i,1));
PolyList{Fingers.PolygonNum(i)}.point_from_edgePosition(...
        Fingers.EdgeNum(i), Fingers.EdgeRange(i,2))];
    plot(allowed_reg(:,1),allowed_reg(:,2),'c','LineWidth',2);
    p(i,:) = f.get_finger_center(finger_d);
    c = fill(p(i,1)+x,p(i,2)+y,'b','FaceAlpha',.2);
%     s = scatter(p(1),p(2),100,'b', 'filled','MarkerFaceAlpha',0.2);
    f.draw_inf_line('k')
end
