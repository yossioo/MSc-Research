%% Demo 1
clc; clear;
warning on verbose
addpath('Classes_Funcs')
%% Create objects
% The script creates a cell array P, which contains the polygonal
% objects.

N = 3; % number of objects
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

%% Display results
% 
% figure(20);
% clf;
% 
% PolyList{1}.plot(); hold on;
% for i = 2:numel(PolyList)
%     PolyList{i}.plot()
%     text(PolyList{i}.Center(1)-5,PolyList{i}.Center(2),num2str(i))
% end
% 
% axis equal
% grid on
