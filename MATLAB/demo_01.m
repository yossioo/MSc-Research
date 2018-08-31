%% Demo 1
clc; clear;
warning on verbose
addpath('Classes_Funcs')
%% Create objects
% The script creates a cell array P, which contains the polygonal
% objects.

N = 3; % number of objects
M = 4; % - number of vertices in polygons
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

% find_fingers_script

%% Display results

figure(2);
clf;

Tree{1}.plot(); hold on;
for i = 2:numel(Tree)
    Tree{i}.plot()
end

axis equal
grid on
