%% Demo 1
clc; clear all;
%#ok<*CLALL>
%#ok<*UNRCH>

% warning on verbose
warning off YOSSI:NoEdgeOppositeToCH
addpath('Classes_Funcs')

global DEBUG
DEBUG = true;
finger_d = 10;
%% Create objects
% The script creates a cell array P, which contains the polygonal
% objects.

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
    %     text(PolyList{i}.Center(1)-5,PolyList{i}.Center(2),num2str(i))
end
t = linspace(0,2*pi);
x = finger_d/2*cos(t); y = finger_d/2*sin(t);
axis manual

if 1 %Show all results
    finger_centers = zeros(numel(Fingers(:,6)),2);
    for i = 1:numel(Fingers(:,7))
        f = Fingers{i,7};
        f.plot_contact('b')
        allowed_reg = [PolyList{Fingers.PolygonNum(i)}.point_from_edgePosition(...
            Fingers.EdgeNum(i), Fingers.EdgeRange(i,1));
            PolyList{Fingers.PolygonNum(i)}.point_from_edgePosition(...
            Fingers.EdgeNum(i), Fingers.EdgeRange(i,2))];
        plot(allowed_reg(:,1),allowed_reg(:,2),'c','LineWidth',2);
        finger_centers(i,:) = f.get_finger_center(finger_d);
        f_c = fill(finger_centers(i,1)+x,finger_centers(i,2)+y,'b','FaceAlpha',.2);
        % s = scatter(p(1),p(2),100,'b', 'filled','MarkerFaceAlpha',0.2);
        % f.draw_inf_line('k')
    end
else % show only selected results
    
    for p_i = 1:numel(PolyList) 
        p_ind = Fingers.PolygonNum == p_i;
        p_Fingers = Fingers(p_ind,:);
        groups = max(Fingers.ContactGroup(p_ind));
        GQM_max_ind = max_ind(p_Fingers.Group_GQM(:));
        for f = p_Fingers.ContactVector(GQM_max_ind)'
            f.plot_contact('b')
            finger_center = f.get_finger_center(finger_d);
%             axis auto
            f_c = fill(finger_center(1)+x,finger_center(2)+y,'b','FaceAlpha',.2);
%             f.draw_inf_line('k')
        end
    end
end
% for i = 1:numel(Fingers(:,7))
%     f = Fingers{i,7};
%     f.plot_contact('b')
%     allowed_reg = [PolyList{Fingers.PolygonNum(i)}.point_from_edgePosition(...
%         Fingers.EdgeNum(i), Fingers.EdgeRange(i,1));
% PolyList{Fingers.PolygonNum(i)}.point_from_edgePosition(...
%         Fingers.EdgeNum(i), Fingers.EdgeRange(i,2))];
%     plot(allowed_reg(:,1),allowed_reg(:,2),'c','LineWidth',2);
%     finger_centers(i,:) = f.get_finger_center(finger_d);
%     f_c = fill(finger_centers(i,1)+x,finger_centers(i,2)+y,'b','FaceAlpha',.2);
% %     s = scatter(p(1),p(2),100,'b', 'filled','MarkerFaceAlpha',0.2);
%     f.draw_inf_line('k')
% end
