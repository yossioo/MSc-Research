clc;

clearvars -except Fingers finger_d PolyList TableVarNames

%% We wish to examine contact combinations
% If there are more than 1 contact groups we test all
N_p_i = numel(PolyList);
Groups_num_per_p = zeros(1,N_p_i);

for p_i = 1:N_p_i
    Groups_num_per_p(p_i) = max(Fingers.ContactGroup(Fingers.PolygonNum == p_i));
end
TotalGroups = prod(Groups_num_per_p);
GroupCombinations = zeros(TotalGroups,N_p_i);

for p_i = 1:N_p_i
%     Groups_num(p_i) = max(Fingers.ContactGroup(Fingers.PolygonNum == p_i));
groups = Fingers.ContactGroup(Fingers.PolygonNum == p_i);
GroupCombinations(:,p_i) = repmat(groups,TotalGroups/numel(groups),1);
end

%% Test whether a group forms force closure contact

t = linspace(0,2*pi);
x = finger_d/2*cos(t); y = finger_d/2*sin(t);

for combination = GroupCombinations'
    contact_set = [];
    for p_i = 1:N_p_i
        f_i = Fingers.PolygonNum == p_i & Fingers.ContactGroup == combination(p_i);
        contact_set = vertcat(contact_set, Fingers.ContactVector(f_i));
    end
    % First check whether there 4 or more contacts
    N_cs = numel(contact_set);
    if N_cs >= 4
        % Get wrench convex hull
        [W_CH,W] = W_CH_from_Contacts(contact_set,Polygon_mkII(get_unified_poly(PolyList(:))).Center);
        K = convhulln([W_CH'; [0 0 0]]);
        if ismember(size(W_CH,2)+1,K)
            % the C-Hull did not contain the origin
            %% WIP
            % We wish to iterate over the finger contacts to see
            % whether some can make the CH to contain the origin
            %%% If no contact adjustment can solve it , skip the
            %%% configuration (or save it for double contact test?)
            ind = 1:numel(combination);
            for i = ind
                cts = contact_set(ind~=i);
                W_CH_temp = W_CH_from_Contacts(contact_set,Polygon_mkII(get_unified_poly(PolyList(:))).Center);
                %% WIP HERE
                while 0 
                    for e_i = 1:p.N_e
%                         n = p.Inner_normals(e_i,:);
%                         Move_from_vertex_ratio = 0.05; % set to positive if want to stay away from the vertices
%                         e1 = (1-Move_from_vertex_ratio)*p.Edges(e_i,:) +...
%                             Move_from_vertex_ratio * p.Edges(e_i+1,:);
%                         e2 = Move_from_vertex_ratio*p.Edges(e_i,:) +...
%                             (1-Move_from_vertex_ratio) * p.Edges(e_i+1,:);
%                         w = [n, cross2d(e1-p.Center,n)/p.Area^0.5;
%                             n, cross2d(e2-p.Center,n)/p.Area^0.5]';
%                         new_EGW = slice_cone(-W_CH,w);
                    end
                end
            end
        else
        end
    else
        % less then 4
        % Add contacts to the unified polygon?
        
    end
    PolyList{1}.plot(); hold on; axis equal; grid on;
    for i = 2:numel(PolyList)
        PolyList{i}.plot()
        text(PolyList{i}.Center(1)-5,PolyList{i}.Center(2),num2str(i))
    end
    axis manual
    p = zeros(numel(contact_set),2);
    for c_i = 1:numel(contact_set)
        f = contact_set(c_i);
        f.plot_contact('b')
        p(c_i,:) = f.get_finger_center(finger_d);
        c = fill(p(c_i,1)+x,p(c_i,2)+y,'b','FaceAlpha',.2);
        %     s = scatter(p(1),p(2),100,'b', 'filled','MarkerFaceAlpha',0.2);
        f.draw_inf_line('k')
    end

end
