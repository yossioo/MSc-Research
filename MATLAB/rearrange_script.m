%% This script implements the search for new configuration of the objects
warning off MATLAB:polyshape:boundary3Points
%% First we wish to find the root object:
max_concave_angle = 0;
Root_Object = [];

for object = P
    p = object{1};
    conc_vert_angles = 360-p.V_angles(p.Concave_ind);
    if max(conc_vert_angles) > max_concave_angle
        Root_Object = p;
        Tree{1} = p;
    end
end

if isempty(Root_Object) % no concave vertices found
    % Look for couple of inner angles higher than 90
    poly_list = {};
    vert_list = {};
    for object = P
        p = object{1};
        if find(p.V_angles > 90)
            poly_list{end+1} = p;
            diff_w_90 = p.V_angles - 90;
            diff_w_90(diff_w_90<0) = 180;
            ind = find(min(diff_w_90) == diff_w_90);
            vert_list{end+1} = ind(1);
        end
    end
    if numel(poly_list) > 1 % Found 2 such vertices
        % We wish to align the vertices together and rotate objects to
        % fit ------- What about case with more than 2 such objects ?
        edge_lengths_1 = repmat(poly_list{1}.E_lengths,3,1);
        edge_lengths_2 = repmat(poly_list{2}.E_lengths,3,1);
        adj_edge(1,1) = edge_lengths_1(vert_list{1}+poly_list{1}.N_e-1);
        adj_edge(1,2) = edge_lengths_1(vert_list{1}+poly_list{1}.N_e);
        adj_edge(2,1) = edge_lengths_2(vert_list{2}+poly_list{2}.N_e-1);
        adj_edge(2,2) = edge_lengths_2(vert_list{2}+poly_list{2}.N_e);
        
        [i,j] = find(adj_edge == max(adj_edge(:)));
        i = i(1);  % In case some lengths are equal
        j = j(1);
        Root_Object = poly_list{i(1)};
        Tree(1) = poly_list(i);
        Tree(2) = poly_list(3-i);
        
        V1 = Tree{1}.Edges(vert_list{i},:);
        V2 = Tree{2}.Edges(vert_list{3-i},:);
        
        if j == 2% Longest edge is the following edge on the root object
            edge_base_direction = diff(Tree{1}.Edges([vert_list{i},vert_list{i}+1],:))
            
            % select leading edge on child object
            if vert_list{3-i} == 1
                edge_2_align_direction = diff(Tree{2}.Edges([end,end-1],:))
            else
                edge_2_align_direction = diff(Tree{2}.Edges([vert_list{3-i},vert_list{3-i}-1],:))
            end
        else % j==1 - on root object longest edge is the leading edge
            
            if vert_list{i} == 1
                edge_base_direction = diff(Tree{1}.Edges([end,end-1],:))
            else
                edge_base_direction = diff(Tree{1}.Edges([vert_list{i},vert_list{i}-1],:))
            end
            edge_2_align_direction = diff(Tree{2}.Edges([vert_list{3-i},vert_list{3-i}+1],:))
        end
        % First we rotate about the contacting vertex
        Tree{2}.rotate(-angle_of_2_vec(edge_base_direction, edge_2_align_direction), V2)
        
        % Then translate
        Tree{2}.translate(V1-V2);
        
        
    else % No luck
        % We will look for 2 longest edges and stack them together
        % Find longest edges
        Tree = {};
        e_lengths = [0 0];
        e_ids= [0 0];
        for object = P
            p = object{1};
            max_e_length = max(p.E_lengths);
            max_e_id = find(max_e_length == p.E_lengths);
            
            if isempty(Tree)
                Tree{1} = p;
                e_lengths(1) = max_e_length;
                e_ids(1) = max_e_id(1);
            else
                if max_e_length>e_lengths(1)
                    Tree{2} = Tree{1};
                    Tree{1} = p;
                    e_lengths(2) = e_lengths(1) ;
                    e_ids(2) = e_ids(1);
                    e_lengths(1) = max_e_length;
                    e_ids(1) = max_e_id(1);
                elseif max_e_length>e_lengths(2)
                    Tree{2} = p;
                    e_lengths(2) = max_e_length;
                    e_ids(2) = max_e_id(1);
                end
            end
        end
        
        % Now we have 2 objects with longest edges
        V1 = mean(Tree{1}.Edges([e_ids(1),e_ids(1)+1],:));
        base_direction = Tree{1}.find_vector_from_vertex(e_ids(1),1);
        align_direction = -Tree{2}.find_vector_from_vertex(e_ids(2),1);
        V2 = Tree{2}.Edges(e_ids(2),:);
        Tree{2}.rotate(-angle_of_2_vec(base_direction, align_direction), V2);
        Tree{2}.translate(V1-V2);
    end
    
end

%% Stacking process
clear p
% Remove items that are stacked from the initial cell array
for i = 1:numel(Tree)
    ind = is_Polygon_in_array(Tree{i},P);
    if ind
        P(ind) = [];
        disp(Tree{i}.Name + " removed from  P")
    end
end

% disp('P contains:')
% for i = 1:numel(P)
%     fprintf("P{%d}: %s\n",i,P{i}.Name)
% end

while ~isempty(P)
    % Unify the shape
    uni = get_unified_poly(Tree);
    UnifiedPolygon = Polygon_mkII(round(uni.Vertices,3));
    outer_angles = 360-UnifiedPolygon.V_angles;
    outer_angles(~UnifiedPolygon.Concave_ind) = 0;
    conc_angles = 360-UnifiedPolygon.V_angles(UnifiedPolygon.Concave_ind);
    biggest_conc_ind = find(max(outer_angles) == outer_angles,1);
    outer_angles(~UnifiedPolygon.Concave_ind) = 1e3;
    % Search for shape with widest angle
    max_angle = 0;
    P_ind = 0;
    v_ind = 0;
    for i = 1:numel(P)
        if max(P{i}.V_angles) > max_angle
            P_ind = i;
            v_ind = find(max(P{i}.V_angles) > max_angle,1);
            max_angle = max(P{i}.V_angles);
        end
    end
    
    if find(max_angle<=conc_angles)
        % Can fit a poly somewhere
        concavity_left_after_stacking = outer_angles-max_angle;
        concavity_left_after_stacking(concavity_left_after_stacking<0) = 1e3;
        conc_ind = find(concavity_left_after_stacking == min(concavity_left_after_stacking),1);
        stack_poly_to_vertex_w_edge(UnifiedPolygon, P{P_ind}, conc_ind, v_ind,1)

        Tree{end+1} = P{P_ind};
%         Tree{end}.Name = strcat(num2str(numel(Tree))," - ",Tree{end}.Name );
        P(P_ind) = [];
        
    else
        % Push back the object, maybe later will be some concavity to
        % use.
        P{end+1} = P{P_ind};
        P(P_ind) = [];
    end
    
    if numel(P) > 0
        disp("Not done yet")
        disp('P contains:')
        for i = 1:numel(P)
            fprintf("P{%d}: %s\n",i,P{i}.Name)
        end
    end
    
    figure(19)
    clf;
    Tree{1}.plot(); hold on;
    for i = 2:numel(Tree)
%         Tree{i}.Name = strcat(num2str(i)," - ",Tree{i}.Name) ;
        Tree{i}.plot()
        
    end
    axis equal
    grid on
    hold off
    
end

% So far so good. Think over the logic of stacking inside of most
% fitting concavity, or the largest one or smth.


