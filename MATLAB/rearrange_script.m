%% This script implements the search for new configuration of the objects

%% First we wish to find the root object:
max_concave_angle = 0;
Root_Object = [];

for object = P
    p = object{1};
    conc_vert_angles = 360-p.V_angles(p.concave_ind);
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
        if find(p.V_angles >= 90)
            poly_list{end+1} = p;
            diff_w_90 = p.V_angles - 90;
            diff_w_90(diff_w_90<=0) = 180;
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
                e_ids(1) = max_e_id;
            else
                if max_e_length>e_lengths(1)
                    Tree{2} = Tree{1};
                    Tree{1} = p;
                    e_lengths(2) = e_lengths(1) ;
                    e_ids(2) = e_ids(1);
                    e_lengths(1) = max_e_length;
                    e_ids(1) = max_e_id;
                elseif max_e_length>e_lengths(2)
                    Tree{2} = p;
                    e_lengths(2) = max_e_length;
                    e_ids(2) = max_e_id;
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

uni = simplify(get_unified_poly(Tree));
Uni_mkII = Polygon_mkII(round(uni.Vertices,3));

figure(3)
for i = 1: Uni_mkII.N_e
    Uni_mkII.plot()
end

axis equal
