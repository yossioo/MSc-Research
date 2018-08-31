%% Contacts search
clearvars -except Tree 
clc; %#ok<*SAGROW>
%% Iterate over each polygon

N = numel(Tree);
ALL = 1:N;
C = {};
poly_C_inds = [];
for o_i = ALL
    % Remove these 2 lines later. Now want to see only for given
    % polygon
    C = {};
    poly_C_inds = [];
    %
    others = ALL ~= o_i;
    Unified = Polygon_mkII(get_unified_poly(Tree(others)));
    Unified.Name = Unified.Name + " - w/o " + num2str(o_i);
    p = Tree{o_i};
    
    % Have to search separately for v2e contacts, e2e end contacts,
    % and v2iv contacts
    
    % Search for common edge segments
    for i = 1:p.N_e
        e = p.Edges([i i+1],:);
        [in,out] = intersect(Unified.Shape,e);
%         [in,out] = intersect(polybuffer(Unified.Shape,1e-5,'JointType','square'),e);
        n = -p.find_normal_at_point(mean(in));
        for in_i = 1:size(in,1)
            C{end+1} = ContactVector(in(in_i,:),n,1,o_i);
            poly_C_inds(end+1) = o_i;
        end
    end
    for i = 1:Unified.N_e
        e = Unified.Edges([i i+1],:);
        [in,out] = intersect(p.Shape,e);
        %         [in,out] = intersect(polybuffer(Unified.Shape,1e-5,'JointType','square'),e);
        n = -p.find_normal_at_point(mean(in));
        for in_i = 1:size(in,1)
            C{end+1} = ContactVector(in(in_i,:),n,1,o_i);
            poly_C_inds(end+1) = o_i;
        end
    end
    
    % Search vertices of this polygon touch another
    [~,on] = Unified.in_polygon(p.Edges(1:end-1,:));
    points = p.Edges(on,:);
    for p_i = 1:size(points,1)
        point = points(p_i,:);
        n = Unified.find_normal_at_point(point);
        for n_i = 1:size(n,1)
            C{end+1} = ContactVector(point,n(n_i,:),1,o_i);
            poly_C_inds(end+1) = o_i;
        end
    end
    % Search vertices of another touch this
    [~,on] = p.in_polygon(Unified.Edges(1:end-1,:));
    points = Unified.Edges(on,:);
    for p_i = 1:size(points,1)
        point = points(p_i,:);
        n = -p.find_normal_at_point(point);
        for n_i = 1:size(n,1)
            C{end+1} = ContactVector(point,n(n_i,:),1,o_i); 
            poly_C_inds(end+1) = o_i;
        end
    end
    
    figure(o_i); clf
    Unified.plot();
    axis equal; grid on; hold on;
    p.plot();
    for c = C
        cont = c{1};
        cont.plot_contact()
    end
end
