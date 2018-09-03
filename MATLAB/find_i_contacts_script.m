%% Contacts search
 %#ok<*SAGROW>
clc; %#ok<*SAGROW>
%% Iterate over each polygon

N = numel(PolyList);
ALL = 1:N;
C = {};
C_poly_inds = [];
for o_i = ALL
    % Remove these 2 lines later. Now want to see only for given
    % polygon
    % C = {};
    % C_poly_inds = [];
    %
    others = ALL ~= o_i;
    Unified = Polygon_mkII(get_unified_poly(PolyList(others)));
    Unified.Name = Unified.Name + " - w/o " + num2str(o_i);
    p = PolyList{o_i};
    
    % Have to search separately for v2e contacts, e2e end contacts,
    % and v2iv contacts
    
    
    % Search for common edge segments
    d = 1e-3*Unified.Area; % Bufffer measure
    for i = 1:p.N_e
        e = p.Edges([i i+1],:);
        [in,out] = intersect(polybuffer(Unified.Shape,d),e);
        if norm(diff(in)) > d*3
            n = -p.find_normal_at_point(mean(in));
            if isempty(n)
                continue
            end
            for in_i = 1:size(in,1)
                C{end+1} = ContactVector(in(in_i,:),n,1,o_i);
                C_poly_inds(end+1) = o_i;
            end
        end
    end
    
    d = 1e-3*Unified.Area; % Bufffer measure
    for i = 1:Unified.N_e
        e = Unified.Edges([i i+1],:);
        try
            [in,out] = intersect(polybuffer(p.Shape,d),e);
        catch
            in = [];
        end
        if norm(diff(in)) > d*3
            n = -p.find_normal_at_point(mean(in));
            if isempty(n)
                continue
            end
            for in_i = 1:size(in,1)
                C{end+1} = ContactVector(in(in_i,:),n,1,o_i);
                C_poly_inds(end+1) = o_i;
            end
        end
    end
    % Search vertices of this polygon touch another
    [~,on] = Unified.in_polygon(p.Edges(1:end-1,:));
    points = p.Edges(on,:);
    for p_i = 1:size(points,1)
        point = points(p_i,:);
        n = Unified.find_normal_at_point(point);
        if isempty(n)
            continue
        end
        for n_i = 1:size(n,1)
            C{end+1} = ContactVector(point,n(n_i,:),1,o_i);
            C_poly_inds(end+1) = o_i;
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
            C_poly_inds(end+1) = o_i;
        end
    end
    
    figure(o_i); clf
    Unified.plot();
    axis equal; grid on; hold on;
    plot(polybuffer(Unified.Shape,d))
    p.plot();
    for c = C(C_poly_inds == o_i)
        cont = c{1};
        cont.plot_contact()
    end
end


%% Remove duplicate contacts ?

% This can be done by iterating over wrenches and `cross`-ing one with
% another. If some cross yields norm smaller than 

% norm(cross([1 0 0 ],[1 0.03 0.01]))
