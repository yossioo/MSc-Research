clc;
clearvars -except PolyList Filtered_Contacts Filtered_Contacts_poly_ind finger_d
warning off MATLAB:polyshape:boolOperationFailed

TableVarNames = {'PolygonNum','ContactGroup','EdgeNum','EdgeRange','OptimalPosition','ContactVector'};
Fingers = table([],[],[],[],[],[],...
    'VariableNames', TableVarNames);


dx = 1e-2;
Qx = dx*cosd([0;120;240]);
Qy = dx*sind([0;120;240]);

for p_i = 1:numel(PolyList)
    p = PolyList{p_i};
    
    I_contacts = Filtered_Contacts(Filtered_Contacts_poly_ind == p_i);
    N = numel(I_contacts);
    W = zeros(3,N);
    
    
    figure(200+p_i);clf;
    p.plot(); hold on; axis equal; grid on;
    for c_i = 1:N
        c = I_contacts{c_i};
        plot(p.Center(1),p.Center(2),'rs','MarkerSize',20)
        c.plot_contact()
        
        W(:,c_i) = round([c.direction_vector(:); c.cross_around_point(p.Center) / p.Area^0.5],2);
    end
    figure(101); clf
    quiver3(0*W(1,:),0*W(1,:),0*W(1,:),...
        W(1,:), W(2,:), W(3,:),'k-','AutoScale','off')
    hold on;
    ax = mean(W,2);
    quiver3(0,0,0, ax(1),ax(2),ax(3),'r-','AutoScale','off')
    hor = cross([0 0 1], ax); hor = hor./norm(hor);
    ver = cross(ax, hor); ver = ver./norm(ver);
    quiver3(0,0,0, hor(1),hor(2),hor(3),'g-','AutoScale','off')
    quiver3(0,0,0, ver(1),ver(2),ver(3),'b-','AutoScale','off')
    Wh = dot(W,repmat(hor(:),1,size(W,2)));
    Wv = dot(W,repmat(ver(:),1,size(W,2)));
    
    % Now we wish to determine whether all vectors lie in the same
    % plane. If dot projections of 3rd and next vectors have 0 size on
    % the normal of the plane of first 2 - all vectors in plane. If
    % so, select 2 marginal.
    cp = zeros(1,size(W,2));
    dp = zeros(1,size(W,2));
    for w_i = 3:size(W,2)
        dp(w_i) = round(dot(cross(W(:,1),W(:,2)),W(:,w_i)),2);
        cp(w_i) = round(dot(cross(W(:,1),W(:,2)),cross(W(:,1),W(:,w_i))),2);
    end
    if find(abs(dp)>1e-2)
        % There are some vectors out of the plane of first 2 vectors
        K = convhull(Wh,Wv);
        W_CH = W(:,K(1:end-1));
    else
        % Select 2 marginal
        min_cp = min(cp);
        max_cp = max(cp);
        
        if min_cp == 0
            W_min = W(:,1);
        else
            W_min = W(:,find(cp==min_cp,1));
        end
        
        if max_cp == 0
            W_max = W(:,2);
        else
            W_max = W(:,find(cp==max_cp,1));
        end
        
        W_CH = [W_min,W_max];
    end
    
    if size(W_CH, 2) > 2  % more than 2 vectors - searching for 1 finger to complete
        
        for e_i = 1:p.N_e
            n = p.Inner_normals(e_i,:);
            Move_from_vertex_ratio = 0.05; % set to positive if want to stay away from the vertices
            e1 = (1-Move_from_vertex_ratio)*p.Edges(e_i,:) +...
                Move_from_vertex_ratio * p.Edges(e_i+1,:);
            e2 = Move_from_vertex_ratio*p.Edges(e_i,:) +...
                (1-Move_from_vertex_ratio) * p.Edges(e_i+1,:);
            w = [n, cross2d(e1-p.Center,n)/p.Area^0.5;
                n, cross2d(e2-p.Center,n)/p.Area^0.5]';
            new_EGW = slice_cone(-W_CH,w);
            
            if ~isempty(new_EGW)
                % Next step is extracting the allowed finger position along the edge
                L = diff(w(3,1:2));
                p1_ = (new_EGW(3,1) - w(3,1))/L;
                p2_ = (new_EGW(3,2) - w(3,1))/L;
                p1_ = Move_from_vertex_ratio+p1_*(1-2*Move_from_vertex_ratio);
                p2_ = Move_from_vertex_ratio+p2_*(1-2*Move_from_vertex_ratio);
                % Search for contacts by using the code from
                % <matlab:inverted_cone Lines:106-111>
                contact_point1 = p.point_from_edgePosition(e_i,p1_+1e-2);
                f1 =  ContactVector(contact_point1,...
                    p.find_normal_at_point(contact_point1),1,p_i);
                contact_point2 = p.point_from_edgePosition(e_i,p2_);
                f2 =  ContactVector(contact_point2,...
                    p.find_normal_at_point(contact_point2),1,p_i);
                
                % We find optimal location by checking the maximum of
                % the minimal of CH face distances. Each face is
                % checked: normal direction created, and then distance
                % from the origin is checked.
                pos_range = linspace(p1_,p2_);
                torq_range = linspace(new_EGW(3,1), new_EGW(3,2));
                d_Full = zeros(size(W_CH,2),numel(pos_range));
                points_alonge_the_line = [repmat(n(:),1,100);torq_range];
                
                V1 = W_CH(:,end)-points_alonge_the_line;
                V2 = W_CH(:,1)-points_alonge_the_line;
                d_Full(4,:) = dot(-points_alonge_the_line, cross(V1,V2)/norm(cross(V1,V2)));
                
                for f_i = 1:size(W_CH,2)-1
                    V1 = W_CH(:,f_i)-points_alonge_the_line;
                    V2 = W_CH(:,f_i+1)-points_alonge_the_line;
                    d_Full(f_i,:) = dot(-points_alonge_the_line, cross(V1,V2)/norm(cross(V1,V2)));
                end
                d = min(d_Full,[],1);
                optimal_pos = pos_range(find(d == max(d),1));
                contact_point = p.point_from_edgePosition(e_i,optimal_pos);
                f =  ContactVector(contact_point,...
                    p.find_normal_at_point(contact_point),finger_d,p_i);
                
                figure(200+p_i)
                f1.plot_contact('c')
                f2.plot_contact('c')
                f.plot_contact('b')
                Fingers = [Fingers;table(p_i,...
                    max([1 1+max(Fingers.ContactGroup(Fingers.PolygonNum==p_i))]),...
                    e_i, [p1_ p2_], optimal_pos, f,...
                    'VariableNames', TableVarNames)]
                
                % This adds one finger that can immobilize the body
            end
            %         DT = delaunayTriangulation([1e-3 1e-3 0; -1e-3 2.5e-3 1e-4 ; w]);
            %         tetramesh(DT,'FaceAlpha',0.1,'FaceColor','y');
            %         quiver3(0*w(:,1),0*w(:,1),0*w(:,1),w(:,1),w(:,2),w(:,3),'AutoScale','off')
        end
        if sum(Fingers.PolygonNum==p_i) == 0
            warning("YOSSI:NoEdgeOppositeToCH",...
                "No edge to complete the grasp.\nIterating over a polygon #%d in the PolyList to find an edge with normal direction to complete the ConvexHull formed by given contacts",p_i)
        end
    else
        % 2 vectors (or 1?. But I believe that I always will have at least 2, since I'm not doing 1.)
        while 0
            search for edge that can span both sides of the plane
            If no such edge found, search for plane couples.
        end
        
        for e_i = 1:p.N_e
            n = p.Inner_normals(e_i,:);
            Move_from_vertex_ratio = 0.00; % set to positive if want to stay away from the vertices
            e1 = (1-Move_from_vertex_ratio)*p.Edges(e_i,:) +...
                Move_from_vertex_ratio * p.Edges(e_i+1,:);
            e2 = Move_from_vertex_ratio*p.Edges(e_i,:) +...
                (1-Move_from_vertex_ratio) * p.Edges(e_i+1,:);
            w = [n, cross2d(e1,n);
                n, cross2d(e2,n)]';
            new_EGW = slice_cone(-W_CH,w);
            DT_test = delaunayTriangulation([W_CH, w(:,2)]');
%             tetramesh(DT_test,'FaceAlpha',0.1,'FaceColor','y');
            if find(size(DT_test) == 0)
                % DT does not form a polyhedral (probably because it
                % is on one surface
            else
                T_test = triangulation(DT_test.ConnectivityList, DT_test.Points);
                ID = pointLocation(T_test,[0 0 0])
                if ID
                    % Convex hull spans the origin
                    % Select marginal vectors of EGW to be 2 contacts
                end
%                 [in,~] = inpolygon(Qx(:), Qy(:), Vert(1,XY_CH_ind), Vert(2,XY_CH_ind));
            end
        end
    end
    
    
    %     zlabel('\tau_z','FontSize',20)
    %     xlabel('f_x','FontSize',20)
    %     ylabel('f_y','FontSize',20)
%     title(p.Name)
    
    if sum(Fingers.PolygonNum==p_i) == 0
        % Meaning no fingers found to complete the grasp
        fprintf('No fingers for polygon %d\n',p_i)
    end
end
