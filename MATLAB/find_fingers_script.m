clc;
clearvars -except PolyList C C_poly_inds
warning off MATLAB:polyshape:boolOperationFailed


Fingers = table(-1,-1,-1,[0 1],ContactVector([0 0],[1 0],1,0),...
    'VariableNames', {'PolygonNum','ContactGroup','EdgeNum','EdgeRange','ContactVector'});
Fingers(:,:) = [];
% Alternatively:
Fingers = table([],[],[],[],[],...
    'VariableNames', {'PolygonNum','ContactGroup','EdgeNum','EdgeRange','ContactVector'});


for p_i = 1:numel(PolyList)
    p = PolyList{p_i};
    
    I_contacts = C(C_poly_inds == p_i);
    N = numel(I_contacts);
    W = zeros(3,N);
    
    
    figure(101); clf
    figure(100); clf
    p.plot(); hold on; axis equal; grid on;
    for c_i = 1:N
        c = I_contacts{c_i};
        figure(100)
        plot(p.Center(1),p.Center(2),'rs','MarkerSize',20)
        c.plot_contact()
        
        figure(101); 
        W(:,c_i) = round([c.direction_vector(:); c.cross_around_point(p.Center)],3);
        quiver3(0,0,0, W(1,c_i), W(2,c_i), W(3,c_i),'k-','AutoScale','off')
        hold on;
    end
    %     W = W./vecnorm(W);
    %     W(:,end+1) = [1;1;0]; % Remove this in release
    %     %     quiver3(0,0,0, W(1,end), W(2,end), W(3,end),'k-','AutoScale','off')
    %     figure(100); clf
    %     axis equal
    %         grid on
    ax = mean(W,2);
    quiver3(0,0,0, ax(1),ax(2),ax(3),'r-','AutoScale','off')
    hor = cross([0 0 1], ax); hor = hor./norm(hor);
    ver = cross(ax, hor); ver = ver./norm(ver);
        quiver3(0,0,0, hor(1),hor(2),hor(3),'g-','AutoScale','off')
        quiver3(0,0,0, ver(1),ver(2),ver(3),'b-','AutoScale','off')
    Wh = dot(W,repmat(hor(:),1,size(W,2)));
    Wv = dot(W,repmat(ver(:),1,size(W,2)));
    
    % Now we wish to determine whether all vectors lie in the same
    % plane
    [p,S,mu] = polyfit(Wh+eps,Wv+eps,1)
    if isnan(p)
        [p,S,mu] = polyfit(Wv+eps,Wh+eps,1)
    end
    % If they are not- create a convex hull and remove duplicates
    % If they are - select 2 marginal (remove all but 2)
    % Remove by crossing vectors and checking norm.
    try
        K = convhull(Wh,Wv);
        W_CH = W(:,K(1:end-1));
    catch
        W_CH = W;
    end
    %     DT = delaunayTriangulation([W_CH, [0;0;0]]');
    %     tetramesh(DT,'FaceAlpha',0.3);
    
    
    %     zlabel('\tau_z','FontSize',20)
    %     xlabel('f_x','FontSize',20)
    %     ylabel('f_y','FontSize',20)
    title(p.Name)
    
    for e_i = 1:p.N_e
        n = p.Inner_normals(e_i,:);
        %         e1 = p.Edges(e_i,:);
        %         e2 = p.Edges(e_i+1,:);
        Move_from_vertex_ratio = 0.00;
        e1 = (1-Move_from_vertex_ratio)*p.Edges(e_i,:) +...
            Move_from_vertex_ratio * p.Edges(e_i+1,:);
        e2 = Move_from_vertex_ratio*p.Edges(e_i,:) +...
            (1-Move_from_vertex_ratio) * p.Edges(e_i+1,:);
        w = [n, cross2d(e1,n);
            n, cross2d(e2,n)]';
        new_EGW = slice_cone(-W_CH,w);
        
        % Next step is extracting the allowed finger position along the edge
        if ~isempty(new_EGW)
            L = diff(w(3,1:2));
            p1_ = (new_EGW(3,1) - w(3,1))/L;
            p2_ = (new_EGW(3,2) - w(3,1))/L;
            
            % Search for contacts by using the code from 
            % <matlab:inverted_cone Lines:106-111>
            f = ContactVector([0 0],[0 0],1,p_i);
            
            Fingers = [Fingers;table(p_i,...
                max([1 1+max(Fingers.ContactGroup(Fingers.PolygonNum==p_i))]),...
                e_i, [p1_ p2_], f,...
                'VariableNames', {'PolygonNum','ContactGroup','EdgeNum','EdgeRange','ContactVector'})]
            
        end
        %         DT_GEW = delaunayTriangulation([1e-3 1e-3 0; -1e-3 2.5e-3 1e-4 ; w]);
        %         tetramesh(DT_GEW,'FaceAlpha',0.1,'FaceColor','y');
        %         quiver3(0*w(:,1),0*w(:,1),0*w(:,1),w(:,1),w(:,2),w(:,3),'AutoScale','off')
    end
    if sum(Fingers.PolygonNum==p_i) == 0
        % Meaning no fingers found to complete the grasp
        fprintf('No fingers for polygon %d\n',p_i)
    end
end
