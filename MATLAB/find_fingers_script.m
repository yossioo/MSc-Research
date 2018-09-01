clc;
clearvars -except PolyList C C_poly_inds

for i = 1:1% numel(PolyList)
    p = PolyList{i};
    
    I_contacts = C(C_poly_inds == i);
    N = numel(I_contacts);
    W = zeros(3,N);
    
    
    figure(100); clf
    p.plot(); hold on; axis equal; grid on;
    for k = 1:N
        c = I_contacts{k};
        
        plot(p.Center(1),p.Center(2),'rs','MarkerSize',20)
        c.plot_contact()
        W(:,k) = round([c.direction_vector(:); c.cross_around_point(p.Center)],3);
%         quiver3(0,0,0, W(1,k), W(2,k), W(3,k),'k-','AutoScale','off')
%         hold on;
    end
%     W = W./vecnorm(W);
    W(:,end+1) = [1;1;0]; % Remove this in release
%     quiver3(0,0,0, W(1,end), W(2,end), W(3,end),'k-','AutoScale','off')
%     figure(100); clf
%     axis equal
%     grid on
    ax = mean(W,2);
%     quiver3(0,0,0, ax(1),ax(2),ax(3),'r-','AutoScale','off')
    hor = cross([0 0 1], ax); hor = hor./norm(hor);
    ver = cross(ax, hor); ver = ver./norm(ver);
%     quiver3(0,0,0, hor(1),hor(2),hor(3),'g-','AutoScale','off')
%     quiver3(0,0,0, ver(1),ver(2),ver(3),'b-','AutoScale','off')
    Wh = dot(W,repmat(hor(:),1,size(W,2)));
    Wv = dot(W,repmat(ver(:),1,size(W,2)));
    
    K = convhull(Wh,Wv);
    W_CH = W(:,K(1:end-1));
    DT = delaunayTriangulation([W_CH, [0;0;0]]');
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
        new_EGW = slice_cone(-W_CH,w)
%         DT_GEW = delaunayTriangulation([1e-3 1e-3 0; -1e-3 2.5e-3 1e-4 ; w]);
%         tetramesh(DT_GEW,'FaceAlpha',0.1,'FaceColor','y');
%         quiver3(0*w(:,1),0*w(:,1),0*w(:,1),w(:,1),w(:,2),w(:,3),'AutoScale','off')
    end
end
