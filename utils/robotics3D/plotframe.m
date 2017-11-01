function plotframe(q,p)

% get a quaternion and plot a frame rotated by it.

or = [0;0;0];
hold on;
plotarrow(or,0.1*[1;0;0],'r');
% hold on
plotarrow(or,0.1*[0;1;0],'g');
plotarrow(or,0.1*[0;0;1],'b');

R = quat2rot(q);

if nargin < 2
    p = or;
end

plotarrow(p,R'*5*[1;0;0],'r--');
plotarrow(p,R'*5*[0;1;0],'g--');
plotarrow(p,R'*5*[0;0;1],'b--');
 
% %% Plot the Pyramid
% polygon_l = min(0.22,0.22*max(norm(p),2));
% x = polygon_l*[0 0 0 0; 1 1 -1 1; 1 -1 -1 -1];
% z = polygon_l*[0 0 0 0; 3 3 3 3; 3 3 3 3];
% y =polygon_l*[0 0 0 0; 1 1 -1 -1; -1 1 1 -1];
% polygons =[x;y;z];
% for k=1:1:size(x,2)
%   for l=1:1:3
%      PP = [x(l,k);y(l,k);z(l,k)];
%      PP = (R'*PP)+p;
%      x(l,k) = PP(1);
%      y(l,k) = PP(2);
%      z(l,k) = PP(3);
%      
%   end
%     
% end
% 
% fill3(x,y,z, 'm','linewidth',3)
% view(100, 30)

% hold off

% R = quat2rot(q);
% 
% or = [0;0;0];
% if nargin < 2
%     p = or;
% end
% 
% e = eye(3);
% 
% X = [or(1)*ones(3,1) ; p(1)*ones(3,1)];
% Y = [or(2)*ones(3,1) ; p(2)*ones(3,1)];
% Z = [or(3)*ones(3,1) ; p(3)*ones(3,1)];
% 
% U = [e(:,1); R'*e(:,1)];
% V = [e(:,2); R'*e(:,2)];
% W = [e(:,3); R'*e(:,3)];

%quiver3(X,Y,Z,U,V,W,'LineWidth',2,...
%    'MarkerEdgeColor','k','MarkerFaceColor',[.49 1 .63],'MarkerSize',12);