function h = drawSphere(n,r, centerx, centery, centerz, N)

% if nargin == 5
%     [x,y,z] = sphere(N);
% else
    [x,y,z] = sphere(50);
% end

h = surf(r*x+centerx, r*y+centery, r*z+centerz);
if n==0
h.EdgeColor = [0,0,0];
end
if n==1
    h.EdgeColor = [1,0,0];
end
if n==2
    h.EdgeColor = [0,1,0];
end
if n==3
    h.EdgeColor = [0,0,1];
end
if n==4
    h.EdgeColor = [1,1,1];
end
h.FaceColor = h.EdgeColor;