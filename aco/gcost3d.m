function g=gcost3d(X,goal)
%和目标点的距离
    g=sqrt(sum((X-goal).^2));
end