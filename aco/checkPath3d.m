function feasible=checkPath3d(n,newPos)
%判断候补航迹段是否合理，即两点之间能否连通
    global DDIR ;%规定可连接的最大距离
    DDIR=30;
    feasible = true;
    a=(newPos-n)/10;
    dist=sqrt(sum((n-newPos).^2));
    if dist>30||dist<3
        feasible=false;
        return;
    end
    x=sqrt((newPos(1)-n(1))^2+(newPos(2)-n(2))^2);
    y=abs(newPos(3)-n(3));
    if y>x
        feasible=false;
        return;
    end
    for i=1:10
        posCheck = n + a*i;
        if ~(feasiblePoint3d(posCheck)) 
              feasible = false;break;
        end
        if ~feasiblePoint3d(newPos), feasible = false; end
    end
end