function fit=fitness3d(g1,g2,o1,o2)
%适应度函数
    global  D1 D2 D3 p;
    global xt xs xt2 xs2;
    global yt ys yt2 ys2;
    global zt zs zt2 zs2;
    n=D1+2;%n代表维数
    [posX,posY,posZ]=poschange3d(g1,g2,D1,xt,xs,yt,ys,zt,zs,p);
    [posXo1,posYo1,posZo1]=poschange3d(o1,o2,D2,xt2,xs2,yt2,ys2,zt2,zs2,p);
    a=[posX;posY;posZ];
    b=[posXo1;posYo1;posZo1];
    global xpos;
    global ypos;
    global zpos;
    posX=[xs,posX,xt];
    posY=[ys,posY,yt];
    posZ=[zs,posZ,zt];
    xpos=posX;
    ypos=posY;
    zpos=posZ;
    u=highenough(posX,posY,posZ,n);%避免和地形相撞
    if  u~=1
        fit=0;
        return;
    end
    fit1=0;
    fit2=0;
    for i=2:n
        dist=((posX(i)-posX(i-1))^2+(posY(i)-posY(i-1))^2+(posZ(i)-posZ(i-1))^2)^0.5;
        fit1=fit1+dist;
        dx=(posX(i)-posX(i-1))/10;
        dy=(posY(i)-posY(i-1))/10;
        dz=(posZ(i)-posZ(i-1))/10;
        for j=1:10
            fit2=fit2+danger3d(posX(i-1)+dx*j,posY(i-1)+dy*j,posZ(i-1)+dz*j)/10;
        end
    end
    n=n-1;
    lastdist=((xt-posX(n))^2+(yt-posY(n))^2+(zt-posZ(n))^2)^0.5;
    if lastdist>3*p
        fit1=fit1+lastdist;
    end
    fit=1/(fit1+fit2+space3d(a,b));
    %最后一次转向也要满足最大转向角的限制
    if abs(atan((yt-posY(n))/(xt-posX(n)))-atan((posY(n)-posY(n-1))/(posX(n)-posX(n-1))))>1
        fit=0;
    end
    if abs(atan((zt-posZ(n))/((yt-posY(n))^2+(xt-posX(n))^2)^0.5)-atan((posZ(n)-posZ(n-1))/((posY(n)-posY(n-1))^2+(posX(n)-posX(n-1))^2)^0.5))>1
        fit=0;
    end
%     disp(fit1);
%     disp(fit2);
%     disp(space3d(a,b));
end