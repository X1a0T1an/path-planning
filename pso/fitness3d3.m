function fit=fitness3d3(g1,g2,o1,o2,o12,o22)
%计算适应度函数
    global D1 D2 D3 p;
    global xt xs xt2 xs2 xt3 xs3;
    global yt ys yt2 ys2 yt3 ys3;
    global zt zs zt2 zs2 zt3 zs3;
    n=D3+2;%n代表维数
    [posX,posY,posZ]=poschange3d(g1,g2,D3,xt3,xs3,yt3,ys3,zt3,zs3,p);%将粒子解码为航迹点
    [posXo1,posYo1,posZo1]=poschange3d(o1,o2,D1,xt,xs,yt,ys,zt,zs,p);%将粒子解码为航迹点
    [posXo2,posYo2,posZo2]=poschange3d(o12,o22,D2,xt2,xs2,yt2,ys2,zt2,zs2,p);%将粒子解码为航迹点
    a=[posX;posY;posZ];
    b=[posXo1;posYo1;posZo1];
    c=[posXo2;posYo2;posZo2];
    global xpos3;
    global ypos3;
    global zpos3;
    xpos3=[xs3,posX,xt3];
    ypos3=[ys3,posY,yt3];
    zpos3=[zs3,posZ,zt3];
    posX=[xs3,posX,xt3];
    posY=[ys3,posY,yt3];
    posZ=[zs3,posZ,zt3];
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
    lastdist=((xt3-posX(n))^2+(yt3-posY(n))^2+(zt3-posZ(n))^2)^0.5;
    if lastdist>3*p
        fit1=fit1+lastdist;
    end
    fit=1/(fit1+fit2+space3d(a,b)+space3d(a,c));
    %最后一次转向也要满足最大转向角的限制
    if abs(atan((yt3-posY(n))/(xt3-posX(n)))-atan((posY(n)-posY(n-1))/(posX(n)-posX(n-1))))>1
        fit=0;
    end
    if abs(atan((zt3-posZ(n))/((yt3-posY(n))^2+(xt3-posX(n))^2)^0.5)-atan((posZ(n)-posZ(n-1))/((posY(n)-posY(n-1))^2+(posX(n)-posX(n-1))^2)^0.5))>1
        fit=0;
    end
end