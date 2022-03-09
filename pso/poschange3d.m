function [posX,posY,posZ]=poschange3d(g1,g2,n,xt,xs,yt,ys,zt,zs,p)%g1为俯仰角，g2为偏差角，n为每个粒子的维数
%将粒子解码成为航迹点
a=0.1;%一份角度
w1=a*g1;
w2=a*g2;
b1(1)=w1(1);
b2(1)=w2(1);
if n>1
    b1(2)=w1(2)+b1(1)+asin(0.5*sin(-w1(2)));
    b2(2)=w2(2)+b2(1)+asin(0.5*sin(-w2(2)));
    if n>2
        for i=3:n
            m1=atan(((i-1)*sin(b1(i-1))-(i-2)*sin(b1(i-2)))/((i-1)*cos(b1(i-1))-(i-2)*cos(b1(i-2))));
            b1(i)=w1(i)+m1+asin((i-1)*sin(b1(i-1)-w1(i)-m1)/i);
            m2=atan(((i-1)*sin(b2(i-1))-(i-2)*sin(b2(i-2)))/((i-1)*cos(b2(i-1))-(i-2)*cos(b2(i-2))));
            b2(i)=w2(i)+m2+asin((i-1)*sin(b2(i-1)-w2(i)-m2)/i);
        end
    end
end
r1=atan2((zt-zs),(((xt-xs)^2+(yt-ys)^2)^0.5));
r2=atan2((yt-ys),(xt-xs));
for i=1:n
    posX(i)=p*i*cos(r1+b1(i))*cos(r2+b2(i))+xs;
    posY(i)=p*i*cos(r1+b1(i))*sin(r2+b2(i))+ys;
    posZ(i)=p*i*sin(r1+b1(i))+zs;
end
end