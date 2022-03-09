clear
clc
load('Z111.mat');%载入初始地形
global  Z3;
%设置山峰高度
h=[26, 30,  25,  28 , 27, 25, 23];
%设置山峰坐标
x0=[20, 45,  45, 15, 57,  50, 10];
y0=[20, 25,  50, 50,  41, 8,  25];
xi=[5.5, 8,  5,  3.5, 4, 4,  4.5];
yi=[5,   7,  6,   4, 5,  3,  4.5];
Z2=CeatHill(7,h,x0,y0,xi,yi,65); 
Z3=max(Z1,Z2);
%创造地形
figure(2);
surf(Z3); %画出三维曲面 
axis([1 65 1 65 1 30])%限制xyz轴范围
hold on;
%此为3d粒子群算法，需要和fitness函数相配合
global Np Rk    %威胁源个数，有效半径
%威胁源坐标
global danareaX;
global danareaY;
global danareaZ;
%载入威胁源坐标
load('danX.mat');
load('danY.mat');
load('danZ.mat');
Np=length(danareaX);
Rk=3;
%设置起始点和目标点
global xt xs xt2 xs2 xt3 xs3;
global yt ys yt2 ys2 yt3 ys3;
global zt zs zt2 zs2 zt3 zs3;
xt=50;xt2=55;xt3=58;
yt=10;yt2=10;yt3=10;
zt=15;zt2=15;zt3=15;
xs=4;xs2=4;xs3=4;
ys=30;ys2=28;ys3=32;
zs=15;zs2=15;zs3=15;
global xpos xpos2 xpos3;
global ypos ypos2 ypos3;
global zpos zpos2 zpos3;
global  p D1 D2 D3;
N=50;                          %群体粒子个数
p=2;                            %步长
D1=round(sqrt((xt-xs)^2+(yt-ys)^2+(zt-zs)^2)/p)-1;%粒子维度
D2=round(sqrt((xt2-xs2)^2+(yt2-ys2)^2+(zt2-zs2)^2)/p)-1;%粒子维度
D3=round(sqrt((xt3-xs3)^2+(yt3-ys3)^2+(zt3-zs3)^2)/p)-1;%粒子维度
T=50;                          %最大迭代次数
c1=1.5;                         %学习因子1
c2=1.5;                         %学习因子2
Wmax=0.8;                       %惯性权重最大值
Wmin=0.4;                       %惯性权重最小值
V1max=6;                        %速度最大值
V1min=-6;                       %速度最小值
X1max=4;                         %偏角最大值
X1min=-4;                        %偏角最小值
X2max=4;                       %俯仰角最大值
X2min=-4;                       %俯仰角最小值
V2max=6;                       
V2min=-6;
%初始化个体
x1=randi([X1min,X1max],N,D1);%粒子的初始位置，
v1=rand(N,D1)*(V1max-V1min)+V1min;
x2=randi([X2min,X2max],N,D1);
v2=rand(N,D1)*(V2max-V2min)+V2min;
x12=randi([X1min,X1max],N,D2);%粒子的初始位置，
v12=rand(N,D2)*(V1max-V1min)+V1min;
x22=randi([X2min,X2max],N,D2);
v22=rand(N,D2)*(V2max-V2min)+V2min;
x13=randi([X1min,X1max],N,D3);%粒子的初始位置，
v13=rand(N,D3)*(V1max-V1min)+V1min;
x23=randi([X2min,X2max],N,D3);
v23=rand(N,D3)*(V2max-V2min)+V2min;
x1(:,1)=round(randi([-20,20],N,1));
x2(:,1)=round(randi([-20,20],N,1));
x12(:,1)=round(randi([-20,20],N,1));
x22(:,1)=round(randi([-20,20],N,1));
x13(:,1)=round(randi([-20,20],N,1));
x23(:,1)=round(randi([-20,20],N,1));
%初始化个体最优位置和最优值
p1=x1;           %p记录每个粒子自身最佳位置 p(i，:)代表第i个粒子的最佳位置
p2=x2;
pbest=zeros(N,1); %pbest记录每个粒子自身最佳值，一开始都为0
p12=x12;           %p记录每个粒子自身最佳位置 p(i，:)代表第i个粒子的最佳位置
p22=x22;
pbest2=zeros(N,1); %pbest记录每个粒子自身最佳值，一开始都为0
p13=x13;           %p记录每个粒子自身最佳位置 p(i，:)代表第i个粒子的最佳位置
p23=x23;
pbest3=zeros(N,1); %pbest记录每个粒子自身最佳值，一开始都为0
%初始化全局最优位置和最优值
g1=zeros(1,D1); %g代表全局最佳粒子的位置
g2=zeros(1,D1);
gbest=0;   %gbest代表目前的最佳值
g12=zeros(1,D2); %g代表全局最佳粒子的位置
g22=zeros(1,D2);
gbest2=0;   %gbest代表目前的最佳值
g13=zeros(1,D3); %g代表全局最佳粒子的位置
g23=zeros(1,D3);
gbest3=0;   %gbest代表目前的最佳值
for i=1:N
     pbest(i)=fitness3d(x1(i,:),x2(i,:),g12,g22); %对每个粒子进行适应度计算，第一次计算所得先设为最佳值
    pbest2(i)=fitness3d2(x12(i,:),x22(i,:),g1,g2); %对每个粒子进行适应度计算，第一次计算所得先设为最佳值
end
for i=1:N    %从初始的自身最佳值找到初始的全局最佳值
    if (pbest(i)>gbest)
        g1=p1(i,:);
        g2=p2(i,:);
        gbest=pbest(i);
    end
     if (pbest2(i)>gbest2)
        g12=p12(i,:);
        g22=p22(i,:);
        gbest2=pbest2(i);
     end
end
%按照公式依次迭代直到迭代次数
for i=1:T
    for j=1:N
         %更新个体最优位置和最优值
         fit=fitness3d(x1(j,:),x2(j,:),g12,g22);
        if (fit>pbest(j))
            p1(j,:)=x1(j,:);
            p2(j,:)=x2(j,:);
            pbest(j)=fit;
        end
        %更新全局最优位置和最优值
        if (pbest(j)>gbest)
            g1=p1(j,:);
            g2=p2(j,:);
            gbest=pbest(j);
        end
         fit2=fitness3d2(x12(j,:),x22(j,:),g1,g2);
        if (fit2>pbest2(j))
            p12(j,:)=x12(j,:);
            p22(j,:)=x22(j,:);
            pbest2(j)=fit2;
        end
        %更新全局最优位置和最优值
        if (pbest2(j)>gbest2)
            g12=p12(j,:);
            g22=p22(j,:);
            gbest2=pbest2(j);
        end
         %计算动态惯性权重值
          w=Wmax-(Wmax-Wmin)*i/T;
          %更新位置和速度
          v1(j,:)=w*v1(j,:)+c1*rand*(p1(j,:)-x1(j,:))+c2*rand*(g1-x1(j,:));
          v2(j,:)=w*v2(j,:)+c1*rand*(p2(j,:)-x2(j,:))+c2*rand*(g2-x2(j,:));
          v12(j,:)=w*v12(j,:)+c1*rand*(p12(j,:)-x12(j,:))+c2*rand*(g12-x12(j,:));
          v22(j,:)=w*v22(j,:)+c1*rand*(p22(j,:)-x22(j,:))+c2*rand*(g22-x22(j,:));
     %边界条件处理
        for ii=1:D1
            if (v1(j,ii)<V1min)||(v1(j,ii)>V1max)
               v1(j,ii)=rand*(V1max-V1min)+V1min;
            end
            if (v2(j,ii)<V2min)||(v2(j,ii)>V2max)
               v2(j,ii)=rand*(V2max-V2min)+V2min;
            end
            x1(j,ii)=x1(j,ii)+v1(j,ii);
            x2(j,ii)=x2(j,ii)+v2(j,ii);
            %第一段航迹不需要满足最大转弯角的约束
            if ii~=1
                if (x1(j,ii)<X1min)||(x1(j,ii)>X1max)
                    x1(j,ii)=rand*(X1max-X1min)+X1min;
                end
                if (x2(j,ii)<X2min)||(x2(j,ii)>X2max)
                    x2(j,ii)=rand*(X2max-X2min)+X2min;
                end
            end
            if ii==1
                if (x1(j,ii)<-20)||(x1(j,ii)>20)
                    x1(j,ii)=rand*(40)-20;
                end
                if (x2(j,ii)<-20)||(x2(j,ii)>20)
                    x2(j,ii)=rand*(40)-20;
                end
                if (x12(j,ii)<-20)||(x12(j,ii)>20)
                    x12(j,ii)=rand*(40)-20;
                end
                if (x22(j,ii)<-20)||(x22(j,ii)>20)
                    x22(j,ii)=rand*(40)-20;
                end
            end
            x1(j,ii)=round(x1(j,ii));
            x2(j,ii)=round(x2(j,ii));
        end
        for ii=1:D2
            if (v12(j,ii)<V1min)||(v12(j,ii)>V1max)
               v12(j,ii)=rand*(V1max-V1min)+V1min;
            end
            if (v22(j,ii)<V2min)||(v22(j,ii)>V2max)
               v22(j,ii)=rand*(V2max-V2min)+V2min;
            end
            x12(j,ii)=x12(j,ii)+v12(j,ii);
            x22(j,ii)=x22(j,ii)+v22(j,ii);
            %第一段航迹不需要满足最大转弯角的约束
            if ii~=1
                if (x12(j,ii)<X1min)||(x12(j,ii)>X1max)
                    x12(j,ii)=rand*(X1max-X1min)+X1min;
                end
                if (x22(j,ii)<X2min)||(x22(j,ii)>X2max)
                    x22(j,ii)=rand*(X2max-X2min)+X2min;
                end
            end
            if ii==1
                if (x12(j,ii)<-20)||(x12(j,ii)>20)
                    x12(j,ii)=rand*(40)-20;
                end
                if (x22(j,ii)<-20)||(x22(j,ii)>20)
                    x22(j,ii)=rand*(40)-20;
                end
            end
            x12(j,ii)=round(x12(j,ii));
            x22(j,ii)=round(x22(j,ii));
        end
    end
end
for i=1:N
     pbest3(i)=fitness3d3(x13(i,:),x23(i,:),g1,g2,g12,g22); %对每个粒子进行适应度计算，第一次计算所得先设为最佳值
     if (pbest3(i)>gbest3)
        g13=p13(i,:);
        g23=p23(i,:);
        gbest3=pbest3(i);
     end
end
for i=1:T
    for j=1:N
        %更新个体最优位置和最优值
        fit3=fitness3d3(x13(j,:),x23(j,:),g1,g2,g12,g22);
        if (fit3>pbest3(j))
            p13(j,:)=x13(j,:);
            p23(j,:)=x23(j,:);
            pbest3(j)=fit3;
        end
        %更新全局最优位置和最优值
        if (pbest3(j)>gbest3)
            g13=p13(j,:);
            g23=p23(j,:);
            gbest3=pbest3(j);
        end
        %更新位置和速度
        v13(j,:)=w*v13(j,:)+c1*rand*(p13(j,:)-x13(j,:))+c2*rand*(g13-x13(j,:));
        v23(j,:)=w*v23(j,:)+c1*rand*(p23(j,:)-x23(j,:))+c2*rand*(g23-x23(j,:));
        for ii=1:D3
            if (v13(j,ii)<V1min)||(v13(j,ii)>V1max)
               v13(j,ii)=rand*(V1max-V1min)+V1min;
            end
            if (v23(j,ii)<V2min)||(v23(j,ii)>V2max)
               v23(j,ii)=rand*(V2max-V2min)+V2min;
            end
            x13(j,ii)=x13(j,ii)+v13(j,ii);
            x23(j,ii)=x23(j,ii)+v23(j,ii);
            %边界条件处理
            %第一段航迹不需要满足最大转弯角的约束
            if ii~=1
            if (x13(j,ii)<X1min)||(x13(j,ii)>X1max)
                x13(j,ii)=rand*(X1max-X1min)+X1min;
            end
            if (x23(j,ii)<X2min)||(x23(j,ii)>X2max)
                x23(j,ii)=rand*(X2max-X2min)+X2min;
            end
            end
            if ii==1
            if (x13(j,ii)<-20)||(x13(j,ii)>20)
                x13(j,ii)=rand*(40)-20;
            end
            if (x23(j,ii)<-20)||(x23(j,ii)>20)
                x23(j,ii)=rand*(40)-20;
            end
            end
            x13(j,ii)=round(x13(j,ii));
            x23(j,ii)=round(x23(j,ii));
        end
    end
end
%适应度函数
figure(2)
fitness3d(g1,g2,g12,g22);
plot3(xpos,ypos,zpos,'r')
fitness3d2(g12,g22,g1,g2);
plot3(xpos2,ypos2,zpos2,'g')
fitness3d3(g13,g23,g1,g2,g12,g22);
plot3(xpos3,ypos3,zpos3,'b')
hold on
r=3;
for j=1:Np
    h1 = drawSphere(0,r,danareaX(j), danareaY(j), danareaZ(j));%画障碍球
end
waitforbuttonpress;%等待鼠标按下触发离线规划和在线规划相结合
%新增障碍
danareaX=[danareaX,18*rand(1,5)+10];
danareaY=[danareaY,18*rand(1,5)+25];
danareaZ=[danareaZ,10*rand(1,5)+15];
Np=length(danareaX);
axis equal
Rk=3;
j=1;j2=1;j3=1;
global currentPosition1 currentPosition2 currentPosition3;
currentPosition1=[xs,ys,zs];
currentPosition2=[xs2,ys2,zs2];
currentPosition3=[xs3,ys3,zs3];
%改变目标点
newgoal1=[xt,yt,zt+5];
newgoal2=[xt2,yt2,zt2+5];
newgoal3=[xt3,yt3,zt3+5];
global nt;
t=1;nn=1;
flag1=0;flag2=0;flag3=0;
while ~(nn>length(xpos))||~(nn>length(xpos2))||~(nn>length(xpos3))
    nt=nn;
    %判断视野范围内是否存在新目标点，如果存在，运用在线规划进行局部航迹调整
    if ~(isempty(newgoal1))&&sqrt((xpos(nn)-newgoal1(1))^2+(ypos(nn)-newgoal1(2))^2+(zpos(nn)-newgoal1(3))^2)<10
        h1 = drawSphere(1,0.5,newgoal1(1), newgoal1(2), newgoal1(3));
        %定义局部航迹的起始点
        newsource=[xpos(nn),ypos(nn),zpos(nn)];
        %定义局部航迹的终点
        newgoal=newgoal1;
        %获取当前的运动方向
        cuDir_a=atan2(ypos(nn)-ypos(nn-1),xpos(nn)-xpos(nn-1)); 
        cuDir_b=atan2(zpos(nn)-zpos(nn-1),sqrt((xpos(nn)-xpos(nn-1))^2+(ypos(nn)-ypos(nn-1))^2));
        %通过人工势场法进行局部航迹规划
        newpath=GETAPM(newsource,newgoal,cuDir_a,cuDir_b,1);
        n=size(newpath,1);
        xpos=[xpos(1:nn),newpath(:,1).'];
        ypos=[ypos(1:nn),newpath(:,2).'];
        zpos=[zpos(1:nn),newpath(:,3).'];
        D1=nn+n;
        newgoal1=[];
        flag1=1;
    end     
    if ~(nn>length(xpos)-5)&&flag1==0
        j=nn;
    %判断离线规划所得航迹是否会和新增威胁发生碰撞，如果有，运用在线规划算法进行局部航迹调整
    for i=0:4
        if~feasiblePoint3d([xpos(j+i),ypos(j+i),zpos(j+i)])
            for m=21:25
                if sqrt((xpos(j+i)-danareaX(m))^2+(ypos(j+i)-danareaY(m))^2+(zpos(j+i)-danareaZ(m))^2)<Rk
                    h1 = drawSphere(4,r,danareaX(m), danareaY(m), danareaZ(m));%画障碍球
                end
            end
            %定义局部航迹的起始点
            newsource=[xpos(j),ypos(j),zpos(j)];
            %获取当前的运动方向
            if j+i==1
            cuDir_a=atan2(yt-ys,xt-xs); 
            cuDir_b=atan2(zt-zs,sqrt((xt-xs)^2+(yt-ys)^2));
            else
            cuDir_a=atan2(ypos(j+i)-ypos(j+i-1),xpos(j+i)-xpos(j+i-1)); 
            cuDir_b=atan2(zpos(j+i)-zpos(j+i-1),sqrt((xpos(j+i)-xpos(j+i-1))^2+(ypos(j+i)-ypos(j+i-1))^2));
            end
            k=j+i+4;
            if k>D1+2
                k=D1+2;
            else
                while k<D1+2&&(~feasiblePoint3d([xpos(k),ypos(k),zpos(k)])||~feasiblePoint3d([xpos(k-3),ypos(k-3),zpos(k-3)]))
                    k=k+1;
                end
            end
                    %定义局部航迹的终点
                    newgoal=[xpos(k),ypos(k),zpos(k)];
                    %通过人工势场法进行局部航迹规划
                    newpath=GETAPM(newsource,newgoal,cuDir_a,cuDir_b,1);
                    n=size(newpath,1);
                    xpos=[xpos(1:j),newpath(:,1).',xpos(k+1:D1+2)];
                    ypos=[ypos(1:j),newpath(:,2).',ypos(k+1:D1+2)];
                    zpos=[zpos(1:j),newpath(:,3).',zpos(k+1:D1+2)];
                    D1=D1+n+j-k;
                    i=4;
        end
    end
    end
    %同第一架无人机
    if ~(isempty(newgoal2))&&sqrt((xpos2(nn)-newgoal2(1))^2+(ypos2(nn)-newgoal2(2))^2+(zpos2(nn)-newgoal2(3))^2)<15
        h1 = drawSphere(2,0.5,newgoal2(1), newgoal2(2), newgoal2(3));
        newsource=[xpos2(nn),ypos2(nn),zpos2(nn)];
        newgoal=newgoal2;
        cuDir_a=atan2(ypos2(nn)-ypos2(nn-1),xpos2(nn)-xpos2(nn-1)); 
        cuDir_b=atan2(zpos2(nn)-zpos2(nn-1),sqrt((xpos2(nn)-xpos2(nn-1))^2+(ypos2(nn)-ypos2(nn-1))^2));
        newpath=GETAPM(newsource,newgoal,cuDir_a,cuDir_b,1);
        n=size(newpath,1);
        xpos2=[xpos2(1:nn),newpath(:,1).'];
        ypos2=[ypos2(1:nn),newpath(:,2).'];
        zpos2=[zpos2(1:nn),newpath(:,3).'];
        D2=nn+n;
        newgoal2=[];
        flag2=1;
    end 
    if ~(nn>length(xpos2)-5)&&flag2==0
        j2=nn;
    for i=0:4
        if~feasiblePoint3d([xpos2(j2+i),ypos2(j2+i),zpos2(j2+i)])
            for m=21:25
                if sqrt((xpos2(j+i)-danareaX(m))^2+(ypos2(j+i)-danareaY(m))^2+(zpos2(j+i)-danareaZ(m))^2)<Rk
                    h1 = drawSphere(4,r,danareaX(m), danareaY(m), danareaZ(m));%画障碍球
                end
            end
            newsource=[xpos2(j2),ypos2(j2),zpos2(j2)];
            if j2+i==1
            cuDir_a=atan2(yt2-ys2,xt2-xs2); 
            cuDir_b=atan2(zt2-zs2,sqrt((xt2-xs2)^2+(yt2-ys2)^2));
            else
            cuDir_a=atan2(ypos2(j2+i)-ypos2(j2+i-1),xpos2(j2+i)-xpos2(j2+i-1)); 
            cuDir_b=atan2(zpos2(j2+i)-zpos2(j2+i-1),sqrt((xpos2(j2+i)-xpos2(j2+i-1))^2+(ypos2(j2+i)-ypos2(j2+i-1))^2));
            end
            k=j2+i+4;
            if k>D2+2
                k=D2+2;
            else
                while k<(D2+2)&&(~feasiblePoint3d([xpos2(k),ypos2(k),zpos2(k)])||~feasiblePoint3d([xpos2(k-3),ypos2(k-3),zpos2(k-3)]))
                    k=k+1;
                end
            end
                    newgoal=[xpos2(k),ypos2(k),zpos2(k)];
                    newpath=GETAPM(newsource,newgoal,cuDir_a,cuDir_b,2);
                    n=size(newpath,1);
                    xpos2=[xpos2(1:j2),newpath(:,1).',xpos2(k+1:D2+2)];
                    ypos2=[ypos2(1:j2),newpath(:,2).',ypos2(k+1:D2+2)];
                    zpos2=[zpos2(1:j2),newpath(:,3).',zpos2(k+1:D2+2)];
                    D2=D2+n+j2-k;
                    i=4;
        end
    end
    end
    %同第一架无人机
    if ~(isempty(newgoal3))&&sqrt((xpos3(nn)-newgoal3(1))^2+(ypos3(nn)-newgoal3(2))^2+(zpos3(nn)-newgoal3(3))^2)<10
        h1 = drawSphere(3,0.5,newgoal3(1), newgoal3(2), newgoal3(3));
        newsource=[xpos3(nn),ypos3(nn),zpos3(nn)];
        newgoal=newgoal3;
        cuDir_a=atan2(ypos3(nn)-ypos3(nn-1),xpos3(nn)-xpos3(nn-1)); 
        cuDir_b=atan2(zpos3(nn)-zpos3(nn-1),sqrt((xpos3(nn)-xpos3(nn-1))^2+(ypos3(nn)-ypos3(nn-1))^2));
        newpath=GETAPM(newsource,newgoal,cuDir_a,cuDir_b,1);
        n=size(newpath,1);
        xpos3=[xpos3(1:nn),newpath(:,1).'];
        ypos3=[ypos3(1:nn),newpath(:,2).'];
        zpos3=[zpos3(1:nn),newpath(:,3).'];
        D2=nn+n;
        newgoal3=[];
        flag3=1;
    end 
    if ~(nn>length(xpos3)-5)&&flag3==0;
        j3=nn;
    for i=0:4
        if~feasiblePoint3d([xpos3(j3+i),ypos3(j3+i),zpos3(j3+i)])
            for m=21:25
                if sqrt((xpos3(j3+i)-danareaX(m))^2+(ypos3(j3+i)-danareaY(m))^2+(zpos3(j3+i)-danareaZ(m))^2)<Rk
                    h1 = drawSphere(4,r,danareaX(m), danareaY(m), danareaZ(m));%画障碍球
                end
            end
            newsource=[xpos3(j3),ypos3(j3),zpos3(j3)];
            if j3+i==1
            cuDir_a=atan2(yt3-ys3,xt3-xs3); 
            cuDir_b=atan2(zt3-zs3,sqrt((xt3-xs3)^2+(yt3-ys3)^2));
            else
            cuDir_a=atan2(ypos3(j3+i)-ypos3(j3+i-1),xpos3(j3+i)-xpos3(j3+i-1)); 
            cuDir_b=atan2(zpos3(j3+i)-zpos3(j3+i-1),sqrt((xpos3(j3+i)-xpos3(j3+i-1))^2+(ypos3(j3+i)-ypos3(j3+i-1))^2));
            end
            k=j3+i+4;
            if k>D3+2
                k=D3+2;
            else
                while k<D3+2&&(~feasiblePoint3d([xpos3(k),ypos3(k),zpos3(k)])||~feasiblePoint3d([xpos3(k-3),ypos3(k-3),zpos3(k-3)]))
                    k=k+1;
                end
            end
                    newgoal=[xpos3(k),ypos3(k),zpos3(k)];
                    newpath=GETAPM(newsource,newgoal,cuDir_a,cuDir_b,3);
                    n=size(newpath,1);
                    xpos3=[xpos3(1:j3),newpath(:,1).',xpos3(k+1:D3+2)];
                    ypos3=[ypos3(1:j3),newpath(:,2).',ypos3(k+1:D3+2)];
                    zpos3=[zpos3(1:j3),newpath(:,3).',zpos3(k+1:D3+2)];
                    D3=D3+n+j3-k;
                    i=4;
        end
    end
    end
    if ~(nn>length(xpos))
        h1 = drawSphere(1,0.5,xpos(nn), ypos(nn), zpos(nn));
        currentPosition1=[xpos(nn), ypos(nn), zpos(nn)];
    end
    if ~(nn>length(xpos2))
        h1 = drawSphere(2,0.5,xpos2(nn), ypos2(nn), zpos2(nn));
        currentPosition2=[xpos2(nn), ypos2(nn), zpos2(nn)];
    end
    if ~(nn>length(xpos3))
        h1 = drawSphere(3,0.5,xpos3(nn), ypos3(nn), zpos3(nn));
        currentPosition3=[xpos3(nn), ypos3(nn), zpos3(nn)];
    end
     M(t)=getframe;t=t+1;
    nn=nn+1;
end               


