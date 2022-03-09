clear
clc
%载入地形
load('Z111.mat');
global  Z3;
h=[26, 30,  25,  28 , 27, 25, 23];
x0=[20, 45,  45, 15, 57,  50, 10];
y0=[20, 25,  50, 50,  41, 8,  25];
xi=[5.5, 8,  5,  3.5, 4, 4,  4.5];
yi=[5,   7,  6,   4, 5,  3,  4.5];
Z2=CeatHill(7,h,x0,y0,xi,yi,65); 
Z3=max(Z1,Z2);
figure(2);
surf(Z3); %画出三维曲面 
axis([1 65 1 65 1 30])%限制xyz轴范围
hold on;
global Np Rk ;  %威胁源数
Rk=3;%威胁半径
%载入威胁源
global danareaX;
global danareaY;
global danareaZ;
load('danX.mat');
load('danY.mat');
load('danZ.mat');
Np=length(danareaX);
for j=1:Np
    h1 = drawSphere(0,5,danareaX(j),danareaY(j),danareaZ(j));
end
%设置起始点和目标点
global xt xs xt2 xs2 xt3 xs3;
global yt ys yt2 ys2 yt3 ys3;
global zt zs zt2 zs2 zt3 zs3;
xt=60;xt2=60;xt3=60;
yt=40;yt2=45;yt3=35;
zt=15;zt2=15;zt3=15;
xs=5;xs2=5;xs3=5;
ys=30;ys2=32;ys3=28;
zs=10;zs2=10;zs3=10;
global source goal source2 goal2 source3 goal3;
goal=[xt,yt,zt];
source=[xs,ys,zs];
goal2=[xt2,yt2,zt2];
source2=[xs2,ys2,zs2];
goal3=[xt3,yt3,zt3];
source3=[xs3,ys3,zs3];
robotDirection_a=atan2(goal(2)-source(2),goal(1)-source(1)); % 水平面上的方向
robotDirection_b=atan2(goal(3)-source(3),sqrt((goal(1)-source(1))^2+(goal(2)-source(2))^2)); % 垂直面上的方向
robotDirection2_a=atan2(goal2(2)-source2(2),goal2(1)-source2(1)); % 水平面上的方向
robotDirection2_b=atan2(goal2(3)-source2(3),sqrt((goal2(1)-source2(1))^2+(goal2(2)-source2(2))^2));% 垂直面上的方向
robotDirection3_a=atan2(goal3(2)-source3(2),goal3(1)-source3(1)); % 水平面上的方向
robotDirection3_b=atan2(goal3(3)-source3(3),sqrt((goal3(1)-source3(1))^2+(goal3(2)-source3(2))^2)); % 垂直面上的方向
robotSize=[1 1 1]; %大小
robotSpeed=1; % 速度
robotSpeed2=1; % 速度
robotSpeed3=1;%速度
maxRobotSpeed=1; % 最大速度
S=10; % 安全距离
distanceThreshold=3; % 目标区域范围 
maxAcceleration=1; % 最大加速度
maxTurn=10*pi/180; % 最大转弯角
k=3; 
attractivePotentialScaling=2; % 引力场参数
repulsivePotentialScaling=100; % 斥力场参数
minAttractivePotential=1; % 最小引力

%%%%% parameters end here %%%%%

currentPosition=source; % 当前位置
cuDir_a=robotDirection_a; % 当前水平面的方向
cuDir_b=robotDirection_b; % 当前垂直面的方向
currentPosition2=source2; % 当前位置
cuDir2_a=robotDirection2_a; % 当前水平面的方向
cuDir2_b=robotDirection2_b; % 当前垂直面的方向
currentPosition3=source3; % 当前位置
cuDir3_a=robotDirection3_a; % 当前水平面的方向
cuDir3_b=robotDirection3_b; % 当前垂直面的方向
robotHalfDiagonalDistance=((robotSize(1)/2)^2+(robotSize(2)/2)^2)^0.5; 
pathFound=0; % 标志位
pathCost=0;
t=1;
pathLength=0; 
pathFound2=0; % 标志位
pathCost2=0;
pathlength2=0;
pathFound3=0; % 标志位
pathCost3=0;
pathlength3=0;
h1=drawSphere(1,0.5,currentPosition(1),currentPosition(2),currentPosition(3));
%目标点设置要合理
if ~feasiblePoint3d(goal), error('goal lies on an obstacle or outside map'); end
h1= drawSphere(2,0.5,currentPosition2(1),currentPosition2(2),currentPosition2(3));
if ~feasiblePoint3d(goal2), error('goal2 lies on an obstacle or outside map'); end
h1=drawSphere(3,0.5,currentPosition3(1),currentPosition3(2),currentPosition3(3));
if ~feasiblePoint3d(goal3), error('goal3 lies on an obstacle or outside map'); end
M(t)=getframe;
t=t+1;
num=0;
%开始进行航迹规划
while ~pathFound||~pathFound2||~pathFound3
    %无人机1还未到达目标点
    if ~pathFound
    %获取不同方向离最近障碍的距离
    i=robotSize(1)/2+1;
    while true
        x=currentPosition+i*[cos(cuDir_b)*[cos(cuDir_a) sin(cuDir_a)] sin(cuDir_b)];
        if ~feasiblePoint3d(x); break; end
        if sqrt(sum(x-(currentPosition2)).^2)<1&&~pathFound2,break;end
        if sqrt(sum(x-(currentPosition3)).^2)<1&&~pathFound3,break;end
        i=i+1;
    end
    distanceFront=i-robotSize(1)/2; % robotSize(1)/2 distance included in i was inside the robot body 
    
    %获取不同方向离最近障碍的距离
    i=robotSize(2)/2+1;
    while true
        x=currentPosition+i*[cos(cuDir_b)*[cos(cuDir_a-pi/2) sin(cuDir_a-pi/2)] sin(cuDir_b)];
        if ~feasiblePoint3d(x); break; end
        if sqrt(sum(x-currentPosition2).^2)<1&&~pathFound2,break;end
        if sqrt(sum(x-currentPosition3).^2)<1&&~pathFound3,break;end
        i=i+1;
    end
    distanceLeft=i-robotSize(2)/2;  
    
    %获取不同方向离最近障碍的距离
    i=robotSize(2)/2+1;
    while true
        x=currentPosition+i*[cos(cuDir_b)*[cos(cuDir_a+pi/2) sin(cuDir_a+pi/2)] sin(cuDir_b)];
        if ~feasiblePoint3d(x); break; end
         if sqrt(sum(x-currentPosition2).^2)<1&&~pathFound2,break;end
         if sqrt(sum(x-currentPosition3).^2)<1&&~pathFound3,break;end
        i=i+1;
    end
    distanceRight=i-robotSize(2)/2;  
    
    %获取不同方向离最近障碍的距离
    i=robotHalfDiagonalDistance+1;
    while true
        x=currentPosition+i*[cos(cuDir_b+pi/2)*[cos(cuDir_a) sin(cuDir_a)] sin(cuDir_b+pi/2)];
        if ~feasiblePoint3d(x), break; end
         if sqrt(sum(x-currentPosition2).^2)<1&&~pathFound2,break;end
         if sqrt(sum(x-currentPosition3).^2)<1&&~pathFound3,break;end
        i=i+1;
    end
    distanceUp=i-robotHalfDiagonalDistance;
    
    %获取不同方向离最近障碍的距离
    i=robotHalfDiagonalDistance+1;
    while true
        x=currentPosition+i*[cos(cuDir_b-pi/2)*[cos(cuDir_a) sin(cuDir_a)] sin(cuDir_b-pi/2)];
        if ~feasiblePoint3d(x), break; end
        if sqrt(sum(x-currentPosition2).^2)<1&&~pathFound2,break;end
        if sqrt(sum(x-currentPosition3).^2)<1&&~pathFound3,break;end
        i=i+1;
    end
    distanceDown=i-robotHalfDiagonalDistance;
    %获取不同方向离最近障碍的距离
    i=robotHalfDiagonalDistance+1;
    while true
        x=currentPosition+i*[cos(cuDir_b)*[cos(cuDir_a+pi/4) sin(cuDir_a+pi/4)] sin(cuDir_b)];
        if ~feasiblePoint3d(x), break; end
        if sqrt(sum(x-currentPosition2).^2)<1&&~pathFound2,break;end
        if sqrt(sum(x-currentPosition3).^2)<1&&~pathFound3,break;end
        i=i+1;
    end
    distanceRF=i-robotHalfDiagonalDistance;
    %获取不同方向离最近障碍的距离
    i=robotHalfDiagonalDistance+1;
    while true
        x=currentPosition+i*[cos(cuDir_b)*[cos(cuDir_a-pi/4) sin(cuDir_a-pi/4)] sin(cuDir_b)];
        if ~feasiblePoint3d(x), break; end
        if sqrt(sum(x-currentPosition2).^2)<1&&~pathFound2,break;end
        if sqrt(sum(x-currentPosition3).^2)<1&&~pathFound3,break;end
        i=i+1;
    end
    distanceLF=i-robotHalfDiagonalDistance;
    %获取不同方向离最近障碍的距离
    i=robotHalfDiagonalDistance+1;
    while true
        x=currentPosition+i*[cos(cuDir_b+pi/4)*[cos(cuDir_a) sin(cuDir_a)] sin(cuDir_b+pi/4)];
        if ~feasiblePoint3d(x), break; end
        if sqrt(sum(x-currentPosition2).^2)<1&&~pathFound2,break;end
        if sqrt(sum(x-currentPosition3).^2)<1&&~pathFound3,break;end
        i=i+1;
    end
    distanceUF=i-robotHalfDiagonalDistance;
    %获取不同方向离最近障碍的距离
    i=robotHalfDiagonalDistance+1;
    while true
        x=currentPosition+i*[cos(cuDir_b-pi/4)*[cos(cuDir_a) sin(cuDir_a)] sin(cuDir_b-pi/4)];
        if ~feasiblePoint3d(x), break; end
        if sqrt(sum(x-currentPosition2).^2)<1&&~pathFound2,break;end
        if sqrt(sum(x-currentPosition3).^2)<1&&~pathFound3,break;end
        i=i+1;
    end
    distanceDF=i-robotHalfDiagonalDistance;
    %获取不同方向离最近障碍的距离
    i=robotHalfDiagonalDistance+1;
    while true
        x=currentPosition+i*[cos(cuDir_b+pi/4)*[cos(cuDir_a+pi/4) sin(cuDir_a+pi/4)] sin(cuDir_b+pi/4)];
        if ~feasiblePoint3d(x), break; end
        if sqrt(sum(x-currentPosition2).^2)<1&&~pathFound2,break;end
        if sqrt(sum(x-currentPosition3).^2)<1&&~pathFound3,break;end
        i=i+1;
    end
    distanceURF=i-robotHalfDiagonalDistance;
    %获取不同方向离最近障碍的距离
    i=robotHalfDiagonalDistance+1;
    while true
        x=currentPosition+i*[cos(cuDir_b+pi/4)*[cos(cuDir_a-pi/4) sin(cuDir_a-pi/4)] sin(cuDir_b+pi/4)];
        if ~feasiblePoint3d(x), break; end
        if sqrt(sum(x-currentPosition2).^2)<1&&~pathFound2,break;end
        if sqrt(sum(x-currentPosition3).^2)<1&&~pathFound3,break;end
        i=i+1;
    end
    distanceULF=i-robotHalfDiagonalDistance;
    %获取不同方向离最近障碍的距离
    i=robotHalfDiagonalDistance+1;
    while true
        x=currentPosition+i*[cos(cuDir_b-pi/4)*[cos(cuDir_a+pi/4) sin(cuDir_a+pi/4)] sin(cuDir_b-pi/4)];
        if ~feasiblePoint3d(x), break; end
        if sqrt(sum(x-currentPosition2).^2)<1&&~pathFound2,break;end
        if sqrt(sum(x-currentPosition3).^2)<1&&~pathFound3,break;end
        i=i+1;
    end
    distanceDRF=i-robotHalfDiagonalDistance;
    %获取不同方向离最近障碍的距离
    i=robotHalfDiagonalDistance+1;
    while true
        x=currentPosition+i*[cos(cuDir_b-pi/4)*[cos(cuDir_a-pi/4) sin(cuDir_a-pi/4)] sin(cuDir_b-pi/4)];
        if ~feasiblePoint3d(x), break; end
        if sqrt(sum(x-currentPosition2).^2)<1&&~pathFound2,break;end
        if sqrt(sum(x-currentPosition3).^2)<1&&~pathFound3,break;end
        i=i+1;
    end
    distanceDLF=i-robotHalfDiagonalDistance;
    
    % 计算目标点和自己的方向
     angleGoal_a=atan2(goal(2)-currentPosition(2),goal(1)-currentPosition(1));
     angleGoal_b=atan2(goal(3)-currentPosition(3),sqrt((goal(1)-currentPosition(1))^2+(goal(2)-currentPosition(2))^2));
    
     % 计算目标点和自己的距离
     distanceGoal=( sqrt(sum((currentPosition-goal).^2)) );
     %已到达目标点，推出航迹规划
     if distanceGoal<distanceThreshold, pathFound=true; end
     %根据不同方向的距离计算斥力场
     repulsivePotential=[0,0,0];
     if distanceFront<20
        repulsivePotential=repulsivePotential+(1.0/distanceFront-0.05)^k*[cos(cuDir_b)*[cos(cuDir_a) sin(cuDir_a)] sin(cuDir_b)];
     end
     if distanceLeft<20
        repulsivePotential=repulsivePotential+(1.0/distanceLeft-0.05)^k*[cos(cuDir_b)*[cos(cuDir_a-pi/2) sin(cuDir_a-pi/2)] sin(cuDir_b)];
     end
     if distanceRight<20
        repulsivePotential=repulsivePotential+(1.0/distanceRight-0.05)^k*[cos(cuDir_b)*[cos(cuDir_a+pi/2) sin(cuDir_a+pi/2)] sin(cuDir_b)];
     end
     if distanceUp<20
        repulsivePotential=repulsivePotential+(1.0/distanceUp-0.05)^k*[cos(cuDir_b+pi/2)*[cos(cuDir_a) sin(cuDir_a)] sin(cuDir_b+pi/2)];
     end
     if distanceDown<20
        repulsivePotential=repulsivePotential+(1.0/distanceDown-0.05)^k*[cos(cuDir_b-pi/2)*[cos(cuDir_a) sin(cuDir_a)] sin(cuDir_b-pi/2)];
     end
     if distanceRF<20
        repulsivePotential=repulsivePotential+(1.0/distanceRF-0.05)^k*[cos(cuDir_b)*[cos(cuDir_a+pi/4) sin(cuDir_a+pi/4)] sin(cuDir_b)];
     end
     if distanceLF<20
        repulsivePotential=repulsivePotential+(1.0/distanceLF-0.05)^k*[cos(cuDir_b)*[cos(cuDir_a-pi/4) sin(cuDir_a-pi/4)] sin(cuDir_b)];
     end
     if distanceUF<20
        repulsivePotential=repulsivePotential+(1.0/distanceUF-0.05)^k*[cos(cuDir_b+pi/4)*[cos(cuDir_a) sin(cuDir_a)] sin(cuDir_b+pi/4)];
     end
     if distanceDF<20
        repulsivePotential=repulsivePotential+(1.0/distanceDF-0.05)^k*[cos(cuDir_b-pi/4)*[cos(cuDir_a) sin(cuDir_a)] sin(cuDir_b-pi/4)];
     end
     if distanceURF<20
        repulsivePotential=repulsivePotential+(1.0/distanceURF-0.05)^k*[cos(cuDir_b+pi/4)*[cos(cuDir_a+pi/4) sin(cuDir_a+pi/4)] sin(cuDir_b+pi/4)];
     end
     if distanceULF<20
        repulsivePotential=repulsivePotential+(1.0/distanceULF-0.05)^k*[cos(cuDir_b+pi/4)*[cos(cuDir_a-pi/4) sin(cuDir_a-pi/4)] sin(cuDir_b+pi/4)];
     end
     if distanceDRF<20
        repulsivePotential=repulsivePotential+(1.0/distanceDRF-0.05)^k*[cos(cuDir_b-pi/4)*[cos(cuDir_a+pi/4) sin(cuDir_a+pi/4)] sin(cuDir_b-pi/4)];
     end
     if distanceDLF<20
        repulsivePotential=repulsivePotential+(1.0/distanceDLF-0.05)^k*[cos(cuDir_b-pi/4)*[cos(cuDir_a-pi/4) sin(cuDir_a-pi/4)] sin(cuDir_b-pi/4)];
     end
     %计算引力场
     attractivePotential=max([(1.0/distanceGoal)^k*attractivePotentialScaling minAttractivePotential ])*[cos(angleGoal_b)*[cos(angleGoal_a) sin(angleGoal_a)] sin(angleGoal_b)];
     %计算综合力场
      totalPotential=attractivePotential-repulsivePotentialScaling*repulsivePotential;
     
     % 计算水平方向需要改变的角度，并对其限幅
     preferredSteer_a=atan2(robotSpeed*cos(cuDir_b)*sin(cuDir_a)+totalPotential(2),robotSpeed*cos(cuDir_b)*cos(cuDir_a)+totalPotential(1))-cuDir_a;
      while preferredSteer_a>pi, preferredSteer_a=preferredSteer_a-2*pi; end % check to get the angle between -pi and pi
      while preferredSteer_a<-pi, preferredSteer_a=preferredSteer_a+2*pi; end % check to get the angle between -pi and pi
     preferredSteer_a=min([maxTurn preferredSteer_a]);
     preferredSteer_a=max([-maxTurn preferredSteer_a]);
     %改变水平方向角
     cuDir_a=cuDir_a+preferredSteer_a;
     % 计算垂直方向需要改变的角度，并对其限幅
     preferredSteer_b=atan2(robotSpeed*sin(cuDir_b)+totalPotential(3),sqrt((robotSpeed*cos(cuDir_b)*cos(cuDir_a)+totalPotential(1))^2+(robotSpeed*cos(cuDir_b)*sin(cuDir_a)+totalPotential(2))^2))-cuDir_b;
      while preferredSteer_b>pi, preferredSteer_b=preferredSteer_b-2*pi; end % check to get the angle between -pi and pi
      while preferredSteer_b<-pi, preferredSteer_b=preferredSteer_b+2*pi; end % check to get the angle between -pi and pi
     preferredSteer_b=min([maxTurn preferredSteer_b]);
     preferredSteer_b=max([-maxTurn preferredSteer_b]);
     %改变垂直方向角
     cuDir_b=cuDir_b+preferredSteer_b;
     %计算速度
     preferredSpeed=sqrt(sum((robotSpeed*[cos(cuDir_b)*[cos(cuDir_a) sin(cuDir_a)] sin(cuDir_b)] + totalPotential).^2));
     preferredSpeed=min([robotSpeed+maxAcceleration preferredSpeed]);
     robotSpeed=max([robotSpeed-maxAcceleration preferredSpeed]);
     robotSpeed=min([robotSpeed maxRobotSpeed]);
     robotSpeed=max([robotSpeed 0]);
     
     
     % 根据速度和角度计算新的航迹点
     newPosition=currentPosition+robotSpeed*[cos(cuDir_b)*[cos(cuDir_a) sin(cuDir_a)] sin(cuDir_b)];
     pathCost=pathCost+distanceCost(newPosition,currentPosition);
     currentPosition=newPosition;
     if ~feasiblePoint3d(currentPosition), error('collission recorded'); end
     
     % 画图
     h1=drawSphere(1,0.5,currentPosition(1),currentPosition(2),currentPosition(3));
     M(t)=getframe;t=t+1;
    end
 %无人机2，如无人机1
if ~pathFound2
    i=robotSize(1)/2+1;
    while true
        x=currentPosition2+i*[cos(cuDir2_b)*[cos(cuDir2_a) sin(cuDir2_a)] sin(cuDir2_b)];
        if ~feasiblePoint3d(x); break; end
        if sqrt(sum(x-(currentPosition)).^2)<1&&~pathFound,break;end
        if sqrt(sum(x-(currentPosition3)).^2)<1&&~pathFound3,break;end
        i=i+1;
    end
    distanceFront=i-robotSize(1)/2; 
    i=robotSize(2)/2+1;
    while true
        x=currentPosition2+i*[cos(cuDir2_b)*[cos(cuDir2_a-pi/2) sin(cuDir2_a-pi/2)] sin(cuDir2_b)];
        if ~feasiblePoint3d(x); break; end
        if sqrt(sum(x-currentPosition).^2)<1&&~pathFound,break;end
        if sqrt(sum(x-currentPosition3).^2)<1&&~pathFound3,break;end
        i=i+1;
    end
    distanceLeft=i-robotSize(2)/2;  
    i=robotSize(2)/2+1;
    while true
        x=currentPosition2+i*[cos(cuDir2_b)*[cos(cuDir2_a+pi/2) sin(cuDir2_a+pi/2)] sin(cuDir2_b)];
        if ~feasiblePoint3d(x); break; end
         if sqrt(sum(x-currentPosition).^2)<1&&~pathFound,break;end
         if sqrt(sum(x-currentPosition3).^2)<1&&~pathFound3,break;end
        i=i+1;
    end
    distanceRight=i-robotSize(2)/2;  
    i=robotHalfDiagonalDistance+1;
    while true
        x=currentPosition2+i*[cos(cuDir2_b+pi/2)*[cos(cuDir2_a) sin(cuDir2_a)] sin(cuDir2_b+pi/2)];
        if ~feasiblePoint3d(x), break; end
         if sqrt(sum(x-currentPosition).^2)<1&&~pathFound,break;end
         if sqrt(sum(x-currentPosition3).^2)<1&&~pathFound3,break;end
        i=i+1;
    end
    distanceUp=i-robotHalfDiagonalDistance;
    i=robotHalfDiagonalDistance+1;
    while true
        x=currentPosition2+i*[cos(cuDir2_b-pi/2)*[cos(cuDir2_a) sin(cuDir2_a)] sin(cuDir2_b-pi/2)];
        if ~feasiblePoint3d(x), break; end
        if sqrt(sum(x-currentPosition).^2)<1&&~pathFound,break;end
        if sqrt(sum(x-currentPosition3).^2)<1&&~pathFound3,break;end
        i=i+1;
    end
    distanceDown=i-robotHalfDiagonalDistance;
    i=robotHalfDiagonalDistance+1;
    while true
        x=currentPosition2+i*[cos(cuDir2_b)*[cos(cuDir2_a+pi/4) sin(cuDir2_a+pi/4)] sin(cuDir2_b)];
        if ~feasiblePoint3d(x), break; end
        if sqrt(sum(x-currentPosition).^2)<1&&~pathFound,break;end
        if sqrt(sum(x-currentPosition3).^2)<1&&~pathFound3,break;end
        i=i+1;
    end
    distanceRF=i-robotHalfDiagonalDistance;
    i=robotHalfDiagonalDistance+1;
    while true
        x=currentPosition2+i*[cos(cuDir2_b)*[cos(cuDir2_a-pi/4) sin(cuDir2_a-pi/4)] sin(cuDir2_b)];
        if ~feasiblePoint3d(x), break; end
        if sqrt(sum(x-currentPosition).^2)<1&&~pathFound,break;end
        if sqrt(sum(x-currentPosition3).^2)<1&&~pathFound3,break;end
        i=i+1;
    end
    distanceLF=i-robotHalfDiagonalDistance;
    i=robotHalfDiagonalDistance+1;
    while true
        x=currentPosition2+i*[cos(cuDir2_b+pi/4)*[cos(cuDir2_a) sin(cuDir2_a)] sin(cuDir2_b+pi/4)];
        if ~feasiblePoint3d(x), break; end
        if sqrt(sum(x-currentPosition).^2)<1&&~pathFound,break;end
        if sqrt(sum(x-currentPosition3).^2)<1&&~pathFound3,break;end
        i=i+1;
    end
    distanceUF=i-robotHalfDiagonalDistance;
    i=robotHalfDiagonalDistance+1;
    while true
        x=currentPosition2+i*[cos(cuDir2_b-pi/4)*[cos(cuDir2_a) sin(cuDir2_a)] sin(cuDir2_b-pi/4)];
        if ~feasiblePoint3d(x), break; end
        if sqrt(sum(x-currentPosition).^2)<1&&~pathFound,break;end
        if sqrt(sum(x-currentPosition3).^2)<1&&~pathFound3,break;end
        i=i+1;
    end
    distanceDF=i-robotHalfDiagonalDistance;
    i=robotHalfDiagonalDistance+1;
    while true
        x=currentPosition2+i*[cos(cuDir2_b+pi/4)*[cos(cuDir2_a+pi/4) sin(cuDir2_a+pi/4)] sin(cuDir2_b+pi/4)];
        if ~feasiblePoint3d(x), break; end
        if sqrt(sum(x-currentPosition).^2)<1&&~pathFound,break;end
        if sqrt(sum(x-currentPosition3).^2)<1&&~pathFound3,break;end
        i=i+1;
    end
    distanceURF=i-robotHalfDiagonalDistance;
    i=robotHalfDiagonalDistance+1;
    while true
        x=currentPosition2+i*[cos(cuDir2_b+pi/4)*[cos(cuDir2_a-pi/4) sin(cuDir2_a-pi/4)] sin(cuDir2_b+pi/4)];
        if ~feasiblePoint3d(x), break; end
        if sqrt(sum(x-currentPosition).^2)<1&&~pathFound,break;end
        if sqrt(sum(x-currentPosition3).^2)<1&&~pathFound3,break;end
        i=i+1;
    end
    distanceULF=i-robotHalfDiagonalDistance;
    i=robotHalfDiagonalDistance+1;
    while true
        x=currentPosition2+i*[cos(cuDir2_b-pi/4)*[cos(cuDir2_a+pi/4) sin(cuDir2_a+pi/4)] sin(cuDir2_b-pi/4)];
        if ~feasiblePoint3d(x), break; end
        if sqrt(sum(x-currentPosition).^2)<1&&~pathFound,break;end
        if sqrt(sum(x-currentPosition3).^2)<1&&~pathFound3,break;end
        i=i+1;
    end
    distanceDRF=i-robotHalfDiagonalDistance;
    i=robotHalfDiagonalDistance+1;
    while true
        x=currentPosition2+i*[cos(cuDir2_b-pi/4)*[cos(cuDir2_a-pi/4) sin(cuDir2_a-pi/4)] sin(cuDir2_b-pi/4)];
        if ~feasiblePoint3d(x), break; end
        if sqrt(sum(x-currentPosition).^2)<1&&~pathFound,break;end
        if sqrt(sum(x-currentPosition3).^2)<1&&~pathFound3,break;end
        i=i+1;
    end
    distanceDLF=i-robotHalfDiagonalDistance;
    
   
     angleGoal_a=atan2(goal2(2)-currentPosition2(2),goal2(1)-currentPosition2(1));
     angleGoal_b=atan2(goal2(3)-currentPosition2(3),sqrt((goal2(1)-currentPosition2(1))^2+(goal2(2)-currentPosition2(2))^2));
    
    
     distanceGoal=( sqrt(sum((currentPosition2-goal2).^2)) );
     if distanceGoal<distanceThreshold, pathFound2=true; end
     repulsivePotential=[0,0,0];
     
     if distanceFront<20
        repulsivePotential=repulsivePotential+(1.0/distanceFront-0.05)^k*[cos(cuDir2_b)*[cos(cuDir2_a) sin(cuDir2_a)] sin(cuDir2_b)];
     end
     if distanceLeft<20
        repulsivePotential=repulsivePotential+(1.0/distanceLeft-0.05)^k*[cos(cuDir2_b)*[cos(cuDir2_a-pi/2) sin(cuDir2_a-pi/2)] sin(cuDir2_b)];
     end
     if distanceRight<20
        repulsivePotential=repulsivePotential+(1.0/distanceRight-0.05)^k*[cos(cuDir2_b)*[cos(cuDir2_a+pi/2) sin(cuDir2_a+pi/2)] sin(cuDir2_b)];
     end
     if distanceUp<20
        repulsivePotential=repulsivePotential+(1.0/distanceUp-0.05)^k*[cos(cuDir2_b+pi/2)*[cos(cuDir2_a) sin(cuDir2_a)] sin(cuDir2_b+pi/2)];
     end
     if distanceDown<20
        repulsivePotential=repulsivePotential+(1.0/distanceDown-0.05)^k*[cos(cuDir2_b-pi/2)*[cos(cuDir2_a) sin(cuDir2_a)] sin(cuDir2_b-pi/2)];
     end
     if distanceRF<20
        repulsivePotential=repulsivePotential+(1.0/distanceRF-0.05)^k*[cos(cuDir2_b)*[cos(cuDir2_a+pi/4) sin(cuDir2_a+pi/4)] sin(cuDir2_b)];
     end
     if distanceLF<20
        repulsivePotential=repulsivePotential+(1.0/distanceLF-0.05)^k*[cos(cuDir2_b)*[cos(cuDir2_a-pi/4) sin(cuDir2_a-pi/4)] sin(cuDir2_b)];
     end
     if distanceUF<20
        repulsivePotential=repulsivePotential+(1.0/distanceUF-0.05)^k*[cos(cuDir2_b+pi/4)*[cos(cuDir2_a) sin(cuDir2_a)] sin(cuDir2_b+pi/4)];
     end
     if distanceDF<20
        repulsivePotential=repulsivePotential+(1.0/distanceDF-0.05)^k*[cos(cuDir2_b-pi/4)*[cos(cuDir2_a) sin(cuDir2_a)] sin(cuDir2_b-pi/4)];
     end
     if distanceURF<20
        repulsivePotential=repulsivePotential+(1.0/distanceURF-0.05)^k*[cos(cuDir2_b+pi/4)*[cos(cuDir2_a+pi/4) sin(cuDir2_a+pi/4)] sin(cuDir2_b+pi/4)];
     end
     if distanceULF<20
        repulsivePotential=repulsivePotential+(1.0/distanceULF-0.05)^k*[cos(cuDir2_b+pi/4)*[cos(cuDir2_a-pi/4) sin(cuDir2_a-pi/4)] sin(cuDir2_b+pi/4)];
     end
     if distanceDRF<20
        repulsivePotential=repulsivePotential+(1.0/distanceDRF-0.05)^k*[cos(cuDir2_b-pi/4)*[cos(cuDir2_a+pi/4) sin(cuDir2_a+pi/4)] sin(cuDir2_b-pi/4)];
     end
     if distanceDLF<20
        repulsivePotential=repulsivePotential+(1.0/distanceDLF-0.05)^k*[cos(cuDir2_b-pi/4)*[cos(cuDir2_a-pi/4) sin(cuDir2_a-pi/4)] sin(cuDir2_b-pi/4)];
     end
     attractivePotential=max([(1.0/distanceGoal)^k*attractivePotentialScaling minAttractivePotential ])*[cos(angleGoal_b)*[cos(angleGoal_a) sin(angleGoal_a)] sin(angleGoal_b)];
      totalPotential=attractivePotential-repulsivePotentialScaling*repulsivePotential;
     
    
     preferredSteer_a=atan2(robotSpeed*cos(cuDir2_b)*sin(cuDir2_a)+totalPotential(2),robotSpeed*cos(cuDir2_b)*cos(cuDir2_a)+totalPotential(1))-cuDir2_a;
      while preferredSteer_a>pi, preferredSteer_a=preferredSteer_a-2*pi; end % check to get the angle between -pi and pi
      while preferredSteer_a<-pi, preferredSteer_a=preferredSteer_a+2*pi; end % check to get the angle between -pi and pi
     preferredSteer_a=min([maxTurn preferredSteer_a]);
     preferredSteer_a=max([-maxTurn preferredSteer_a]);
     cuDir2_a=cuDir2_a+preferredSteer_a;
     preferredSteer_b=atan2(robotSpeed*sin(cuDir2_b)+totalPotential(3),sqrt((robotSpeed*cos(cuDir2_b)*cos(cuDir2_a)+totalPotential(1))^2+(robotSpeed*cos(cuDir2_b)*sin(cuDir2_a)+totalPotential(2))^2))-cuDir2_b;
      while preferredSteer_b>pi, preferredSteer_b=preferredSteer_b-2*pi; end % check to get the angle between -pi and pi
      while preferredSteer_b<-pi, preferredSteer_b=preferredSteer_b+2*pi; end % check to get the angle between -pi and pi
     preferredSteer_b=min([maxTurn preferredSteer_b]);
     preferredSteer_b=max([-maxTurn preferredSteer_b]);
     cuDir2_b=cuDir2_b+preferredSteer_b;
     
     preferredSpeed=sqrt(sum((robotSpeed2*[cos(cuDir2_b)*[cos(cuDir2_a) sin(cuDir2_a)] sin(cuDir2_b)] + totalPotential).^2));
     preferredSpeed=min([robotSpeed2+maxAcceleration preferredSpeed]);
     robotSpeed2=max([robotSpeed2-maxAcceleration preferredSpeed]);
     robotSpeed2=min([robotSpeed2 maxRobotSpeed]);
     robotSpeed2=max([robotSpeed2 0]);
     
     
     if robotSpeed2==0, error('robot had to stop to avoid collission'); end
     
     % calculating new position based on steer and speed
     newPosition=currentPosition2+robotSpeed2*[cos(cuDir2_b)*[cos(cuDir2_a) sin(cuDir2_a)] sin(cuDir2_b)];
     pathCost2=pathCost2+distanceCost(newPosition,currentPosition2);
     currentPosition2=newPosition;
     if ~feasiblePoint3d(currentPosition2), error('collission recorded'); end
     
     % plotting robot
     h1=drawSphere(2,0.5,currentPosition2(1),currentPosition2(2),currentPosition2(3));
     M(t)=getframe;t=t+1;
end
%无人机3，如无人机1
if ~pathFound3
    i=robotSize(1)/2+1;
    while true
        x=currentPosition3+i*[cos(cuDir3_b)*[cos(cuDir3_a) sin(cuDir3_a)] sin(cuDir3_b)];
        if ~feasiblePoint3d(x); break; end
        if sqrt(sum(x-(currentPosition)).^2)<1&&~pathFound,break;end
        if sqrt(sum(x-(currentPosition2)).^2)<1&&~pathFound2,break;end
        i=i+1;
    end
    distanceFront=i-robotSize(1)/2; 
    i=robotSize(2)/2+1;
    while true
        x=currentPosition3+i*[cos(cuDir3_b)*[cos(cuDir3_a-pi/2) sin(cuDir3_a-pi/2)] sin(cuDir3_b)];
        if ~feasiblePoint3d(x); break; end
        if sqrt(sum(x-currentPosition).^2)<1&&~pathFound,break;end
        if sqrt(sum(x-currentPosition2).^2)<1&&~pathFound2,break;end
        i=i+1;
    end
    distanceLeft=i-robotSize(2)/2;  
    i=robotSize(2)/2+1;
    while true
        x=currentPosition3+i*[cos(cuDir3_b)*[cos(cuDir3_a+pi/2) sin(cuDir3_a+pi/2)] sin(cuDir3_b)];
        if ~feasiblePoint3d(x); break; end
         if sqrt(sum(x-currentPosition).^2)<1&&~pathFound,break;end
         if sqrt(sum(x-currentPosition2).^2)<1&&~pathFound2,break;end
        i=i+1;
    end
    distanceRight=i-robotSize(2)/2;  
    i=robotHalfDiagonalDistance+1;
    while true
        x=currentPosition3+i*[cos(cuDir3_b+pi/2)*[cos(cuDir3_a) sin(cuDir3_a)] sin(cuDir3_b+pi/2)];
        if ~feasiblePoint3d(x), break; end
         if sqrt(sum(x-currentPosition).^2)<1&&~pathFound,break;end
         if sqrt(sum(x-currentPosition2).^2)<1&&~pathFound2,break;end
        i=i+1;
    end
    distanceUp=i-robotHalfDiagonalDistance;
    i=robotHalfDiagonalDistance+1;
    while true
        x=currentPosition3+i*[cos(cuDir3_b-pi/2)*[cos(cuDir3_a) sin(cuDir3_a)] sin(cuDir3_b-pi/2)];
        if ~feasiblePoint3d(x), break; end
        if sqrt(sum(x-currentPosition).^2)<1&&~pathFound,break;end
        if sqrt(sum(x-currentPosition2).^2)<1&&~pathFound2,break;end
        i=i+1;
    end
    distanceDown=i-robotHalfDiagonalDistance;
    
    i=robotHalfDiagonalDistance+1;
    while true
        x=currentPosition3+i*[cos(cuDir3_b)*[cos(cuDir3_a+pi/4) sin(cuDir3_a+pi/4)] sin(cuDir3_b)];
        if ~feasiblePoint3d(x), break; end
        if sqrt(sum(x-currentPosition).^2)<1&&~pathFound,break;end
        if sqrt(sum(x-currentPosition2).^2)<1&&~pathFound2,break;end
        i=i+1;
    end
    distanceRF=i-robotHalfDiagonalDistance;
    i=robotHalfDiagonalDistance+1;
    while true
        x=currentPosition3+i*[cos(cuDir3_b)*[cos(cuDir3_a-pi/4) sin(cuDir3_a-pi/4)] sin(cuDir3_b)];
        if ~feasiblePoint3d(x), break; end
        if sqrt(sum(x-currentPosition).^2)<1&&~pathFound,break;end
        if sqrt(sum(x-currentPosition2).^2)<1&&~pathFound2,break;end
        i=i+1;
    end
    distanceLF=i-robotHalfDiagonalDistance;
    i=robotHalfDiagonalDistance+1;
    while true
        x=currentPosition3+i*[cos(cuDir3_b+pi/4)*[cos(cuDir3_a) sin(cuDir3_a)] sin(cuDir3_b+pi/4)];
        if ~feasiblePoint3d(x), break; end
        if sqrt(sum(x-currentPosition).^2)<1&&~pathFound,break;end
        if sqrt(sum(x-currentPosition2).^2)<1&&~pathFound2,break;end
        i=i+1;
    end
    distanceUF=i-robotHalfDiagonalDistance;
    i=robotHalfDiagonalDistance+1;
    while true
        x=currentPosition3+i*[cos(cuDir3_b-pi/4)*[cos(cuDir3_a) sin(cuDir3_a)] sin(cuDir3_b-pi/4)];
        if ~feasiblePoint3d(x), break; end
        if sqrt(sum(x-currentPosition).^2)<1&&~pathFound,break;end
        if sqrt(sum(x-currentPosition2).^2)<1&&~pathFound2,break;end
        i=i+1;
    end
    distanceDF=i-robotHalfDiagonalDistance;
    i=robotHalfDiagonalDistance+1;
    while true
        x=currentPosition3+i*[cos(cuDir3_b+pi/4)*[cos(cuDir3_a+pi/4) sin(cuDir3_a+pi/4)] sin(cuDir3_b+pi/4)];
        if ~feasiblePoint3d(x), break; end
        if sqrt(sum(x-currentPosition).^2)<1&&~pathFound,break;end
        if sqrt(sum(x-currentPosition2).^2)<1&&~pathFound2,break;end
        i=i+1;
    end
    distanceURF=i-robotHalfDiagonalDistance;
    i=robotHalfDiagonalDistance+1;
    while true
        x=currentPosition3+i*[cos(cuDir3_b+pi/4)*[cos(cuDir3_a-pi/4) sin(cuDir3_a-pi/4)] sin(cuDir3_b+pi/4)];
        if ~feasiblePoint3d(x), break; end
        if sqrt(sum(x-currentPosition).^2)<1&&~pathFound,break;end
        if sqrt(sum(x-currentPosition2).^2)<1&&~pathFound2,break;end
        i=i+1;
    end
    distanceULF=i-robotHalfDiagonalDistance;
    i=robotHalfDiagonalDistance+1;
    while true
        x=currentPosition3+i*[cos(cuDir3_b-pi/4)*[cos(cuDir3_a+pi/4) sin(cuDir3_a+pi/4)] sin(cuDir3_b-pi/4)];
        if ~feasiblePoint3d(x), break; end
        if sqrt(sum(x-currentPosition).^2)<1&&~pathFound,break;end
        if sqrt(sum(x-currentPosition2).^2)<1&&~pathFound2,break;end
        i=i+1;
    end
    distanceDRF=i-robotHalfDiagonalDistance;
    i=robotHalfDiagonalDistance+1;
    while true
        x=currentPosition3+i*[cos(cuDir3_b-pi/4)*[cos(cuDir3_a-pi/4) sin(cuDir3_a-pi/4)] sin(cuDir3_b-pi/4)];
        if ~feasiblePoint3d(x), break; end
        if sqrt(sum(x-currentPosition).^2)<1&&~pathFound,break;end
        if sqrt(sum(x-currentPosition2).^2)<1&&~pathFound2,break;end
        i=i+1;
    end
    distanceDLF=i-robotHalfDiagonalDistance;
    
    % calculate angle from goal
     angleGoal_a=atan2(goal3(2)-currentPosition3(2),goal3(1)-currentPosition3(1));
     angleGoal_b=atan2(goal3(3)-currentPosition3(3),sqrt((goal3(1)-currentPosition3(1))^2+(goal3(2)-currentPosition3(2))^2));
    
     % calculate diatnce from goal
     distanceGoal=( sqrt(sum((currentPosition3-goal3).^2)) );
     if distanceGoal<distanceThreshold, pathFound3=true; end
     repulsivePotential=[0,0,0];
     % compute potentials
     if distanceFront<20
        repulsivePotential=repulsivePotential+(1.0/distanceFront-0.05)^k*[cos(cuDir3_b)*[cos(cuDir3_a) sin(cuDir3_a)] sin(cuDir3_b)];
     end
     if distanceLeft<20
        repulsivePotential=repulsivePotential+(1.0/distanceLeft-0.05)^k*[cos(cuDir3_b)*[cos(cuDir3_a-pi/2) sin(cuDir3_a-pi/2)] sin(cuDir3_b)];
     end
     if distanceRight<20
        repulsivePotential=repulsivePotential+(1.0/distanceRight-0.05)^k*[cos(cuDir3_b)*[cos(cuDir3_a+pi/2) sin(cuDir3_a+pi/2)] sin(cuDir3_b)];
     end
     if distanceUp<20
        repulsivePotential=repulsivePotential+(1.0/distanceUp-0.05)^k*[cos(cuDir3_b+pi/2)*[cos(cuDir3_a) sin(cuDir3_a)] sin(cuDir3_b+pi/2)];
     end
     if distanceDown<20
        repulsivePotential=repulsivePotential+(1.0/distanceDown-0.05)^k*[cos(cuDir3_b-pi/2)*[cos(cuDir3_a) sin(cuDir3_a)] sin(cuDir3_b-pi/2)];
     end
     if distanceRF<20
        repulsivePotential=repulsivePotential+(1.0/distanceRF-0.05)^k*[cos(cuDir3_b)*[cos(cuDir3_a+pi/4) sin(cuDir3_a+pi/4)] sin(cuDir3_b)];
     end
     if distanceLF<20
        repulsivePotential=repulsivePotential+(1.0/distanceLF-0.05)^k*[cos(cuDir3_b)*[cos(cuDir3_a-pi/4) sin(cuDir3_a-pi/4)] sin(cuDir3_b)];
     end
     if distanceUF<20
        repulsivePotential=repulsivePotential+(1.0/distanceUF-0.05)^k*[cos(cuDir3_b+pi/4)*[cos(cuDir3_a) sin(cuDir3_a)] sin(cuDir3_b+pi/4)];
     end
     if distanceDF<20
        repulsivePotential=repulsivePotential+(1.0/distanceDF-0.05)^k*[cos(cuDir3_b-pi/4)*[cos(cuDir3_a) sin(cuDir3_a)] sin(cuDir3_b-pi/4)];
     end
     if distanceURF<20
        repulsivePotential=repulsivePotential+(1.0/distanceURF-0.05)^k*[cos(cuDir3_b+pi/4)*[cos(cuDir3_a+pi/4) sin(cuDir3_a+pi/4)] sin(cuDir3_b+pi/4)];
     end
     if distanceULF<20
        repulsivePotential=repulsivePotential+(1.0/distanceULF-0.05)^k*[cos(cuDir3_b+pi/4)*[cos(cuDir3_a-pi/4) sin(cuDir3_a-pi/4)] sin(cuDir3_b+pi/4)];
     end
     if distanceDRF<20
        repulsivePotential=repulsivePotential+(1.0/distanceDRF-0.05)^k*[cos(cuDir3_b-pi/4)*[cos(cuDir3_a+pi/4) sin(cuDir3_a+pi/4)] sin(cuDir3_b-pi/4)];
     end
     if distanceDLF<20
        repulsivePotential=repulsivePotential+(1.0/distanceDLF-0.05)^k*[cos(cuDir3_b-pi/4)*[cos(cuDir3_a-pi/4) sin(cuDir3_a-pi/4)] sin(cuDir3_b-pi/4)];
     end
     attractivePotential=max([(1.0/distanceGoal)^k*attractivePotentialScaling minAttractivePotential ])*[cos(angleGoal_b)*[cos(angleGoal_a) sin(angleGoal_a)] sin(angleGoal_b)];
      totalPotential=attractivePotential-repulsivePotentialScaling*repulsivePotential;
     
     % perform steer
     preferredSteer_a=atan2(robotSpeed3*cos(cuDir3_b)*sin(cuDir3_a)+totalPotential(2),robotSpeed3*cos(cuDir3_b)*cos(cuDir3_a)+totalPotential(1))-cuDir3_a;
      while preferredSteer_a>pi, preferredSteer_a=preferredSteer_a-2*pi; end % check to get the angle between -pi and pi
      while preferredSteer_a<-pi, preferredSteer_a=preferredSteer_a+2*pi; end % check to get the angle between -pi and pi
     preferredSteer_a=min([maxTurn preferredSteer_a]);
     preferredSteer_a=max([-maxTurn preferredSteer_a]);
     cuDir3_a=cuDir3_a+preferredSteer_a;
     preferredSteer_b=atan2(robotSpeed3*sin(cuDir3_b)+totalPotential(3),sqrt((robotSpeed3*cos(cuDir3_b)*cos(cuDir3_a)+totalPotential(1))^2+(robotSpeed3*cos(cuDir3_b)*sin(cuDir3_a)+totalPotential(2))^2))-cuDir3_b;
      while preferredSteer_b>pi, preferredSteer_b=preferredSteer_b-2*pi; end % check to get the angle between -pi and pi
      while preferredSteer_b<-pi, preferredSteer_b=preferredSteer_b+2*pi; end % check to get the angle between -pi and pi
     preferredSteer_b=min([maxTurn preferredSteer_b]);
     preferredSteer_b=max([-maxTurn preferredSteer_b]);
     cuDir3_b=cuDir3_b+preferredSteer_b;
     % setting the speed based on vehicle acceleration and speed limits. the vehicle cannot move backwards.
     preferredSpeed=sqrt(sum((robotSpeed3*[cos(cuDir3_b)*[cos(cuDir3_a) sin(cuDir3_a)] sin(cuDir3_b)] + totalPotential).^2));
     preferredSpeed=min([robotSpeed3+maxAcceleration preferredSpeed]);
     robotSpeed3=max([robotSpeed3-maxAcceleration preferredSpeed]);
     robotSpeed3=min([robotSpeed3 maxRobotSpeed]);
     robotSpeed3=max([robotSpeed3 0]);
     
     
     if robotSpeed3==0, error('robot had to stop to avoid collission'); end
     
     % calculating new position based on steer and speed
     newPosition=currentPosition3+robotSpeed3*[cos(cuDir3_b)*[cos(cuDir3_a) sin(cuDir3_a)] sin(cuDir3_b)];
     pathCost3=pathCost3+distanceCost(newPosition,currentPosition3);
     currentPosition3=newPosition;
     if ~feasiblePoint3d(currentPosition3), error('collission recorded'); end
     
     % plotting robot
     h1=drawSphere(3,0.5,currentPosition3(1),currentPosition3(2),currentPosition3(3));
     M(t)=getframe;t=t+1;
      end
end 
