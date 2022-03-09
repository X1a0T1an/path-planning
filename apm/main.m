clear
clc
%�������
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
surf(Z3); %������ά���� 
axis([1 65 1 65 1 30])%����xyz�᷶Χ
hold on;
global Np Rk ;  %��вԴ��
Rk=3;%��в�뾶
%������вԴ
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
%������ʼ���Ŀ���
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
robotDirection_a=atan2(goal(2)-source(2),goal(1)-source(1)); % ˮƽ���ϵķ���
robotDirection_b=atan2(goal(3)-source(3),sqrt((goal(1)-source(1))^2+(goal(2)-source(2))^2)); % ��ֱ���ϵķ���
robotDirection2_a=atan2(goal2(2)-source2(2),goal2(1)-source2(1)); % ˮƽ���ϵķ���
robotDirection2_b=atan2(goal2(3)-source2(3),sqrt((goal2(1)-source2(1))^2+(goal2(2)-source2(2))^2));% ��ֱ���ϵķ���
robotDirection3_a=atan2(goal3(2)-source3(2),goal3(1)-source3(1)); % ˮƽ���ϵķ���
robotDirection3_b=atan2(goal3(3)-source3(3),sqrt((goal3(1)-source3(1))^2+(goal3(2)-source3(2))^2)); % ��ֱ���ϵķ���
robotSize=[1 1 1]; %��С
robotSpeed=1; % �ٶ�
robotSpeed2=1; % �ٶ�
robotSpeed3=1;%�ٶ�
maxRobotSpeed=1; % ����ٶ�
S=10; % ��ȫ����
distanceThreshold=3; % Ŀ������Χ 
maxAcceleration=1; % �����ٶ�
maxTurn=10*pi/180; % ���ת���
k=3; 
attractivePotentialScaling=2; % ����������
repulsivePotentialScaling=100; % ����������
minAttractivePotential=1; % ��С����

%%%%% parameters end here %%%%%

currentPosition=source; % ��ǰλ��
cuDir_a=robotDirection_a; % ��ǰˮƽ��ķ���
cuDir_b=robotDirection_b; % ��ǰ��ֱ��ķ���
currentPosition2=source2; % ��ǰλ��
cuDir2_a=robotDirection2_a; % ��ǰˮƽ��ķ���
cuDir2_b=robotDirection2_b; % ��ǰ��ֱ��ķ���
currentPosition3=source3; % ��ǰλ��
cuDir3_a=robotDirection3_a; % ��ǰˮƽ��ķ���
cuDir3_b=robotDirection3_b; % ��ǰ��ֱ��ķ���
robotHalfDiagonalDistance=((robotSize(1)/2)^2+(robotSize(2)/2)^2)^0.5; 
pathFound=0; % ��־λ
pathCost=0;
t=1;
pathLength=0; 
pathFound2=0; % ��־λ
pathCost2=0;
pathlength2=0;
pathFound3=0; % ��־λ
pathCost3=0;
pathlength3=0;
h1=drawSphere(1,0.5,currentPosition(1),currentPosition(2),currentPosition(3));
%Ŀ�������Ҫ����
if ~feasiblePoint3d(goal), error('goal lies on an obstacle or outside map'); end
h1= drawSphere(2,0.5,currentPosition2(1),currentPosition2(2),currentPosition2(3));
if ~feasiblePoint3d(goal2), error('goal2 lies on an obstacle or outside map'); end
h1=drawSphere(3,0.5,currentPosition3(1),currentPosition3(2),currentPosition3(3));
if ~feasiblePoint3d(goal3), error('goal3 lies on an obstacle or outside map'); end
M(t)=getframe;
t=t+1;
num=0;
%��ʼ���к����滮
while ~pathFound||~pathFound2||~pathFound3
    %���˻�1��δ����Ŀ���
    if ~pathFound
    %��ȡ��ͬ����������ϰ��ľ���
    i=robotSize(1)/2+1;
    while true
        x=currentPosition+i*[cos(cuDir_b)*[cos(cuDir_a) sin(cuDir_a)] sin(cuDir_b)];
        if ~feasiblePoint3d(x); break; end
        if sqrt(sum(x-(currentPosition2)).^2)<1&&~pathFound2,break;end
        if sqrt(sum(x-(currentPosition3)).^2)<1&&~pathFound3,break;end
        i=i+1;
    end
    distanceFront=i-robotSize(1)/2; % robotSize(1)/2 distance included in i was inside the robot body 
    
    %��ȡ��ͬ����������ϰ��ľ���
    i=robotSize(2)/2+1;
    while true
        x=currentPosition+i*[cos(cuDir_b)*[cos(cuDir_a-pi/2) sin(cuDir_a-pi/2)] sin(cuDir_b)];
        if ~feasiblePoint3d(x); break; end
        if sqrt(sum(x-currentPosition2).^2)<1&&~pathFound2,break;end
        if sqrt(sum(x-currentPosition3).^2)<1&&~pathFound3,break;end
        i=i+1;
    end
    distanceLeft=i-robotSize(2)/2;  
    
    %��ȡ��ͬ����������ϰ��ľ���
    i=robotSize(2)/2+1;
    while true
        x=currentPosition+i*[cos(cuDir_b)*[cos(cuDir_a+pi/2) sin(cuDir_a+pi/2)] sin(cuDir_b)];
        if ~feasiblePoint3d(x); break; end
         if sqrt(sum(x-currentPosition2).^2)<1&&~pathFound2,break;end
         if sqrt(sum(x-currentPosition3).^2)<1&&~pathFound3,break;end
        i=i+1;
    end
    distanceRight=i-robotSize(2)/2;  
    
    %��ȡ��ͬ����������ϰ��ľ���
    i=robotHalfDiagonalDistance+1;
    while true
        x=currentPosition+i*[cos(cuDir_b+pi/2)*[cos(cuDir_a) sin(cuDir_a)] sin(cuDir_b+pi/2)];
        if ~feasiblePoint3d(x), break; end
         if sqrt(sum(x-currentPosition2).^2)<1&&~pathFound2,break;end
         if sqrt(sum(x-currentPosition3).^2)<1&&~pathFound3,break;end
        i=i+1;
    end
    distanceUp=i-robotHalfDiagonalDistance;
    
    %��ȡ��ͬ����������ϰ��ľ���
    i=robotHalfDiagonalDistance+1;
    while true
        x=currentPosition+i*[cos(cuDir_b-pi/2)*[cos(cuDir_a) sin(cuDir_a)] sin(cuDir_b-pi/2)];
        if ~feasiblePoint3d(x), break; end
        if sqrt(sum(x-currentPosition2).^2)<1&&~pathFound2,break;end
        if sqrt(sum(x-currentPosition3).^2)<1&&~pathFound3,break;end
        i=i+1;
    end
    distanceDown=i-robotHalfDiagonalDistance;
    %��ȡ��ͬ����������ϰ��ľ���
    i=robotHalfDiagonalDistance+1;
    while true
        x=currentPosition+i*[cos(cuDir_b)*[cos(cuDir_a+pi/4) sin(cuDir_a+pi/4)] sin(cuDir_b)];
        if ~feasiblePoint3d(x), break; end
        if sqrt(sum(x-currentPosition2).^2)<1&&~pathFound2,break;end
        if sqrt(sum(x-currentPosition3).^2)<1&&~pathFound3,break;end
        i=i+1;
    end
    distanceRF=i-robotHalfDiagonalDistance;
    %��ȡ��ͬ����������ϰ��ľ���
    i=robotHalfDiagonalDistance+1;
    while true
        x=currentPosition+i*[cos(cuDir_b)*[cos(cuDir_a-pi/4) sin(cuDir_a-pi/4)] sin(cuDir_b)];
        if ~feasiblePoint3d(x), break; end
        if sqrt(sum(x-currentPosition2).^2)<1&&~pathFound2,break;end
        if sqrt(sum(x-currentPosition3).^2)<1&&~pathFound3,break;end
        i=i+1;
    end
    distanceLF=i-robotHalfDiagonalDistance;
    %��ȡ��ͬ����������ϰ��ľ���
    i=robotHalfDiagonalDistance+1;
    while true
        x=currentPosition+i*[cos(cuDir_b+pi/4)*[cos(cuDir_a) sin(cuDir_a)] sin(cuDir_b+pi/4)];
        if ~feasiblePoint3d(x), break; end
        if sqrt(sum(x-currentPosition2).^2)<1&&~pathFound2,break;end
        if sqrt(sum(x-currentPosition3).^2)<1&&~pathFound3,break;end
        i=i+1;
    end
    distanceUF=i-robotHalfDiagonalDistance;
    %��ȡ��ͬ����������ϰ��ľ���
    i=robotHalfDiagonalDistance+1;
    while true
        x=currentPosition+i*[cos(cuDir_b-pi/4)*[cos(cuDir_a) sin(cuDir_a)] sin(cuDir_b-pi/4)];
        if ~feasiblePoint3d(x), break; end
        if sqrt(sum(x-currentPosition2).^2)<1&&~pathFound2,break;end
        if sqrt(sum(x-currentPosition3).^2)<1&&~pathFound3,break;end
        i=i+1;
    end
    distanceDF=i-robotHalfDiagonalDistance;
    %��ȡ��ͬ����������ϰ��ľ���
    i=robotHalfDiagonalDistance+1;
    while true
        x=currentPosition+i*[cos(cuDir_b+pi/4)*[cos(cuDir_a+pi/4) sin(cuDir_a+pi/4)] sin(cuDir_b+pi/4)];
        if ~feasiblePoint3d(x), break; end
        if sqrt(sum(x-currentPosition2).^2)<1&&~pathFound2,break;end
        if sqrt(sum(x-currentPosition3).^2)<1&&~pathFound3,break;end
        i=i+1;
    end
    distanceURF=i-robotHalfDiagonalDistance;
    %��ȡ��ͬ����������ϰ��ľ���
    i=robotHalfDiagonalDistance+1;
    while true
        x=currentPosition+i*[cos(cuDir_b+pi/4)*[cos(cuDir_a-pi/4) sin(cuDir_a-pi/4)] sin(cuDir_b+pi/4)];
        if ~feasiblePoint3d(x), break; end
        if sqrt(sum(x-currentPosition2).^2)<1&&~pathFound2,break;end
        if sqrt(sum(x-currentPosition3).^2)<1&&~pathFound3,break;end
        i=i+1;
    end
    distanceULF=i-robotHalfDiagonalDistance;
    %��ȡ��ͬ����������ϰ��ľ���
    i=robotHalfDiagonalDistance+1;
    while true
        x=currentPosition+i*[cos(cuDir_b-pi/4)*[cos(cuDir_a+pi/4) sin(cuDir_a+pi/4)] sin(cuDir_b-pi/4)];
        if ~feasiblePoint3d(x), break; end
        if sqrt(sum(x-currentPosition2).^2)<1&&~pathFound2,break;end
        if sqrt(sum(x-currentPosition3).^2)<1&&~pathFound3,break;end
        i=i+1;
    end
    distanceDRF=i-robotHalfDiagonalDistance;
    %��ȡ��ͬ����������ϰ��ľ���
    i=robotHalfDiagonalDistance+1;
    while true
        x=currentPosition+i*[cos(cuDir_b-pi/4)*[cos(cuDir_a-pi/4) sin(cuDir_a-pi/4)] sin(cuDir_b-pi/4)];
        if ~feasiblePoint3d(x), break; end
        if sqrt(sum(x-currentPosition2).^2)<1&&~pathFound2,break;end
        if sqrt(sum(x-currentPosition3).^2)<1&&~pathFound3,break;end
        i=i+1;
    end
    distanceDLF=i-robotHalfDiagonalDistance;
    
    % ����Ŀ�����Լ��ķ���
     angleGoal_a=atan2(goal(2)-currentPosition(2),goal(1)-currentPosition(1));
     angleGoal_b=atan2(goal(3)-currentPosition(3),sqrt((goal(1)-currentPosition(1))^2+(goal(2)-currentPosition(2))^2));
    
     % ����Ŀ�����Լ��ľ���
     distanceGoal=( sqrt(sum((currentPosition-goal).^2)) );
     %�ѵ���Ŀ��㣬�Ƴ������滮
     if distanceGoal<distanceThreshold, pathFound=true; end
     %���ݲ�ͬ����ľ�����������
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
     %����������
     attractivePotential=max([(1.0/distanceGoal)^k*attractivePotentialScaling minAttractivePotential ])*[cos(angleGoal_b)*[cos(angleGoal_a) sin(angleGoal_a)] sin(angleGoal_b)];
     %�����ۺ�����
      totalPotential=attractivePotential-repulsivePotentialScaling*repulsivePotential;
     
     % ����ˮƽ������Ҫ�ı�ĽǶȣ��������޷�
     preferredSteer_a=atan2(robotSpeed*cos(cuDir_b)*sin(cuDir_a)+totalPotential(2),robotSpeed*cos(cuDir_b)*cos(cuDir_a)+totalPotential(1))-cuDir_a;
      while preferredSteer_a>pi, preferredSteer_a=preferredSteer_a-2*pi; end % check to get the angle between -pi and pi
      while preferredSteer_a<-pi, preferredSteer_a=preferredSteer_a+2*pi; end % check to get the angle between -pi and pi
     preferredSteer_a=min([maxTurn preferredSteer_a]);
     preferredSteer_a=max([-maxTurn preferredSteer_a]);
     %�ı�ˮƽ�����
     cuDir_a=cuDir_a+preferredSteer_a;
     % ���㴹ֱ������Ҫ�ı�ĽǶȣ��������޷�
     preferredSteer_b=atan2(robotSpeed*sin(cuDir_b)+totalPotential(3),sqrt((robotSpeed*cos(cuDir_b)*cos(cuDir_a)+totalPotential(1))^2+(robotSpeed*cos(cuDir_b)*sin(cuDir_a)+totalPotential(2))^2))-cuDir_b;
      while preferredSteer_b>pi, preferredSteer_b=preferredSteer_b-2*pi; end % check to get the angle between -pi and pi
      while preferredSteer_b<-pi, preferredSteer_b=preferredSteer_b+2*pi; end % check to get the angle between -pi and pi
     preferredSteer_b=min([maxTurn preferredSteer_b]);
     preferredSteer_b=max([-maxTurn preferredSteer_b]);
     %�ı䴹ֱ�����
     cuDir_b=cuDir_b+preferredSteer_b;
     %�����ٶ�
     preferredSpeed=sqrt(sum((robotSpeed*[cos(cuDir_b)*[cos(cuDir_a) sin(cuDir_a)] sin(cuDir_b)] + totalPotential).^2));
     preferredSpeed=min([robotSpeed+maxAcceleration preferredSpeed]);
     robotSpeed=max([robotSpeed-maxAcceleration preferredSpeed]);
     robotSpeed=min([robotSpeed maxRobotSpeed]);
     robotSpeed=max([robotSpeed 0]);
     
     
     % �����ٶȺͽǶȼ����µĺ�����
     newPosition=currentPosition+robotSpeed*[cos(cuDir_b)*[cos(cuDir_a) sin(cuDir_a)] sin(cuDir_b)];
     pathCost=pathCost+distanceCost(newPosition,currentPosition);
     currentPosition=newPosition;
     if ~feasiblePoint3d(currentPosition), error('collission recorded'); end
     
     % ��ͼ
     h1=drawSphere(1,0.5,currentPosition(1),currentPosition(2),currentPosition(3));
     M(t)=getframe;t=t+1;
    end
 %���˻�2�������˻�1
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
%���˻�3�������˻�1
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
