function path=GETAPM(source,goal,cuDir_a,cuDir_b,flag)
global xpos ypos zpos xpos2 ypos2 zpos2 xpos3 ypos3 zpos3 nt;
dnear=1;
t=1;
    path=[];
    robotSize=[1 1 1]; %length and breadth
    robotSpeed=1; % arbitrary units 
    maxRobotSpeed=1; % arbitrary units 
distanceThreshold=3; % a threshold distace. points within this threshold can be taken as same. 
maxAcceleration=1; % maximum speed change per unit time
maxTurn=30*pi/180; % potential outputs to turn are restriect to -60 and 60 degrees.
k=3; % degree of calculating potential
attractivePotentialScaling=3000; % scaling factor for attractive potential
repulsivePotentialScaling=50; % scaling factor for repulsive potential
minAttractivePotential=1.5; % minimum attractive potential at any point
currentPosition=source; % position of the centre of the robot
robotHalfDiagonalDistance=((robotSize(1)/2)^2+(robotSize(2)/2)^2)^0.5; % used for distance calculations 
pathFound=0; % has goal been reached
pathCost=0;
pathLength=0;
while ~pathFound
    if nt<length(xpos)
        currentPosition1=[xpos(nt),ypos(nt),zpos(nt)];
    else
       currentPosition1=[200,200,200];
    end
    if nt<length(xpos2)
        currentPosition2=[xpos2(nt),ypos2(nt),zpos2(nt)];
    else
        currentPosition2=[200,200,200];
    end
    if nt<length(xpos3)
        currentPosition3=[xpos3(nt),ypos3(nt),zpos3(nt)];
    else
        currentPosition3=[200,200,200];
    end
    nt=nt+1;
    % calculate distance from obstacle at front
    i=robotSize(1)/2+1;
    while i<20
        x=currentPosition+i*[cos(cuDir_b)*[cos(cuDir_a) sin(cuDir_a)] sin(cuDir_b)];
        if ~feasiblePoint3d(x); break; end
        if dnear==1
            if ~(flag==1)&&sqrt(sum(x-(currentPosition1)).^2)<1,break;end
            if ~(flag==2)&&sqrt(sum(x-(currentPosition2)).^2)<1,break;end
            if ~(flag==3)&&sqrt(sum(x-(currentPosition3)).^2)<1,break;end
        end
        i=i+1;
    end
    distanceFront=i-robotSize(1)/2; % robotSize(1)/2 distance included in i was inside the robot body 
    
    % calculate distance from obstacle at left
    i=robotSize(2)/2+1;
    while i<20
        x=currentPosition+i*[cos(cuDir_b)*[cos(cuDir_a-pi/2) sin(cuDir_a-pi/2)] sin(cuDir_b)];
        if ~feasiblePoint3d(x); break; end
        if dnear==1
            if sqrt(sum(x-(currentPosition1)).^2)<1&&~(flag==1),break;end
            if sqrt(sum(x-(currentPosition2)).^2)<1&&~(flag==2),break;end
            if sqrt(sum(x-(currentPosition3)).^2)<1&&~(flag==3),break;end
        end
        i=i+1;
    end
    distanceLeft=i-robotSize(2)/2;  
    
    % calculate distance from obstacle at right
    i=robotSize(2)/2+1;
    while i<20
        x=currentPosition+i*[cos(cuDir_b)*[cos(cuDir_a+pi/2) sin(cuDir_a+pi/2)] sin(cuDir_b)];
        if ~feasiblePoint3d(x); break; end
        if dnear==1
            if sqrt(sum(x-(currentPosition1)).^2)<1&&~(flag==1),break;end
            if sqrt(sum(x-(currentPosition2)).^2)<1&&~(flag==2),break;end
            if sqrt(sum(x-(currentPosition3)).^2)<1&&~(flag==3),break;end
        end
        i=i+1;
    end
    distanceRight=i-robotSize(2)/2;  
    
    % calculate distance from obstacle at front-left diagonal
    i=robotHalfDiagonalDistance+1;
    while i<20
        x=currentPosition+i*[cos(cuDir_b+pi/2)*[cos(cuDir_a) sin(cuDir_a)] sin(cuDir_b+pi/2)];
        if ~feasiblePoint3d(x), break; end
        if dnear==1
            if sqrt(sum(x-(currentPosition1)).^2)<1&&~(flag==1),break;end
            if sqrt(sum(x-(currentPosition2)).^2)<1&&~(flag==2),break;end
            if sqrt(sum(x-(currentPosition3)).^2)<1&&~(flag==3),break;end
        end
        i=i+1;
    end
    distanceUp=i-robotHalfDiagonalDistance;
    
    % calculate distance from obstacle at front-right diagonal
    i=robotHalfDiagonalDistance+1;
    while i<20
        x=currentPosition+i*[cos(cuDir_b-pi/2)*[cos(cuDir_a) sin(cuDir_a)] sin(cuDir_b-pi/2)];
        if ~feasiblePoint3d(x), break; end
        if dnear==1
            if sqrt(sum(x-(currentPosition1)).^2)<1&&~(flag==1),break;end
            if sqrt(sum(x-(currentPosition2)).^2)<1&&~(flag==2),break;end
            if sqrt(sum(x-(currentPosition3)).^2)<1&&~(flag==3),break;end
        end
        i=i+1;
    end
    distanceDown=i-robotHalfDiagonalDistance;
    
    i=robotHalfDiagonalDistance+1;
    while i<20
        x=currentPosition+i*[cos(cuDir_b)*[cos(cuDir_a+pi/4) sin(cuDir_a+pi/4)] sin(cuDir_b)];
        if ~feasiblePoint3d(x), break; end
        if dnear==1
            if sqrt(sum(x-(currentPosition1)).^2)<1&&~(flag==1),break;end
            if sqrt(sum(x-(currentPosition2)).^2)<1&&~(flag==2),break;end
            if sqrt(sum(x-(currentPosition3)).^2)<1&&~(flag==3),break;end
        end
        i=i+1;
    end
    distanceRF=i-robotHalfDiagonalDistance;
    i=robotHalfDiagonalDistance+1;
    while i<20
        x=currentPosition+i*[cos(cuDir_b)*[cos(cuDir_a-pi/4) sin(cuDir_a-pi/4)] sin(cuDir_b)];
        if ~feasiblePoint3d(x), break; end
        if dnear==1
            if sqrt(sum(x-(currentPosition1)).^2)<1&&~(flag==1),break;end
            if sqrt(sum(x-(currentPosition2)).^2)<1&&~(flag==2),break;end
            if sqrt(sum(x-(currentPosition3)).^2)<1&&~(flag==3),break;end
        end
        i=i+1;
    end
    distanceLF=i-robotHalfDiagonalDistance;
    i=robotHalfDiagonalDistance+1;
    while i<20
        x=currentPosition+i*[cos(cuDir_b+pi/4)*[cos(cuDir_a) sin(cuDir_a)] sin(cuDir_b+pi/4)];
        if ~feasiblePoint3d(x), break; end
        if dnear==1
            if sqrt(sum(x-(currentPosition1)).^2)<1&&~(flag==1),break;end
            if sqrt(sum(x-(currentPosition2)).^2)<1&&~(flag==2),break;end
            if sqrt(sum(x-(currentPosition3)).^2)<1&&~(flag==3),break;end
        end
        i=i+1;
    end
    distanceUF=i-robotHalfDiagonalDistance;
    i=robotHalfDiagonalDistance+1;
    while i<20
        x=currentPosition+i*[cos(cuDir_b-pi/4)*[cos(cuDir_a) sin(cuDir_a)] sin(cuDir_b-pi/4)];
        if ~feasiblePoint3d(x), break; end
        if dnear==1
            if sqrt(sum(x-(currentPosition1)).^2)<1&&~(flag==1),break;end
            if sqrt(sum(x-(currentPosition2)).^2)<1&&~(flag==2),break;end
            if sqrt(sum(x-(currentPosition3)).^2)<1&&~(flag==3),break;end
        end
        i=i+1;
    end
    distanceDF=i-robotHalfDiagonalDistance;
    i=robotHalfDiagonalDistance+1;
    while i<20
        x=currentPosition+i*[cos(cuDir_b+pi/4)*[cos(cuDir_a+pi/4) sin(cuDir_a+pi/4)] sin(cuDir_b+pi/4)];
        if ~feasiblePoint3d(x), break; end
        if dnear==1
            if sqrt(sum(x-(currentPosition1)).^2)<1&&~(flag==1),break;end
            if sqrt(sum(x-(currentPosition2)).^2)<1&&~(flag==2),break;end
            if sqrt(sum(x-(currentPosition3)).^2)<1&&~(flag==3),break;end
        end
        i=i+1;
    end
    distanceURF=i-robotHalfDiagonalDistance;
    i=robotHalfDiagonalDistance+1;
    while i<20
        x=currentPosition+i*[cos(cuDir_b+pi/4)*[cos(cuDir_a-pi/4) sin(cuDir_a-pi/4)] sin(cuDir_b+pi/4)];
        if ~feasiblePoint3d(x), break; end
        if dnear==1
            if sqrt(sum(x-(currentPosition1)).^2)<1&&~(flag==1),break;end
            if sqrt(sum(x-(currentPosition2)).^2)<1&&~(flag==2),break;end
            if sqrt(sum(x-(currentPosition3)).^2)<1&&~(flag==3),break;end
        end
        i=i+1;
    end
    distanceULF=i-robotHalfDiagonalDistance;
    i=robotHalfDiagonalDistance+1;
    while i<20
        x=currentPosition+i*[cos(cuDir_b-pi/4)*[cos(cuDir_a+pi/4) sin(cuDir_a+pi/4)] sin(cuDir_b-pi/4)];
        if ~feasiblePoint3d(x), break; end
        if dnear==1
            if sqrt(sum(x-(currentPosition1)).^2)<1&&~(flag==1),break;end
            if sqrt(sum(x-(currentPosition2)).^2)<1&&~(flag==2),break;end
            if sqrt(sum(x-(currentPosition3)).^2)<1&&~(flag==3),break;end
        end
        i=i+1;
    end
    distanceDRF=i-robotHalfDiagonalDistance;
    i=robotHalfDiagonalDistance+1;
    while i<20
        x=currentPosition+i*[cos(cuDir_b-pi/4)*[cos(cuDir_a-pi/4) sin(cuDir_a-pi/4)] sin(cuDir_b-pi/4)];
        if ~feasiblePoint3d(x), break; end
        if dnear==1
            if sqrt(sum(x-(currentPosition1)).^2)<1&&~(flag==1),break;end
            if sqrt(sum(x-(currentPosition2)).^2)<1&&~(flag==2),break;end
            if sqrt(sum(x-(currentPosition3)).^2)<1&&~(flag==3),break;end
        end
        i=i+1;
    end
    distanceDLF=i-robotHalfDiagonalDistance;
    
    % calculate angle from goal
     angleGoal_a=atan2(goal(2)-currentPosition(2),goal(1)-currentPosition(1));
     angleGoal_b=atan2(goal(3)-currentPosition(3),sqrt((goal(1)-currentPosition(1))^2+(goal(2)-currentPosition(2))^2));
    
     % calculate diatnce from goal
     distanceGoal=( sqrt(sum((currentPosition-goal).^2)) );
     if distanceGoal<distanceThreshold, pathFound=true; end
     repulsivePotential=[0,0,0];
     % compute potentials
     if distanceFront<10
        repulsivePotential=repulsivePotential+(1.0/distanceFront-0.1)^k*[cos(cuDir_b)*[cos(cuDir_a) sin(cuDir_a)] sin(cuDir_b)];
     end
     if distanceLeft<10
        repulsivePotential=repulsivePotential+(1.0/distanceLeft-0.1)^k*[cos(cuDir_b)*[cos(cuDir_a-pi/2) sin(cuDir_a-pi/2)] sin(cuDir_b)];
     end
     if distanceRight<10
        repulsivePotential=repulsivePotential+(1.0/distanceRight-0.1)^k*[cos(cuDir_b)*[cos(cuDir_a+pi/2) sin(cuDir_a+pi/2)] sin(cuDir_b)];
     end
     if distanceUp<10
        repulsivePotential=repulsivePotential+(1.0/distanceUp-0.1)^k*[cos(cuDir_b+pi/2)*[cos(cuDir_a) sin(cuDir_a)] sin(cuDir_b+pi/2)];
     end
     if distanceDown<10
        repulsivePotential=repulsivePotential+(1.0/distanceDown-0.1)^k*[cos(cuDir_b-pi/2)*[cos(cuDir_a) sin(cuDir_a)] sin(cuDir_b-pi/2)];
     end
     if distanceRF<10
        repulsivePotential=repulsivePotential+(1.0/distanceRF-0.1)^k*[cos(cuDir_b)*[cos(cuDir_a+pi/4) sin(cuDir_a+pi/4)] sin(cuDir_b)];
     end
     if distanceLF<10
        repulsivePotential=repulsivePotential+(1.0/distanceLF-0.1)^k*[cos(cuDir_b)*[cos(cuDir_a-pi/4) sin(cuDir_a-pi/4)] sin(cuDir_b)];
     end
     if distanceUF<10
        repulsivePotential=repulsivePotential+(1.0/distanceUF-0.1)^k*[cos(cuDir_b+pi/4)*[cos(cuDir_a) sin(cuDir_a)] sin(cuDir_b+pi/4)];
     end
     if distanceDF<10
        repulsivePotential=repulsivePotential+(1.0/distanceDF-0.1)^k*[cos(cuDir_b-pi/4)*[cos(cuDir_a) sin(cuDir_a)] sin(cuDir_b-pi/4)];
     end
     if distanceURF<10
        repulsivePotential=repulsivePotential+(1.0/distanceURF-0.1)^k*[cos(cuDir_b+pi/4)*[cos(cuDir_a+pi/4) sin(cuDir_a+pi/4)] sin(cuDir_b+pi/4)];
     end
     if distanceULF<10
        repulsivePotential=repulsivePotential+(1.0/distanceULF-0.1)^k*[cos(cuDir_b+pi/4)*[cos(cuDir_a-pi/4) sin(cuDir_a-pi/4)] sin(cuDir_b+pi/4)];
     end
     if distanceDRF<10
        repulsivePotential=repulsivePotential+(1.0/distanceDRF-0.1)^k*[cos(cuDir_b-pi/4)*[cos(cuDir_a+pi/4) sin(cuDir_a+pi/4)] sin(cuDir_b-pi/4)];
     end
     if distanceDLF<10
        repulsivePotential=repulsivePotential+(1.0/distanceDLF-0.1)^k*[cos(cuDir_b-pi/4)*[cos(cuDir_a-pi/4) sin(cuDir_a-pi/4)] sin(cuDir_b-pi/4)];
     end
     attractivePotential=max([(1.0/distanceGoal)^k*attractivePotentialScaling minAttractivePotential ])*[cos(angleGoal_b)*[cos(angleGoal_a) sin(angleGoal_a)] sin(angleGoal_b)];
      totalPotential=attractivePotential-repulsivePotentialScaling*repulsivePotential;
     
     % perform steer
     preferredSteer_a=atan2(robotSpeed*cos(cuDir_b)*sin(cuDir_a)+totalPotential(2),robotSpeed*cos(cuDir_b)*cos(cuDir_a)+totalPotential(1))-cuDir_a;
      while preferredSteer_a>pi, preferredSteer_a=preferredSteer_a-2*pi; end % check to get the angle between -pi and pi
      while preferredSteer_a<-pi, preferredSteer_a=preferredSteer_a+2*pi; end % check to get the angle between -pi and pi
     preferredSteer_a=min([maxTurn preferredSteer_a]);
     preferredSteer_a=max([-maxTurn preferredSteer_a]);
     cuDir_a=cuDir_a+preferredSteer_a;
     preferredSteer_b=atan2(robotSpeed*sin(cuDir_b)+totalPotential(3),sqrt((robotSpeed*cos(cuDir_b)*cos(cuDir_a)+totalPotential(1))^2+(robotSpeed*cos(cuDir_b)*sin(cuDir_a)+totalPotential(2))^2))-cuDir_b;
      while preferredSteer_b>pi, preferredSteer_b=preferredSteer_b-2*pi; end % check to get the angle between -pi and pi
      while preferredSteer_b<-pi, preferredSteer_b=preferredSteer_b+2*pi; end % check to get the angle between -pi and pi
     preferredSteer_b=min([maxTurn preferredSteer_b]);
     preferredSteer_b=max([-maxTurn preferredSteer_b]);
     cuDir_b=cuDir_b+preferredSteer_b;
     % setting the speed based on vehicle acceleration and speed limits. the vehicle cannot move backwards.
     preferredSpeed=sqrt(sum((robotSpeed*[cos(cuDir_b)*[cos(cuDir_a) sin(cuDir_a)] sin(cuDir_b)] + totalPotential).^2));
     preferredSpeed=min([robotSpeed+maxAcceleration preferredSpeed]);
     robotSpeed=max([robotSpeed-maxAcceleration preferredSpeed]);
     robotSpeed=min([robotSpeed maxRobotSpeed]);
     robotSpeed=max([robotSpeed 0]);
     
     
     if robotSpeed==0, error('robot had to stop to avoid collission'); end
     
     % calculating new position based on steer and speed
     newPosition=currentPosition+robotSpeed*[cos(cuDir_b)*[cos(cuDir_a) sin(cuDir_a)] sin(cuDir_b)];
     pathCost=pathCost+distanceCost(newPosition,currentPosition);
     currentPosition=newPosition;
     path=[path;currentPosition];
end
end
