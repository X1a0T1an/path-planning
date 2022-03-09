clear
clc
load('Z111.mat');
%�����׼����
global  Z3;
h=[26, 30,  25,  28 , 27, 25, 23];
x0=[20, 45,  45, 15, 57,  50, 10];
y0=[20, 25,  50, 50,  41, 8,  25];
xi=[5.5, 8,  5,  3.5, 4, 4,  4.5];
yi=[5,   7,  6,   4, 5,  3,  4.5];
Z2=CeatHill(7,h,x0,y0,xi,yi,65); 
Z3=max(Z1,Z2);
%�����������
figure(2);
surf(Z3); %������ά���� 
axis([1 65 1 65 1 30])%����xyz�᷶Χ
hold on;
%����Ȩ��ϵ��
Alpha=1;
Beta=5;
p1=0.3;
Q=1;
global r;%���۷���Ȩ��ϵ��
r=0.4;
ANTN=20;%������
Nm=15;%��������
global Np Rk K;  %��вԴ��
Rk=3;%��в�뾶
K=0.2;%��вϵ��
global Lmax Lmax2;
Lmax=1000;
Lmax2=1000;
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
%%  �����򲹺�����
Nk=150;%�򲹺���������
vertex=[source;goal;source2;goal2;source3;goal3];%���Ƚ���ʼ���Ŀ������򲹺���������
while length(vertex)<Nk+6
    %������ɺ���������
    x = 64*rand(1)+1; 
    y=64*rand(1)+1;
    z=30*rand(1);
    x=[x,y,z];
    %�жϺ������Ƿ����
    if feasiblePoint3d(x)
        vertex=[vertex;x];
    end
end
for j=1:Nk+6
    h1 = drawSphere(1,0.5,vertex(j,1),vertex(j,2),vertex(j,3));
end
%%  ���Ӻ򲹺������γɺ򲹺�����
Wi=zeros(Nk+6,Nk+6);%��Ϣ�ؾ���
Wi2=zeros(Nk+6,Nk+6);%��Ϣ�ؾ���
Wi3=zeros(Nk+6,Nk+6);%��Ϣ�ؾ���
edges = cell(Nk+6,1);  % ��¼������֮����������
for i=1:Nk+6
    for j=i+1:Nk+6
        %�жϺ򲹺������Ƿ����
        if checkPath3d(vertex(i,:),vertex(j,:))
            edges{i}=[edges{i};j];
            edges{j}=[edges{j};i];
            Wi(i,j)=1;
            Wi(j,i)=1;
            Wi2(i,j)=1;
            Wi2(j,i)=1;
            Wi3(i,j)=1;
            Wi3(j,i)=1;
            line([vertex(i,1);vertex(j,1)],[vertex(i,2);vertex(j,2)],[vertex(i,3);vertex(j,3)]); 
        end
    end
end
waitforbuttonpress; 
%��Ⱥ�㷨
Wf=zeros(Nk+6,Nk+6);%���̾���
Wm=zeros(Nk+6,Nk+6);%��в����
for i=1:length(edges)
    for j=1:length(edges{i})
        Wf(i,edges{i}(j))=Wfuel(vertex(i,:),vertex(edges{i}(j),:));
        Wm(i,edges{i}(j))=Wmenace3d(vertex(i,:),vertex(edges{i}(j),:));
    end
end
bestroad=[];
bestpath=[];
bestroad2=[];
bestpath2=[];
bestroad3=[];
bestpath3=[];
Wmin=9999999999999;%��ʼ��С����ֵ
Wmin2=9999999999999;%��ʼ��С����ֵ
Wmin3=9999999999999;%��ʼ��С����ֵ
for i=1:Nm
    for j=1:Nk+6
        for k=1:Nk+6
            Wi(j,k)=(1-p1)*Wi(j,k);%��Ϣ�ػӷ�
            Wi2(j,k)=(1-p1)*Wi2(j,k);%��Ϣ�ػӷ�
        end
    end
    for ant=1:ANTN
        n=1;%��ǰ���ں����㣬��ʼ��Ϊ���
        closed=[];%���ɱ��ѵ��ù��ĺ�����
        road=[n];%�����������·������
        W=0;%��ʼ������Ϣ
        dist2=0;
        while n~=2%���˻�1��û�е����յ㣬��Ѱ����һ����Ѻ�����
            closed=[closed;n];%���½��ɱ�
            open=edges{n};%�͵�ǰ�򲹺�������ͨ�����к�����
            P=zeros(1,length(open));
            for k=1:length(open)%����������������Ϣ
                if isempty(closed) || isempty(find(closed(:,1)==open(k), 1))
                    if n==1
                    eta=1/(W+r*Wf(n,open(k))+(1-r)*Wm(n,open(k))+gcost3d(vertex(open(k),:),goal)); 
                    else
                    eta=1/(W+r*Wf(n,open(k))+(1-r)*Wm(n,open(k))+gcost3d(vertex(open(k),:),goal)+Wtooclose(vertex(n,:),vertex(open(k),:),bestpath2));
                    end
                last=length(road)-1;
                if last>0
                x0=vertex(road(last),1);
                y0=vertex(road(last),2);
                z0=vertex(road(last),3);
                x1=vertex(n,1);
                y1=vertex(n,2);
                z1=vertex(n,3);
                x2=vertex(open(k),1);
                y2=vertex(open(k),2);
                z2=vertex(open(k),3);
                a0=sqrt((y1-y0)^2+(x1-x0)^2+(z1-z0)^2);
                a1=sqrt((y2-y1)^2+(x2-x1)^2+(z2-z1)^2);
                a2=sqrt((y2-y0)^2+(x2-x0)^2+(z2-z0)^2);
                degree=180-acos((a0^2+a1^2-a2^2)/(2*a0*a1))*180/pi; %���㺽����֮��ļнǣ������Ƕ���������
                if degree>60
                    eta=0;
                end
                end
                    P(k)=Wi(n,open(k))^Alpha*(eta^Beta);%���ո�����������Ϣ����Ϣ��Ũ���Լ���Ӧ��ϵ������
                end
            end
            if sum(P)==0
                break;
            end
            P=P/sum(P);
            %������ԭ��ѡȡ��һ������
            Pcum=cumsum(P);     %cumsum��Ԫ���ۼӼ����
            Select=find(Pcum>=rand); %������ĸ��ʴ���ԭ���ľ�ѡ������·��
            W=W+r*Wf(n,open(Select(1)))+(1-r)*Wm(n,open(Select(1)))+Wtooclose(vertex(n,:),vertex(open(k),:),bestpath2);
            dist2=dist2+sqrt((vertex(n,1)-vertex(open(Select(1)),1))^2+(vertex(n,2)-vertex(open(Select(1)),2))^2+(vertex(n,3)-vertex(open(Select(1)),3))^2);
            if dist2>Lmax%���������������
                break;
            end
            n=open(Select(1));
            road=[road,n];%ѡ����Ѻ��������·������           
        end
        if n==2%���˻�1�����յ�
            len=length(road);
            path=source;
            for j=2:len
                path=[path;vertex(road(j),:)];
            end
            %�����ܴ��۸�����Ϣ��Ũ��
            Wk=totalcost3d(path,bestpath2);
            for j=1:len-1
                Wi(road(j),road(j+1))=Wi(road(j),road(j+1))+p1/(Wk);
            end
            %��¼���µ�ǰ���·��
            if Wk<Wmin
                Wmin=Wk;
                bestroad=road;
                bestpath=path;
            end
            
        end
        %���˻�2�������˻�1����
        dist2=0;
        n2=3;
        closed2=[];
        road2=[n2];
        W2=0;%��ʼ������Ϣ
        dist22=0; 
        while n2~=4
            closed2=[closed2;n2];
            open2=edges{n2};
            P2=zeros(1,length(open2));
            for k=1:length(open2)
                if isempty(closed2) || isempty(find(closed2(:,1)==open2(k), 1))
                    if n2==3
                     eta=1/(W2+r*Wf(n2,open2(k))+(1-r)*Wm(n2,open2(k))+gcost3d(vertex(open2(k),:),goal2)); 
                    else
                    eta=1/(W2+r*Wf(n2,open2(k))+(1-r)*Wm(n2,open2(k))+gcost3d(vertex(open2(k),:),goal2)+Wtooclose(vertex(n2,:),vertex(open2(k),:),bestpath));
                    end
                    last=length(road2)-1;
                if last>0
                x0=vertex(road2(last),1);
                y0=vertex(road2(last),2);
                z0=vertex(road2(last),3);
                x1=vertex(n2,1);
                y1=vertex(n2,2);
                z1=vertex(n2,3);
                x2=vertex(open2(k),1);
                y2=vertex(open2(k),2);
                z2=vertex(open2(k),3);
                a0=sqrt((y1-y0)^2+(x1-x0)^2+(z1-z0)^2);
                a1=sqrt((y2-y1)^2+(x2-x1)^2+(z2-z1)^2);
                a2=sqrt((y2-y0)^2+(x2-x0)^2+(z2-z0)^2);
                degree=180-acos((a0^2+a1^2-a2^2)/(2*a0*a1))*180/pi;
                if degree>60
                     eta=0;
%                     disp('no');
                end
                end
                    P2(k)=Wi2(n2,open2(k))^Alpha*(eta^Beta);
                end
            end
            if sum(P2)==0
                break;
            end
            P2=P2/sum(P2);
            %������ԭ��ѡȡ��һ������
            Pcum2=cumsum(P2);     %cumsum��Ԫ���ۼӼ����
            Select=find(Pcum2>=rand); %������ĸ��ʴ���ԭ���ľ�ѡ������·��
            W2=W2+r*Wf(n2,open2(Select(1)))+(1-r)*Wm(n2,open2(Select(1)))+Wtooclose(vertex(n2,:),vertex(open2(k),:),bestpath);
            dist22=dist22+sqrt((vertex(n2,1)-vertex(open2(Select(1)),1))^2+(vertex(n2,2)-vertex(open2(Select(1)),2))^2+(vertex(n2,3)-vertex(open2(Select(1)),3))^2);
            if dist22>Lmax2
                break;
            end
            n2=open2(Select(1));
            road2=[road2,n2];           
        end
        if n2==4
            len=length(road2);
            path=source2;
            for j=2:len
                path=[path;vertex(road2(j),:)];
            end
            Wk=totalcost3d2(path,bestpath);
            for j=1:len-1
                Wi2(road2(j),road2(j+1))=(1-p1)*Wi2(road2(j),road2(j+1))+p1/(Wk);
            end
            if Wk<Wmin2
                Wmin2=Wk;
                bestroad2=road2;
                bestpath2=path;
            end
        end          
    end
end
for i=1:Nm
    for j=1:Nk+6
        for k=1:Nk+6
            Wi3(j,k)=(1-p1)*Wi3(j,k);%��Ϣ�ػӷ�
        end
    end
    %���˻�3�������˻�1����
    for ant=1:ANTN
        n3=5;
        closed3=[];
        road3=[n3];
        W3=0;%��ʼ������Ϣ
        bestlen3=0;%����·�������ڵ���
        singlenm3=0;
        dist23=0;
        while n3~=6
            closed3=[closed3;n3];
            open3=edges{n3};
            P=zeros(1,length(open3));
            for k=1:length(open3)
                if isempty(closed3) || isempty(find(closed3(:,1)==open3(k), 1))
                    if n3==5
                    eta=1/(W+r*Wf(n3,open3(k))+(1-r)*Wm(n3,open3(k))+gcost3d(vertex(open3(k),:),goal3));
                    else
                    eta=1/(W+r*Wf(n3,open3(k))+(1-r)*Wm(n3,open3(k))+gcost3d(vertex(open3(k),:),goal3)+Wtooclose(vertex(n3,:),vertex(open3(k),:),bestpath)+Wtooclose(vertex(n3,:),vertex(open3(k),:),bestpath2));                
                    if eta==0
                        disp(22);
                        disp(W);
                        disp(r*Wf(n3,open3(k)));
                        disp((1-r)*Wm(n3,open3(k)));
                        disp(10*gcost3d(vertex(open3(k),:),goal3,source3));
                        disp(Wtooclose(vertex(n3,:),vertex(open3(k),:),bestpath));
                        disp(Wtooclose(vertex(n3,:),vertex(open3(k),:),bestpath2));
                    end
                    end
                    last=length(road3)-1;
                if last>0
                x0=vertex(road3(last),1);
                y0=vertex(road3(last),2);
                z0=vertex(road3(last),3);
                x1=vertex(n3,1);
                y1=vertex(n3,2);
                z1=vertex(n3,3);
                x2=vertex(open3(k),1);
                y2=vertex(open3(k),2);
                z2=vertex(open3(k),3);
                a0=sqrt((y1-y0)^2+(x1-x0)^2+(z1-z0)^2);
                a1=sqrt((y2-y1)^2+(x2-x1)^2+(z2-z1)^2);
                a2=sqrt((y2-y0)^2+(x2-x0)^2+(z2-z0)^2);
                degree=180-acos((a0^2+a1^2-a2^2)/(2*a0*a1))*180/pi;
                if degree>60
                      eta=0;
                end
                end
                    P(k)=Wi3(n3,open3(k))^Alpha*(eta^Beta);
                end
            end
            if sum(P)==0
                break;
            end
            P=P/sum(P);
            %������ԭ��ѡȡ��һ������
            Pcum=cumsum(P);     %cumsum��Ԫ���ۼӼ����
            Select=find(Pcum>=rand); %������ĸ��ʴ���ԭ���ľ�ѡ������·��
            W=W+r*Wf(n3,open3(Select(1)))+(1-r)*Wm(n3,open3(Select(1)))+Wtooclose(vertex(n3,:),vertex(open3(k),:),bestpath)+Wtooclose(vertex(n3,:),vertex(open3(k),:),bestpath2);
            dist23=dist23+sqrt((vertex(n3,1)-vertex(open3(Select(1)),1))^2+(vertex(n3,2)-vertex(open3(Select(1)),2))^2+(vertex(n3,3)-vertex(open3(Select(1)),3))^2);
            if dist23>Lmax*2
                disp(dist23);
                break;
            end
            n3=open3(Select(1));
            road3=[road3,n3]; 
        end
        if n3==6
            len=length(road3);
            path=source3;
            for j=2:len
                path=[path;vertex(road3(j),:)];
            end
            Wk=totalcost3d3(path,bestpath,bestpath2);
            for j=1:len-1
                Wi3(road3(j),road3(j+1))=Wi3(road3(j),road3(j+1))+p1/(Wk);
            end
            if Wk<Wmin3
                Wmin3=Wk;
                bestroad3=road3;
                bestpath3=path;
            end
        end
    end
end
hold off;
surf(Z3);
hold on;
for j=1:Np
    h1 = drawSphere(0,Rk,danareaX(j),danareaY(j),danareaZ(j));
end
%�����ú�����ƽ������
bestpath=smooth(bestpath);
bestpath2=smooth(bestpath2);
bestpath3=smooth(bestpath3);
line(bestpath(:,1),bestpath(:,2),bestpath(:,3),'color','r');
line(bestpath2(:,1),bestpath2(:,2),bestpath2(:,3),'color','g');
line(bestpath3(:,1),bestpath3(:,2),bestpath3(:,3),'color','b');
 waitforbuttonpress;
%%ʵʱ���,�ò���������Ⱥ�㷨��ͬ
figure(2)
danareaX=[danareaX,18*rand(1,5)+10];
danareaY=[danareaY,18*rand(1,5)+25];
danareaZ=[danareaZ,10*rand(1,5)+15];
Np=length(danareaX);
global xpos ypos zpos xpos2 ypos2 zpos2 xpos3 ypos3 zpos3 nt;
xpos=bestpath(:,1).';ypos=bestpath(:,2).';zpos=bestpath(:,3).';
xpos2=bestpath2(:,1).';ypos2=bestpath2(:,2).';zpos2=bestpath2(:,3).';
xpos3=bestpath3(:,1).';ypos3=bestpath3(:,2).';zpos3=bestpath3(:,3).';
D1=length(xpos)-2;
D2=length(xpos2)-2;
D3=length(xpos3)-2;
j=1;j2=1;j3=1;
global currentPosition1 currentPosition2 currentPosition3;
currentPosition1=[xs,ys,zs];
currentPosition2=[xs2,ys2,zs2];
currentPosition3=[xs3,ys3,zs3];
newgoal1=[xt,yt,zt+5];
newgoal2=[xt2,yt2,zt2+5];
newgoal3=[xt3,yt3,zt3+5];
t=1;nn=1;
flag1=0;flag2=0;flag3=0;
while ~(nn>length(xpos))||~(nn>length(xpos2))||~(nn>length(xpos3))
    nt=nn;
    if ~(isempty(newgoal1))&&sqrt((xpos(nn)-newgoal1(1))^2+(ypos(nn)-newgoal1(2))^2+(zpos(nn)-newgoal1(3))^2)<10
        h1 = drawSphere(1,0.5,newgoal1(1), newgoal1(2), newgoal1(3));
        newsource=[xpos(nn),ypos(nn),zpos(nn)];
        newgoal=newgoal1;
        cuDir_a=atan2(ypos(nn)-ypos(nn-1),xpos(nn)-xpos(nn-1)); 
        cuDir_b=atan2(zpos(nn)-zpos(nn-1),sqrt((xpos(nn)-xpos(nn-1))^2+(ypos(nn)-ypos(nn-1))^2));
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
    for i=0:4
        if~feasiblePoint3d([xpos(j+i),ypos(j+i),zpos(j+i)])
            for m=21:25
                if sqrt((xpos(j+i)-danareaX(m))^2+(ypos(j+i)-danareaY(m))^2+(zpos(j+i)-danareaZ(m))^2)<Rk
                    h1 = drawSphere(4,Rk,danareaX(m), danareaY(m), danareaZ(m));%���ϰ���
                end
            end
            newsource=[xpos(j),ypos(j),zpos(j)];
            cuDir_a=atan2(ypos(j+i)-ypos(j+i-1),xpos(j+i)-xpos(j+i-1)); 
            cuDir_b=atan2(zpos(j+i)-zpos(j+i-1),sqrt((xpos(j+i)-xpos(j+i-1))^2+(ypos(j+i)-ypos(j+i-1))^2));
            k=j+i+4;
            if k>D1+2
                k=D1+2;
            else
                while ~feasiblePoint3d([xpos(k),ypos(k),zpos(k)])||~feasiblePoint3d([xpos(k-3),ypos(k-3),zpos(k-3)])
                    k=k+1;
                end
            end
                    newgoal=[xpos(k),ypos(k),zpos(k)];
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
    if ~(isempty(newgoal2))&&sqrt((xpos2(nn)-newgoal2(1))^2+(ypos2(nn)-newgoal2(2))^2+(zpos2(nn)-newgoal2(3))^2)<10
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
                    h1 = drawSphere(4,Rk,danareaX(m), danareaY(m), danareaZ(m));%���ϰ���
                end
            end
            newsource=[xpos2(j2),ypos2(j2),zpos2(j2)];
            cuDir_a=atan2(ypos2(j2+i)-ypos2(j2+i-1),xpos2(j2+i)-xpos2(j2+i-1)); 
            cuDir_b=atan2(zpos2(j2+i)-zpos2(j2+i-1),sqrt((xpos2(j2+i)-xpos2(j2+i-1))^2+(ypos2(j2+i)-ypos2(j2+i-1))^2));
            k=j2+i+4;
            if k>D2+2
                k=D2+2;
            else
                while ~feasiblePoint3d([xpos2(k),ypos2(k),zpos2(k)])||~feasiblePoint3d([xpos2(k-3),ypos2(k-3),zpos2(k-3)])
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
    if ~(nn>length(xpos3)-5)&&flag3==0
        j3=nn;
    for i=0:4
        if~feasiblePoint3d([xpos3(j3+i),ypos3(j3+i),zpos3(j3+i)])
            for m=21:25
                if sqrt((xpos3(j+i)-danareaX(m))^2+(ypos3(j+i)-danareaY(m))^2+(zpos3(j+i)-danareaZ(m))^2)<Rk
                    h1 = drawSphere(4,Rk,danareaX(m), danareaY(m), danareaZ(m));%���ϰ���
                end
            end
            newsource=[xpos3(j3),ypos3(j3),zpos3(j3)];
            cuDir_a=atan2(ypos3(j3+i)-ypos3(j3+i-1),xpos3(j3+i)-xpos3(j3+i-1)); 
            cuDir_b=atan2(zpos3(j3+i)-zpos3(j3+i-1),sqrt((xpos3(j3+i)-xpos3(j3+i-1))^2+(ypos3(j3+i)-ypos3(j3+i-1))^2));
            k=j3+i+4;
            if k>D3+2
                k=D3+2;
            else
                while ~feasiblePoint3d([xpos3(k),ypos3(k),zpos3(k)])||~feasiblePoint3d([xpos3(k-3),ypos3(k-3),zpos3(k-3)])
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
