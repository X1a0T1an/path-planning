function newpath=smooth(bestpath)
%通过插入中间航迹点对所得航迹做平滑处理
for j=1:4
n=size(bestpath,1);
i=1;
while ~(i>n-2)
    d1=(bestpath(i+1,:)-bestpath(i,:))/5;
    d2=(bestpath(i+2,:)-bestpath(i+1,:))/5;
    newpos1=bestpath(i+1,:)-d1;
    newpos2=bestpath(i+1,:)+d2;
    k=2;
    while ~feasiblePoint3d(newpos1)&&k<5
        newpos1=bestpath(i+1,:)-d1/k;
        k=k+1;
    end
    k=2;
    while ~feasiblePoint3d(newpos2)&&k<5
        newpos2=bestpath(i+1,:)+d2/k;
        k=k+1;
    end
    bestpath=[bestpath(1:i,:);newpos1;newpos2;bestpath(i+2:n,:)];
    n=n+1;
    i=i+2;
end
end
newpath=bestpath;
d=(newpath(2,:)-newpath(1,:))/4;
newpos1=bestpath(1,:)+d;
newpos2=bestpath(1,:)+d*2;
newpos3=bestpath(1,:)+d*3;
newpath=[newpath(1,:);newpos1;newpos2;newpos3;newpath(2:n,:)];
d=(newpath(end,:)-newpath(end-1,:))/4;
newpos1=bestpath(end-1,:)+d;
newpos2=bestpath(end-1,:)+d*2;
newpos3=bestpath(end-1,:)+d*3;
newpath=[newpath(1:end-1,:);newpos1;newpos2;newpos3;newpath(end,:)];
end