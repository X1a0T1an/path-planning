function cost=Wtooclose(a,b,path)
%�����˻�֮�������صĴ��ۺ���������ԽС������Խ��
cost=0;
n=size(path,1);
if n==0
    return;
end
dd=(b-a)/10;
mindist=10;
for i=1:n
    for j=1:10
        dist=sqrt(sum((a+dd*j-path(i,:)).^2));
        if dist<mindist
            mindist=dist;
        end
    end
end
if mindist<2
    mindist=max(0.1,mindist/2);
    cost=50/mindist;
end
end
        