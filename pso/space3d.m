function cost=space3d(a,b)
%关于无人机之间距离的代价函数，无人机之间距离越小，代价越高
n1=length(a);
n2=length(b);
cost=0;
% global distance;
mindist=10;
for i=1:n1
    for j=1:n2
        dist=sqrt((a(1,i)-b(1,j))^2+(a(2,i)-b(2,j))^2+(a(3,i)-b(3,j))^2);
        if dist<mindist
            mindist=dist;
        end
    end
end
if mindist<2
    mindist=mindist/2;
    cost=50/mindist;
end
end