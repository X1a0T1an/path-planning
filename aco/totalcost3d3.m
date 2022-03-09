function tcost=totalcost3d3(a,path,path2)
    global source goal source2 goal2;
    if isempty(path)
        d=(goal-source)/10;
        for i=1:10
            path=[path;source+d*i];
        end
    end
    if isempty(path2)
        d=(goal2-source2)/10;
        for i=1:10
            path2=[path2;source2+d*i];
        end
    end
    dist=0;
    menace=0;
    tooclose=0;
    for j=1:length(a)-1
        dist=dist+sqrt((a(j,1)-a(j+1,1))^2+(a(j,2)-a(j+1,2))^2+(a(j,3)-a(j+1,3))^2);
        menace=menace+Wmenace3d(a(j,:),a(j+1,:));
        tooclose=tooclose+Wtooclose(a(j,:),a(j+1,:),path)+Wtooclose(a(j,:),a(j+1,:),path2);
    end
    tcost=dist+menace+tooclose;
%      disp(dist);
%       disp(menace);
%      disp(tooclose);
end