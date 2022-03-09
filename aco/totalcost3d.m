function tcost=totalcost3d(a,path)
    global source2 goal2;
    if isempty(path)
        d=(goal2-source2)/10;
        for i=1:10
            path=[path;source2+d*i];
        end
    end
    dist=0;
    menace=0;
    tooclose=0;
    for j=1:length(a)-1
        dist=dist+sqrt((a(j,1)-a(j+1,1))^2+(a(j,2)-a(j+1,2))^2+(a(j,3)-a(j+1,3))^2);
        menace=menace+Wmenace3d(a(j,:),a(j+1,:));
        tooclose=tooclose+Wtooclose(a(j,:),a(j+1,:),path);
    end
    tcost=dist+menace+tooclose;
%     disp(dist);
%      disp(menace);
%      disp(tooclose);
end