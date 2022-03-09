function feasible=checkPath3d(n,newPos)
%�жϺ򲹺������Ƿ����������֮���ܷ���ͨ
    global DDIR ;%�涨�����ӵ�������
    DDIR=30;
    feasible = true;
    a=(newPos-n)/10;
    dist=sqrt(sum((n-newPos).^2));
    if dist>30||dist<3
        feasible=false;
        return;
    end
    x=sqrt((newPos(1)-n(1))^2+(newPos(2)-n(2))^2);
    y=abs(newPos(3)-n(3));
    if y>x
        feasible=false;
        return;
    end
    for i=1:10
        posCheck = n + a*i;
        if ~(feasiblePoint3d(posCheck)) 
              feasible = false;break;
        end
        if ~feasiblePoint3d(newPos), feasible = false; end
    end
end