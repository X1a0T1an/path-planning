function menace=Wmenace3d(a,b)
%计算两点之间生成航迹段受威胁大小，作为后续启发信息的一部分
    global danareaX;
    global danareaY;
    global danareaZ;
    global Np Rk;
    menace=0;
    for i=1:Np
        for j=1:10
            dist=sqrt(sum((a+j*(b-a)/10-[danareaX(i),danareaY(i),danareaZ(i)]).^2));         
            if dist<Rk
                dist=dist/Rk;
                ddan=min(10000,100/(dist^4));
                 menace=menace+ddan/10;          
            end
        end
    end
end