function dan=danger3d(x,y,z)
%关于威胁源的代价函数，离威胁源越近，代价越大
    global danareaX;
    global danareaY;
    global danareaZ;
    global Np Rk;
    dan=0;
    for i=1:Np
        dist=((x-danareaX(i))^2+(y-danareaY(i))^2+(z-danareaZ(i))^2)^0.5;
        if dist>Rk
            ddan=0;
        end
        if dist<Rk
            dist=dist/Rk;
            ddan=min(1000,100/(dist^4));
        end
        dan=dan+ddan;
    end
end