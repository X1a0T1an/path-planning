function feasible=feasiblePoint3d(point)
%≈–∂œ∫Ú≤π∫Ωº£µ„ «∑Ò∫œ¿Ì
    global Z3;
    global danareaX;
    global danareaY;
    global danareaZ;
    global Np Rk;
    feasible=true;
    x=floor(point(1));
    y=floor(point(2));
    z=point(3);
    if x<1||x>63||y<1||y>63||z>50
        feasible=false;
        return;
    end
    dx=point(1)-x;
    dy=point(2)-y;
    z1=(1-dx)*(Z3(y,x)*(1-dy)+Z3(y,x+1)*dy)+dx*(Z3(y+1,x)*(1-dy)+Z3(y+1,x+1)*dy);
    if z-z1<1
        feasible=false;
    end
    for i=1:Np
        if sqrt((point(1)-danareaX(i))^2+(point(2)-danareaY(i))^2+(point(3)-danareaZ(i))^2)<Rk
            feasible=false;
        end
    end
end