function highe=highenough(posX,posY,posZ,n)
%�жϷ��и߶��Ƿ��ܹ��㿪���Σ�ͬʱ������߷��и߶�
    global Z3
    c=15;%��ÿ���ٷֽ��1/c
    for i=1:n-1
%         if posX(i)>1&&posX(i+1)>1&&posY(i)>1&&posY(i+1)>1
            %dist=(((posZ(i+1)-posZ(i))^2+(posY(i+1)-posY(i))^2+(posX(i+1)-posX(i))^2)^0.5)/10;
            for j=0:c-1
            %����ƽ����ֵ��˼����������ĸ߶�
                x=floor(posX(i)+j*(posX(i+1)-posX(i))/c);
                y=floor(posY(i)+j*(posY(i+1)-posY(i))/c);
%                 disp(y);                
                if x>60||y>60||x<1||y<1
                    highe=0;
                    return
                end
                z=(posZ(i)+j*(posZ(i+1)-posZ(i))/c);
                 dx=posX(i)+j*(posX(i+1)-posX(i))/c-x;
                 dy=posY(i)+j*(posY(i+1)-posY(i))/c-y;
                 z1=(1-dx)*(Z3(y,x)*(1-dy)+Z3(y,x+1)*dy)+dx*(Z3(y+1,x)*(1-dy)+Z3(y+1,x+1)*dy);
                if z-z1<1.5
                    highe=0;
                    return;
                end
                if z>30
                    highe=0;
                    return;
                end
            end          
     end
    highe=1;
end
            