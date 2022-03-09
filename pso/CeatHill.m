function [data]=CeatHill(N,h,x0,y0,xi,yi,num) 
%创建小山丘模型，N座山，H为山的高度，X0,Y0为山的坐标。XI,YI为山丘的斜度，NUM为山丘模型数据的维数。
%XI,YI的值越大，山丘就越平坦
% Example: 
%          tic; 
%          h=[20,60,40];
%          x0=[20,40,50];
%          y0=[20,50,60];
%          xi=[];
%          yi=[];
%          d=CeatHill(3,h,x0,y0,xi,yi,65); 
%          colormap(gray); %设置渲染的颜色 
%          surf(d); %画出三维曲面 
%          shading flat; %各小曲面之间不要网格 
%          toc %测试运行时间 
x=1:1:num;y=1:1:num;
%z2=zeros(1,4);
for m=1:num
    for n=1:num
        Sum=0;
        for k=1:N
            s=h(k)*exp(-((x(m)-x0(k))/xi(k))^2-((y(n)-y0(k))/yi(k))^2);
            Sum=Sum+s;
        end
        data(m,n)=Sum;
    end
end