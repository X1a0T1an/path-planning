function [data]=CeatHill(N,h,x0,y0,xi,yi,num) 
%����Сɽ��ģ�ͣ�N��ɽ��HΪɽ�ĸ߶ȣ�X0,Y0Ϊɽ�����ꡣXI,YIΪɽ���б�ȣ�NUMΪɽ��ģ�����ݵ�ά����
%XI,YI��ֵԽ��ɽ���Խƽ̹
% Example: 
%          tic; 
%          h=[20,60,40];
%          x0=[20,40,50];
%          y0=[20,50,60];
%          xi=[];
%          yi=[];
%          d=CeatHill(3,h,x0,y0,xi,yi,65); 
%          colormap(gray); %������Ⱦ����ɫ 
%          surf(d); %������ά���� 
%          shading flat; %��С����֮�䲻Ҫ���� 
%          toc %��������ʱ�� 
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