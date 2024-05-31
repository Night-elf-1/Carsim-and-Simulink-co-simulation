%% 最终需要的参考信息为：x,y,yaw,v,前轮转角，共5个量
 clear 
 clc

%% 生成双移线参考轨迹
 %设置相关参数，最终生成 双移线 参考轨迹
 %最终 cx, cy 为生成双移线的 x,y坐标
 shape=2.4;           
 dx1=25;dx2=21.95;     
 dy1=4.05;dy2=5.7;     
 Xs1=27.19;Xs2=56.46;  
 cx=0:0.1:120;         %x坐标从0开始，每隔0.1m生成一个点，一共120m
 z1=shape/dx1*(cx-Xs1)-shape/2;
 z2=shape/dx2*(cx-Xs2)-shape/2;
 cy=dy1/2.*(1+tanh(z1))-dy2/2.*(1+tanh(z2));

%% 将生成的双移线x,y坐标整理，以方便复制进CarSim
 path=[cx',cy'];

%% 生成参考曲率与参考横摆角
 ref_x=cx;
 ref_y=cy;

 dif_x=diff(ref_x);                   %差分x
 dif_xxxx=[dif_x,dif_x(end)];         %差分后与原来数据比少一个，所以加一个
 
 dif_y=diff(ref_y);                   %差分y
 dif_yyyy=[dif_y,dif_y(end)];         %差分后与原来数据比少一个，所以加一个
 
 r_yaw = atan2(dif_yyyy , dif_xxxx);  %生成参考轨迹的横摆角
 
 d_y=gradient(ref_y) ./ abs(dif_xxxx);
 dd_y=gradient(d_y) ./ abs(dif_xxxx);
 
 r_k=abs(dd_y) ./ (1+d_y.^2).^(3/2);  %生成参考轨迹的参考曲率

 %% 画图
 figure(1)
 plot(cx,cy,'LineWidth',2)            %画生成的双移线轨迹
 
 figure(2)
 plot(cx,r_yaw )                      %画参考横摆角

 figure(3)
 plot(cx,r_k)                         %画参考曲率







