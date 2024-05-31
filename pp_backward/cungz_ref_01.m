%% 双移线轨迹
clear 
clc

%双移线------------------11111111111111111111111111111111111111
 shape=2.4;%参数名称，用于参考轨迹生成
 dx1=25;dx2=21.95;%没有任何实际意义，只是参数名称
 dy1=4.05;dy2=5.7;%没有任何实际意义，只是参数名称
 Xs1=27.19;Xs2=56.46;%参数名称
 cx=0:0.1:120;%%cx就是规划轨迹点的x坐标
 t1=0:0.05:17.9;
 t2=t1';
 z1=shape/dx1*(cx-Xs1)-shape/2;
 z2=shape/dx2*(cx-Xs2)-shape/2;
 cy=dy1/2.*(1+tanh(z1))-dy2/2.*(1+tanh(z2));%cy就是规划轨迹点的y坐标

 %这个path用于carsim道路模型搭建
path=[cx',cy'];


figure(1)

plot(cx,cy,'LineWidth',2)

ref_x=cx;
ref_y=cy;

%以下求参考横摆角，在纯跟踪算法里没用到
dif_x=diff(ref_x);
dif_xxxx=[dif_x,dif_x(end)];
dif_y=diff(ref_y);
dif_yyyy=[dif_y,dif_y(end)];
refHeading = atan2(dif_yyyy , dif_xxxx);


figure(2)
plot(cx,refHeading )


