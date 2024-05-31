
%% 双移线轨迹
clear 
clc

%双移线--------------------------------------------------------
 shape=2.4;%参数名称，用于参考轨迹生成
 dx1=25;dx2=21.95;%没有任何实际意义，只是参数名称
 dy1=4.05;dy2=5.7;%没有任何实际意义，只是参数名称
 Xs1=27.19;Xs2=56.46;%参数名称
 cx=0:0.1:120;
 
 z1=shape/dx1*(cx-Xs1)-shape/2;
 z2=shape/dx2*(cx-Xs2)-shape/2;
 cy=dy1/2.*(1+tanh(z1))-dy2/2.*(1+tanh(z2));

%双移线-------------------------------------------------------
%得到双移线轨迹每个点对应的X,Y坐标，用于在carsim中生成参考轨迹
path=[cx',cy'];

figure(1)
plot(cx,cy,'LineWidth',2)

%参考轨迹切线角***********************************************
ref_x=cx;
ref_y=cy;

%差分函数
dif_x=diff(ref_x);
dif_xxxx=[dif_x,dif_x(end)];

dif_y=diff(ref_y);
dif_yyyy=[dif_y,dif_y(end)];

r_yaw = atan2(dif_yyyy , dif_xxxx);
%参考轨迹切线角***********************************************
figure(2)
plot(cx,r_yaw )
