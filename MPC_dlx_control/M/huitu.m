
load("tu2.mat")
 
load("cx1.mat")
load("cy1.mat")
figure(1)
plot(cx,cy,'-.k','LineWidth',1.5);%黑色
hold on
plot(out.x.Data,out.y.Data,'-.m','LineWidth',1.5);%粉色
legend('参考轨迹','MPC-30km/h');%图例
xlabel('X/m')
ylabel('Y/m')
 grid %网格
% 
figure(2)
plot(out.steer.Time,out.steer.Data,'-.m','LineWidth',1.5);
xlabel('X/s')
ylabel('Y/°')
legend('steer');
grid 

