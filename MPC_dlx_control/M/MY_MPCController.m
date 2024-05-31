%%S函数的输入参数 t（当前时间）,x（状态向量）,u（输入向量）,flag；由simulink传入
%S函数的输出 sys：通用返回参数，取决于flag的值，如flag=3时，sys返回S函数的输出；
% x0：初始状态值，如果系统中没有状态，则向量为空。除flag为0外，x0被忽略；
% str：保留以后使用，S函数必须设置该元素为空矩阵，即[]；
% ts：一个两列矩阵，用于指定采样时间与时间的偏移量。

%主函数
function [sys,x0,str,ts] = MY_MPCController(t,x,u,flag)

switch flag
 case 0
  [sys,x0,str,ts] = mdlInitializeSizes; %初始化
  
 case 2
  sys = mdlUpdates(t,x,u);              %更新离散状态
  
 case 3
  sys = mdlOutputs(t,x,u);              %计算输出
 
 case {1,4,9}                           % 
  sys = [];
  
 otherwise
  error(['unhandled flag = ',num2str(flag)]); % Error handling
end

%==============================================================
% 1.初始化子函数
%==============================================================
function [sys,x0,str,ts] = mdlInitializeSizes

sizes = simsizes;             %读入参数初始化模板
sizes.NumContStates  = 0;     %无连续状态
sizes.NumDiscStates  = 4;     %4个离散状态
sizes.NumOutputs     = 1;     %1个输出，前轮转角 
sizes.NumInputs      = 5;     %5个输入，需要状态量 与 速度
sizes.DirFeedthrough = 1;     %输入直接受控于一个输入口的值
sizes.NumSampleTimes = 1;     %单个采样周期
sys = simsizes(sizes);        %由设定参数进行系统初始化

x0 =[0.001;0.001;0.001;0.001];%初值设定 4个误差状态：[ed,d_ed,eyaw,d_eyaw]初始值

str = [];                     %规定设置为空
ts  = [0.05 0];               %采样时间

%==============================================================
% 3.更新离散状态量---Update the discrete states
%==============================================================
function sys = mdlUpdates(t,x,u)
  
sys = x;


%==============================================================
% 2.计算输出---Calculate outputs
%==============================================================
function sys = mdlOutputs(~,~,u)

  %定义静态变量 n
    persistent uk_1;
    if isempty(uk_1)
    uk_1=0;
    end

%     global uk_1 
    % uk_1 = 0;
    Nx =4;                    %状态量的个数 4个误差状态：[ed,d_ed,eyaw,d_eyaw]
    Nu =1;                    %控制量的个数 前轮转角：deita
    Np =20;                   %预测步长
    Nc=20;                    %控制步长
    Row=5;                    %松弛因子权重矩阵

 % 车辆参数
    cf = -138970;
    cr = -82204;
    m = 1412;
    Iz=1536.7;
    lf=1.3;
    lr=2.6-1.3;
    T=0.05; 
 % 路径跟踪误差连续状态空间方程系数矩阵
    vx = u(5)+0.1; %u(5)是vx , vx在分母，避免vx=0 
    a1=[0,1,0,0;
       0,(cf+cr)/(m*vx),-(cf+cr)/m,(lf*cf-lr*cr)/(m*vx);
       0,0,0,1;
       0,(lf*cf-lr*cr)/(Iz*vx),-(lf*cf-lr*cr)/Iz,(lf^2*cf+lr^2*cr)/(Iz*vx)];
    b1=[0;
       -cf/m;
       0;
       -lf*cf/Iz];
 % 路径跟踪误差离散状态空间方程系数矩阵
    %a = T*a1+eye(Nx);% 向前欧拉法离散化
    a = (eye(Nx)-a1*T/2)\(eye(Nx)+a1*T/2);%中点欧拉法离散化
    b = T*b1;

 %构建新的状态量---kesi
    %u()看simulink中s函数的输入
    kesi=zeros(Nx+Nu,1);      %（4+1）行*1列
    kesi(1) = u(1);           %ed
    kesi(2) = u(2);           %ed_dot
    
    heading_offset = u(3);    %e_phi，让角度在 (-pi,pi)之内，apollo同样处理
    if (heading_offset < -pi)
        heading_offset = heading_offset + 2*pi;
    end
    if (heading_offset > pi)
        heading_offset = heading_offset - 2*pi;
    end
    kesi(3)=heading_offset;
    kesi(4)=u(4);             %ephi_dot
    kesi(5)=uk_1;             %上个时刻控制量 
                                                                                  

 %构建新状态量，新状态空间方程系数矩阵为：
    A_cell=cell(2,2);
    B_cell=cell(2,1);
    A_cell{1,1}=a;
    A_cell{1,2}=b;
    A_cell{2,1}=zeros(Nu,Nx);
    A_cell{2,2}=eye(Nu);
    B_cell{1,1}=b;
    B_cell{2,1}=eye(Nu);
    A=cell2mat(A_cell);
    B=cell2mat(B_cell);
 %输出方程的系数矩阵为：
    C=[ 1 0 0 0 0;
        0 1 0 0 0;
        0 0 1 0 0;
        0 0 0 1 0];
  %预测方程的系数矩阵
    W_cell=cell(Np,1);
    Z_cell=cell(Np,Nc);
    for j=1:1:Np
        W_cell{j,1}=C*A^j;
        for k=1:1:Nc
            if k<=j
                Z_cell{j,k}=C*A^(j-k)*B;
            else 
                Z_cell{j,k}=zeros(Nx,Nu);
            end
        end
    end
    W=cell2mat(W_cell);
    Z=cell2mat(Z_cell);

 % 权重矩阵设计  
    Q=[30,0,0,0;
        0,1,0,0;
        0,0,6,0
        0,0,0,1];
    F=[40,0,0,0;
        0,1,0,0;
        0,0,5,0
        0,0,0,1];
    Q1 = kron(eye(Np-1),Q);
    QB = blkdiag(Q1,F);
    
    R=10;
    RB=kron(eye(Nc),R);
 % 转化为标准二次型后的H,f矩阵
    H_cell=cell(2,2);
    H_cell{1,1}=2*(Z'*QB*Z+RB);
    H_cell{1,2}=zeros(Nu*Nc,1);
    H_cell{2,1}=zeros(1,Nu*Nc);
    H_cell{2,2}=2*Row;
    H=cell2mat(H_cell);
    H=(H+H')/2;
    G=W*kesi;
    fz_cell=cell(1,2);
    fz_cell{1,1} = 2*(G'*QB*Z);
    fz_cell{1,2} = 0;
    fz=cell2mat(fz_cell);
    f=fz';
 
 %控制量约束
    A_t=zeros(Nc,Nc);%
    for p=1:1:Nc
        for q=1:1:Nc
            if q<=p 
                A_t(p,q)=1;
            else 
                A_t(p,q)=0;
            end
        end 
    end 
    AE=kron(A_t,eye(Nu));
    UK_1=kron(ones(Nc,1), uk_1);
    
    umin= -0.523;%控制量约束设计弧度制：0.523rad = 30deg
    umax= 0.523; 
    Umin=kron(ones(Nc,1),umin);
    Umax=kron(ones(Nc,1),umax);
    A_cons_cell={AE zeros(Nu*Nc, 1); -AE zeros(Nu*Nc, 1)};
    b_cons_cell={Umax-UK_1;-Umin+UK_1};
    A_cons=cell2mat(A_cons_cell);%
    b_cons=cell2mat(b_cons_cell);%

 %控制量增量约束
    delta_umin = -0.013; %控制量增量约束设计弧度制：0.013rad = 0.75deg
    delta_umax =  0.013;
    delta_Umin = kron(ones(Nc,1),delta_umin);
    delta_Umax = kron(ones(Nc,1),delta_umax);
    lb = [delta_Umin; 0];%状态量增量下界
    ub = [delta_Umax; 5];%状态量增量上界
  
 %开始求解过程
    % options = optimset('Algorithm','active-set');
    options = optimset('Algorithm','interior-point-convex'); 
    [X, fval,exitflag]=quadprog(H, f, A_cons, b_cons,[], [],lb,ub,[],options);

 %作用于系统的控制量   
    u_piao=X(1);      %取出优化求解的第一个量
    uk=uk_1+ u_piao;  %上个时刻控制量+本时刻优化求解的控制增量
    
    uk_1=uk;          %下个时刻，本时刻的控制量成为上个时刻控制量
    sys= uk;          %输出
