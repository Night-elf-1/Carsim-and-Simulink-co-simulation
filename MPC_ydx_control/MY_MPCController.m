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
sizes.NumDiscStates  = 3;     %3个离散状态
sizes.NumOutputs     = 2;     %2个输出，速度，前轮转角 
sizes.NumInputs      = 6;     %6个输入，需要状态量 与 速度
sizes.DirFeedthrough = 1;     %输入直接受控于一个输入口的值
sizes.NumSampleTimes = 1;     %单个采样周期
sys = simsizes(sizes);        %由设定参数进行系统初始化

x0 =[0.001;0.001;0.001];%初值设定 3个误差状态：[ex,ey,eyaw]初始值

str = [];                     %规定设置为空
ts  = [0.05 0];               %采样时间

%==============================================================
% 3.更新离散状态量---用不到---Update the discrete states
%==============================================================
function sys = mdlUpdates(t,x,u)
  
sys = x;


%==============================================================
% 2.计算输出---核心---Calculate outputs
%==============================================================
function sys = mdlOutputs(~,~,u)

  %定义静态变量 
    persistent u_piao_s; %存储上一时刻控制量：u_piao(k-1)
    if isempty(u_piao_s)
       u_piao_s=[0;0];
    end

    persistent vrs;      %存储上一时刻参考的速度控制量：vr(k-1)
    if isempty(vrs)
       vrs=0;
    end

    persistent drs;      %存储上一时刻参考的前轮转角控制量：dr(k-1)
    if isempty(drs)
       drs=0;
    end

    persistent n;
    if isempty(n)
       n=0;
    end

  %相关参数
    Nx =3;                    %状态量的个数 3个误差状态：[ex,ey,eyaw]
    Nu =2;                    %控制量的个数 速度：v  前轮转角：deita
    Np =20;                   %预测步长
    Nc=20;                    %控制步长
    Row=3;                    %松弛因子权重矩阵
    T=0.05;                   %采样步长
  %路径跟踪误差连续状态空间方程系数矩阵
    L=2.6;
    vr = u(6);                %u(6)是vr  
    yawr=u(5);                %u(5)是yawr
    dr=u(4);                  %u(4)是前轮转角 dr
    A1=[0,   0,  -vr*sin(yawr);
        0,   0,   vr*cos(yawr);
        0,   0,       0];
    B1=[cos(yawr) ,   0;
        sin(yawr) ,   0;
        tan(dr)/L,vr/(cos(dr)*L*cos(dr))];
 %路径跟踪误差离散状态空间方程系数矩阵
    %A2 = T*A1+eye(Nx);% 向前欧拉法离散化
    A2 = (eye(Nx)-A1*T/2)\(eye(Nx)+A1*T/2);%中点欧拉法离散化
    B2 = T*B1;                             %前向欧拉法

 %构建新的状态量---kesi
    kesi=zeros(Nx+Nu,1);      %（3+2）行*1列
    kesi(1) = u(1);           %ex
    kesi(2) = u(2);           %ey
    kesi(3) = u(3);           %eyaw
    kesi(4) = u_piao_s(1);    %上个时刻控制量
    kesi(5) = u_piao_s(2);    %上个时刻控制量 
                                                                                  
 %新状态空间方程系数矩阵为：
    A3_cell=cell(2,2);
    B3_cell=cell(2,1);
    A3_cell{1,1}=A2;
    A3_cell{1,2}=B2;
    A3_cell{2,1}=zeros(Nu,Nx);
    A3_cell{2,2}=eye(Nu);
    B3_cell{1,1}=B2;
    B3_cell{2,1}=eye(Nu);
    A3=cell2mat(A3_cell);
    B3=cell2mat(B3_cell);
 %输出方程的系数矩阵为：
    C=[ 1 0 0 0 0;
        0 1 0 0 0;
        0 0 1 0 0];
 %预测方程的系数矩阵
    W_cell=cell(Np,1);
    Z_cell=cell(Np,Nc);
    for j=1:1:Np
        W_cell{j,1}=C*A3^j;
        for k=1:1:Nc
            if k<=j
                Z_cell{j,k}=C*A3^(j-k)*B3;
            else 
                Z_cell{j,k}=zeros(Nx,Nu);
            end
        end
    end
    W=cell2mat(W_cell);
    Z=cell2mat(Z_cell);

 %权重矩阵设计  
    Q=[80,0,0;
        0,80,0;
        0,0,20];
    F=[50,0,0;
        0,50,0;
        0,0,20];
    Q1 = kron(eye(Np-1),Q);
    QB = blkdiag(Q1,F);
    
    R=[75,0;
        0,60];
%     R=[50,0;
%         0,50];
    RB=kron(eye(Nc),R);
 %转化为标准二次型后的H,f矩阵
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
    UK_1=kron(ones(Nc,1), u_piao_s);
 
    umin= [(0/3.6-vr);(-30/57.297-dr)];  %控制量约束设计弧度制：0.523rad = 30deg
    umax= [ (17/3.6-vr); (30/57.297-dr)]; 
    Umin=kron(ones(Nc,1),umin);
    Umax=kron(ones(Nc,1),umax);
    A_cons_cell={AE zeros(Nu*Nc, 1); -AE zeros(Nu*Nc, 1)};
    b_cons_cell={Umax-UK_1;-Umin+UK_1};
    A_cons=cell2mat(A_cons_cell);%
    b_cons=cell2mat(b_cons_cell);%

 %控制量增量约束
    if n==0       
       vrs=vr;  %在初始时刻，我们让本时刻与上一时刻的参考速度相等，因为初始时刻没有上一时刻
       drs=dr;  %在初始时刻，我们让本时刻与上一时刻的参考前轮转角相等，因为初始时刻没有上一时刻
       n=1;
    end
    delta_umin = [-0.198-vr+vrs;-0.013-dr+drs]; %控制量增量约束设计，弧度制：度数转化弧度 m /（180/pi）
    delta_umax = [ 0.198-vr+vrs; 0.013-dr+drs];
    delta_Umin = kron(ones(Nc,1),delta_umin);
    delta_Umax = kron(ones(Nc,1),delta_umax);
    lb = [delta_Umin; 0];%状态量增量下界
    ub = [delta_Umax; 5];%状态量增量上界
  
 %开始求解过程
    options = optimset('Algorithm','interior-point-convex'); 
    [X, fval,exitflag]=quadprog(H, f, A_cons, b_cons,[], [],lb,ub,[],options);

 %作用于系统的控制量   
    deita_u_piao(1)=X(1);               %第一步，取出优化求解的第一个量
    deita_u_piao(2)=X(2);               %第一步，取出优化求解的第二个量
    u_piao_s(1)=kesi(4)+deita_u_piao(1);%第二步，上个时刻优化求解的控制量+本时刻优化求解的控制增量
    u_piao_s(2)=kesi(5)+deita_u_piao(2);%第二步，上个时刻优化求解的控制量+本时刻优化求解的控制增量
    u_real(1)=u_piao_s(1)+vr;           %第三步，最终速度控制量
    u_real(2)=u_piao_s(2)+dr;           %第三步，最终前轮控制量  
    
    vrs=vr;                             %本时刻速度参考量，作为下个时刻的上一个时刻速度参考量
    drs=dr;                             %本时刻前轮转角参考量，作为下个时刻的上一个时刻前轮转角参考量
    
    sys= u_real;                        %输出
