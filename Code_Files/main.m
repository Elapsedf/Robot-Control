clear;clc;
addpath('/Users/dwl/Downloads/mrlib/mrlib');

% ----------------------------- Initial ----------------------------- %

W1 = 109e-3; W2 = 82e-3; L1 = 425e-3; L2 = 392e-3; H1 = 89e-3; H2 = 95e-3;
% input 1: The initial configuration of the end-effector in the 
Tseini = [-1  0  0  L1+L2;
           0  0  1  W1+W2;
           0  1  0  H1-H2;
           0  0  0  1;]; 
% input 2: The desired configuration (test case 1) 
% Tsegoal = [-0.500 -0.500 -0.707  0.108;
%            -0.500 -0.500  0.707  0.379;
%            -0.707  0.707  0.000  0.725;
%             0.000  0.000  0.000  1.000;];
% % input 2: The desired configuration (test case 2)
% Tsegoal = [-0.491 -0.533 -0.689  0.372;
%            -0.512 -0.463  0.723  0.589;
%            -0.705  0.708 -0.046  0.358;
%             0.000  0.000  0.000  1.000;];
% input 2: use FK to calculate a valid conf
Slist = [0   0   0   0      0      0;
         0   1   1   1      0      1;
         1   0   0   0     -1      0;
         0  -H1 -H1 -H1    -W1     H2-H1;
         0   0   0   0      L1+L2  0; 
         0   0   L1  L1+L2  0      L1+L2];
theta_d = [30; -45; -20; 0; 0; 0]*pi/180; % it has to be coloumn. Weird.
Tsegoal = FKinSpace(Tseini, Slist, theta_d);

% time setp
dt = 0.01; 

% ----------------------------- Traj Gen ----------------------------- %
% set Tf
% PUT YOUR CODE HERE! %
% Tf = ...
Tf=5;
% set N
% PUT YOUR CODE HERE! %
% N = ...
N=100;
% Set method
% PUT YOUR CODE HERE! %
% method = ...
method=5;
% compute trajectory
% PUT YOUR CODE HERE! %
% result_traj = ScrewTrajectory(...
result_traj=ScrewTrajectory(Tseini,Tsegoal,Tf,N,method);



% ----------------------------- Traj Gen ----------------------------- %
% 初始化矩阵存储每次的控制量
result_conf = zeros(N-2, 6);
result_Xerr = zeros(N-2, 6);
result_control = zeros(N-2, 6);

% 初始化迭代值
X = Tseini;     % 当前位姿
Xd = result_traj{1};    % 期望的下一个位姿
Xdnext = result_traj{2};    % 期望的下两个位姿
thetalist = [0; 0; 0; 0; 0; 0]; 
current_conf = [0 0 0 0 0 0];   % 当前关节角度
% PI控制器的参数

M = [-1 0 0 L1+L2;
    0 0 1 W1+W2;
    0 1 0 H1-H2;
    0 0 0 1];
% STEP3
% PUT YOUR CODE HERE! %
Kp = eye(6);
Ki = eye(6);
dt = 0.01;
result_conf = zeros(N-2,6);
result_Xerr = zeros(N-2,6);
result_control = zeros(N-2, 6);

X = Tseini;
Xd = result_traj{1};

Xdnext = result_traj{2};

thetalist = [0;0;0;0;0;0];
current_conf = [0 0 0 0 0 0];


for i = 1:(N-2)
    % 读取当前的关节角度
    thetalist = current_conf';

    % 调用PI控制器
    [control, Xerr] = controller(X, Xd, Xdnext, Kp, Ki, dt, thetalist);

    % 求取下一个关节角度
    next_conf = nextState(current_conf, control, dt);


    result_Xerr(i,:) = Xerr;
    result_conf(i,:) = next_conf;
    result_control(i,:) = control;

    norm(current_conf' - theta_d)

    % 计算末端执行器位形
    thetalist = next_conf';
    X = FKinSpace(X,Slist,thetalist);

    % 设置末端执行器的期望位形为轨迹的第 i+1 个元素
    Xd = result_traj{i+1};

    % 设置末端执行器的下一个期望位形为轨迹的第 i+2 个元素
    Xdnext = result_traj{i+2};

    % 更新当前机器人关节位形
    current_conf = next_conf;

end

current_conf
theta_d'

% 保存为csv文件
disp('Configuration Computation Done.');
csvwrite('result_conf.csv',result_conf);
disp('CSV file Written Done.');