clear;clc;

% STEP1
% PUT YOUR CODE HERE! %
run("trajectoryGenerator.m")
result_traj
% STEP2
% PUT YOUR CODE HERE! %

W1 = 109e-3; W2 = 82e-3;
L1 = 425e-3; L2 = 392e-3;
H1 = 89e-3; H2 = 95e-3;
Slist = [0 0 0 0 0 0 ;
         0 1 1 1 0 1 ;
         1 0 0 0 -1 0 ;
         0 -H1 -H1 -H1 -W1 H2-H1 ;
         0 0 0 0 L1+L2 0 ;
         0 0 L1 L1+L2 0 L1+L2];
Blist = [0 0 0 0 0 0;
         1 0 0 0 -1 0;
         0 1 1 1 0 1;
         W1+W2 H2 H2 H2 -W2 0;
         0 -L1-L2 -L2 0 0 0;
         L1+L2 0 0 0 0 0];
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

% STEP4
% PUT YOUR CODE HERE! %
X = Tseini;
Xd = result_traj{1};
Xd

Xdnext = result_traj{2};
Xdnext
thetalist = [0;0;0;0;0;0];
current_conf = [0 0 0 0 0 0];

% STEP5
for i = 1:(N-2)
thetalist = current_conf';
[control, Xerr] = controller(X, Xd, Xdnext, Kp, Ki, dt, thetalist);
next_conf = nextState(current_conf, control, dt);
result_Xerr(i,:) = Xerr;
result_conf(i,:) = next_conf;
result_control(i,:) = control;
thetalist = next_conf';
X = FKinSpace(M, Slist, thetalist);

Xd = result_traj{i+1};
Xdnext = result_traj{i+2};
current_conf = next_conf;
% PUT YOUR CODE HERE! %
end

% STEP6
disp('Configuration Computation Done.');
csvwrite('result_conf.csv',result_conf);
csvwrite('result_Xerr.csv',result_Xerr);
disp('CSV file Written Done.');
% PUT YOUR CODE HERE! %