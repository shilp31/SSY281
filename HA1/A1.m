clear;clc;close all;

%% Q1a (discretization of a state-space model)

% Discrete Model
h = 0.1;
a = -0.9421;
b = 82.7231;
c = 14.2306;
p = -3.7808;
q = 4.9952;
r = 57.1120;

A_c = [0 1 0 0
      b 0 0 a
      0 0 0 1
      q 0 0 p];

B_c = [0 c 0 r]';

C_c = [1 0 0 0];

D_c = 0;

sys_d = expm([A_c B_c;[0 0 0 0], 0].*h);

A_1a = sys_d(1:4, 1:4);
B_1a = sys_d(1:4, 5);
C_1a = C_c;

eig_1a = eig(A_1a);

%% Q1b

% Delayed model
tau = 0.8*h;

sys_d_tau = expm([A_c B_c;[0 0 0 0], 0].*tau);
A_tau = sys_d_tau(1:4, 1:4);
B_tau = sys_d_tau(1:4, 5);

sys_d_h_tau = expm([A_c B_c;[0 0 0 0], 0].*(h-tau));
A_h_tau = sys_d_h_tau(1:4, 1:4);
B_h_tau = sys_d_h_tau(1:4, 5);

A = A_h_tau*A_tau;
B_1 = A_h_tau*B_tau;
B_2 = B_h_tau;

Aa_1b = [A B_1;[0 0 0 0] 0];
Ba_1b = [B_2; 1];
Ca_1b = [C_1a 0];

eig_1b = eig(Aa_1b);

%% Q2a (Dynamic Programming solution of the LQ problem)

% finite horizon problem
h = 0.01;
A = [1.0041 0.0100 0 0;
     0.8281 1.0041 0 -0.0093;
     0.0002 0.0000 1 0.0098;
     0.0491 0.0002 0 0.9629];
B = [0.0007;
     0.1398;
     0.0028;
     0.5605];
C = [1 0 0 0;
     0 0 1 0];
Q = eye(4);
P_f = 10*eye(4);
R = 1;


[N_2a, K_2a] = min_n_dp(A, B, Q, R, P_f);
disp('Minimum horizon length to stabilize the system');
disp(N_2a)
disp('Stabilizing feedback gain');
disp(K_2a)

%% Q2b

% infinite horizon problem

P_inf_2b = idare(A, B, Q, R);
P_inf_DP_2b = sta_sol_dp(A, B, Q, R, P_f);
disp('Stationary solution of the Riccati equation')
disp(P_inf_2b);
disp('Stationary solution of the Riccati equation obtained through Dynamic Programming)')
disp(P_inf_DP_2b)

%% Q2c
[N_2c, K_2c] = min_n_dp(A, B, Q, R, P_inf_2b);
disp('Minimum horizon length to stabilize the system')
disp(N_2c)
disp('Stabilizing feedback gain')
disp(K_2c)

%% Q3a (batch solution of the LQ problem)
N_3a = 'Minimum horizon length to stabilize the system';
K0_3a = 'Stabilizing feedback gain';
%[N_3a, K0_3a] = min_n_batch(A, B, Q, R, P_f);
%disp('Minimum horizon length to stabilize the system')
%disp(N_3a)
%disp('Stabilizing feedback gain')
%disp(K0_3a);

%% Q4 (Receding horizon control)

% plot x1, x2, x3 and u as per the instructions
Q = eye(4);
P_f = 10*eye(4);
[x1, u1] = simulate_dp(A, B, Q, 1, P_f, 40);
[x2, u2] = simulate_dp(A, B, Q, 1, P_f, 80);
[x3, u3] = simulate_dp(A, B, Q, 0.1, P_f, 40);
[x4, u4] = simulate_dp(A, B, Q, 0.1, P_f, 80);

plot_sim(x1, u1, x2, u2, x3, u3, x4, u4)


%% Q5 (constrained receding horizon control)

% same plot as for Q4 but for the constrained problem
constraint = [0 1 0 0 8];
[x1, u1] = sim_CRHC(A, B, Q, 1, P_f, 40, constraint);
[x2, u2] = sim_CRHC(A, B, Q, 1, P_f, 80, constraint);
[x3, u3] = sim_CRHC(A, B, Q, 0.1, P_f, 40, constraint);
[x4, u4] = sim_CRHC(A, B, Q, 0.1, P_f, 80, constraint);

plot_sim(x1, u1, x2, u2, x3, u3, x4, u4)


%% Functions

% To find minimum N (DP)
function [N, K_N] = min_n_dp(A, B, Q, R, P_f)
P = P_f;
N = 0;
while 1
    N = N+1;
    % Riccatti Eq
    P = Q + A'*P*A - (A'*P*B/(R + B'*P*B)*B'*P*A);
    K_N = -(R + B.'*P*B)\(B.'*P*A);
    % Stability check
    poles = abs(eig(A + B*K_N));
    unstable = sum(poles>=1);

    if unstable == 0
        poles = eig(A + B*K_N);
        if N ~= 1
            N = N+1;
        end
        disp(poles)
        break
    end
    if N>1000
        break
    end
end
end


% To find stationary solution (DP)
function[P_final] = sta_sol_dp(A, B, Q, R, P_f)
P = P_f;
N = 0;
while 1
    N = N +1;
    P_old = P;
    P = Q + A'*P*A - A'*P*B/(R + B'*P*B)*B'*P*A;
    
    if norm(P - P_old) <= 0.1
        P_final = P;
        N = N+1;
        disp('Stationary solution found at N =');
        disp(N);
        break
    end
end     
end

% To find minimum N (Batch)
%function [N, K_b] = min_n_batch(A, B, Q, R, P_f)

%end

% To simulate systems (DP)
function [x, u] = simulate_dp(A, B, Q, R, P_f, N)
    x = [pi/38; 0; 0; 0];
    u = zeros(size(B, 2), 210);
    for k = 1:210
        P = P_f;
        for i = 1:N-1
            P = Q + A'*P*A-(A'*P*B)/(R+B'*P*B)*B'*P*A;
        end
        K_N = -(R + B.'*P*B)\(B.'*P*A);
        u(:, k) = K_N * x(:, k);
        x(:, k+1) = A * x(:, k) + B * u(:, k);
    end
end

% To plot simulations
function [] = plot_sim(x1, u1, x2, u2, x3, u3, x4, u4)
figure()
subplot(4,1,1)
hold on; grid on
title('Ball angle')
plot(0:100,x1(1,1:101))
plot(0:100,x2(1,1:101))
plot(0:100,x3(1,1:101))
plot(0:100,x4(1,1:101))
legend('R=1, N=40','R=1, N=80','R=0.1, N=40','R=0.1, N=80')

subplot(4,1,2)
hold on; grid on
title('Ball angular speed')
plot(0:100,x1(2,1:101))
plot(0:100,x2(2,1:101))
plot(0:100,x3(2,1:101))
plot(0:100,x4(2,1:101))
legend('R=1, N=40','R=1, N=80','R=0.1, N=40','R=0.1, N=80')

subplot(4,1,3)
hold on; grid on
title('Wheel angle')
plot(0:200,x1(3,1:201))
plot(0:200,x2(3,1:201))
plot(0:200,x3(3,1:201))
plot(0:200,x4(3,1:201))
legend('R=1, N=40','R=1, N=80','R=0.1, N=40','R=0.1, N=80')

subplot(4,1,4)
hold on; grid on
title('Input')
plot(0:30,u1(1:31))
plot(0:30,u2(1:31))
plot(0:30,u3(1:31))
plot(0:30,u4(1:31))
legend('R=1, N=40','R=1, N=80','R=0.1, N=40','R=0.1, N=80')

end

% constrained RHC
function [Z, VN] = CRHC(A, B, N, Q, R, P_f, F, G, h, x0, n)
% Objective function
Qbar = blkdiag(kron(eye(N-1),Q),P_f);
Rbar = kron(eye(N),R);
H = 2*blkdiag(Qbar,Rbar);
f = [];
% Equality constraints
I = eye(n);
Aeq1 = kron(eye(N),I)+kron(diag(ones(N-1,1),-1),-A);
Aeq2 = kron(eye(N),-B);
Aeq = [Aeq1 Aeq2];
beq = [A*x0;zeros(n*(N-1),1)];
% Inequality constraints
Ain = [F G];
bin = h;
% Solve QP
[Z,VN,exitflag,~,~] = quadprog(H,f,Ain,bin,Aeq,beq);
% Check if the problem has been solved to optimality
if exitflag ~= 1
    disp(['The exitflag is ' num2str(exitflag) ', which means that something is wrong.'])
    pause
end
end

%simulate CRHC
function [x_t, u_t] = sim_CRHC(A, B, Q, R, P_f, N, constraint)
x_max = constraint(1:4);
u_max = constraint(5);

x0 = [pi/38 0 0 0]';
n  = length(A);    
m  = size(B,2);

F = kron([eye(N); zeros(N)], [0 1 0 0; 0 -1 0 0]);
G = kron([zeros(N); eye(N)], [1; -1]);
h = [x_max(2)*ones(2*N, 1); u_max*ones(2*N, 1)];

t_f = 210;

x_t = [x0 zeros(n, t_f)];
u_t = zeros(m, t_f);

x = x0;

for t_step = 1:t_f
    [solution, ~] = CRHC(A, B, N, Q, R, P_f, F, G, h, x, n);
    x = solution(1:n);
    u = solution(n*N + 1);

    x_t(:, t_step+1) = x;
    u_t(:, t_step) = u;
end
end













