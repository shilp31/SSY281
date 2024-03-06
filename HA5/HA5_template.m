clc
close all
clear

%% Q1a
% find terminal weight to guarantee asymptotic stability and plot X_N
A = [1.2  1;
     0    1];
B = [0; 1];

model = LTISystem('A', A, 'B', B);

model.x.min = [-15; -15];
model.x.max = [15; 15];
model.u.min = -1;
model.u.max = 1;
X = Polyhedron('lb', model.x.min, 'ub', model.x.max);
U = Polyhedron('lb', model.u.min, 'ub', model.u.max);

N = 4;           
Q = eye(2);      
R = 100;         
model.x.penalty = QuadFunction(Q);
model.u.penalty = QuadFunction(R);

Xf = Polyhedron([0 0]);
model.x.with('terminalSet');
model.x.terminalSet = Xf;

[H,~,~] = idare(A, B, Q, R);  
Pf_1a = QuadFunction(H);         
model.x.with('terminalPenalty');
model.x.terminalPenalty = Pf_1a; 

figure()
hold on
for i = 1:N
    X_N_1a = X.intersect(model.reachableSet('X', Xf, 'U', U, 'N', i, 'direction', 'backward'));
    plot(X_N_1a, 'color', 'green', 'alpha', 0.5)
    hold on
end

plot(0, 0, 'x', 'MarkerFaceColor', 'auto', 'MarkerEdgeColor', 'k', 'MarkerSize', 8)
xlabel('x_1')
ylabel('x_2')
legend('X_N, N=1', 'X_N, N=2', 'X_N, N=3', 'X_N, N=4', 'Origin')
title('X_N')
axis equal

disp('terminal weigth to guarantee asymptotic stability')
disp(Pf_1a)

% X_N_1a = 'set of feasible initial states for horizon length N (to be plotted)';

%% Q1b
% design 3 different MPC controllers, simulate them and plot the results
% (both for the simulated states and for the actual ones)

%% Q1c
% find the explicit MPC solution to reach the target set and plot the
% state-space partitions

%% Q1d
% choose a target set Xf such that persistent feasibility is guaranteed for
% all initial state x0 in C_inf

Xf_1d = 'target set that guarantees persistent feasibility for all initial states inside C_inf';

%% Q2a
% find the matrices for the discretized system

A_2a = 'A matrix for the discretized system';
B_2a = 'B matrix for the discretized system';
C_2a = 'C matrix for the discretized system';

%% Q2b
% design a minimum time (i.e. minimum N) controller that brings the system
% to standstill and plot the predicted states and the output

N_2b = 'minimum horizon length that allows the system to reach standstill';

%% Q2c
% design a minimum time controller that brings the system to standstill,
% while having a constrained torsional torque, and plot the predicted states
% and the output

N_2c = 'minimum horizon length that allows the system to reach standstill while respecting the constraint on T';

%% Q2d
% design a minimum time controller that brings the system close to
% standstill and plot the predicted states and output

% then, perform a closed loop simulation with a controller having the N you
% just found and plot the states and outputs for 2s

N_2d = 'minimum horizon length that allows the system to reach almost standstill';