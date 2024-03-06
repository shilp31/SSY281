clc
close all
clear all

%% Q3c
% solve the LP using the formulation shown in (5)
A = [0.4889 0.2939;
 1.0347 -0.7873;
 0.7269 0.8884;
 -0.3034 -1.1471];
b = [-1.0689;
    -0.8095;
    -2.9443;
    1.4384];
% determine number of constraints, coeffts
m = size(A, 1);
n = size(A, 2);

c_T = [zeros(n, 1);
    1];

F = [A -ones(m, 1);
    -A -ones(m, 1)];
g = [b;
    -b];
disp('Solution to the linear problem')
[z_3c, ~ ] = linprog(c_T, F, g)

%% Q3e
% solve the dual problem derived in Q3d
Aeq = F';
beq = -[0 0 1]';
lb = zeros(length(g), 1);
ub = [];
disp('Solution to the dual problem')
[mu_3e, ~, ~, ~, ~] = linprog(g, [], [], Aeq, beq, lb, ub)

%% Q3f
% Solve the primal problem by using the solution of the dual problem
% obtained in Q3e

z_3f = 'Solution to the primal problem obtained through the dual one';

%% Q4a
% solve the QP
x_4a = 'SOlution to the QP (x1 and x2)';
u_4a = 'Solution to the QP (u0 and u1)';
