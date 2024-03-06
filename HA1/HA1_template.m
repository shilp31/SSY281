clear;clc;close all;
% Use this template for Home Assignment 1.
% You may add as many variables as you want but DO NOT CHANGE THE NAME OF
% THE VARIABLES WHICH ARE ALREADY IN THE TEMPLATE.
% Also, DO NOT REUSE THE VARIABLES WHICH ARE ALREADY IN THE TEMPLATE FOR
% SOMETHING ELSE.
% The grading will be partially automatized, so it is very important that
% the names of the variables are what the grader script expects to see.
% All the variables whose name must not be changed have a suffix related to
% the number of the question, so just pay extra attention to them.
% Keep also in mind that the variables are expected to have NUMERICAL
% VALUES and not strings of characters.

%% Q1a (discretization of a state-space model)

% Discrete Model

A_1a = 'Matrix A for discrete time model';
B_1a = 'Matrix B for discrete time model';
C_1a = 'Matrix C for discrete time model';

eig_1a = 'Eigenvalues of A_1a';

%% Q1b

% Delayed model

Aa_1b = 'Matrix A for delayed discrete time model';
Ba_1b = 'Matrix B for delayed discrete time model';
Ca_1b = 'Matrix C for delayed discrete time model';

eig_1b = 'Eigenvalues of Aa_1b';

%% Q2a (Dynamic Programming solution of the LQ problem)

% finite horizon problem

N_2a = 'Minimum horizon length to stabilize the system';
K_2a = 'Stabilizing feedback gain';

%% Q2b

% infinite horizon problem

P_inf_2b = 'Stationary solution of the Riccati equation';
P_inf_DP_2b = 'Stationary solution of the Riccati equation obtained through Dynamic Programming)';

%% Q2c

N_2c = 'Minimum horizon length to stabilize the system';
K_2c = 'Stabilizing feedback gain';

%% Q3a (batch solution of the LQ problem)

N_3a = 'Minimum horizon length to stabilize the system';
K0_3a = 'Stabilizing feedback gain';

%% Q4 (Receding horizon control)

% plot x1, x2, x3 and u as per the instructions

%% Q5 (constrained receding horizon control)

% same plot as for Q4 but for the constrained problem