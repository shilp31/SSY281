clc
close all
clear

%% Q1a
% represent the polyhedron in V- and H-representation and plot the two
% versions separately

A = [0 1
    -1 0
    -1 -1
     1 1];
b = [0;
     0;
     1;
     1];

P_H_1a = Polyhedron('A', A, 'b', b);
P_V_1a = Polyhedron('V', P_H_1a.V, 'R', P_H_1a.R);

figure()
subplot(1, 2, 1)
plot(P_H_1a, 'color', 'plum', 'alpha', 0.5)
title('H-representation of the polyhedron')
subplot(1, 2, 2)
plot(P_V_1a, 'color', 'plum')
title('V-representation of the polyhedron')

%% Q1b
% perform some sets operations
A1 = [0 1
      1 0
      0 -1
     -1 0];
b1 = [2; 2; 2; 2];
P_1b = Polyhedron('A', A1, 'b', b1);

A2 = [-1 -1
      1 1
      1 -1
     -1 1];
b2 = [1; 1; 1; 1];
Q_1b = Polyhedron('A', A2, 'b', b2);


Mink_sum_1b = P_1b + Q_1b;
Pontr_diff_1b = P_1b - Q_1b;
PminusQplusQ_1b = (P_1b - Q_1b) + Q_1b;
PplusQminusQ_1b = (P_1b + Q_1b) - Q_1b;
QminusPplusP_1b = (Q_1b - P_1b) + P_1b;
QplusPminusP_1b = (Q_1b + P_1b) - P_1b;


figure();
subplot(4, 2, 1);
plot(P_1b, 'color', 'plum');
title('Polyhedron P');

subplot(4, 2, 2);
plot(Q_1b, 'color', 'plum');
title('Polyhedron Q');

subplot(4, 2, 3);
plot(Mink_sum_1b, 'color', 'plum');
title('P + Q');

subplot(4, 2, 4);
plot(Pontr_diff_1b, 'color', 'plum');
title('P - Q');

subplot(4, 2, 5);
plot(PminusQplusQ_1b, 'color', 'plum');
title('(P - Q) + Q');

subplot(4, 2, 6);
plot(PplusQminusQ_1b, 'color', 'plum');
title('(P + Q) - Q');

subplot(4, 2, 7);
plot(QminusPplusP_1b, 'color', 'plum');
title('(Q - P) + P');

subplot(4, 2, 8);
plot(QplusPminusP_1b, 'color', 'plum');
title('(Q + P) - P');


%% Q2a
% show that the set S is positively invariant for the system and plot it
A = [ 0.8 0.4;
     -0.4 0.8];
A_in = [1 0
        0 1;
        -1 0;
        0 -1;
        1 1;
        1 -1;
        -1 1;
        -1 -1];
b_in = [1; 1; 1; 1; 1.5; 1.5; 1.5; 1.5];

S_2a = Polyhedron('A', A_in, 'b', b_in);
S_2a_Reach = Polyhedron('A', A_in/A, 'b', b_in);
figure()
plot(S_2a, 'color', 'green', 'alpha', 0.6)
hold on
plot(S_2a_Reach, 'color', 'green', 'alpha', 0.8)
legend('Set S', 'reach(S)')

%% Q2b
% calculate the one-step reachable set from S and plot it together with S
B = [0; 1];
U = Polyhedron('lb', -1, 'ub', 1);
A_u = [1; -1];
b_u = [1; 1];

reach_set_S_2b = projection(Polyhedron('A', [A_in/A, -(A_in/A)*B; zeros(2, 2), A_u], 'b', [b_in; b_u]), 1:2);

figure
plot(S_2a, 'color', 'green')
hold on
plot(reach_set_S_2b, 'color', 'green', 'alpha', 0.5)
legend('Set S', 'One-step Reachable Set from S')


%% Q2c
% calculate the one-step precursor set of S and plot it together with S

pre_set_S_2c = projection(Polyhedron('A', [A_in*A, A_in*B; zeros(2, 2), A_u], 'b', [b_in; b_u]), 1:2); % project tau into 1st and 2nd dimensions

figure
plot(S_2a, 'color', 'green')
hold on
plot(pre_set_S_2c, 'color', 'green', 'alpha', 0.5)
legend('Set S', 'One-step precursor set of S')

%% Q3a
% find shortest prediction horizon such that the RH controller is feasible

A=[0.9   0.4
  -0.4   0.9];
B = [0; 1];

Q = eye(2);
R = 1;
x0 = [2 0]';

N_3a = 'Shortest prediction horizon that makes the RH controller feasible';

%% Q3b
% find out if the RH controller is persistently feasible all the way to the
% origin

maxContrInvSet_3b = 'Maximal control invariant set for the system';

%% Q3c
% plot the set of feasible initial states for the two controllers
% previously designed and discuss about the size of the problems

X0_1_3c = 'Set of feasible initial states for controller designed in Q3a';
X0_2_3c = 'Set of feasible initial states for controller designed in Q3b';




