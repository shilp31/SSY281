clear;clc;close all;


%% Q1a

A = [1.0041  0.0100  0  0;
     0.8281  1.0041  0  -0.0093;
     0.0002  0.0000  1  0.0098;
     0.0491  0.0002  0  0.9629];
B = [0.0007  0.01;
     0.1398  1;
     0.0028  0;
     0.5605  0];
C = [1  0  0  0;
     0  0  1  0];

y_s = [pi/18 -pi]';

I = eye(length(A));

result_1a = [I-A -B;
       C   zeros(2)]\[zeros(4,1); y_s];

disp("State set-point")
xs_1a = result_1a(1:4, 1)
disp("Input set-point")
us_1a = result_1a(5:end, 1)

%% Q1b

B_1b = B(:, 2);

m = size(B_1b, 2);
n = size(A, 1);
p = size(C, 1);

Q = eye(p);

H = 2* blkdiag(C'*Q*C, zeros(m));
f = [(-2*y_s'*Q*C)'; 
    zeros(m,1)];
I = eye(n);

% define constraints, AX = B
A_cons_1b = [I-A -B_1b];
B_cons_1b = zeros(n, 1);

result_1b = quadprog(H, f, [], [], A_cons_1b, B_cons_1b, [], []);
disp("State set-point that minimizes the 2-norm of the output error")
xs_1b = result_1b(1:4, 1)
disp("Input set-point that minimizes the 2-norm of the output error")
us_1b = result_1b(end, 1)

%% Q1c

C_1c = C(1, :);
z_sp = pi/18;

m = size(B, 2);
n = size(A, 1);
p = size(C_1c, 1);

R_s = eye(m);

H = 2*blkdiag(zeros(n), R_s);
f = [zeros(n, 1); 
    zeros(m, 1)];
I = eye(n);

A_cons_1c = [I-A -B;
    C_1c zeros(size(C_1c*B))];
B_cons_1c = [zeros(n, 1); 
             z_sp];

options = optimoptions('quadprog','Display','iter');
result_1c = quadprog(H, f, [], [], A_cons_1c, B_cons_1c, [], [], [], options);

disp("State set-point that minimizes the 2-norm of the input and drives the output error to 0")
xs_1c = result_1c(1:4, 1)
disp("Input set-point whose 2-norm is minimized")
us_1c = result_1c(5:end, 1)

%% Q2a

B = B(:, 1);
B_p = B;
C_p = [0 1]';

% disturbance models

Bd_1 = zeros(4,1);
Cd_1 = [1; 1];

Bd_2 = zeros(4, 2);
Cd_2 = eye(2);

Bd_3 = [zeros(4, 1) B_p];
Cd_3 = eye(2);

disp("Augmented model for the first disturbance model")
[Ae_1_2a, Be_1_2a, Ce_1_2a] = find_augmod(A, B, C, Bd_1, Cd_1)
disp("Augmented model for the second disturbance model")
[Ae_2_2a, Be_2_2a, Ce_2_2a] = find_augmod(A, B, C, Bd_2, Cd_2)
disp("Augmented model for the third disturbance model")
[Ae_3_2a, Be_3_2a, Ce_3_2a] = find_augmod(A, B, C, Bd_3, Cd_3)

%% Q2b

Q = eye(5);
R = eye(2);

disp("Stationary Kalman filter for the first augmented model (write 0 if you found that augmented model 1 is undetectable)")
[~, Le_1_2b, ~] = idare(Ae_1_2a', Ce_1_2a', Q, R)
disp("Stationary Kalman filter for the second augmented model (write 0 if you found that augmented model 2 is undetectable)")
Le_2_2b = 0
Q = eye(6);
disp("Stationary Kalman filter for the third augmented model (write 0 if you found that augmented model 3 is undetectable)")
[~, Le_3_2b, ~] = idare(Ae_3_2a', Ce_3_2a', Q, R)

%% Q2c

H = [0 1];
disp("Matrix Mss for the first augmented model (write 0 if you found that augmented model 1 is undetectable)")
Mss_1_2c = [eye(length(A))-A, -B;
              H*C     zeros(size(H*C,1),size(B,2))]\[Bd_1; -H*Cd_1]
disp("Matrix Mss for the second augmented model (write 0 if you found that augmented model 2 is undetectable)")
Mss_2_2c = 0
disp("Matrix Mss for the third augmented model (write 0 if you found that augmented model 3 is undetectable)")
Mss_3_2c = [eye(length(A))-A, -B;
              H*C     zeros(size(H*C,1),size(B,2))]\[Bd_3; -H*Cd_3]

%% Q2d

% provide plots showing how the states evolve for all the observable systems


%% Functions

% construct augmented model 
function[A_a, B_a, C_a] = find_augmod(A, B, C, B_d, C_d)
A_a = [A B_d;
    zeros(size(B_d, 2), size(A, 2)) eye(size(B_d, 2))];
B_a = [B; zeros(size(B_d, 2), size(B, 2))];
C_a = [C C_d];

rank_abc = rank(obsv(A, C));
rank_a_abc = rank([eye(length(A))-A  -B_d; C  C_d ]);
if rank_abc ~= rank(A)
    disp("Original system is rank deficient")
end
if rank_a_abc ~= rank(A_a)
    disp("Aug system is rank deficient")
end
end
