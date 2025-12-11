%% Pollinator–Flower State-Space Model
% 5 states, 2 inputs, 3 outputs

% State matrix A
A = [
    -0.08   0.02    0       -0.01   0;
     0.04  -0.06    0        0      0;
     0.10   0      -0.05     0.02  -0.01;
     0      0       0.03    -0.04   0;
     0      0       0        0.01  -0.03
];

% Input matrix B
B = [
    0.10   0.00;
    0.02   0.00;
    0.00   1.00;
    0.00   0.10;
    0.00   0.05
];

% Output matrix C
C = [
    1  1  0  0  0;   % y1 = x1 + x2
    0  0  1  0  0;   % y2 = x3
    0  0  0  0  1;    % y3 = x5
];

% Feedthrough matrix D
D = zeros(3,2);

Q = [1/1000^2, 0, 0, 0, 0;
    0, 1/500^2, 0, 0, 0;
    0, 0, 1/100^2, 0, 0;
    0, 0, 0, 0.1, 0;
    0, 0, 0, 0, 0.1];
R = [1/430^2, 0;
    0, 1/160^2;
    ];

eigenvalues = eig(A)

%% Checking Controllability and Observability

Cont = [B A*B A^2*B A^3*B A^4*B];
rank(Cont);

O = [C; C*A; C*A*A; C*A*A*A; C*A*A*A*A];
rank(O);

%% Open loop response
x0 = [ 30000;9500;5000;0.5;0.95];
ref = [0 ; 10];

%% State feedback controller 

%Step 1: Find optimal state feedback gains via LQR (w = v = 0), u = kx
xnom = [5000; 500; 500; 0.5; 0.00005]; 
Qn = diag(1./(xnom.^2));             % penalize relative errors

% scale control penalty similarly (units of u1 = bees/day, u2 = flowers/day)
% choose R so that one "max" action is reasonably expensive:
R = diag([700/(1000^2), 700/(100^2)]); % tune these to increase/decrease aggressiveness
[K,~,~] = lqr(A,B,Qn,R);


% Step 2: Design precompensator
Csel = C([2 3],:);      % only care about number of flowers and flower health
M = -Csel * ((A - B*K)\B);
F = inv(M);     

% Step 3: State Feedback 
G = eye(5); %process noise input matrix, usually I
Q = [1000, 0, 0, 0, 0;
    0, 50, 0, 0, 0;
    0,0,10,0 ,0;
    0, 0, 0,0.00002,0;
    0, 0, 0, 0, 0.00002]; %process noise covariance
R = [1000, 0, 0;
    0, 50, 0;
    0, 0, 0.0002]; %measurement noise variance
[L, Y, CLP] = lqe(A,G,C,Q,R);

A_obs = A-L*C;
B_obs = [B L];
C_obs = eye(5);
D_obs = 0;


ref2 = [10000 ;1]; %I want 10,000 flowers in perfect health

load_system('FinalProjectSim');
Out = sim("FinalProjectSim.slx");
uncontrolled2 = Out.uncontrolled2;
controlled = Out.controlled;

tu = uncontrolled2.time;
yu = uncontrolled2.signals.values;   % (N x 3)

tc = controlled.time;
yc = controlled.signals.values;      % (N x 3)

ee = Out.estimator_error.signals.values;

ci = Out.control_input.signals.values;

%plot for uncontrolled system
figure;
subplot(3,1,1)
plot(tu, yu(:,1), 'LineWidth', 1.4)
ylabel('Total Bees')
title('Uncontrolled System')

subplot(3,1,2)
plot(tu, yu(:,2), 'LineWidth', 1.4)
ylabel('Number of Flowers')

subplot(3,1,3)
plot(tu, yu(:,3), 'LineWidth', 1.4)
ylabel('Flower Health')
xlabel('Time (days)')

%plot for Controlled System
figure;
subplot(3,1,1)
plot(tc, yc(:,1), 'LineWidth', 1.4)
ylabel('Total Bees')
title('Controlled System')

subplot(3,1,2)
plot(tc, yc(:,2), 'LineWidth', 1.4)
ylabel('Number of Flowers')

subplot(3,1,3)
plot(tc, yc(:,3), 'LineWidth', 1.4)
ylabel('Flower Health')
xlabel('Time (days)')

%plot for estimator error
figure;
subplot(3,1,1)
plot(tc, ee(:,1)+ee(:,2), 'LineWidth', 1.4)
ylabel('Total Bees')
title('Estimator Error')

subplot(3,1,2)
plot(tc, ee(:,3), 'LineWidth', 1.4)
ylabel('Total Flowers')

subplot(3,1,3)
plot(tc, ee(:,3), 'LineWidth', 1.4)
plot(tc, ee(:,4), 'LineWidth', 1.4)
ylabel('Flower Health/ Nectar Level')
xlabel('Time (days)')

%plot for control effort
figure;
subplot(2,1,1)
plot(tc, ci(:,1), 'LineWidth', 1.4)
ylabel('Adult Bees')
title('Controller Input')

subplot(2,1,2)
plot(tc, ci(:,2), 'LineWidth', 1.4)
ylabel('Flowers Planted')


