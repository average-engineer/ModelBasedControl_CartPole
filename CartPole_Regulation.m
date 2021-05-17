clearvars
close all
clc
format bank

%% Description
% Regulation problem of Cart with inverted pole system
% For regulation, system can be linearlized about the unstable equilibrium
% point of the pole
% Model based control technique is used where control partitioning method
% is made use of

%% System Parameters
% Cart mass
M = 10;%kg

% Pole mass
m = 2;%kg

% Pole length
L = 4;%meters

g = 9.81;

% Dynamic coefficient matrices
% Mass Matrix
M_mat = [m + M,(m*L)/2;
    (m*L)/2,(m*(L)^2)/3];
% Stiffness Matrix
K_mat = [0,0;
    0,(-m*g*L)/2];

% Coefficient matrices
% Open Loop State weighing matrix
A = [zeros(2,2),eye(2,2);-M_mat\K_mat,zeros(2,2)];

% Control Cost Matrix
B = [zeros(size(M_mat)) ; M_mat\eye(size(M_mat))];

% Acceleration due to gravity
g = 9.81; %m/s^2

% Time Step Size
dt = 0.01;

% Time Vector
t_span = [0:dt:20];

% Initial Conditions
w_0 = [0;0.5;0;0];

%% Disturbance force on the control system
% variable for deciding the type of disturbance force
dist = 'Impulse'; % None/Impulse/Harmonic/Static

switch dist
    case 'None'
        % No external disturbance force on the system
        f_dist = zeros(2,1,length(t_span));
    
    case 'Impulse'
        % The disturbance force is modelled as an unit impulse function force
        % applied horizontally on the cart
        % Resembles a hand flicking the cart
        % There is no disturbance torque given to the pole joint
        
        % Time instant at which force is applied (seconds)
        a = 5;
        
        % Magnitude of disturbance force
        F = 5;
        f_dist = ImpulseForce(t_span,a,F,dt);
        
    case 'Harmonic'
        % Harmonic Disturbance Force
        for i = 1:length(t_span)
            f_dist(:,i) = zeros(2,1);
            f_dist(1,i) = 5*sin(10*t_span(i));
        end
        
    case 'Static'
        for i = 1:length(t_span)
            f_dist(:,i) = zeros(2,1);
            f_dist(1,i) = 200;
        end
end

%% Controller Gains
% PD Controller is made use of 

% Proportional Gain
Kp = [1,5;-5,2];

% Derivative Gain
Kd = [5,1;-1,5];

%% State space model of the closed loop control system
% There were 2 approaches to state space modelling of this control system
% which were thought of by me
% The results for both the methods will be obtained and compared

% First Method:
% Linearized Open Loop Dynamics of Cart Pole
% zdot = Az + Bu
% replace u by u = M(-KpX - KdXdot + fdist) + KX (control law for model
% based regulator design)

[t,w1] = ode45(@(t,w1)ClosedLoopDynamics_1(t,w1,M_mat,K_mat,A,B,Kp,Kd,dist),t_span,w_0);

% Second Method
% The linear closed loop dynamic equation obtained for regulator design
% using control partitioning method:
% X_ddot + KdX_dot + KpX = f_dist

[t,w2] = ode45(@(t,w2)ClosedLoopDynamics_2(t,w2,M_mat,B,Kp,Kd,dist),t_span,w_0);

% Plotting the two types of responses

% Cart Displacement
figure
hold on
plot(t_span,w1(:,1),'linewidth',2)
plot(t_span,w2(:,1),'linewidth',2)
legend('First Method','Second Method')
grid on

% Pole Angle
figure
hold on
plot(t_span,w1(:,2),'linewidth',2)
plot(t_span,w2(:,2),'linewidth',2)
legend('First Method','Second Method')
grid on

% Technically both methods are correct so should overlap


%% Actuator Effort
% Both the Cart and the Pole are assumed to be actuated
for i = 1:length(t_span)
    u(:,i) = f_dist(:,i) + (K_mat - M_mat*Kp)*[w2(i,1);w2(i,2)] - M_mat*Kd*[w2(i,3);w2(i,4)];
    % Cart Actuation (N)
    uCart(i) = u(1,i);
    % Pole Actuation (Nm)
    uPole(i) = u(2,i);
end

figure
hold on
plot(t_span,uCart,'linewidth',2)
plot(t_span,uPole,'linewidth',2)
legend('Cart Actuation (N)','Pole Actuation (Nm)')
grid on
