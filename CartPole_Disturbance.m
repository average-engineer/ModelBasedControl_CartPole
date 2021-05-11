clearvars
close all
clc
format bank
%% Model Based Control of Inverted Cart Pole System with disturbance force present
% For an external disturbance, the closed loop control system will no
% longer behave like an unforced system
% the disturbance force will be the control system input and will have some
% effect on the dynamics of the closed loop control system

%% System Parameters
% Cart mass
M = 10;%kg

% Pole mass
m = 2;%kg

% Pole length
L = 4;%meters

% Acceleration due to gravity
g = 9.81; %m/s^2

% Time Step Size
dt = 0.01;

% Time Vector
t_span = [0:dt:50];

% Initial Conditions
w_0 = [0;0.5;0;0];

%% Disturbance Force
% The disturbance force is modelled as an unit impulse function force
% applied horizontally on the cart
% Resembles a hand flicking the cart
% There is no disturbance torque given to the pole joint

% Time instant at which force is applied (seconds)
a = 5;
    
% Magnitude of disturbance force 
F = 5;

% Impulse Disturbance force
% f_dist = ImpulseForce(t_span,a,F,dt);

% Harmonic Disturbance Force
% for i = 1:length(t_span)
%     f_dist(:,i) = zeros(2,1);
%     f_dist(1,i) = 5*sin(10*t_span(i));
% end

% Static Disturbance
for i = 1:length(t_span)
    f_dist(:,i) = zeros(2,1);
    f_dist(1,i) = 5;
end

%% Type of actuation provide to Cart Pole plant
% Both cart and pole joint are actuated
act = 'Both';

%% Dynamic coefficient matrices
% Mass Matrix
M_mat = [m + M,(m*L)/2;
    (m*L)/2,(m*(L)^2)/3];
% Stiffness Matrix
K_mat = [0,0;
    0,(-m*g*L)/2];
% State Weighing matrix
A = [zeros(2,2),eye(2,2);-M_mat\K_mat,zeros(2,2)];
% Controller Cost matrix
switch act
    case 'Both'
        B = [zeros(size(M_mat)) ; M_mat\eye(size(M_mat))];
        D = zeros(2,2);
        
    case 'Pole'
        B = [0;0;(-6)/(L*(4*M + m));(12*(M + m))/((m*(L^2))*(4*M + m))];
        D = zeros(2,1);
        
    case 'Cart'
        B = [0;0;(4)/(4*M + m);(-6)/(4*M*L + m*L)];
        D = zeros(2,1);
end

%% Type of controller used
controller = 'PD';
%% Controller Gains
% The cart horizontal and pole angular positions and velocities are fedback
% to the controller 

% General Controller Gain Matrices
Kc = [100,100;
    100,100];

% Proportional Gain Matrix : Kp 
% Proportional Gain Matrix
Kp = [1,5;
     -5,2];
% Derivative Gain Matrix : Kd
Kd = [5,1;
    -1,5];
%% Dynamic Response of the Closed Loop System
[t,w] = ode45(@(t,w)ModelBasedControlSSR_wDisturbance(t,w,M_mat,K_mat,Kc,Kp,Kd,A,B,controller,act), t_span, w_0);

% Closed Loop Control System dynamics and control law
switch act
    case 'Cart'
        A_cl = A + [B*[1,0]*(K_mat - M_mat*Kp), -B*[1,0]*M_mat*Kd];
        % Controller/Actuator Effort
        for i = 1:max(size(w))
            u1 = -M_mat*Kd*[w(i,3);w(i,4)] + (K_mat - M_mat*Kp)*[w(i,1);w(i,2)];
            u(:,i) = [1,0]*u1;
        end
        
        % Plotting Actuator Effort
        figure
        plot(t_span,u)
        grid on
        
    case 'Pole'
        A_cl = A + [B*[0,1]*(K_mat - M_mat*Kp), -B*[0,1]*M_mat*Kd];
        % Controller/Actuator Effort
        for i = 1:max(size(w))
            u1 = -M_mat*Kd*[w(i,3);w(i,4)] + (K_mat - M_mat*Kp)*[w(i,1);w(i,2)];
            u(:,i) = [0,1]*u1;
        end
        
        % Plotting Actuator Effort
        figure
        plot(t_span,u)
        grid on
        
    case 'Both'
        A_cl = A + [B*(K_mat - M_mat*Kp), -B*M_mat*Kd];
        % Controller/Actuator Effort
        for i = 1:max(size(w))
            u(:,i) = -M_mat*Kd*[w(i,3);w(i,4)] + (K_mat - M_mat*Kp)*[w(i,1);w(i,2)] + f_dist(:,i);
            uCart(i) = u(1,i);
            uPole(i) = u(2,i);
        end
        
        figure
        plot(t_span,uCart,'linewidth',2)
        grid on
        title('Cart Actuation Force (N)')
        
        figure
        plot(t_span,uPole,'linewidth',2)
        grid on
        title('Pole Joint Actuation Torque (N-m)')
end

% Closed Loop Eigen-values
ClosedLoopPoles = eig(A_cl)

%% Response of the Closed Loop Control System
figure
hold on
plot(t_span,w(:,1),'linewidth',2)
plot(t_span,w(:,2),'linewidth',2)
legend('Cart Horizontal Position','Pole Angular Position')
grid on

%% Distrubance Force
figure
plot(t_span,f_dist(1,:),'linewidth',2)
title('Disturbance Force (N)')
xlabel('Time(s)')
grid on
axis([0,50,-F - 5, F + 5])