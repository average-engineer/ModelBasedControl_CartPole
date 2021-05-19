clearvars
close all
clc

%% Description
% Regulation of a simple mass with spring attached to a wall
% The mass slides on a frictional surface which provides Columb Damping to
% the system

%% Controller Details
% Controller used: PD
% Control algorithm: Control Partitioning (MODEL BASED CONTROL)

%% Controller Gains
Kp = 16;
Kd = 8;

%% Plant parameters
% Mass
m = 1;
% Columb damping coefficient
b = 1;
% spring stiffness
k = 1;

% DOFs of plant
dof = 1;

% Simulation time vector
dt = 0.01; % step size
t_span = 0:dt:10;

% Initial condition
w_0 = [2;0];

%% Disturbance Force
% variable for deciding the type of disturbance force
dist = 'None'; % None/Impulse/Harmonic/Static

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
        F = 200;
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
            f_dist(1,i) = 5;
        end
end

%% Computing various responses
[~,w1] = ode45(@(t,w1)ClosedLoopDynamics_2(t,w1,m,Kp,Kd,dist,dof),t_span,w_0);

[~,w2] = ode45(@(t,w2)ClosedLoopDynamics_2(t,w2,m,Kp,3*Kd,dist,dof),t_span,w_0);

[~,w3] = ode45(@(t,w3)ClosedLoopDynamics_2(t,w3,m,Kp,0.2*Kd,dist,dof),t_span,w_0);

% Mass position
figure
hold on
plot(t_span,w1(:,1),'linewidth',2)
plot(t_span,w2(:,1),'linewidth',2)
plot(t_span,w3(:,1),'linewidth',2)
grid on
xlabel('Time(s)')
ylabel('Mass Position (m)')
legend('Critically Damped','Overdamped','Underdamped')


