function [dw_dt] = ClosedLoopDynamics_1(t,w,M_mat,K_mat,A,B,Kp,Kd,disturbance)

% State Space Model for a Model Based Control System Closed Loop
% w: State Vector
% dw_dt: First Time Differential of State Vector
% X = [w1;w2];
% X_dot = [w3;w4];
% X_ddot: First Time Differential of X_dot
% [X_dot;X_ddot] <=> [dw_dt1;dw_dt2:dw_dt3:dw_dt4] = dw_dt
% [X;X_dot] <=> [w1;w2;w3;w4] = w
% u: Input Vector to the plant
% f =  any external disturbance force/torque (closed loop control system
% input)
% For Model based Control:
% u = -M*Kp*X -M*Kd*Xdot +  K*X + M*f
% u = -M_mat*Kc*[w(1);w(2)] + K_mat*[w(1);w(2)] + f;
% dw_dt = A*w + B*u
% dw_dt = A_cl*w + B*f;


% Disturbance Force
switch disturbance
    case 'None'
        f = [0;0];
        
    case 'Impulse'
        if t<5
            f = [0;0]; 
        elseif t>=5 && t<= 5 + (1/5)
            f = [5;0];
        else
            f = [0;0];
        end
        
    case 'Harmonic'
        f = [5*sin(10*t);0];
        
    case 'Static'
        f = [5;0];       
end

% act: variable defining the type of actuation in the system
% controller: variable defining the kind of controller used in the system

% Closed Loop State Weighing Matrix
A_cl = A + [B*(K_mat - M_mat*Kp), -B*M_mat*Kd];

dw_dt = A_cl*w + B*f;
end