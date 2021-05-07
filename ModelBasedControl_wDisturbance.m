function [dw_dt] = ModelBasedControl_wDisturbance(t,w,M_mat,K_mat,Kc,Kp,Kd,A,B,controller,act)

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
% u = -M*Kc*X + K*X + f
% u = -M_mat*Kc*[w(1);w(2)] + K_mat*[w(1);w(2)] + f;
% dw_dt = A*w + B*u
% dw_dt = A_cl*w + B*f;

if t<5
    f = [0;0];
elseif t>=5 && t<= 5 + (1/5)
    f = [5;0];
else
    f = [0;0];
end

% f = [50*sin(100*t);0];

% act: variable defining the type of actuation in the system
% controller: variable defining the kind of controller used in the system

% Closed Loop State Weighing Matrix
switch controller
    case 'General'
        A_cl = A + [B*(K_mat - M_mat*Kc), zeros(4,2)];;
    case 'PD'
        switch act
            case 'Both'
                A_cl = A + [B*(K_mat - M_mat*Kp), -B*M_mat*Kd];
            case 'Cart'
                A_cl = A + [B*[1,0]*(K_mat - M_mat*Kp), -B*[1,0]*M_mat*Kd];
            case 'Pole'
                A_cl = A + [B*[0,1]*(K_mat - M_mat*Kp), -B*[0,1]*M_mat*Kd];
        end      
end

dw_dt = A_cl*w + B*f;
end