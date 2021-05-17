function [dw_dt] = ClosedLoopDynamics_2(t,w,M,B,Kp,Kd,disturbance)

% State Space model of the closed loop dynamic equation obtained for
% control partitioning in model based control design of a Regulator

% X_ddot + KdX_dot + KpX = inv(M)*f_dist

% dw_dt = A_cl*w + B*u
% u = f_dist (external disturbance force on the system)
% A_cl = closed loop state weighing matrix
% B = Control cost matrix

% Disturbance Force
switch disturbance
    case 'None'
        f = [0;0];
        
    case 'Impulse'
        if t<5
            f = [0;0]; 
        elseif t>=5 && t<= 5 + (1/5)
            f = [200;0];
        else
            f = [0;0];
        end
        
    case 'Harmonic'
        f = [5*sin(10*t);0];
        
    case 'Static'
        f = [5;0];       
end

% Closed Loop State weighing matrix
A_cl = [zeros(size(Kp)),eye(size(Kp));-Kp,-Kd];

% Control cost matrix for this state space model
B = [zeros(2,2);M\eye(2,2)];

% State Space equation
dw_dt = A_cl*w + B*f;

end