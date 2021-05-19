function [dw_dt] = ClosedLoopDynamics_2(t,w,M,Kp,Kd,disturbance,n)

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
        f = zeros(n,1);
        
    case 'Impulse'
        f = zeros(n,1);
        if t>=5 && t<= 5 + (1/5)
            f(1) = 200;
        end
        
    case 'Harmonic'
        f = zeros(n,1);
        f(1) = 5*sin(10*t);
        
    case 'Static'
        f = zeros(n,1);
        f(1) = 5;       
end

% Closed Loop State weighing matrix
A_cl = [zeros(n,n),eye(n,n);-Kp,-Kd];

% Control cost matrix for this state space model
B = [zeros(n,n);M\eye(size(M))];

% State Space equation
dw_dt = A_cl*w + B*f;

end