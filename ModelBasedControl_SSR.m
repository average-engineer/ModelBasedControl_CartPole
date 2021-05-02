function [dw_dt] = ModelBasedControl_SSR(t,w,M_mat,K_mat,Kc,A,B)

% State Space Model fora Model Based Control System Closed Loop
% w: State Vector
% dw_dt: First Time Differential of State Vector
% X = [w1;w2];
% X_dot = [w3;w4];
% X_ddot: First Time Differential of X_dot
% [X_dot;X_ddot] <=> [dw_dt1;dw_dt2:dw_dt3:dw_dt4] = dw_dt
% [X;X_dot] <=> [w1;w2;w3;w4] = w
% u: Input Vector
% For Model based Control:
% u = -M*Kc*X + K*X
% u = -M_mat*Kc*[w(1);w(2)] + K_mat*[w(1);w(2)];
% dw_dt = A*w + B*u;

% Closed Loop State Weighing Matrix
A_cl = A + [B*(K_mat - M_mat*Kc), zeros(4,2)];
% A_cl = A + [B*(K_mat - M_mat*Kp), -B*M_mat*Kd];
dw_dt = A_cl*w;
end