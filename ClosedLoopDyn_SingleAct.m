function [dwdt] = ClosedLoopDyn_SingleAct(t,w,Kp,Kd,n,A,B,M,C,K)

% Function for state space modelling of the Cart Pole system which is
% actuated only at the cart and Model Based Control Algorithm is applied
% for its Regulation

% Acl = [zeros(n,n),eye(n,n);
%       -[0;1]*Kp,-[0;1]*Kd];
% dwdt = Acl*w;

% Input Force
% u = [[1,0]*(M*(-Kp*[w(1);w(2)] - Kd*[w(3);w(4)]) + C*([w(3);w(4)]) + K*([w(1);w(2)]));0];

dwdt = A*w + B*u;
end