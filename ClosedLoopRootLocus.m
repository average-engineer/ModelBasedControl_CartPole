function ClosedLoopRootLocus(A,B,K_mat,M_mat,Kp,Kd,act)

% Function for plotting the root locus of the closed loop model based
% control with PD controller
% Kp is the varied proportional gain matrix
% Kd is the varied derivative gain matrix
% A is the state weighing matrix of the open loop plant
% B is the control cost matrix of the open loop plant
% act is the variable defining the type of actuation in the system
% M_mat is the mass matrix of the dynamic system of equations of the plant
% K_mat is the stiffness matrix of the dynamic system of equations of the plant

count = 0;
for ii = 1:length(Kp)
    switch act
        case 'Both'
            A_cl(:,:,ii) = A + [B*(K_mat - M_mat*Kp(:,:,ii)), -B*M_mat*Kd(:,:,ii)];
        case 'Cart'
            A_cl(:,:,ii) = A + [B*[1,0]*(K_mat - M_mat*Kp(:,:,ii)), -B*[1,0]*M_mat*Kd(:,:,ii)];
        case 'Pole'
            A_cl(:,:,ii) = A + [B*[0,1]*(K_mat - M_mat*Kp(:,:,ii)), -B*[0,1]*M_mat*Kd(:,:,ii)];
    end
    
    % Eigen Values (Poles) of the closed loop system
    eigValues(:,ii) = eig(A_cl(:,:,ii));
    Pole1(ii) = eigValues(1,ii);
    Pole2(ii) = eigValues(2,ii);
    Pole3(ii) = eigValues(3,ii);
    Pole4(ii) = eigValues(4,ii);   
    
    % Values of controller gain matrices for which all the eigenvalues of
    % the closed loop system are stable
    if real(Pole1(ii)) < 0 && real(Pole2(ii)) < 0 && real(Pole3(ii)) < 0 && real(Pole4(ii)) < 0
        count = count + 1;
    end
end

if count ~= 0
    fprintf('There are controller gain matrices for which the closed loop system is stable')
end

figure
hold on
plot(real(Pole1),imag(Pole1),'o')
plot(real(Pole2),imag(Pole2),'o')
plot(real(Pole3),imag(Pole3),'o')
plot(real(Pole4),imag(Pole4),'o','color','k')
legend('Pole 1' , 'Pole 2', 'Pole 3', 'Pole 4')
title('Root Locus')
xlabel('Real Axis')
ylabel('Imaginary Axis')
grid on

% figure
% subplot(4,1,1)
% plot(real(Pole1),'o')
% grid on
% title('1st Pole')
% subplot(4,1,2)
% plot(real(Pole2),'o')
% grid on
% title('2nd Pole')
% subplot(4,1,3)
% plot(real(Pole3),'o')
% grid on
% title('3rd Pole')
% subplot(4,1,4)
% plot(real(Pole4),'o')
% grid on
% title('4th Pole')
