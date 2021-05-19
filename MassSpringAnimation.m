function MassSpringAnimation(w1,w2,w3)

% Function for animating spring mass system
% The mass will have a dynamic response 
% xc,yc: coordinates of the mass COM
% xp1,yp1: Left hand coordinates of spring
% xp2, yp2: Right hand coordinates of spring

% 3 cases
yc1 = 0;
yc2 = -5;
yc3 = 5;

% Mass block parameters
h = 2; % Height
b = 2; % Width

% Spring parameters
R = 1;
num_coil = 10;

figure
for i = 1:max(size(w1))
    xc1 = w1(i,1); % Position of mass
    xp11 = xc1 + (b/2);
    yp11 = yc1;
    xp12 = 5;
    yp12 = yc1;
    
    xc2 = w2(i,1); % Position of mass
    xp21 = xc2 + (b/2);
    yp21 = yc2;
    xp22 = 5;
    yp22 = yc2;
    
    xc3 = w3(i,1); % Position of mass
    xp31 = xc3 + (b/2);
    yp31 = yc3;
    xp32 = 5;
    yp32 = yc3;
    
    hold on
    %% Using spring creation function
    % Xue-She Wang (2021). Plot 2D Spring (https://github.com/wangxueshe/Plot-2D-Spring-in-Matlab), GitHub. Retrieved May 19, 2021.
    % Create an intance of spring
    spr = Spring(R, num_coil);
    
    [x1, y1] = spr.getSpr([xp12,yp12], [xp11,yp11]);
    [x2, y2] = spr.getSpr([xp22,yp22], [xp21,yp21]);
    [x3, y3] = spr.getSpr([xp32,yp32], [xp31,yp31]);
    h1 = plot(x1,y1,'linewidth',2,'color','b');
    h2 = plot(x2,y2,'linewidth',2,'color','b');
    h3 = plot(x3,y3,'linewidth',2,'color','b');
    
    %% Drawing mass block
    h4 = drawCart(xc1,yc1,h,b,'r');
    h5 = drawCart(xc2,yc2,h,b,'r');
    h6 = drawCart(xc3,yc3,h,b,'r');
    
    pause(0.01)
    delete(h1)
    delete(h2)
    delete(h3)
    delete(h4)
    delete(h5)
    delete(h6)
    
    grid on
    axis([-5,5,-7,7])
    
end