clear
close all

dt = 0.001;
t_f = 5;
n = t_f/dt;

g = 9.81;
rho = 1.29;
m = 0.149;
D = 0.0737;
A = pi*(D/2)^2;
c_d = 0.3;

%%
v_0 = 30;
theta_0 = 45;

v_x = [v_0*cosd(theta_0) zeros(1,n)];
v_y = [v_0*sind(theta_0) zeros(1,n)];

p_x = [zeros(1,n+1)];
p_y = [zeros(1,n+1)];

for i = 1:n
    p_x(i+1) = p_x(i) + v_x(i)*dt;
    p_y(i+1) = p_y(i) + v_y(i)*dt;

    F_d = 0.5*rho*(v_x(i)^2+v_y(i)^2)*c_d*A;
    theta = atan(v_y(i)/v_x(i));

    v_x(i+1) = v_x(i) - F_d/m*cos(theta)*dt;
    v_y(i+1) = v_y(i) - g*dt - F_d/m*sin(theta)*dt;
end

t = 0:dt:t_f;
s_x = v_0*cosd(theta_0)*t;
s_y = v_0*sind(theta_0)*t - 0.5*g*t.^2;

figure(1)
clf
hold on
plot(s_x,s_y)
plot(p_x,p_y)

grid on
axis equal
axis([0 100 0 50])
title('Baseball Trajectory')
xlabel('Distance (m)')
ylabel('Height (m)')
legend('Ideal','Real')
set(gca,'Fontsize',12)

%%
t_f = 10;
n = t_f/dt;

v_ = 1:1:50;
p_ = zeros(1,length(v_));

for j = 1:length(v_)
    v_0 = v_(j);
    theta_0 = 45;
    
    v_x = [v_0*cosd(theta_0) zeros(1,n)];
    v_y = [v_0*sind(theta_0) zeros(1,n)];
    
    p_x = [zeros(1,n+1)];
    p_y = [zeros(1,n+1)];
    
    for i = 1:n
        p_x(i+1) = p_x(i) + v_x(i)*dt;
        p_y(i+1) = p_y(i) + v_y(i)*dt;

        if p_y(i) < 0
            break
        end
    
        F_d = 0.5*rho*(v_x(i)^2+v_y(i)^2)*c_d*A;
        theta = atan(v_y(i)/v_x(i));
    
        v_x(i+1) = v_x(i) - F_d/m*cos(theta)*dt;
        v_y(i+1) = v_y(i) - g*dt - F_d/m*sin(theta)*dt;

    end

    p_(j) = p_x(i);
end

% coefs = polyfit(v_,p_,4);
% q = polyval(coefs,v_);

figure(2)
clf
hold on
plot(v_,v_.^2/g)
plot(v_,p_)
% plot(v_,q)

grid on
title('Baseball Maximum Horizontal Range')
xlabel('Initial Velocity (m/s)')
ylabel('Range (m)')
legend('Ideal','Real')
set(gca,'Fontsize',12)