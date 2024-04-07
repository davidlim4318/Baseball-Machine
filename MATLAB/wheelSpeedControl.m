clear

J = 0.06;
h = 0.001;
c = 0.1;

G_c = tf(1,[J c]);
G_z = c2d(G_c,h,'zoh');

rlocaxis = 1.5*[-1 1 -1 1];

G = G_z;

% %% Plant with Delay
% P_delay = tf(1,[1 zeros(1,d/h)],h);
% 
% G = G_z*P_delay;

figure(1)
subplot(2,5,1); rlocus(G); axis(rlocaxis); axis equal; shg
subplot(2,5,6); bode(G); shg

%% Lag Controller 1
omega_target = 0.2;
alpha = 0.01;
p = omega_target*sqrt(alpha);
z = omega_target/sqrt(alpha);

D_lag_1 = tf([1 z*h-1],[1 p*h-1],h);

L1 = G*D_lag_1;
figure(1)
subplot(2,5,2); rlocus(L1); axis(rlocaxis); axis equal; shg
subplot(2,5,7); bode(L1); shg

% %% Low Pass Filter 1
% omega_c = 10;
% F_lpf = tf(omega_c*h,[1 omega_c*h-1],h);
% 
% L = L*F_lpf;
% 
% figure(2)
% subplot(2,3,2); rlocus(L); axis(rlocaxis); axis equal; shg
% subplot(2,3,5); bode(L); shg

% %% Lag Controller
% omega_target = 0.1;
% alpha = 10;
% p = omega_target/sqrt(alpha);
% z = omega_target*sqrt(alpha);
% 
% D_lag_1 = tf([1 z*h-1],[1 p*h-1],h);
% 
% L = G_P*D_lead_1*D_lag_1;
% 
% figure(1)
% subplot(2,5,3); rlocus(L); axis(rlocaxis); axis equal; shg
% subplot(2,5,8); bode(L); shg

%% Adjust Gain
omega_target = 2;
K = 1/abs(freqresp(L1,omega_target))

poles = rlocus(L1,K);
figure(1)
subplot(2,5,4); rlocus(L1); hold on; 
plot(real(poles),imag(poles),'r*'); hold off; axis(rlocaxis); axis equal

%% Closed Loop 1
D = K*D_lag_1;
L = D*G;
T = L/(1+L);
if any(abs(pole(T)) > 1)
    error('Closed loop system is unstable!')
end

tlim = [0 3];
figure(1)
subplot(2,5,5); step(T,tlim); shg
subplot(2,5,9); bode(T); shg

S_u = D/(1+G*D);
subplot(2,5,10); step(S_u,tlim); shg

%% Difference Equation 1
syms z Theta(z) E(z)
[num, den] = tfdata(D);
eqn = simplify( poly2sym(cell2mat(num), z)*E(z) == poly2sym(cell2mat(den), z)*Theta(z) );
eqn_diff = vpa(iztrans(eqn))

%% Ad Hoc PID
PID = pid(0.04,0,0.032,0,h);
L_PID = G*PID;

figure(3)
subplot(2,1,1)
poles = rlocus(L_PID,
rlocus(L_PID); hold on; 
plot(real(poles),imag(poles),'r*'); hold off; axis(rlocaxis); axis equal

subplot(2,1,2)
T_PID = L_PID/(1+L_PID);
step(T_PID,tlim); shg

%% PID Difference Equation
syms z Theta(z) E(z) theta_k x_k
[num, den] = tfdata(PID);
eqn = simplify( poly2sym(cell2mat(num), z)*E(z) == poly2sym(cell2mat(den), z)*Theta(z) );
eqn_diff = vpa(iztrans(eqn))

