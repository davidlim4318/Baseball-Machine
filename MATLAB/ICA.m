clear
close all

syms omega

%omega_nls = [8300 8000 5000 11000 5676 20000];
%tau_sts = [0.718 0.274 0.637 0.951 2.6 0.153];
%key = {'775 Size','4260BL Series','NEMA 17 Size','NEO 550 V1.0','NEO 550 V1.1','775 Amazon'};

%omega_nls = [8000 11000 5676 20000 25200];
%tau_sts = [0.274 0.951 2.6 0.153 3.18];
%key = {'4260BL Series','NEO 550 V1.0','NEO 550 V1.1','775 Amazon','Turnigy TrackStar 1/8'};

omega_nls = [8000 5676 20000 25200];
tau_sts = [0.274 2.6 0.153 0.6];
key = {'4260BL Series','NEO 550 V1.1','775 Amazon','Turnigy TrackStar 1/8 (Estimated)'};

color = ['r','b','g','m','c'];
style = ["-","--","-.",":","-"];

figure(1)
subplot(2,1,1)
hold on
subplot(2,1,2)
hold on

for i = 1:length(omega_nls)
    omega_nl = omega_nls(i);
    tau_st = tau_sts(i);
    tau = tau_st*(1-omega/omega_nl);
    %tau_ast = 0.3*tau_st;
    %  tau = piecewise(omega>0 & omega<0.7*omega_nl, 0.3*tau_st, omega>=.7*omega_nl, tau_st*(1-omega/omega_nl));
    P = tau*omega*2*pi/60;
    %P_ast = tau_ast*omega*2*pi/60;
    subplot(2,1,1)
    fplot(tau,[0,omega_nl],'LineWidth',2,'Color',color(i),'LineStyle',style(i))
    %fplot(tau_ast,[0,0.7*omega_nl],'LineWidth',1,'Color',color(i),'LineStyle',style(i))
    subplot(2,1,2)
    fplot(P,[0,omega_nl],'LineWidth',2,'Color',color(i),'LineStyle',style(i))
    %fplot(P_ast,[0,0.7*omega_nl],'LineWidth',1,'Color',color(i),'LineStyle',style(i))
end

subplot(2,1,2)
plot(1500,6.13,'r*')
axis([0 12000 0 400])
title('DC Motor Power vs. Speed','FontWeight','normal')
xlabel('Rotational Speed (rpm)')
ylabel('Max Continuous Power (W)')
legend([key,{'Operational Req.'}])
set(gca,'FontSize',12,'FontName','Cambria Math','FontWeight','normal')

subplot(2,1,1)
axis([0 12000 0 0.9])
plot(1500,0.05,'r*')
title('DC Motor Torque vs. Speed','FontWeight','normal')
xlabel('Rotational Speed (rpm)')
ylabel('Max Continuous Torque (N-m)')
legend([key,{'Operational Req.'}]);
set(gca,'FontSize',12,'FontName','Cambria Math','FontWeight','normal')