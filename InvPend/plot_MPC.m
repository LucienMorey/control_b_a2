%% Plotting Results when usin MPC
% Create your MPC plot

clear

load('data_MPC.mat');   % Data stored after simulation

xc=y_n(:,1);            % Noisy measurement of cart position
theta=y_n(:,2)*180/pi;  % Noisy measurement of pendulum angle

xc_hat=x_hat(:,1);
theta_hat=x_hat(:,3)*180/pi;

xc_ref=x_ref(:,1);

figure(2021)
subplot(311)
plot(time,u)
ylabel('$u~(Nm)$','fontsize',16,'interpreter','latex')
hold on
plot([0 60], [-15 -15],'--k')
plot([0 60], [15 15],'--k')
yticks([-25 -10 0 10 25])
hold off
grid
axis([0 60 -30 30])
title('MPC','fontsize',16,'interpreter','latex')

subplot(312)
plot(time,xc,time,xc_hat,time,xc_ref,'--k')
legend('$x_c$','$\hat{x}_c$','$x_c^{\star}$','fontsize',16,'interpreter','latex')
ylabel('$x_c~ (m)$','fontsize',16,'interpreter','latex')
grid
subplot(313)
plot(time,theta,time,theta_hat)
ylabel('$\theta~ (deg)$','fontsize',16,'interpreter','latex')
xlabel('Time (s)','fontsize',16,'interpreter','latex')
legend('$\theta(t)$','$\hat{\theta}(t)$','fontsize',16,'interpreter','latex')
grid

