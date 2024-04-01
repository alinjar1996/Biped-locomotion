% ReDySim plot_motion module. This module plot joint positions and velocities
% Contibutors: Dr. Suril Shah and Prof S. K. Saha @IIT Delhi

function[]=plot_motion()
disp('------------------------------------------------------------------');
disp('Generating plots for joint motions');

[n]=inputs();
m=n-1;
load statevar.dat;
load timevar.dat;
Y=statevar;T=timevar;
clear statevar;
clear timevar;
FF = 20;
set(0,'DefaultLineLineWidth',1.5)
fh1=figure(1);
set(fh1, 'color', 'white'); % sets the color to white 

plot(T,Y(:,1:3));
set (gca,'fontsize',FF,'fontweight','n','fontname','Euclid','linewidth',0.5,'Box', 'off','TickDir','out' );
hl=legend('X_0','Y_0','Z_0');
set(hl,'Orientation','h','Color', 'none','Box', 'off','Location','best','FontAngle','italic','fontsize',FF,'fontweight','normal','fontname','Euclid','linewidth',0.5)
xlabel('time(s)','FontSize',FF);
ylabel('Base COM (m)','FontSize',FF)
% xlim([0 1]) ;
% set(h,'FontSize',FF);
%subplot(2,2,2)
figure()
plot(T,Y(:,4:6));
% axis([0 1 -100 FF])
set (gca,'fontsize',FF,'fontweight','n','fontname','Euclid','linewidth',0.5,'Box', 'off','TickDir','out' );
hl=legend('\phi_0','\theta_0','\psi_0');
set(hl,'Orientation','h','Color', 'none','Box', 'off','Location','best','FontAngle','italic','fontsize',FF,'fontweight','normal','fontname','Euclid','linewidth',0.5)
xlabel('time(s)','FontSize',FF);
ylabel('Euler angles (rad)','FontSize',FF);
% xlim([0 1]) ;
figure()
plot(T,Y(:,7+m:9+m));
set (gca,'fontsize',FF,'fontweight','n','fontname','Euclid','linewidth',0.5,'Box', 'off','TickDir','out' );
hl=legend('$\dot{X_0}$','$\dot{Y_0}$','$\dot{Z_0}$');
set(hl,'interpreter','latex','Orientation','h','Color', 'none','Box', 'off','Location','best','FontAngle','italic','fontsize',FF,'fontweight','normal','fontname','Euclid','linewidth',0.5)
xlabel('time(s)','FontSize',FF);
ylabel('Rates of Base COM (m/s)','FontSize',FF)
% xlim([0 1]) ;
% set(h,'FontSize',FF);
figure()
plot(T,Y(:,10+m:12+m));
% axis([0 1 -100 FF])
set (gca,'fontsize',FF,'fontweight','n','fontname','Euclid','linewidth',0.5,'Box', 'off','TickDir','out' );
hl=legend('$\dot{\phi_0}$','$\dot{\theta_0}$','$\dot{\psi_0}$');
set(hl,'interpreter','latex','Orientation','h','Color', 'none','Box', 'off','Location','best','FontAngle','italic','fontsize',FF,'fontweight','normal','fontname','Euclid','linewidth',0.5)
xlabel('time(s)','FontSize',FF);
ylabel('Rates of Euler angles (rad/s)','FontSize',FF);

fh = figure(2); % returns the handle to the figure object
set(fh, 'color', 'white'); % sets the color to white 
%subplot(1,2,1)
plot(T,Y(:,7:m+6));
set (gca,'fontsize',FF,'fontweight','n','fontname','Euclid','linewidth',0.5,'Box', 'off','TickDir','out' );
l1=legend('\theta_1','\theta_2','\theta_3','\theta_4','\theta_5','\theta_6','\theta_7', '\theta_8');
%set(l1,'Orientation','h','Location','best','Color', 'none','Box', 'off','FontAngle','italic','fontsize',FF,'fontweight','normal','fontname','Euclid','linewidth',0.5)
set(l1, 'NumColumns',4)
xlabel('time(s)','FontSize',FF);
ylabel('Joint angle (rad)','FontSize',FF);

figure
plot(T,Y(:,m+13:2*m+12))
set (gca,'fontsize',FF,'fontweight','n','fontname','Euclid','linewidth',0.5,'Box', 'off','TickDir','out');
l1=legend('$\dot{\theta_1}$','$\dot{\theta_2}$','$\dot{\theta_3}$','$\dot{\theta_4}$','$\dot{\theta_5}$','$\dot{\theta_6}$','$\dot{\theta_7}$', '$\dot{\theta_8}$');
%set(l1,'interpreter','latex','Orientation','h','Location','best','Color', 'none','Box', 'off','FontAngle','italic','fontsize',FF,'fontweight','normal','fontname','Euclid','linewidth',0.5)
set(l1, 'NumColumns',4, 'interpreter','latex')
xlabel('time(s)','FontSize',FF);
ylabel('Rates of joint angle (rad/s)','FontSize',FF);
