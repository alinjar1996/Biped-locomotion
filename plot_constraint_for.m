% ReDySim plot_tor module. This module plots the generalized forces
% Contibutors: Dr. Suril Shah and Prof S. K. Saha @IIT Delhi

function[]=plot_constraint_for()
disp('------------------------------------------------------------------');
disp('Generating plots for joint constraint forces');
%Plots
[n]=inputs();

load constraintforce.dat;
load timevar.dat;
S=timevar;
tauf=constraintforce;
clear tor;
clear timevar;


S = S(3:length(S));
tauf = tauf(3:length(tauf),:);

set(0,'DefaultLineLineWidth',1.5)
% set(0,'DefaultLineLineWidth',1.5,'DefaultAxesColorOrder',[0 0 0],...
%    'DefaultAxesLineStyleOrder','-|--|:|-.', 'DefaultLineMarkerSize',1.5)
% whitebg('w') %create a figure with a white color scheme

FF=40;
% if n-1 <= 6
%     fh1=figure(5);
%     set(fh1, 'color', 'white'); % sets the color to white
%     plot(S,tauf)
%     set (gca,'fontsize',10,'fontweight','n','fontname','Euclid','linewidth',0.5,'Box', 'off','TickDir','out' );
%     hl=legend('F_1','F_2','F_3','F_4','F_5','F_6','F_7','F_8');
%     set(hl,'Orientation','h','Color', 'none','Box', 'off','Location','south','FontAngle','italic','fontsize',10,'fontweight','normal','fontname','Euclid','linewidth',0.5)
%     xlabel('time(s)','FontSize',10);
%     ylabel('Joint Reaction Force (N)','FontSize',10);
%else
   wpass = 0.999999;
   for i =1:8
     %tauf_filt(:,i)= lowpass(tauf(:,i),wpass);
     tauf_filt(:,i)= tauf(:,i);
   end
%'FontAngle','italic',
    fh1=figure();
 %   subplot(1,2,1)
    set(fh1, 'color', 'white'); % sets the color to white

 markers = {'none','s'};
% List a bunch of colors; like the markers, they 
% will be selected circularly. 
   colors = {'b','r','k','c','m',[0,0.7,0.3], [0.6,0.2,0.2], [0.33,0.33,0.33]};
 % Same with line styles
  % linestyle = {'-','--','-.',':'};
  linestyle = {'-','--'};
% this function will do the circular selection
% Example:  getprop(colors, 7) = 'b'
   getFirst = @(v)v{1}; 
   getprop = @(options, idx)getFirst(circshift(options,-idx+1));

for j = 1:8
    plot(S,tauf_filt(:,j),...
        'Marker',getprop(markers,j),...
        'color',getprop(colors,j),...
        'linestyle',getprop(linestyle,j));
    hold on

end
 
    set (gca,'fontsize',FF,'fontweight','n','fontname','Euclid','linewidth',0.5,'Box', 'off','TickDir','out' );
    hl=legend('$\overline{f}_{r1}$','$\overline{f}_{r2}$','$\overline{f}_{r3}$','$\overline{f}_{r4}$','$\overline{f}_{r5}$','$\overline{f}_{r6}$', '$\overline{f}_{r7}$', '$\overline{f}_{r8}$','interpreter','latex');
   % set(hl,'Orientation','h','Color', 'none','Box', 'off','Location','north','fontsize',FF,'fontweight','normal','fontname','Euclid','linewidth',0.5)
    set(hl, 'NumColumns',8)
    xlabel('Time(s)','FontSize',FF);
    ylabel('JARF (N)','FontSize',FF);
    
%     subplot(1,2,2)
%     set(fh1, 'color', 'white'); % sets the color to white
%     plot(S,tau(:,7:n-1))
%     set (gca,'fontsize',10,'fontweight','n','fontname','Euclid','linewidth',0.5,'Box', 'off','TickDir','out' );
%     hl=legend('F_7','F_8','F_9','F_1_0','F_1_1','F_1_2','F_1_3');
%     set(hl,'Orientation','h','Color', 'none','Box', 'off','Location','south','FontAngle','italic','fontsize',10,'fontweight','normal','fontname','Euclid','linewidth',0.5)
%     xlabel('time(s)','FontSize',10);
%     ylabel('Joint torque (Nm)','FontSize',10);    
%end
