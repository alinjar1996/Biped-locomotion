% ReDySim plot_tor module. This module plots the generalized forces
% Contibutors: Dr. Suril Shah and Prof S. K. Saha @IIT Delhi

function[]=plot_tor()
disp('------------------------------------------------------------------');
disp('Generating plots for joint torques');
%Plots
[n]=inputs();

load tor.dat;
load timevar.dat;
S=timevar;
tau=tor;
clear tor;
clear timevar;

S = S(3:length(S));
tau = tau(3:length(tau),:);

set(0,'DefaultLineLineWidth',1.5)
% set(0,'DefaultLineLineWidth',1.5,'DefaultAxesColorOrder',[0 0 0],...
%    'DefaultAxesLineStyleOrder','-|--|:|-.', 'DefaultLineMarkerSize',1.5)
% whitebg('w') %create a figure with a white color scheme
wpass = 0.99999999999;
FF = 30;
for i =1:8
    % tau_filt(:,i)= lowpass(tau(:,i),wpass);

    tau_filt(:,i) = tau(:,i);
end

% if n-1 <= 6
%     fh1=figure(5);
%     set(fh1, 'color', 'white'); % sets the color to white
%     plot(S,tau)
%     set (gca,'fontsize',10,'fontweight','n','fontname','Euclid','linewidth',0.5,'Box', 'off','TickDir','out' );
%     hl=legend('\tau_1','\tau_2','\tau_3','\tau_4','\tau_5','\tau_6','\tau_7','\tau_8');
%     set(hl,'Orientation','h','Color', 'none','Box', 'off','Location','south','FontAngle','italic','fontsize',10,'fontweight','normal','fontname','Euclid','linewidth',0.5)
%     xlabel('time(s)','FontSize',10);
%     ylabel('Joint torque (Nm)','FontSize',10);
% else
    fh1=figure();
 %   subplot(1,2,1)
    set(fh1, 'color', 'white'); % sets the color to white
    
    % List a bunch of markers; they will be selected in 
% order and then the selection will start again if 
% there are more lines than markers.  
  %  markers = {'o','+','*','s','d','v','>','h'};
  markers = {'none','s'};
% List a bunch of colors; like the markers, they 
% will be selected circularly. 
   colors = {'b','r','k','c','m',[0.6,0.2,0.2], [0.5,0.5,0], [0.33,0.33,0.33]};
 % Same with line styles
  % linestyle = {'-','--','-.',':'};
  linestyle = {'-','-.'};
% this function will do the circular selection
% Example:  getprop(colors, 7) = 'b'
   getFirst = @(v)v{1}; 
   getprop = @(options, idx)getFirst(circshift(options,-idx+1));

for j = 1:8
    plot(S,tau_filt(:,j),...
        'Marker',getprop(markers,j),...
        'color',getprop(colors,j),...
        'linestyle',getprop(linestyle,j));
    hold on

end
    set (gca,'fontsize',FF,'fontweight','n','fontname','Euclid','linewidth',0.5,'Box', 'off','TickDir','out' );
    hl=legend('\tau_1','\tau_2','\tau_3','\tau_4','\tau_5','\tau_6', '\tau_7', '\tau_8');
    set(hl, 'NumColumns',4)
    %set(hl,'Orientation','h','Color', 'none','Box', 'off','Location','south','FontAngle','italic','fontsize',FF,'fontweight','normal','fontname','Euclid','linewidth',0.5)
    xlabel('Time(s)','FontSize',FF);
    ylabel('Joint torque (N-m)','FontSize',FF);
    
%     subplot(1,2,2)
%     set(fh1, 'color', 'white'); % sets the color to white
%     plot(S,tau(:,7:n-1))
%     set (gca,'fontsize',10,'fontweight','n','fontname','Euclid','linewidth',0.5,'Box', 'off','TickDir','out' );
%     hl=legend('\tau_7','\tau_8','\tau_9','\tau_1_0','\tau_1_1','\tau_1_2','\tau_1_3');
%     set(hl,'Orientation','h','Color', 'none','Box', 'off','Location','south','FontAngle','italic','fontsize',10,'fontweight','normal','fontname','Euclid','linewidth',0.5)
%     xlabel('time(s)','FontSize',10);
%     ylabel('Joint torque (Nm)','FontSize',10);    
%end
