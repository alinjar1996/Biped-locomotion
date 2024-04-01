% ReDySim trajectory module. The desired indpendent joint trejectories are 
% enterd here
% Contibutors: Dr. Suril Shah and Prof S. K. Saha @IIT Delhi

function [thh dthh ddthh xh yh zh dxh dyh dzh xt1 yt1 zt1]=trajectory(s)

% %Planar biped-In this program hip trjectories are found using IPM whlie joint
% %trejectories by direct relation ships
%Trejectories 2

Ts=0.55*2;  %time interval for one step
%shank and thigh length and angles
%sl2=0.5;tl2=0.5;tl1=0.5;sl1=0.5;
sl2=0.1;tl2=0.108;tl1=0.108;sl1=0.1;

%xi=-0.15;zc=0.96;

xi = -0.03; zc =0.13;

gg=9.81;w=sqrt(gg/zc);
hf=0.03;%max foot-tip height
Sl=0.06;%stride
%hf=0.01;%max foot-tip height
%Sl=0.04;%stride
wf=pi/Ts;
%Step size, cycle time, and max lift of the foot
% Biped Trajectories
% Main for loop starts
ti=s;
% i=1;
% for ti=0:step:Tfi
if rem(ti,Ts)~= 0
    t=rem(ti,Ts);
    chk=ceil(ti/Ts);
elseif ti==0
    t=0;
    chk=1;
else
    t=Ts;
    chk=ti/Ts;
end

%Leg 1 foot trajectory generation
%X, dX, ddX
xt1=-Sl*cos(wf*t);
dxt1=wf*Sl*sin(wf*t);
ddxt1=wf*wf*Sl*cos(wf*t);
%Z, dZ, ddZ
zt1=(hf/2)*(1-cos(2*wf*t));
dzt1=(2*wf)*(hf/2)*(sin(2*wf*t));
ddzt1=(2*wf)*(2*wf)*(hf/2)*(cos(2*wf*t));

%Leg 2 foot trajectory generation
%X, dX, ddX
xt2=0;
dxt2=0;
ddxt2=0;
%Z, dZ, ddZ
zt2=0;
dzt2=0;
ddzt2=0;

%Hip trajectory generation
%X, dX, ddX
dxi=((1+exp(w*Ts))/(1-exp(w*Ts)))*xi*w;
Co1=0.5*(xi+(dxi/w));Co2=0.5*(xi-(dxi/w));
xh=Co1*exp(w*t)+Co2*exp(-w*t);
dxh=w*Co1*exp(w*t)-w*Co2*exp(-w*t);
ddxh=w*w*(Co1*exp(w*t)+Co2*exp(-w*t));
%Z, dZ, ddZ
zh=zc;
dzh=0;
ddzh=0;

yh=0;
dyh=0;
ddyh=0;

yt1=0;dyt1=0;ddyt1=0;
yt2=0;dyt2=0;ddyt2=0;

%lo1
lo1s=(xh-xt2)*(xh-xt2)+(yh-yt2)*(yh-yt2)+(zh-zt2)*(zh-zt2);
lo1=sqrt(lo1s);
dlo1=((xh-xt2)*(dxh-dxt2)+(yh-yt2)*(dyh-dyt2)+(zh-zt2)*(dzh-dzt2))/lo1;
ddlo1=((xh-xt2)*(ddxh-ddxt2)+(yh-yt2)*(ddyh-ddyt2)+(zh-zt2)*(ddzh-ddzt2)+...
    (dxh-dxt2)*(dxh-dxt2)+(dyh-dyt2)*(dyh-dyt2)+(dzh-dzt2)*(dzh-dzt2)-dlo1*dlo1)/lo1;
%alp4,th4
alp4=acos((lo1*lo1-tl1*tl1-sl1*sl1)/(2*sl1*tl1));
dalp4=-(lo1*dlo1)/(sin(alp4)*sl1*tl1);
ddalp4=-(lo1*ddlo1+dlo1*dlo1)/(sin(alp4)*sl1*tl1)-cot(alp4)*dalp4*dalp4;
th4=-alp4;
dth4=-dalp4;
ddth4=-ddalp4;
%gama1
gama1=asin(tl1*sin(alp4)/lo1);
dgama1=(tl1*cos(alp4)*dalp4-dlo1*sin(gama1))/(lo1*cos(gama1));
ddgama1=(tl1*(cos(alp4)*ddalp4-sin(alp4)*dalp4*dalp4)+...
    (lo1*dgama1*dgama1-ddlo1)*sin(gama1)-2*dlo1*cos(gama1)*dgama1)/(lo1*cos(gama1));
%bita1
bita1=asin((xh-xt2)/lo1);
dbita1=((dxh-dxt2)/(lo1*cos(bita1)))-(dlo1/lo1)*tan(bita1);
ddbita1=((ddxh-ddxt2)/(lo1*cos(bita1)))-((ddlo1/lo1)-dbita1*dbita1)*tan(bita1)-2*(dlo1/lo1)*dbita1;
%th2
th2=alp4-(gama1+bita1);
dth2=dalp4-(dgama1+dbita1);
ddth2=ddalp4-(ddgama1+ddbita1);
%th5
th5=pi/2+bita1+gama1;
dth5=dbita1+dgama1;
ddth5=ddbita1+ddgama1;

%Leg 2
%lo2
lo2s=(xh-xt1)*(xh-xt1)+(yh-yt1)*(yh-yt1)+(zh-zt1)*(zh-zt1);
lo2=sqrt(lo2s);
dlo2=((xh-xt1)*(dxh-dxt1)+(yh-yt1)*(dyh-dyt1)+(zh-zt1)*(dzh-dzt1))/lo2;
ddlo2=((xh-xt1)*(ddxh-ddxt1)+(yh-yt1)*(ddyh-ddyt1)+(zh-zt1)*(ddzh-ddzt1)+...
    (dxh-dxt1)*(dxh-dxt1)+(dyh-dyt1)*(dyh-dyt1)+(dzh-dzt1)*(dzh-dzt1)-dlo2*dlo2)/lo2;
%alp10,th10
alp10=acos((lo2*lo2-tl2*tl2-sl2*sl2)/(2*sl2*tl2));
dalp10=-(lo2*dlo2)/(sin(alp10)*sl2*tl2);
ddalp10=-(lo2*ddlo2+dlo2*dlo2)/(sin(alp10)*sl2*tl2)-cot(alp10)*dalp10*dalp10;
th10=-alp10;
dth10=-dalp10;
ddth10=-ddalp10;
%gama2
gama2=asin(tl2*sin(alp10)/lo2);
dgama2=(tl2*cos(alp10)*dalp10-dlo2*sin(gama2))/(lo2*cos(gama2));
ddgama2=(tl2*(cos(alp10)*ddalp10-sin(alp10)*dalp10*dalp10)+...
    (lo2*dgama2*dgama2-ddlo2)*sin(gama2)-2*dlo2*cos(gama2)*dgama2)/(lo2*cos(gama2));
%bita2
bita2=asin((xh-xt1)/lo2);
dbita2=((dxh-dxt1)/(lo2*cos(bita2)))-(dlo2/lo2)*tan(bita2);
ddbita2=((ddxh-ddxt1)/(lo2*cos(bita2)))-((ddlo2/lo2)-dbita2*dbita2)*tan(bita2)-2*(dlo2/lo2)*dbita2;
%th8
th8=alp10-(gama2+bita2);
dth8=dalp10-(dgama2+dbita2);
ddth8=ddalp10-(ddgama2+ddbita2);
%th11
th11=pi/2+bita2+gama2;
dth11=dbita2+dgama2;
ddth11=ddbita2+ddgama2;

if rem(chk,2)==1
    ath2=th2;
    ath4=th4;
    ath5=th5;
    ath8=th8;
    ath10=th10;
    ath11=th11;
    dath2=dth2;
    dath4=dth4;
    dath5=dth5;
    dath8=dth8;
    dath10=dth10;
    dath11=dth11;
    ddath2=ddth2;
    ddath4=ddth4;
    ddath5=ddth5;
    ddath8=ddth8;
    ddath10=ddth10;
    ddath11=ddth11;
else
    ath2=th8;
    ath4=th10;
    ath5=th11;
    ath8=th2;
    ath10=th4;
    ath11=th5;
    dath2=dth8;
    dath4=dth10;
    dath5=dth11;
    dath8=dth2;
    dath10=dth4;
    dath11=dth5;
    ddath2=ddth8;
    ddath4=ddth10;
    ddath5=ddth11;
    ddath8=ddth2;
    ddath10=ddth4;
    ddath11=ddth5;
end


%planar
thh2=ath2;  dthh2=dath2;   ddthh2=ddath2;
thh3=ath4; dthh3=dath4;  ddthh3=ddath4;
thh4=ath5; dthh4=dath5;  ddthh4=ddath5;

thh5=ath8;  dthh5=dath8;   ddthh5=ddath8;
thh6=ath10;  dthh6=dath10;   ddthh6=ddath10;
thh7=ath11;  dthh7=dath11;   ddthh7=ddath11;


%     i=i+1;
% end
% ti=0:step:Tfi;
% Biped Kinematics
%thh=[0; thh2; thh3; thh4; thh5; thh6; thh7];
%dthh=[0; dthh2; dthh3; dthh4; dthh5; dthh6; dthh7];
thh=[0; thh2; -0*(0.0436); thh3; thh4; thh5; -0*(0.0436); thh6; thh7];
dthh=[0; dthh2; 0; dthh3; dthh4; dthh5; 0; dthh6; dthh7];
ddthh=[ddthh2; 0; ddthh3; ddthh4; ddthh5; 0; ddthh6; ddthh7];

%ddthh=[ddthh2; ddthh3; ddthh4; ddthh5; ddthh6; ddthh7];
% th=[-pi/2; -10*pi/180; -10*pi/180; 110*pi/180; 20*pi/180; -10*pi/180; 80*pi/180];
% dth=[0; 0; 0; 0; 0; 0; 0];
% ddthh=[0; 0; 0; 0; 0; 0];
% set(0,'DefaultLineLineWidth',2.5)
% figure(12)
% r2d=180/pi;
% plot(ti,thh2*r2d,ti,thh3*r2d,ti,thh4*r2d,ti,thh5*r2d,ti,thh6*r2d,ti,thh7*r2d)
% % plot(ti,thh3*r2d,ti,dthh3*r2d,ti,ddthh3*r2d)
