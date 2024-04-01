% ReDySim inputs module. The model parameters are entered in this module
% Contibutors: Dr. Suril Shah and Prof S. K. Saha @IIT Delhi

function [n nq alp a b bt dx dy dz al alt m g  Icxx Icyy Iczz Icxy Icyz Iczx g_prop]=inputs()
         %[n nq alp a b bt dx dy dz al alt m g  Icxx Icyy Iczz Icxy Icyz Iczx g_prop]

% 7 Link Biped
%NO. OF LINK
%n=7;
n= 9;
nq=1;%1 for spatial and 0 for planar
%ENTER DH PARAMETER HERE   
%  dh=[al b alp th];
t_h = 0.05; %torso_height
sl2=0.1;tl2=0.108;tl1=0.108;sl1=0.1; %shank and thigh link lengths obtained from URDF
hl = 0.0358; %hip length
%fl = 0.09; %foot_length
fl = 0.09; %foot_length

alp=[0; 0; -1*pi/2; 1*pi/2; 0; 0; -1*pi/2; 1*pi/2; 0];
%alp=[0; 0; pi/2; 0; 0; 0; pi/2; 0; 0];
a=[0; t_h/2; hl; tl1; sl1; t_h/2; hl; tl2; sl2];
b=[0; -0*0.055; 0; 0; 0; 0*0.055; 0; 0; 0];
%Parent array bt and corrosponding vectors
bt=[0 1 2 3 4 1 6 7 8];

%alp = [0; 0; 0; 0; 0; 0; 0; 0; 0];
%a=[0; 0.25; 0; 0.5; 0.5; 0.25; 0; 0.5; 0.5];
%b=[0; 0; 0; 0; 0; 0; 0; 0; 0];
%bt=[0 1 2 3 4 1 5 6 7];
%Link Length
%al(1) = trunk length = 2*a(2)= 2*a(6)
%al=[t_h; 0.0; 0.5; 0.5; 0.15; 0.0; 0.5; 0.5; 0.15];
al=[t_h;hl;tl1;sl1;fl;hl;tl2;sl2;fl];
% %Distance from origin to link tip in term of link length
%alt=[t_h/2; 0.0; 0.5; 0.5; 0.075; 0.0; 0.5; 0.5; 0.075];
alt=[t_h/2; hl; tl1; sl1; fl/2; hl; tl2; sl2; fl/2];

%ENTER VECTOR rm
%dx=[     0     al(2)/2 al(3)/2     0  al(5)/2  al(6)/2      0  ];
%dy=[     0         0       0       0       0        0       0      ];
%dz=[     0         0       0       0       0        0       0     ];
dx = [0  0  al(3)/2 al(4)/2  0  0 al(7)/2 al(8)/2 0];
dy=  [0  0       0       0   0  0    0       0    0];
dz=  [0  0       0       0   0  0    0       0    0];

%MASS
%m=1.*[5; 0.05; 1; 1; 0.2; 0.05; 1; 1; 0.2];
%m = [0.1334; 0.0728; 0.01123; 0.25259; 0.1334; 0.0728; 0.011328; 0.2634;0.1504]; % gave result
m = [0.1504;0.1334; 0.0728; 0.01123; 0.25259; 0.1334; 0.0728; 0.011328; 0.25259]; 

%;0.2634

g=[0 ; -9.81; 0];

%MOMENT OF INERTIA
Icxx=zeros(n,1);Icyy=zeros(n,1);Iczz=zeros(n,1); % Initialization 
Icxy=zeros(n,1);Icyz=zeros(n,1);Iczx=zeros(n,1); % Initialization 
%Icxx(1)=(1/12)*0.1*0.1;   Icyy(1)=(1/12)*m(1)*(al(1)*al(1)+0.1*0.1);  Iczz(1)=(1/12)*m(1)*al(1)*al(1);
% Icxx(2)=0;   Icyy(2)=(1/12)*m(2)*al(2)*al(2);  Iczz(2)=(1/12)*m(2)*al(2)*al(2);
% Icxx(3)=0;   Icyy(3)=(1/12)*m(3)*al(3)*al(3);  Iczz(3)=(1/12)*m(3)*al(3)*al(3);
% Icxx(4)=0;   Icyy(4)=(1/12)*m(4)*al(4)*al(4);  Iczz(4)=(1/12)*m(4)*al(4)*al(4);
% Icxx(5)=0;   Icyy(5)=(1/12)*m(5)*al(5)*al(5);  Iczz(5)=(1/12)*m(5)*al(5)*al(5);
% Icxx(6)=0;   Icyy(6)=(1/12)*m(6)*al(6)*al(6);  Iczz(6)=(1/12)*m(6)*al(6)*al(6);
% Icxx(7)=0;   Icyy(7)=(1/12)*m(7)*al(7)*al(7); Iczz(7)=(1/12)*m(7)*al(7)*al(7);
% Icxx(8)=0;   Icyy(8)=(1/12)*m(8)*al(8)*al(8); Iczz(8)=(1/12)*m(8)*al(8)*al(8);
% Icxx(9)=0;   Icyy(9)=(1/12)*m(9)*al(9)*al(9); Iczz(9)=(1/12)*m(9)*al(9)*al(9);

Icxx = [8.90079991e-04;1.46119348e-04;11.18188756e-05;1.32756718e-04;4.65693015e-04;1.46119348e-04;11.18188756e-05;1.32756718e-04;4.65693015e-04];
Icyy = [2.44281295e-03;7.24989160e-05;12.24593674e-05;1.43711638e-04;2.70756846e-04;7.24989160e-05;12.24593674e-05;1.43711638e-04;2.70756846e-04];
Iczz = [2.62099605e-03;1.24221192e-04;4.84181807e-05;2.88232176e-05;3.78883043e-04;1.24221192e-04;4.84181807e-05;2.88232176e-05;3.78883043e-04];

%Ground paprameters
%Parameters for vertical reaction
K=10000/10; odf=100;                    
Cv=odf*2*sqrt(K*sum(m));
vzdmax=8; 
%Parameters for horizontal reactioin
mu=1*0.3;Ch=50000*0.03;
gr=[2 1 3];
g_prop=[K Cv vzdmax mu Ch gr];