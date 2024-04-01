% ReDySim runinv module. This module perform inverse dynamics.
% Contibutors: Dr. Suril Shah and Prof S. K. Saha @IIT Delhi

function []=runinv()
disp('------------------------------------------------------------------');
disp('Recursive Inverse Dynamics');
disp('Contibutors: Dr. Suril Shah and Prof. S. K. Saha @IIT Delhi ');
disp('------------------------------------------------------------------');

[yo, ti, tf, incr, rtol, atol,type]=initials();%ODE CALL
disp('Simulation Started');

tic
% option=odeset('stats','on');
options=odeset('AbsTol',atol,'RelTol',rtol,'stats','on');
if type==0
    [S,Y]=ode45(@sys_ode,ti:incr:tf,yo,options);
else
    [S,Y]=ode15s(@sys_ode,ti:incr:tf,yo,options);
end
% Y=ode5(@recurode,T,y0);
toc

% SIMULATION SPEED
s=sprintf('Simulation %6.4f times faster',tf/toc);
disp(s);

[n nq alp a b bt dx dy dz al alt m g  Icxx Icyy Iczz Icxy Icyz Iczx g_prop]=inputs();

%OPENING DATA FILE
fomode='w';
fip1=fopen('timevar.dat',fomode);%time
fip2=fopen('statevar.dat',fomode);%all state variables
fip3=fopen('tor.dat','w');
fip4=fopen('base.dat',fomode);%time
fip5=fopen('constraintforce.dat',fomode);%constraintforce
fip6=fopen('jointacc.dat',fomode);%jointaccelaration
len=length(S);

for k = 1 : len
    s=S(k);
    q=Y(k,1:6)';
    dq=Y(k,6+1:2*6)';
    grnen=Y(k,2*6+1);
    acten=Y(k,2*6+2);
    % Trejectories
    [thh dthh ddthh]=trajectory(s);
    th=thh;dth=dthh;
    ddth=[0;ddthh];
    tsim=s;
    YY=[q' th(2:n)' dq' dth(2:n)' grnen acten];
    % C+Tug- TERM USING INVERSE DYNAMIC ALGORITHM
    [tu ddq grnden phib phim tuf] = invdyn_float2(q, dq, th, dth,ddth, n,alp,a,b,bt,dx,dy,dz,al,alt, m,g,Icxx,Icyy,Iczz,Icxy,Icyz,Iczx, g_prop);
    
    %WRITING SOLUTION FOR EACH INSTANT IN FILES
    fprintf(fip1,'%e\n',tsim);
    fprintf(fip2,'%e ',YY);fprintf(fip2,'\n');
    fprintf(fip3,'%e ',tu);fprintf(fip3,'\n');
    fprintf(fip4,'%e ',tsim, q, dq);fprintf(fip4,'\n');
    fprintf(fip5,'%e ',tuf);fprintf(fip5,'\n');
    fprintf(fip6,'%e ',ddth);fprintf(fip6,'\n');
end
fclose all;

disp('------------------------------------------------------------------');
disp('Recursive Inverse Dynamics');
disp('Contibutors: Dr. Suril Shah and Prof. S. K. Saha @IIT Delhi ');
disp('------------------------------------------------------------------');
