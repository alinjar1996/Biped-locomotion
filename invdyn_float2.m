% ReDySim invdyn_float module. This module calculates generalized force
% Contibutors: Dr. Suril Shah and Prof S. K. Saha @IIT Delhi

function [tu ddq grnden phib phim tuf] = invdyn_float2 (q, dq, th, dth,ddth, n,alp,a,b,bt,dx,dy,dz,al,alt, m,g,Icxx,Icyy,Iczz,Icxy,Icyz,Iczx, g_prop)
%This code was dveloped by Mr. Suril Shah
g=-g;
% FORWARD RECURSION _FINDING TWIST AND TWIST RATE
%Initialization
e=[0;0;1];
o3=eye(3);
z31=zeros(3,1);
z33=zeros(3,3);
tt=zeros(3,n);
% tb=zeros(3,n);
dtt=zeros(3,n);
dtb=zeros(3,n);
tu=zeros(n-1,1);
tuf=zeros(n-1,1);
h=zeros(n,1);
hforce=zeros(n,1);
twt=zeros(3,n);
twb=zeros(3,n);
Oi=zeros(3,n);
so=zeros(3,n);
st=zeros(3,n);
vo=zeros(3,n);
vt=zeros(3,n);
grnden=0;
p=zeros(6,n);
pforce=zeros(6,n);
Ai0=zeros(6,6,n);
A=zeros(6,6,n);
Mt=zeros(6,6,n);
Q=zeros(3,3,n);
F=zeros(n-1,6);
Fforce=zeros(n-1,6);
Dxx=zeros(n,1);Dyy=zeros(n,1);Dzz=zeros(n,1);
Dxy=zeros(n,1);Dyz=zeros(n,1);Dzx=zeros(n,1);
Ixx=zeros(n,1);Iyy=zeros(n,1);Izz=zeros(n,1);
Ixy=zeros(n,1);Iyz=zeros(n,1);Izx=zeros(n,1);

phi1=q(4);th1=q(5);si1=q(6);
% dphi1=dq(4);
dth1=dq(5);dsi1=dq(6);
Q1=[cos(phi1)*cos(si1)-sin(phi1)*sin(th1)*sin(si1)   -sin(phi1)*cos(th1)     cos(phi1)*sin(si1)+sin(phi1)*sin(th1)*cos(si1)
    sin(phi1)*cos(si1)+cos(phi1)*sin(th1)*sin(si1)    cos(phi1)*cos(th1)     sin(phi1)*sin(si1)-cos(phi1)*sin(th1)*cos(si1)
    -cos(th1)*sin(si1)              sin(th1)                                  cos(th1)*cos(si1)                         ];
Q(:,:,1)=Q1;

L1=[0   cos(phi1)  -sin(phi1)*cos(th1)
    0   sin(phi1)   cos(phi1)*cos(th1)
    1           0   sin(th1)  ];
QL1=[-sin(si1)*cos(th1)   cos(si1)  0
    sin(th1)            0         1
    cos(si1)*cos(th1)   sin(si1)  0    ];
QdL1=[-cos(si1)*cos(th1)*dsi1+sin(si1)*sin(th1)*dth1     -sin(si1)*dsi1   0
    cos(th1)*dth1                                      0               0
    -sin(si1)*cos(th1)*dsi1-cos(si1)*sin(th1)*dth1      cos(si1)*dsi1   0];

P1=[z33 QL1
    Q1'  z33];
P1_gim=[z33 L1
        o3  z33];

dth1=dq(4:6);
v1=dq(1:3);
tt(:,1)=QL1*dth1;
tb(:,1)=Q1'*v1;
% dt1=dP1*dq+G;
dtt(:,1)=QdL1*dth1;
dtb(:,1)=Q1'*g;

tti=tt(:,1);
Oi(:,1)=Q1'*[q(1);q(2);q(3)];
% aoi=[alt(1)-al(1);0;0];
ati=[alt(1);0;0];
so(:,1)=Oi(:,1);
st(:,1)=Oi(:,1)+ati;
% ttixaoi=[tti(2)*aoi(3)-aoi(2)*tti(3);-(tti(1)*aoi(3)-aoi(1)*tti(3));tti(1)*aoi(2)-aoi(1)*tti(2)];
ttixati=[tti(2)*ati(3)-ati(2)*tti(3);-(tti(1)*ati(3)-ati(1)*tti(3));tti(1)*ati(2)-ati(1)*tti(2)];
vo(:,1)=tb(:,1);
vt(:,1)=tb(:,1)+ttixati;

i=1;
% Transfer of inertia tencor form Center of mass to link origin
dxxs=dx(i)*dx(i);dyys=dy(i)*dy(i);dzzs=dz(i)*dz(i);dxy=dx(i)*dy(i);dyz=dy(i)*dz(i);dzx=dz(i)*dx(i);
Dxx(i)=-m(i)*(dzzs+dyys);   Dyy(i)=-m(i)*(dzzs+dxxs);   Dzz(i)=-m(i)*(dxxs+dyys);
Dxy(i)=m(i)*dxy;            Dyz(i)=m(i)*dyz;            Dzx(i)=m(i)*dzx;
Ixx(i)=Icxx(i)-Dxx(i);      Iyy(i)=Icyy(i)-Dyy(i);      Izz(i)=Iczz(i)-Dzz(i);
Ixy(i)=Icxy(i)-Dxy(i);      Iyz(i)=Icyz(i)-Dyz(i);      Izx(i)=Iczx(i)-Dzx(i);
Ii=[Ixx(i) Ixy(i)    Izx(i)
    Ixy(i)  Iyy(i)    Iyz(i)
    Izx(i)  Iyz(i)    Izz(i)];

%Wt Euler equations of motion
%mass x d
mdi(1,1)=m(i)*dx(i);
mdi(2,1)=m(i)*dy(i);
mdi(3,1)=m(i)*dz(i);
tti=tt(:,i);
Itt=Ii*tt(:,i);
ttixItt=[tti(2)*Itt(3)-Itt(2)*tti(3);-(tti(1)*Itt(3)-Itt(1)*tti(3));tti(1)*Itt(2)-Itt(1)*tti(2)];
dtbi=dtb(:,i);
mdixdtbi=[mdi(2)*dtbi(3)-dtbi(2)*mdi(3);-(mdi(1)*dtbi(3)-dtbi(1)*mdi(3));mdi(1)*dtbi(2)-dtbi(1)*mdi(2)];
Wit=Ii*dtt(:,i)+mdixdtbi+ttixItt;
%Wb Newton's equaitons of motion
dtti=dtt(:,i);
dttixmdi=[dtti(2)*mdi(3)-mdi(2)*dtti(3);-(dtti(1)*mdi(3)-mdi(1)*dtti(3));dtti(1)*mdi(2)-mdi(1)*dtti(2)];
tti=tt(:,i);
ttixmdi=[tti(2)*mdi(3)-mdi(2)*tti(3);-(tti(1)*mdi(3)-mdi(1)*tti(3));tti(1)*mdi(2)-mdi(1)*tti(2)];
ttixttixmdi=[tti(2)*ttixmdi(3)-ttixmdi(2)*tti(3);-(tti(1)*ttixmdi(3)-ttixmdi(1)*tti(3));tti(1)*ttixmdi(2)-ttixmdi(1)*tti(2)];
Wib=m(i)*dtb(:,i)+dttixmdi+ttixttixmdi;
twt(:,1)=-Wit;
twb(:,1)=-Wib;

Ifi=Q(:,:,i)*Ii*Q(:,:,i)';
dii=[dx(i); dy(i); dz(i)];
di=Q(:,:,i)*dii;
sdi=[0 -di(3) di(2); di(3) 0 -di(1); -di(2) di(1) 0];
Mi=[Ifi        m(i)*sdi
    -m(i)*sdi  m(i)*o3];
Mt(:,:,i)=Mi;

Ai0(:,:,1)=eye(6);

% FOR LOOP STARTS
for i=2:n
    Qi=[cos(th(i))              -sin(th(i))              0
        cos(alp(i))*sin(th(i))   cos(alp(i))*cos(th(i)) -sin(alp(i))
        sin(alp(i))*sin(th(i))   sin(alp(i))*cos(th(i))  cos(alp(i))];
    %position vector from origin of link to origin of next link
    ai=[a(i)
        - b(i)*sin(alp(i))
        b(i)*cos(alp(i))];
    %ai=Qi*am(:,i);% in frame n
    
    ttbi=tt(:,bt(i));
    %w angular velocity
    tt(:,i)=Qi'*ttbi+e*dth(i);
    
    ttbixai=[ttbi(2)*ai(3)-ai(2)*ttbi(3);-(ttbi(1)*ai(3)-ai(1)*ttbi(3));ttbi(1)*ai(2)-ai(1)*ttbi(2)];
    
    %v  linear velocity
    tb(:,i)=Qi.'*(tb(:,bt(i))+ttbixai);
    
    %dw angular acceleration
    tti=tt(:,i);
    ttixe=[tti(2);-tti(1);0];
    dtt(:,i)=Qi.'*dtt(:,bt(i))+e*ddth(i)+ttixe*dth(i);
    
    %dv linear acceleration
    dttbi=dtt(:,bt(i));
    dttbixai=[dttbi(2)*ai(3)-ai(2)*dttbi(3);-(dttbi(1)*ai(3)-ai(1)*dttbi(3));dttbi(1)*ai(2)-ai(1)*dttbi(2)];
    ttbixttbixai=[ttbi(2)*ttbixai(3)-ttbixai(2)*ttbi(3);-(ttbi(1)*ttbixai(3)-ttbixai(1)*ttbi(3));...
        ttbi(1)*ttbixai(2)-ttbixai(1)*ttbi(2)];
    dtb(:,i)=Qi.'*(dtb(:,bt(i))+dttbixai+ttbixttbixai);
    
    % Transfer of inertia tencor form Center of mass to link origin
    dxxs=dx(i)*dx(i);dyys=dy(i)*dy(i);dzzs=dz(i)*dz(i);dxy=dx(i)*dy(i);dyz=dy(i)*dz(i);dzx=dz(i)*dx(i);
    Dxx(i)=-m(i)*(dzzs+dyys);   Dyy(i)=-m(i)*(dzzs+dxxs);   Dzz(i)=-m(i)*(dxxs+dyys);
    Dxy(i)=m(i)*dxy;            Dyz(i)=m(i)*dyz;            Dzx(i)=m(i)*dzx;
    Ixx(i)=Icxx(i)-Dxx(i);      Iyy(i)=Icyy(i)-Dyy(i);      Izz(i)=Iczz(i)-Dzz(i);
    Ixy(i)=Icxy(i)-Dxy(i);      Iyz(i)=Icyz(i)-Dyz(i);      Izx(i)=Iczx(i)-Dzx(i);
    Ii=[Ixx(i) Ixy(i)    Izx(i)
        Ixy(i)  Iyy(i)    Iyz(i)
        Izx(i)  Iyz(i)    Izz(i)];
      
    %Wt Euler equations of motion
    %mass x d
    mdi(1,1)=m(i)*dx(i);
    mdi(2,1)=m(i)*dy(i);
    mdi(3,1)=m(i)*dz(i);
    tti=tt(:,i);
    Itt=Ii*tt(:,i);
    ttixItt=[tti(2)*Itt(3)-Itt(2)*tti(3);-(tti(1)*Itt(3)-Itt(1)*tti(3));tti(1)*Itt(2)-Itt(1)*tti(2)];
    dtbi=dtb(:,i);
    mdixdtbi=[mdi(2)*dtbi(3)-dtbi(2)*mdi(3);-(mdi(1)*dtbi(3)-dtbi(1)*mdi(3));mdi(1)*dtbi(2)-dtbi(1)*mdi(2)];
    Wit=Ii*dtt(:,i)+mdixdtbi+ttixItt;
    
    %Wb Newton's equaitons of motion
    dtti=dtt(:,i);
    dttixmdi=[dtti(2)*mdi(3)-mdi(2)*dtti(3);-(dtti(1)*mdi(3)-mdi(1)*dtti(3));dtti(1)*mdi(2)-mdi(1)*dtti(2)];
    tti=tt(:,i);
    ttixmdi=[tti(2)*mdi(3)-mdi(2)*tti(3);-(tti(1)*mdi(3)-mdi(1)*tti(3));tti(1)*mdi(2)-mdi(1)*tti(2)];
    ttixttixmdi=[tti(2)*ttixmdi(3)-ttixmdi(2)*tti(3);-(tti(1)*ttixmdi(3)-ttixmdi(1)*tti(3));tti(1)*ttixmdi(2)-ttixmdi(1)*tti(2)];
    Wib=m(i)*dtb(:,i)+dttixmdi+ttixttixmdi;
    
    %% Calculation for foot ground interaction
    Oi(:,i)=Qi'*(Oi(:,bt(i))+ai);
    Q(:,:,i)=Q(:,:,bt(i))*Qi;
       
    aoi=[alt(i)-al(i);0;0];
    ati=[alt(i);0;0];
    so(:,i)=Oi(:,i)+aoi;
    st(:,i)=Oi(:,i)+ati;
    ttixaoi=[tti(2)*aoi(3)-aoi(2)*tti(3);-(tti(1)*aoi(3)-aoi(1)*tti(3));tti(1)*aoi(2)-aoi(1)*tti(2)];
    ttixati=[tti(2)*ati(3)-ati(2)*tti(3);-(tti(1)*ati(3)-ati(1)*tti(3));tti(1)*ati(2)-ati(1)*tti(2)];
    vo(:,i)=tb(:,i)+ttixaoi;
    vt(:,i)=tb(:,i)+ttixati;
    
    stf=Q(:,:,i)*st(:,i);
    sof=Q(:,:,i)*so(:,i);
    vof=Q(:,:,i)*vo(:,i);
    vtf=Q(:,:,i)*vt(:,i);
    fti=fcalc(stf,vtf,g_prop);
    foi=fcalc(sof,vof,g_prop);
    %deriv of ground energy (ground reaction force*velocity)
    grnden=grnden+fti'*vtf+foi'*vof;
    fti=Q(:,:,i)'*fti;
    foi=Q(:,:,i)'*foi;
    
    atixfti=[ati(2)*fti(3)-fti(2)*ati(3);-(ati(1)*fti(3)-fti(1)*ati(3));ati(1)*fti(2)-fti(1)*ati(2)];
    aoixfoi=[aoi(2)*foi(3)-foi(2)*aoi(3);-(aoi(1)*foi(3)-foi(1)*aoi(3));aoi(1)*foi(2)-foi(1)*aoi(2)];
    Wft=atixfti+aoixfoi;
    Wfb=fti+foi;
    twt(:,i)=Wft-Wit;
    twb(:,i)=Wfb-Wib;
   
    %GIM I11F
    ei=Q(:,3,i);
    ao=Q(:,:,bt(i))*ai;
    sao=[0 -ao(3) ao(2); ao(3) 0 -ao(1); -ao(2) ao(1) 0];
    A(:,:,i)=[o3    z33
        -sao   o3];
    Ai0(:,:,i)=A(:,:,i)*Ai0(:,:,bt(i));
    p(:,i)=[ei
        z31];

    pforce(:,i)=[z31
                 ei];
    
    Ifi=Q(:,:,i)*Ii*Q(:,:,i)';
    dii=[dx(i); dy(i); dz(i)];
    di=Q(:,:,i)*dii;
    sdi=[0 -di(3) di(2); di(3) 0 -di(1); -di(2) di(1) 0];
    Mi=[Ifi        m(i)*sdi
        -m(i)*sdi  m(i)*o3];
    Mt(:,:,i)=Mi;
end

% BACKWARD RECURSION_FINDING JOINT TORQUE
for i=n:-1:2
    %Caluation of the generalized foces
    h(i)=twt(3,i);
    hforce(i) = twb(3,i); %constraint force
    Qi=[cos(th(i))              -sin(th(i))              0
        cos(alp(i))*sin(th(i))   cos(alp(i))*cos(th(i)) -sin(alp(i))
        sin(alp(i))*sin(th(i))   sin(alp(i))*cos(th(i))  cos(alp(i))];
    ai=[a(i)
        - b(i)*sin(alp(i))
        b(i)*cos(alp(i))];
    twbi=Qi*twb(:,i);
    aixtwbi=[ai(2)*twbi(3)-twbi(2)*ai(3);-(ai(1)*twbi(3)-twbi(1)*ai(3));ai(1)*twbi(2)-twbi(1)*ai(2)];
    twt(:,bt(i))=twt(:,bt(i))+Qi*twt(:,i)+aixtwbi;
    twb(:,bt(i))= twb(:,bt(i))+twbi;
    
    %GIM I11F
    Mt(:,:,bt(i))=Mt(:,:,bt(i))+A(:,:,i)'*Mt(:,:,i)*A(:,:,i);
    psi=p(:,i)'*Mt(:,:,i);
    psiforce = pforce(:,i)'*Mt(:,:,i);
    F(i-1,:)=psi*Ai0(:,:,i)*P1_gim;
    Fforce(i-1,:)=psiforce*Ai0(:,:,i)*P1_gim;
end
phi21=h(2:n);
phim=-phi21;
% phi11=[Q1*twb(:,1);QL1'*twt(:,1)];
tw1=[twt(:,1);twb(:,1)];
phi11=P1'*tw1;
phib=-phi11;
%GIM I11F
I11=P1_gim'*Mt(:,:,1)*P1_gim;

%Forward recursion
ddq=I11\phi11;
%ddq=pinv(I11)*phi11;

%ddq = 0*ones(6,1);

for i=2:n
    tu(i-1)=F(i-1,:)*ddq-h(i);
    tuf(i-1)=Fforce(i-1,:)*ddq-hforce(i);
end
%i
%size(F)
%size(F(i,:))
%size(ddq)