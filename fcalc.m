%calculation of ground reactions
function [grf,Cyi]=fcalc(p,v,g_prop)
K=g_prop(1);
Cy=g_prop(2);
vydmax=g_prop(3);
mu=g_prop(4);
Ch=g_prop(5);
gr=g_prop(6:8);
Cyi=0;
ground=0;
grf=[0;0;0];
%this part of the program calculates damping value of the ground
zl=1.2*vydmax*Cy/K;  %factor of saftey 1.2

%pen=(groundlevel - foottip z)
%lmbvi=a4*pen^3+ a3*pen^2+ a2*pen+ a1;
% a1=0;
% a2=0;
% ADr=[zl*zl zl*zl*zl ;
%      2*zl    3*zl*zl];
DADr=3*zl*zl*zl*zl-2*zl*zl*zl*zl;
IADr=[3*zl*zl -zl*zl*zl ;
    -2*zl    zl*zl];
BDr=[Cy; 0];
% XDr=ADr\BDr;
XDr=(IADr*BDr)/DADr;
a3=XDr(1);
a4=XDr(2);

h=p(gr(1));
pen=ground-h;
vv=v(gr(1));vh1=v(gr(2));vh2=v(gr(3));
if pen >= 0.00001    %leg on ground
    if vv>0 && pen<=zl
        %Cyi=a4*(pen*pen*pen)+ a3*(pen*pen)+a2*(pen)+ a1;
        Cyi=a4*(pen*pen*pen)+ a3*(pen*pen);
    else
        Cyi=Cy;
    end
    grf(gr(1)) = K*pen - Cyi*vv;
    pcf=(2/pi)*mu*abs(grf(gr(1)));
    grf(gr(2)) =-pcf*atan(Ch*vh1/pcf);
    grf(gr(3)) =-pcf*atan(Ch*vh2/pcf);
else                %leg in air
    grf=[0;0;0];
end