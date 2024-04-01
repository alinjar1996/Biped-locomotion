function[]=energy()
disp('------------------------------------------------------------------');
disp('Performing Energy balance');

load statevar.dat;
load timevar.dat;
Y=statevar;T=timevar;
let=length(T);
kine=zeros(let,1);
pote=zeros(let,1);

[n nq alp a b bt dx dy dz al alt m g  Icxx Icyy Iczz Icxy Icyz Iczx]=inputs();
for k =1:let
    q=Y(k,1:6)';
    th=Y(k,6:6+n-1)';
    nqn=6+n;
    dq=Y(k,nqn:nqn+6-1)';
    dth=Y(k,nqn+6-1:2*(n+6-1))';
    [tt vc  scf]=for_kine(q,th, dq, dth, n, alp, a, b, bt, dx, dy, dz, al, alt);

    KE=0;
    PE=0;
    for j=1:n
        % FINDING W=M*dt+W*M*E*T+wg  
        It=[Icxx(j) Icxy(j)    Iczx(j)
            Icxy(j)  Icyy(j)    Icyz(j)
            Iczx(j)  Icyz(j)    Iczz(j)]; 
       PE=PE+m(j)*-g'*scf(:,j);
       KE=KE+0.5*(m(j)*vc(:,j)'*vc(:,j)+tt(:,j)'*It'*tt(:,j));
    end
    kine(k)=KE;
    pote(k)=PE;
end
GE=Y(:,2*(n+6-1)+1);
AE=Y(:,2*(n+6-1)+2);
TE=kine+pote+GE+AE;

%OPENING DATA FILE
fomode='w';
fip4=fopen('envar.dat',fomode);%time
% disp('2:Writing data into file');
%FOR LOOP FOR READING & WRITING SOLUTIONS FOR EACH INSTANT
for j=1:let
    %WRITING SOLUTION FOR EACH INSTANT IN FILES
    fprintf(fip4,'%e ',pote(j), kine(j), AE(j), TE(j),GE(j));
    fprintf(fip4,'\n');
end