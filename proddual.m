function [Re1,Re2,Du1,Du2]=proddual(a1,a2,b1,b2,c1,c2,d1,d2)

Re1=a2*c1+a1*c2+a1*d1;
Re2=a2*c2+a1*d2;

Du1=a2*d1+b1*c2+b2*c1+b1*d1;
Du2=a2*d2+b2*c2+b1*d2;

end