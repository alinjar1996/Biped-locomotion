% ReDySim main module. Use this module to run your program
% Contibutors: Dr. Suril Shah and Prof S. K. Saha @IIT Delhi

function [] = run_me()
%Use this file to run your program
clear all;
fclose all;
clc;
%Run the main code
runinv;
%Plots the joint motion
plot_motion;
%Plots the joint torques and JARF
 plot_tor;
 plot_constraint_for;

%Energy Calculation
energy;
%Plots Energy Balance
plot_en;
%Animate
%animate_new(0.1);
%animate_new2(0.25);
animate_newside(0.25);

disp('------------------------------------------------------------------');
disp('Contibutors: Mr. Alinjar Dan, Dr. Suril Shah and Prof S. K. Saha @IIT Delhi ');
disp('------------------------------------------------------------------');


