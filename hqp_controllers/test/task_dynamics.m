clear all; close all; clc;

e0=1;      %initial state
ds=0.035;  %safety value
di=0.2;    %influence zone
lmbd=-1;   %lineardynamics gain

dt = 0.01;
t=0:dt:1;
u=ones(size(t));

A=lmbd/(di-ds); 
B=-lmbd*ds/(di-ds);
C=1;
D=0;
sys=ss(A,B,C,D);
y=lsim(sys,u,t,e0);

subplot(2,1,1);
plot(t,y,'b'); grid on;
xlabel('t [s]');
ylabel('e');
title('Task function');

subplot(2,1,2);
dy=diff(y)/dt;
plot(t(1:end-1),dy,'r'); grid on;
xlabel('t [s]');
ylabel('de');
title('Task velocity');