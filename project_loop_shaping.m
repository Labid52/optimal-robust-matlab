
clc
close all
clear
%% model parameters
m = 1;
M = 5;
L = 2;
g = 10;
d = 1;
d = ureal('d',1,'Percentage', [-15 15]);

b = 1; % pendulum up (b=1)

%% system model

A = [0 1 0 0;
    0 -d/M b*m*g/M 0;
    0 0 0 1;
    0 -b*d/(M*L) -b*(m+M)*g/(M*L) 0];
B2 = [0; 1/M; 0; b*1/(M*L)];
B1 = [0;0.01;0;0];
C1 = [0 0 1 0];
C2 = [1 0 0 0];
D =0;

B = [B1 B2];
C = [C1;C2 ]


G = ss(A,B,C,D);

G.InputName = {'w','u'};
G.OutputName = {'z','y'};

%% 
figure
sigma(G)

figure
step(G)

%% 
Gd = tf(2.5,[1 0]);
figure
sigma(Gd,{0.1 100},'k')
grid on

%%
[K0,CL0,gamma0,info0] = loopsyn(G,Gd);
gamma0

%%
L0 = G*K0;  
figure
sigma(L0,"b",Gd,"r--",{.1,100});
grid
legend("L0 (actual loop shape)","Gd (target loop shape)");
%% 
figure
step(CL0,5)


%% satisfactory design

alpha = 0.25;
[K,CL,gamma,info] = loopsyn(G,Gd,alpha);
gamma
figure
L = G*K;            
sigma(L0,L,Gd,"k--",{.1,100});
grid
legend("L0 (inital design)","L (final design)","Gd (target loop shape)");

DM = diskmargin(G,K);
DM.DiskMargin

figure
step(CL0,CL,5)
legend("initial (alpha = 0.5)","final (alpha = 0.25)","Location","southeast")






