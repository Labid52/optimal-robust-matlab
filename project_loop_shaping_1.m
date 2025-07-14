
clc
close all
clear
%% model parameters
m = 1;
M = 5;
L = 2;
g = 10;
d = 1;
d = ureal('d',1,'Percentage', [-15 15])

b = 1; % pendulum up (b=1)

%% system model

A = [0 1 0 0;
    0 -d/M b*m*g/M 0;
    0 0 0 1;
    0 -b*d/(M*L) -b*(m+M)*g/(M*L) 0];
B2 = [0; 1/M; 0; b*1/(M*L)];
%B1 = [0;0.01;0;0];
%C1 = [0 0 1 0];
C2 = [1 0 0 0];
D =0;

B = B2;
C = C2;


G = ss(A,B,C,D);

%G.InputName = {'w','u'};
%G.OutputName = {'z','y'};

W1 = makeweight(100,[1 0.5],0.25);
W3 = makeweight(0.25,[1 0.5],100);
figure
bodemag(W1,W3)

P = augw(G,W1,[],W3);

size(P)
P.OutputGroup

[K,CL,gamma] = hinfsyn(P); 
figure
L = P*K;            
sigma(L,"k--",{.1,100}); hold on

grid


%%
figure
step(CL,5)