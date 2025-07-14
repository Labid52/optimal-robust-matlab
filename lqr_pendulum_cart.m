clc
clear
close all
%% model parameters
m = 1;
M = 5;
L = 2;
g = 10;
d = 1;

b = 1; % pendulum up (b=1)

%% system model

A = [0 1 0 0;
    0 -d/M b*m*g/M 0;
    0 0 0 1;
    0 -b*d/(M*L) -b*(m+M)*g/(M*L) 0];
B = [0; 1/M; 0; b*1/(M*L)];
C = [1 0 0 0];
D =0;

%% checking controllability and observability (PBH test)
open_loop_stability_flag = 1;
lambda = eig(A);
for i = 1:length(lambda)
    if lambda(i) < 0
        open_loop_stability_flag = 0;
        disp('Unstable Open Loop System');
        break;
    end
end
controllability_matrix = ctrb(A,B);
if rank(controllability_matrix)==length(controllability_matrix)
    disp('the system is fully controllable')
else
    disp('the system is not fully controllable')
end
observability_matrix = obsv(A,C);
if rank(observability_matrix)==length(observability_matrix)
    disp('the system is fully observable')
else
    disp('the system is not fully observable')
end
%% Controllability and observability gramian
if open_loop_stability_flag == 0
    Wc = controllability_matrix*controllability_matrix'
else
    Wc = gram(ss(A,B,C,D),'c')
end
degree_of_controllability = diag(Wc)
if open_loop_stability_flag == 0
    Wb = observability_matrix*observability_matrix'
else
    Wc = gram(ss(A,B,C,D),'b')
end
degree_of_observability = diag(Wb)



%%  Design LQR controller
Q = [0.01 0 0 0;
    0 0.01 0 0;
    0 0 10 0;
    0 0 0 0.01];
R = 1;

K = lqr(A,B,Q,R);

%% Simulate closed-loop system
tspan = 0:.001:10;
x0 = [-2; 0; pi+0.1; 0];  % initial condition 
wr = [2; 0; pi; 0];      % reference position
u=@(x)-K*(x - wr);       % control law
[t,x] = ode45(@(t,x)pendcart(x,m,M,L,g,d,u(x)),tspan,x0);

for k=1:length(t)
    JLQR(k) = (x(k,:)-wr')*Q*(x(k,:)'-wr) + u(x(k,:)')^2*R;
end

% for k=1:100:length(t)
%     drawpend(x(k,:),m,M,L);
% end

%% frequency domain
Ac = A-B*K; Bc = B*wr'; Cc = C; Dc= zeros(1,4);
[num, den] = ss2tf(A,B,C,D);
G = tf(num,den);
GK = G*K(1);
sysCl = ss(Ac,Bc,Cc,Dc);
figure
margin(GK)
%% visualization
figure
plot(t,x,'LineWidth',2); hold on
l1 = legend('x','v','\theta','\omega');
set(l1,'Location','SouthEast');
set(gcf,'Position',[100 100 500 200]);
xlabel('Time')
ylabel('State')
grid on
set(gcf,'PaperPositionMode','auto')
hold off
figure
plot(t,cumtrapz(t,JLQR),'k','LineWidth',2)
set(gcf,'Position',[100 100 500 300])
xlabel('Time')
ylabel('Cost')
grid on
set(gcf,'PaperPositionMode','auto')
