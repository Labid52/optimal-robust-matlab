clc
close all
clear
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

B = [ones(4,1) B];
C = [ones(1,4); C];

D = [0 1;1 0];

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
%% Augmentation
A = A-B*(lqr(A,B,C'*C,1));
B1 = [0;0;0;0];
B2 = B;
C = C;
%%  Design Hloop controller

% Horizon
N = 4000;

% boundary conditions
x0 = [2; 0; pi+0.1;0];
h = 0.01;


% Dynamic programming backward pass
P(:,:,1) = C'*C;
gamma = 0.001;

for k = 1:1:N
    P(:,:,k+1) = P(:,:,k)+ h*(P(:,:,k)*A + A'*P(:,:,k) - P(:,:,k)*(B2*0.0001*B2'-gamma^2*B1*B1')*P(:,:,k) + C'*C);
    K(:,:,k+1) = B2'*P(:,:,k+1);
end
P(:,:,1:end) = P(:,:,end:-1:1);
K(:,:,1:end) = K(:,:,end:-1:1);
% Simulation
x(:,1) = x0;
t(1) = 0;
J(1) = x(:,1)'*C'*C*x(:,1);
for k = 1:1:N
    u(:,k) = -K(:,:,k)*x(:,k);
    x(:,k+1) = x(:,k)+h*(A*x(:,k) + B2*u(:,k)+B1);
    J(k+1) = J(k)+x(:,k)'*C'*C*x(:,k) + u(:,k)'*u(:,k);
    t(k+1) = t(k) + h;
end
%J(N+1) = J(end) + (x(:,N+1)-wr)'*P(:,:,N+1)*(x(:,N+1)-wr);

% Plot results
figure(1)
subplot(2,2,1)
plot(t,x(1,:),'k','LineWidth',1.5)
grid on
xlabel('Time')
ylabel('x_1')
grid on
subplot(2,2,2)
plot(t,x(2,:),'k','LineWidth',1.5)
xlabel('Time')
ylabel('x_2')
grid on
subplot(2,2,3)
plot(t,x(3,:),'k','LineWidth',1.5)
xlabel('Time')
ylabel('x_3')
grid on
subplot(2,2,4)
plot(t,x(4,:),'k','LineWidth',1.5)
xlabel('Time')
ylabel('x_4')
grid on

figure(2)
plot(t(1:end-1),u,'k','LineWidth',1.5)
xlabel('Time')
ylabel('Control input')
grid on

figure(3)
plot(t,J,'k','LineWidth',1.5)
xlabel('Time')
ylabel('Cost')
grid on
