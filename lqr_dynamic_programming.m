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
Q = [1 0 0 0;
    0 1 0 0;
    0 0 1 0;
    0 0 0 1];
R = 0.0001;


% Horizon
N = 20;

% Initial condition
x0 = [4; 2; 3;-3];

% Dynamic programming backward pass
P(:,:,N+1) = Q;
for k = N:-1:1
    K(:,:,k) = -inv(R+B'*P(:,:,k+1)*B)*B'*P(:,:,k+1)*A;
    P(:,:,k) = Q + K(:,:,k)'*R*K(:,:,k) + (A+B*K(:,:,k))'*P(:,:,k+1)*(A+B*K(:,:,k));
end
Jint(1) = 0;
% Simulation
x(:,1) = x0;
for k = 1:N
    u(:,k) = K(:,:,k)*x(:,k);
    x(:,k+1) = A*x(:,k) + B*u(:,k);
    J(k) = x(:,k)'*Q*x(:,k) + u(:,k)'*R*u(:,k);
    Jint(k+1) = Jint(k) + J(k)
end
J(N+1) = x(:,N+1)'*P(:,:,N+1)*x(:,N+1);

% Plot results
figure(1)
plot(0:N,x(1,:),'-o',0:N,x(2,:),'-x',0:N,x(3,:),0:N,x(4,:),'LineWidth',1.2)
xlabel('Time step')
ylabel('State')
legend('x_1','x_2','x_3','x_4')
grid on

figure(2)
plot(0:N-1,u,'k-o','LineWidth',1.2)
xlabel('Time step')
ylabel('Control input')
grid on

figure(3)
plot(0:N,Jint,'k-o','LineWidth',1.2)
xlabel('Time step')
ylabel('Cost')
grid on
