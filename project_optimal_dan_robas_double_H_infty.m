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
B2 = [0; 1/M; 0; b*1/(M*L)];
B1 = [0;0.01;0;0];
C1 = [0 0.001 0 0];
C2 = [1 0 0 0];
D =0;
sysOl = ss(A,B2,C2,D);

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
controllability_matrix = ctrb(A,B2);
if rank(controllability_matrix)==length(controllability_matrix)
    disp('the system is fully controllable')
else
    disp('the system is not fully controllable')
end
observability_matrix = obsv(A,C2);
if rank(observability_matrix)==length(observability_matrix)
    disp('the system is fully observable')
else
    disp('the system is not fully observable')
end
%% Controllability and observability gramian
if open_loop_stability_flag == 0
    Wc = controllability_matrix*controllability_matrix'
else
    Wc = gram(ss(A,B2,C,D),'c')
end
degree_of_controllability = diag(Wc)
if open_loop_stability_flag == 0
    Wb = observability_matrix*observability_matrix'
else
    Wc = gram(ss(A,B2,C,D),'b')
end
degree_of_observability = diag(Wb)
%% lqr controller
K_pp = place(A,B2,[-0.6,-0.46,-0.04,-0.1]);
A = A-B2*K_pp;
B = [B1 B2];
C = [C1;C2];
gamma = 0.1;
Q1 = C1'*C1;
R1 = [gamma^2 0;0 1];
Q2 = B1*B1';
R2 = R1;
[P1,K,lam] =icare(A,B,Q1,R1,[],[],[]);
[P2,K,lam] =icare(A',C',Q2,R2,[],[],[]);

k_21 = -B2'*P1;
k_12 = -pinv(eye(4)-gamma^(-2)*P2*P1)*P2*C2';
k_11 = A + gamma^(-2)*B1*B1'*P1-B2*k_21-k_12*C2;

K = [k_11 k_12;k_21 0]
z = 0.001*ones(4,1)
A_a = [A z;0 0 0 0 0]
B_a = [B ;0 0]
C_a(1,:) = [C(1,:) 0]
C_a(2,:) = [C(2,:) 0]
 Ac = A_a-B_a*K(1:2,:);
 sysCl = ss(Ac,B_a(:,1),C_a(1,:) ,D)
 sysClf = ss(Ac,B_a(:,2),C_a(2,:),D)
 K_f = K(2,:);
 Nbar = feedforwardC(sysClf,0.1*K_f)
 sysClf = sysClf*Nbar;
 tfn = 500;
 [y_clf,t21] = step(sysClf,tfn);
 [Im_clf,t22] = impulse(sysClf,tfn);
 save('time_domain_double_riccati_u2.mat','t21','t22',"Im_clf","y_clf")
  [y_clf,t23] = step(sysCl,tfn);
 [Im_clf,t24] = impulse(sysCl,tfn);
 save('time_domain_double_riccati_u1.mat','t23','t24',"Im_clf","y_clf")

%% Visualization
%step response
figure
subplot(2,1,1)
step(sysOl)
subplot(2,1,2)
step(sysClf)
% frequency response
G = tf(sysOl);
GK = G*K(2);
figure
margin(GK,'k')
grid
Fh = gcf;                                                   % Handle To Current Figure
Kids = Fh.Children;                                         % Children
AxAll = findobj(Kids,'Type','Axes');                        % Handles To Axes
Ax1 = AxAll(1);                                             % First Set Of Axes
LinesAx1 = findobj(Ax1,'Type','Line');                      % Handle To Lines
LinesAx1(2).LineWidth = 1.2;                                  % Set 1LineWidth.
Ax2 = AxAll(2);                                             % Second Set Of Axes
LinesAx2 = findobj(Ax2,'Type','Line');                      % Handle To Lines
LinesAx2(2).LineWidth = 1.2;                                  % Set 1LineWidth.
figure
nyquist(GK,'k')
grid
Fh = gcf;                                                   % Handle To Current Figure
Kids = Fh.Children;                                         % Children
AxAll = findobj(Kids,'Type','Axes');                        % Handles To Axes
Ax1 = AxAll(1);                                             % First Set Of Axes
LinesAx1 = findobj(Ax1,'Type','Line');                      % Handle To Lines
LinesAx1(2).LineWidth = 1.2;                                  % Set 1LineWidth.
                                 % Set 1LineWidth.
