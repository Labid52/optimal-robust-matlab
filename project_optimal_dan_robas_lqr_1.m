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
B = [0 0; 1/M 0.01; 0 0; b*1/(M*L) 0];
C = [1 0 0 0];
D =0;
sysOl = ss(A,B(:,1),C,D);

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
%% lqr controller
Q = [1000 0 0 0;
    0 0.01 0 0;
    0 0 0.01 0;
    0 0 0 0.01];
R = eye(2);


[P,K,lam] =icare(A,B,Q,R)

Ac = A-B*K;
sysCl = ss(Ac,B,C,D)
 [yCl_lqr,t01] = step(sysCl(1));
 [Im_lqr,t02] = impulse(sysCl(1));
 save('time_domain_lqr_u2.mat','t01','t02',"Im_lqr","yCl_lqr")
 [yCl_lqr,t03,] = step(sysCl(2));
 [Im_lqr,t04] = impulse(sysCl(2));
 save('time_domain_lqr_u1.mat','t03','t04',"Im_lqr","yCl_lqr")
%% Visualization
% step response
figure
subplot(2,1,1)
step(sysOl)
subplot(2,1,2)
step(sysCl)
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



