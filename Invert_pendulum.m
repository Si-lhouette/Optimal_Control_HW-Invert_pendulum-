%% Invert_Pendulum_Problem
%% Init parameters
M = 10;
m = 1;
g = 9.8;
L = 1;

%% State-Space Model
A1 = [0,1,0,0;
    0,0,-m*g/M,0;
    0,0,0,1;
    0,0,(M+m)*g/M*L,0];

B1 = [0;1/M;0;-1/M*L];

C = [1 0 0 0;
    0,0,1,0];

%% Cost-Fnc wight matrix init
Q = [100,0,0,0;
    0,0,0,0;
    0,0,10,0;
    0,0,0,0];
R = 1;

%% Generate sys
S1 = ss(A1,B1,C,0); % define the sys
Ts = 0.1; % sample time

Sd = c2d(S1,Ts); % transfer to disperse sys
[Ad,Bd,Cd,Dd,TS] = ssdata(Sd); % get disperse-sys state-space matrix

%% LQR
[K,S,e] = dlqr(Ad,Bd,Q,R);

%% Generate new sys with state-feedback
tS = ss(Ad-Bd*K,Bd,Cd,Dd,Ts); % get new sys with state-feedback

%% Given initial state & Plot the result
x0 = [0,0.1,0.05,0]'; % init state: P'=0.1;theta=0.05
t=[0:0.1:20]; % timespan
[Y,~] = initial(tS,x0,t); % calculates the response of sys

% Plot optimal state trajectory
figure;
pos = Y(:,1);
angle = Y(:,2);
plot(t,angle,'linewidth',1.3);hold on
plot(t,pos,'linewidth',1.3);
legend('angle','pos')
grid on
title('Optimal State Trajectory')

% Plot optimal control u(t)
dpos = [];dangle = [];
for i = 1:length(pos)-1
    dpos = [dpos; pos(i+1)-pos(i)];
    dangle = [dangle; angle(i+1)-angle(i)]
end
x = [pos(1:end-1),dpos,angle(1:end-1),dangle];
u = -K*x';
figure;
plot(t(1:end-1),u,'linewidth',1.3)
grid on
title('Optimal U(t)=F')


