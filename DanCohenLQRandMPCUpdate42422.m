%% MAE 507 Project Satellite Model
% (Auer, Matthew), (Cohen, Dan), (Skowronek, Olga) 
clear all
close all
clc

%------------------------- Motor Parameters ------------------------------%
% Maxon 500266: 
%   https://www.maxongroup.us/maxon/view/product/motor/ecmotor/ecflat/ecflat90/500266

k = 0.136;  % Torque constant and speed constant both in kg*m^2*s^-2*A^-1
R = 0.28;   % Winding resistance in Ohms
L = 0.369*10^-3;    % Winding inductance in Henries

m_mot = 0.985;  % Motor mass in kg
r_mot = 90*(10^-3)/2;   % Motor radius

T_nom = 0.988;  % Nominal torque of the motor, Nm

% Estimated damping rate of the motor, bearings, etc. Based on nominal 
% torque of the motor per rotation with a scale factor. WAG.
b = 0.1*T_nom/(2*pi);   % Nm/(rad/s)

% Input and state constraints defined by motor
v_min = -30; % Min motor voltage is min input
v_max =  30; % Max motor voltage is max input

omega_f_min = -1780*(1/60)*(2*pi);  % Min reachable motor speed [rad/s]
omega_f_max =  1780*(1/60)*(2*pi);  % Max reachable motor speed [rad/s]

%------------------------- Flywheel Inertia ------------------------------%
% Approximate the flywheel as a disk with inner radius (r1) and outer
% radius (r2), and a plate thickness (t).
% Assume the material is aluminum.

r1 = 100*10^-3; % Inner radius in m
r2 = 150*10^-3; % Outer radius in m
t = 5*10^-3;    % Disc axial thickness in m

rho_Al = 2710;  % Aluminum density in kg/m^3

Jf = (pi/2)*rho_Al*t*(r2^4 - r1^4); % Compute polar moment of inertia (PMoI)

%------------------------- Satelite Inertia ------------------------------%
% Approximate the satelite as a cube made from aluminum with the inner
% space filled with material with density of plastic (ABS), electronics,
% sparse wires, etc. Assume the motor mass is in the axis of the satelite
% centroid and plastic inertia is removed equal to the size of the motor
% and flywheel.

rho_elec = 1006;    % density of the electronics. Assumed to be ABS

l = 350*10^-3;  % side length of cube. Should be larger than 2*r2 above.
l_elec = l - 2*5*10^-3; % side length of inner cube. Based on assumed Al thickness.

I_cube_Al = (rho_Al/6)*l^5; % Compute the PMoI of a solid cube

% Compute the PMoI of the solid elec cube minus Al cube.
I_elec = ((rho_elec - rho_Al)/6)*l_elec^5;  

% Compute the PMoI of the motor minus the space taken up by motor in elec.
I_motor = ((r_mot^2)/2)*(m_mot - rho_elec*pi*(r_mot^2));  

% Compute the PMoI the space taken up by flywheel in elec.
I_fly = (pi/2)*rho_elec*t*(r2^4 - r1^4);  

% Compute the satelite PMoI by summing all parts and subtracting the PMoI
% where the wheel is in elec.
Js = I_cube_Al + I_elec + I_motor - I_fly; 

%-------------------------- System Matrices ------------------------------%
% Define A, B, C, D matrices using above parameters.
% 
% State variables are angle of the satelite (externally observed 
% fixed reference), flywheel angular velocity (relative to the satelite 
% frame) and its derivative.

% Inputs to the system are motor voltage and external torques
A = [0, -Jf/(Jf + Js)       , 0                   ;
     0,  0                  , 1                   ;
     0,  -(R*b + k^2)/(L*Jf), -(R*Jf + L*b)/(L*Jf)];

% Input system matrix (using both control input (voltage) and disturbance
% (external torque))
B = [0        , 0    ;
     0        , -1/Jf;
     k/(L*Jf) , 0    ];

% Input controllable system matrix (using only control input (voltage))
B_ctrl = [0        ;
          0        ;
          k/(L*Jf)];

C = [1 0 0];
D = [0 0];
D_ctrl = 0;

% Setup system models for the complete system and the control only system
sys = ss(A,B,C,D);
sys_ctrl = ss(A,B_ctrl,C,D_ctrl);

% Convert the continuous dynamics from continuous-time to discrete-time
% using a Zero-Order-Hold method. 
sampling_frequency = 1000;  % Hz. Assume the control loop is running at 1kHz
Ts = 1.0/sampling_frequency;    % sec. Sampling time

sysd = c2d(sys,Ts);
sysd_ctrl = c2d(sys_ctrl,Ts);

Ad_ctrl = sysd_ctrl.A;
Bd_ctrl = sysd_ctrl.B;
Cd_ctrl = sysd_ctrl.C;
Dd_ctrl = sysd_ctrl.D;

Ad = sysd.A;
Bd = sysd.B;
Cd = sysd.C;
Dd = sysd.D;

%------------------------ System Characteristics -------------------------%
% Check controllability of the continous-time MISO and SISO systems
ctrb_sys = rank(ctrb(sys));
ctrb_sys_ctrl = rank(ctrb(sys_ctrl));

% Check observability of the continous-time MISO and SISO systems
obsv_sys = rank(obsv(sys));
obsv_sys_ctrl = rank(obsv(sys_ctrl));

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Check the system characteristics  VERFIY THAT THIS IS CORRECT FOR DT
% Check controllability of the discrete-time MISO and SISO systems
ctrb_sysd = rank(ctrb(sysd));
ctrb_sysd_ctrl = rank(ctrb(sysd_ctrl));

% Check observability of the discrete-time MISO and SISO systems
obsv_sysd = rank(obsv(sys));
obsv_sysd_ctrl = rank(obsv(sys_ctrl));
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

ctrb_list = [ctrb_sys, ctrb_sys_ctrl, ctrb_sysd, ctrb_sysd_ctrl];
obsv_list = [obsv_sys, obsv_sys_ctrl, obsv_sysd, obsv_sysd_ctrl];
for sys_step = 1:length(ctrb_list)
    % Check rank of each system controllability matrix
    if ctrb_list(sys_step) < size(A,1)
        all_controllable = false;
    else
        all_controllable = true;
    end
    % Check rank of each system observability matrix
    if obsv_list(sys_step) < size(A,1)
        all_observable = false;
    else
        all_observable = true;
    end
end
all_controllable
all_observable

%--------------------- Simulate open-loop dynamics -----------------------%
x0 = [0;0;0];
v0 = v_max; % Constant max motor voltage signal
T_ext0 = 1; % Constant disturbance torque.

duration = 5;  % number of sec runtime
N = round(duration/Ts); % compute the number of time steps
t = linspace(0,duration,N);    % simulate over duration with N steps
v = linspace(v0,v0,N);  % Constant input voltage
T_ext = linspace(0,T_ext0,N);

% continuous-time solution to the controllable system (no disturbance)
y_ct_ctrl = lsim(sys_ctrl,v,t,x0);   

% continuous-time solution to the full system
y_ct = lsim(sys,[v',T_ext'],t,x0);   

% discrete-time solution
x = zeros(size(A,1),length(t));
y = zeros(size(C,1),length(t));
U = zeros(size(B,2),length(t));
x(:,1) = x0;
for k = 1:length(t)
    U(:,k) = [v(k); T_ext(k)];
    x(:,k+1) = Ad*x(:,k) + Bd*U(:,k);
    y(:,k)   = Cd*x(:,k) + Dd*U(:,k);
end

figure(1)
title('Continuous-Time and Discrete-Time Solutions')
hold on
plot(t,y_ct_ctrl,'--k','DisplayName','CT Contrtollable Portion Solution','LineWidth',2)
plot(t,y_ct,'Color','#0072BD','DisplayName','CT Full Solution', 'LineWidth',2)
plot(t,y,'o','MarkerEdgeColor','k','MarkerFaceColor','#D95319','DisplayName', ...
    'DT Solution','LineWidth',1,'MarkerIndices',[1:(N/50):N])
stairs(t,y,'k','DisplayName','DT Full Solution','LineWidth',1);
ylim([1.1*min(y)-.01, 1.1*max(y)+.01])
xlim([t(1), t(N)])
hold off
legend('Location','best')
grid on

%------------------- Simulate closed-loop dynamics -----------------------%
x0 = [1;0;0];
T_ext0 = 1; % Constant disturbance torque.
r0 = 0; % Reference signal value.

K = place(Ad_ctrl,Bd_ctrl,[0.999 0.985 0.980]);

duration = 20;  % number of sec runtime
N = round(duration/Ts); % compute the number of time steps
t = linspace(0,duration,N);    % simulate over duration with N steps
r = linspace(r0,r0,N);  % Constant input voltage
T_ext = linspace(0,0,N);
T_ext(round(N/2):N) = T_ext0; %round(N/2) + round(1/Ts)

% discrete-time solution
x = zeros(size(A,1),length(t));
y = zeros(size(C,1),length(t));
u = zeros(size(Bd_ctrl,2),length(t));
x(:,1) = x0;
for k = 1:length(t)
    u(k) = -K*x(:,k);
    x(:,k+1) = Ad*x(:,k) + Bd_ctrl*u(k) + Bd(:,2)*T_ext(k);
    y(:,k)   = Cd*x(:,k) + Dd_ctrl*u(k) + Dd(:,2)*T_ext(k);
end

% Calculate theta of the flywheel assuming it starts from zero
theta_f = -((Js + Jf)/Jf)*x(1,1:N) + ((Js + Jf)/Jf)*x0(1) ;

figure(2)
title('Closed-Loop Discrete-Time Solution, Angles')
hold on
plot(t,x(1,1:N),'Color','#0072BD','DisplayName','Theta Satellite','LineWidth',2)
plot(t,theta_f,'Color',	'#7E2F8E','DisplayName','Theta Flywheel','LineWidth',2)
xlim([t(1), t(N)])
hold off
legend('Location','best')
grid on

figure(3)
title('Closed-Loop Discrete-Time Solution, Full State')
hold on
plot(t,x(1,1:N),'Color','#0072BD','DisplayName','Theta Satellite','LineWidth',2)
plot(t,theta_f,'Color',	'#7E2F8E','DisplayName','Theta Flywheel','LineWidth',2)
plot(t,x(2,1:N),'Color','#D95319','DisplayName','Flywheel Rate','LineWidth',2)
plot(t,x(3,1:N),'Color','#EDB120','DisplayName','Flywheel Rate Dot','LineWidth',2)
xlim([t(1), t(N)])
hold off
legend('Location','best')
grid on


Q = [100 0 0; 0 0.01 0; 0 0 0.01]
%N = [100,100,100]'
R = 0.01
K = dlqr(Ad,Bd_ctrl,Q,R);
              % If we used the classic LQR what would our gain look like for each state space var
CLP = eig(Ad-Bd_ctrl*K)
tspan = 0:Ts:5;
%tspan = linspace(0,10,10000); % time
y0 = [pi/2; 0; 0]; % [x xdot theta theta_dot]

T_ext = 1;
[t,y] = ode45(@(t,y)((Ad-Bd_ctrl*K)*(y-[0; 0; 0])),tspan,y0); % LQR sol
plot(y)
legend('global theta','global theta dot','wheel angular rate','wheel angular rate dot')
%------------------- LQR CONT closed-loop dynamics -----------------------%





Q = [100 0 0; 0 0.01 0; 0 0 0.01]
%N = [100,100,100]'
R = 0.01
K = lqr(sys_ctrl,Q,R);
              % If we used the classic LQR what would our gain look like for each state space var
%Des_poles = [-1.0,-2.0,-3.0];
%K = place(A,B_ctrl,Des_poles);
%CLP = eig(A-B_ctrl*K)
tspan = 0:Ts:5;
%tspan = linspace(0,10,10000); % time
y0 = [pi/2; 0; 0]; % [x xdot theta theta_dot]

T_ext = 1;
%[t,y] = ode45(@(t,y)((A-B_ctrl*K)*(y-[0; 0; 0])),tspan,y0); % LQR sol


x = zeros(size(A,1),length(tspan));
y = zeros(size(C,1),length(tspan));
u = zeros(size(Bd_ctrl,2),length(tspan));

x0 = [pi/2;0;0];
x(:,1) = x0;
for k=1:length(x)

    u(k) = -K*x(:,k);
    x(:,k+1) = Ad*x(:,k) + Bd_ctrl*u(k);
    y(:,k)   = Cd*x(:,k) + Dd_ctrl*u(k);
end

plot(y)
legend('global theta','global theta dot','wheel angular rate','wheel angular rate dot')
%------------------- LQR CONT closed-loop dynamics -----------------------%

% DLQR From class


clear N
clear Q 
clear R 
clear X 
clear F 
clear P
clear u

Q = [10000 0 0; 0 0.001 0; 0 0 0.4];
%N = [100,100,100]'
R = 0.001;




N = 5001; %length(t_dis);
P{N} =1*eye(3)*Q;
for k = N-1:-1:1
    F{k} = ((R+Bd_ctrl'*P{k+1}*Bd_ctrl)^-1)*Bd_ctrl'*P{k+1}*Ad;
    P{k}= Q+F{k}'*R*F{k}+(Ad-Bd_ctrl*F{k})'*P{k+1}*(Ad-Bd_ctrl*F{k});
end 
%N-1:-1:1
X(:,1) = [pi/2;0;0];
xgoal = [0;0;0];
%xprime = X(:,1)-xgoal
%z = [xprime;1];
%xgoaltest = [linspace(0, 10, 200)];


rng(0) % same seed any time
mu=(9.6*10^(-7)+3.7*10^(-7)+2.1*10^(-5)+1.5*10^(-6))/4;
sigma1=sqrt(1*10^(-7));
disturbance=abs(sigma1*randn(1))+mu*sin(x(1));
E = [1;0;0];

for i = 1:N-1  
    u(i) = -F{i}*(X(:,i)); %X(:,i);%X(:,i);
    %f = A*xgoal-xgoal;
    %xprime = X(:,i)-xgoal;
    %z = [A f; 0 0 1]*[xprime;1]+[B;0]*u(i);
    X(:,i+1) = Ad * X(:,i) + Bd_ctrl*u(i)+Bd(:,2)*disturbance;
    disturbance=abs(sigma1*randn(1))+mu*sin(x(1));

end


real_x = X(1,:);
disp(min(find(real_x<=0.1571 )))
TimeToRise = min(find(real_x <=0.1571 ))

Z_vec(TimeToRise:end)

RMSE = sqrt(mean((Z_vec(TimeToRise:end)  - realx(TimeToRise:end)).^2))

figure;
plot(1:1:N,X(1,:),'LineWidth',3)
title('State 1: CubeSat Theta(rad) vs time steps')
grid on
hold on
Z_vec = zeros(1,length(real_x));
plot(Z_vec,'LineWidth',3)
hold on
plot((pi/2)*.10*ones(1,length(real_x)),'LineWidth',3)
hold on 
grid on
plot(-(pi/2)*.10*ones(1,length(real_x)),'LineWidth',3)
legend('CubeSat Theta(rad)','Theta Goal(rad)','+10% Threshold(rad)','-10% Threshold(rad)')
xlabel('Time Steps')
ylabel('State 1: CubeSat Theta(rad) ')

figure;
plot(1:1:N,X(2,:),'LineWidth',3)
grid on
title('State 2 vs time steps')
xlabel('Time Steps')
ylabel('State 2 Omega(Rad/sec)')

figure;

plot(1:1:N,X(3,:),'LineWidth',3)
title('State 3 vs time steps')
xlabel('Time Steps')
ylabel('State 3 Omega Dot (Rad/sec^2)')
grid on



figure;
plot(1:1:N-1,u,'b','LineWidth',3)
title('input u vs time')
xlabel('Time steps')
ylabel('Input u')
grid on







% 
% 
% 
% figure(2)
% plot(real_x,'LineWidth',3)
% grid on
% hold on
% Z_vec = zeros(1,length(real_x));
% plot(Z_vec,'LineWidth',3)
% hold on
% plot((pi/2)*.10*ones(1,length(real_x)),'LineWidth',3)
% hold on 
% grid on
% plot(-(pi/2)*.10*ones(1,length(real_x)),'LineWidth',3)
% title('State 1: CubeSat Theta(rad) vs time steps')
% legend('CubeSat Theta(rad)','Theta Goal(rad)','+10% Threshold(rad)','-10% Threshold(rad)')
% xlabel('Time Steps')
% ylabel('State 1: CubeSat Theta(rad) ')
% 
% disp(min(find(real_x <=0.1571 )))
% TimeToRise = min(find(real_x <=0.1571 ))
% 
% Z_vec(TimeToRise:end)
% 
% RMSE = sqrt(mean((Z_vec(TimeToRise:end)  - real_x(TimeToRise:end)).^2))
% 
% 
% 
% figure(3)
% plot(real_x2,'LineWidth',3)
% title('State 2 vs time steps')
% xlabel('Time Steps')
% ylabel('State 2 Omega(Rad/sec)')
% grid on
% 
% 
% figure(4)
% plot(real_x3,'LineWidth',3)
% title('State 3 vs time steps')
% xlabel('Time Steps')
% ylabel('State 3 Omega Dot (Rad/sec^2)')
% grid on
% 





%% 

plot_3d_cube_DANCOHEN(X(1,:),'DanCohenTestingLQR.avi')




% %%%% MPC $$$$
% 
% 
% % MPC data
% Q = [1000 0 0; 0 0.0001 0; 0 0 0.0001]
% %N = [100,100,100]'
% R = 0.0001
% H =  eye(4)*1e6;
% 
% % Initial state
% nx = 3; % Number of states
% nu = 1; % Number of inputs
% 
% 
% xgoal = [0;0;0];
% x = [pi/2;0;0];% x0
% %f = Ad*xgoal-xgoal;
% %xprime = x-xgoal;
%   
% %z = [Ad f; 0 0 0 1]*[xprime;1];%+[B;0]*U(1);
% 
% %Bz = [Bd_ctrl;0];
% %Az = [Ad f; 0 0 0 1];
% 
% N = 500;
% 
% 
% % Initial state
% x0 = [pi/2;0;0];
% 
% 
% u = sdpvar(repmat(nu,1,N),repmat(1,1,N));
% x = sdpvar(repmat(nx,1,N+1),repmat(1,1,N+1));
% 
% constraints = [];
% objective = 0;%z{N}'*H*z{N};
% for k = 1:N
%  objective = objective +x{k}'*Q*x{k} + u{k}*R*u{k};
%  constraints = [constraints, x{k+1} == Ad*x{k} + Bd_ctrl*u{k}];
%  constraints = [constraints, -25 <= u{k}<= 25, [-pi/2;-180;-2000]<=x{k+1}<=[(pi/2)+0.1;180;2000]];
% end
% %ops = sdpsettings('verbose',2);
% 
% controller = optimizer(constraints, objective,sdpsettings('solver','gurobi'),x{1},[u{:}]);
% 
% xgoal = [0;0;0];
% x = [pi/2;0;0];% x0
% clf;
% hold on
% real_x = [pi/2];
% real_x2 = [0];
% real_x3 = [0];
% implementedU = [];
% real_z = [];
% real_z2 = [];
% %z = [A f; 0 0 0 1]*[xprime;1];
% 
% figure(1)
% for i = 1:5000
%   xprime = x-xgoal;
%   %f = A*xgoal-xgoal;
%   U = controller{x};  
%   stairs(i:i+length(U)-1,U,'r')
%   x = Ad*x + Bd_ctrl*U(1); % foward simulation starting with x0
% 
%   %z = [Ad f; 0 0 0 1]*z+[Bd_ctrl;0]*U(1);
%   
%   %real_z = [real_z;z(1)];
%   %real_z2 = [real_z2;z(2)];
%   real_x = [real_x;x(1)];
%   real_x2 = [real_x2;x(2)];
%   real_x3 = [real_x3;x(3)];
%   %pause(0.05)
%   stairs(i:i+length(U)-1,U,'k')
%   implementedU = [implementedU;U(1)];
% end
% %plotting 
% hold on
% stairs(implementedU,'b')
% title('Input vs time steps')
% xlabel('Time Steps')
% ylabel('Input')
% 
% 
% 
% figure(2)
% plot(real_x)
% title('State 1  vs time steps')
% xlabel('Time Steps')
% ylabel('State 1 ')
% 
% 
% figure(3)
% plot(real_x2)
% title('State 2 vs time steps')
% xlabel('Time Steps')
% ylabel('State 2')
% 
% 
% figure(4)
% plot(real_x3)
% title('State 3 vs time steps')
% xlabel('Time Steps')
% ylabel('State 3')
% 
% 
% figure(5)
% plot(real_z2)
% title('State 2 q vs time steps')
% xlabel('Time Steps')
% ylabel('State 2 q')
% 
% 

% %% Hinf
% load('OLGAGAIN.mat','K')
% 
% Pinf = ss(A,B_ctrl,C,D_ctrl)
% 
% NMEAS= 1;NCON = 1;
% [K,CL,GAM,INFO] = hinfsyn(Pinf,NMEAS,NCON)
% tspan = [0,10];
% %tspan = linspace(0,10,10000); % time
% y0 = [pi/2; 0; 0]; % [x xdot theta theta_dot]
% y = [0;0;0];
% T_ext = 1;
% [t,y] = ode45(@(t,y)((A-B*INFO.Ku)*(y-y0)),tspan,y0); % LQR sol
% plot(y)
% legend('global theta','global theta dot','wheel angular rate','wheel angular rate dot')





