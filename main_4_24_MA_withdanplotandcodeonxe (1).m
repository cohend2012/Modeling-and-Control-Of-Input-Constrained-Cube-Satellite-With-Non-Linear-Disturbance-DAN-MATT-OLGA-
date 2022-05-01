%% MAE 507 Project Satellite Model
% (Auer, Matthew), (Cohen, Dan), (Skowronek, Olga) 
clear all
close all
clc
%% Declare system models

sampling_freq = 1000;   % Hz
Ts = 1/sampling_freq;

[sys_ctrl2 , ctrb2ct, obsv2ct, state_constraints2ct, input_constraints2ct] = ...
    system_with_characteristics('Satelite_theta_omega_omega_dot_ss','ct');
[sysd_ctrl2, ctrb2dt, obsv2dt, state_constraints2dt, input_constraints2dt] = ...
    system_with_characteristics('Satelite_theta_omega_omega_dot_ss','dt',sampling_freq);
[sys_full3 , ctrb3ct, obsv3ct, state_constraints3ct, input_constraints3ct] = ...
    system_with_characteristics('Satelite_sys_with_disturbance_ss','ct');
[sysd_full3, ctrb3dt, obsv3dt, state_constraints3dt, input_constraints3dt] = ...
    system_with_characteristics('Satelite_sys_with_disturbance_ss','dt',sampling_freq);
[sys_mot4ct, ctrb_na, obsv_na, state_constraints_na, input_constraints4ct] = ...
    system_with_characteristics('Fixed_motor_model_tf','ct');


% %% ------------------------ Open-Loop Dynamics --------------------------%%
% x0 = [0;0;0];
% v0 = input_constraints3ct(1,2); % Constant max motor voltage signal
% T_ext0 = 1; % Constant disturbance torque.
% 
% duration = 5;  % number of sec runtime
% N = round(duration/Ts); % compute the number of time steps
% t = linspace(0,duration,N);    % simulate over duration with N steps
% v = linspace(v0,v0,N);  % Constant input voltage
% T_ext = linspace(0,T_ext0,N);
% 
% % continuous-time solution to the controllable system (no disturbance)
% y_ct_ctrl = lsim(sys_ctrl2,v,t,x0);   
% 
% % continuous-time solution to the full system
% y_ct = lsim(sys_full3,[v',T_ext'],t,x0);   
% 
% Ad = sysd_full3.A; Bd = sysd_full3.B; Cd = sysd_full3.C; Dd = sysd_full3.D;
% % discrete-time solution
% x = zeros(size(Ad,1),length(t));
% y = zeros(size(Cd,1),length(t));
% U = zeros(size(Bd,2),length(t));
% x(:,1) = x0;
% for k = 1:length(t)
%     U(:,k) = [v(k); T_ext(k)];
%     x(:,k+1) = Ad*x(:,k) + Bd*U(:,k);
%     y(:,k)   = Cd*x(:,k) + Dd*U(:,k);
% end
% 
% figure(1)
% title('Continuous-Time and Discrete-Time Solutions')
% hold on
% plot(t,y_ct_ctrl,'--k','DisplayName','CT Contrtollable Portion Solution','LineWidth',2)
% plot(t,y_ct,'Color','#0072BD','DisplayName','CT Full Solution', 'LineWidth',2)
% plot(t,y,'o','MarkerEdgeColor','k','MarkerFaceColor','#D95319','DisplayName', ...
%     'DT Solution','LineWidth',1,'MarkerIndices',[1:(N/50):N])
% stairs(t,y,'k','DisplayName','DT Full Solution','LineWidth',1);
% ylim([1.1*min(y)-.01, 1.1*max(y)+.01])
% xlim([t(1), t(N)])
% hold off
% legend('Location','best')
% grid on
% 
% %% ------------------------ Place ---------------------------------------%%
% x0 = [1;0;0];
% T_ext0 = 1; % Constant disturbance torque.
% r0 = 0; % Reference signal value.
% 
% % This type of system is missing an integrator and therefore can't null out
% % constant disturbance torques. If the torque was a ramp, there would need
% % to be 2 integrators and so-on.
% Des_poles = [0.999 0.985 0.980];
% Ad_ctrl = sysd_ctrl2.A; Bd_ctrl = sysd_ctrl2.B; Cd_ctrl = sysd_ctrl2.C; Dd_ctrl = sysd_ctrl2.D;
% K = place(Ad_ctrl,Bd_ctrl,Des_poles);
% 
% Ad = sysd_full3.A; Bd = sysd_full3.B; Cd = sysd_full3.C; Dd = sysd_full3.D;
% duration = 20;  % number of sec runtime
% N = round(duration/Ts); % compute the number of time steps
% t = linspace(0,duration,N);    % simulate over duration with N steps
% r = linspace(r0,r0,N);  % Constant input voltage
% T_ext = linspace(0,0,N);
% T_ext(round(N/2):round(N/2) + round(1/Ts)) = T_ext0; %round(N/2) + round(1/Ts)
% 
% v_min = input_constraints3dt(1,1);
% v_max = input_constraints3dt(1,2);
% omega_f_min = state_constraints3dt(2,1);
% omega_f_max = state_constraints3dt(2,2);
% 
% % discrete-time solution
% x = zeros(size(Ad,1),length(t));
% y = zeros(size(Cd,1),length(t));
% u = zeros(size(Bd_ctrl,2),length(t));
% x(:,1) = x0;
% for k = 1:length(t)
%     u(k) = -K*x(:,k);
%     x(:,k+1) = Ad*x(:,k) + Bd_ctrl*u(k) + Bd(:,2)*T_ext(k);
%     y(:,k)   = Cd*x(:,k) + Dd_ctrl*u(k) + Dd(:,2)*T_ext(k);
% end
% 
% figure(2)
% title('Closed-Loop Discrete-Time Solution, Angles')
% hold on
% plot(t,x(1,1:N),'Color','#0072BD','DisplayName','Theta Satellite','LineWidth',2)
% xlim([t(1), t(N)])
% hold off
% legend('Location','best')
% grid on
% 
% figure(3)
% title('Closed-Loop Discrete-Time Solution, Full State')
% hold on
% % Plot the satellite angle
% plot(t,x(1,1:N),'Color','#0072BD','DisplayName','Theta Satellite','LineWidth',2)
% % Plot the voltage input to the system
% plot(t,u(1:N),'Color',	'#7E2F8E','DisplayName','Voltage Input','LineWidth',2)
% % Plot the disturbance input to the system
% plot(t,T_ext(1:N),'Color','#D95319','DisplayName','External Disturbance Torque','LineWidth',2)
% % Plot the voltage bounds
% plot(t,linspace(v_min,v_min,N),'-.','Color','#7E2F8E','DisplayName','Min Voltage','LineWidth',2)
% plot(t,linspace(v_max,v_max,N),'-.','Color','#7E2F8E','DisplayName','Max Voltage','LineWidth',2)
% 
% % Plot the flywheel angular rate
% plot(t,x(2,1:N),'Color','#EDB120','DisplayName','Flywheel Rate','LineWidth',2)
% % Plot the flywheel rate bounds
% plot(t,linspace(omega_f_min,omega_f_min,N),'-.','Color','#EDB120','DisplayName','Min Omega','LineWidth',2)
% plot(t,linspace(omega_f_max,omega_f_max,N),'-.','Color','#EDB120','DisplayName','Max Omega','LineWidth',2)
% 
% xlim([t(1), t(N)])
% ylim([1.1*min(omega_f_min,v_min), 1.1*max(omega_f_max,v_max)])
% hold off
% legend('Location','best')
% grid on
% 
% 
%% ------------------------ LQR -----------------------------------------%%
B_dist = sysd_full3.B(:,2);
D_dist = sysd_full3.D(:,2);

Q = [100 0 0; 0 .01 0; 0 0 .01];
R = .01;
K = lqr(sys_ctrl2,Q,R);

A = sys_ctrl2.A; B = sys_ctrl2.B; C = sys_ctrl2.C; D = sys_ctrl2.D;
CLP = eig(A - B*K);
A_cl = (A - B*K); 
B_cl = B*0;
C_cl = (C - D*K);
D_cl = D*0;

sys2_ctrl_cl = ss(A_cl,B_cl,C_cl,D_cl);
sys2_ctrl_cl_d = c2d(sys2_ctrl_cl,Ts);

A_cld = sys2_ctrl_cl_d.A;
B_cld = sys2_ctrl_cl_d.B;
C_cld = sys2_ctrl_cl_d.C;
D_cld = sys2_ctrl_cl_d.D;

x0 = [1;0;0];
T_ext0 = 1; % Constant disturbance torque.
r0 = 0; % Reference signal value.

duration = 20;  % number of sec runtime
N = round(duration/Ts); % compute the number of time steps
t = linspace(0,duration,N);    % simulate over duration with N steps
r = linspace(r0,r0,N);  % Constant input voltage
T_ext = linspace(0,0,N);
T_ext(round(N/2):round(N)) = T_ext0; %round(N/2) + round(1/Ts)  OR round(N)

% discrete-time solution
x = zeros(size(A,1),length(t));
y = zeros(size(C,1),length(t));
u = zeros(size(B,2),length(t));
x(:,1) = x0;
x_int = [0;0;0];
for k = 1:length(t)
    u(k) = -K*x(:,k);
    x(:,k+1) = A_cld*x(:,k) + B_dist*T_ext(k);
    y(:,k)   = C_cld*x(:,k) + D_dist*T_ext(k);
end
u(1) = 0;

figure(4)
title('Closed-Loop Discrete-Time Solution, Angles')
hold on
plot(t,x(1,1:N),'Color','#0072BD','DisplayName','Theta Satellite','LineWidth',2)
%plot(t,theta_f,'Color',	'#7E2F8E','DisplayName','Theta Flywheel','LineWidth',2)
xlim([t(1), t(N)])
ylim([min(0,min(x(1,1:N)))-.1, max(x(1,1:N))+.1])
hold off
legend('Location','best')
grid on

figure(5)
title('Closed-Loop Discrete-Time Solution, Full State')
hold on
% Plot the satellite angle
plot(t,x(1,1:N),'Color','#0072BD','DisplayName','Theta Satellite','LineWidth',2)
% Plot the voltage input to the system
plot(t,u(1:N),'Color',	'#7E2F8E','DisplayName','Voltage Input','LineWidth',2)
% Plot the disturbance input to the system
plot(t,T_ext(1:N),'Color','#D95319','DisplayName','External Disturbance Torque','LineWidth',2)
% Plot the voltage bounds
plot(t,linspace(input_constraints2ct(1),input_constraints2ct(1),N), ...
    'k-.','DisplayName','Min Voltage','LineWidth',2)
plot(t,linspace(input_constraints2ct(2),input_constraints2ct(2),N), ...
    'k-.','DisplayName','Max Voltage','LineWidth',2)

% Plot the flywheel angular rate
plot(t,x(2,1:N),'Color','#EDB120','DisplayName','Flywheel Rate','LineWidth',2)
% Plot the flywheel rate bounds
plot(t,linspace(state_constraints2ct(2,1),state_constraints2ct(2,1),N), ...
    'b-.','DisplayName','Min Omega','LineWidth',2)
plot(t,linspace(state_constraints2ct(2,2),state_constraints2ct(2,2),N), ...
    'b-.','DisplayName','Max Omega','LineWidth',2)

% %plot(t,theta_f,'Color',	'#7E2F8E','DisplayName','Theta Flywheel','LineWidth',2)
% plot(t,x(2,1:N),'Color','#D95319','DisplayName','Flywheel Rate','LineWidth',2)
% %plot(t,x(3,1:N),'Color','#EDB120','DisplayName','Flywheel Rate Dot','LineWidth',2)
% plot(t,u(1:N),'Color',	'#7E2F8E','DisplayName','Theta Flywheel','LineWidth',2)
xlim([t(1), t(N)])
% ylim([-1 1])
% ylim([1.1*input_constraints2dt(1),1.1*input_constraints2ct(2)])
ylim([1.1*min(state_constraints2ct(2,1),input_constraints2ct(1)), 1.1*max(state_constraints2ct(2,2),input_constraints2ct(2))])
hold off
legend('Location','best')
grid on

%% ------------------------ LQR + integrator Code -----------------------%
antiwindup_sample = round(1*sampling_freq);

B_dist = sysd_full3.B(:,2);
D_dist = sysd_full3.D(:,2);

Q = [1000 0 0; 0 .01 0; 0 0 .01];
R = .01;
[K,S,e] = lqr(sys_ctrl2,Q,R);
Ki = 0.0000005*[1 0 0;
                0 0 0;
                0 0 0];

A = sys_ctrl2.A; B = sys_ctrl2.B; C = sys_ctrl2.C; D = sys_ctrl2.D;
CLP = eig(A - B*K);
A_cl = (A - B*K); 
B_cl = B*0;
C_cl = (C - D*K);
D_cl = D*0;

sys2_ctrl_cl = ss(A_cl,B_cl,C_cl,D_cl);
sys2_ctrl_cl_d = c2d(sys2_ctrl_cl,Ts);

A_cld = sys2_ctrl_cl_d.A;
B_cld = sys2_ctrl_cl_d.B;
C_cld = sys2_ctrl_cl_d.C;
D_cld = sys2_ctrl_cl_d.D;

x0 = [1;0;0];
T_ext0 = 1; % Constant disturbance torque.
r0 = 0; % Reference signal value.

duration = 20;  % number of sec runtime
N = round(duration/Ts); % compute the number of time steps
t = linspace(0,duration,N);    % simulate over duration with N steps
r = linspace(r0,r0,N);  % Constant input voltage
T_ext = linspace(0,0,N);
T_ext(round(N/2):round(N)) = T_ext0; %round(N/2) + round(1/Ts)  OR round(N)

% discrete-time solution
x = zeros(size(A,1),length(t));
y = zeros(size(C,1),length(t));
u = zeros(size(B,2),length(t));
x(:,1) = x0;
x_int = [0;0;0];
for k = 1:length(t)
    u(k) = -K*x(:,k);
    if k > antiwindup_sample
        x_int = sum(x(:,antiwindup_sample:k)')';
    end
    x(:,k+1) = A_cld*x(:,k) - Ki*x_int + B_dist*T_ext(k);
    y(:,k)   = C_cld*x(:,k) + D_dist*T_ext(k);
end
u(1) = 0;
% Calculate theta of the flywheel assuming it starts from zero
%%%%%theta_f = -((Js + Jf)/Jf)*x(1,1:N) + ((Js + Jf)/Jf)*x0(1) ;

figure(6)
title('Closed-Loop Discrete-Time Solution, Angles')
hold on
plot(t,x(1,1:N),'Color','#0072BD','DisplayName','Theta Satellite','LineWidth',2)
%plot(t,theta_f,'Color',	'#7E2F8E','DisplayName','Theta Flywheel','LineWidth',2)
xlim([t(1), t(N)])
ylim([min(0,min(x(1,1:N)))-.1, max(x(1,1:N))+.1])
hold off
legend('Location','best')
grid on

figure(7)
title('Closed-Loop Discrete-Time Solution, Full State')
hold on
% Plot the satellite angle
plot(t,x(1,1:N),'Color','#0072BD','DisplayName','Theta Satellite','LineWidth',2)
% Plot the voltage input to the system
plot(t,u(1:N),'Color',	'#7E2F8E','DisplayName','Voltage Input','LineWidth',2)
% Plot the disturbance input to the system
plot(t,T_ext(1:N),'Color','#D95319','DisplayName','External Disturbance Torque','LineWidth',2)
% Plot the voltage bounds
plot(t,linspace(input_constraints2ct(1),input_constraints2ct(1),N), ...
    'k-.','DisplayName','Min Voltage','LineWidth',2)
plot(t,linspace(input_constraints2ct(2),input_constraints2ct(2),N), ...
    'k-.','DisplayName','Max Voltage','LineWidth',2)

% Plot the flywheel angular rate
plot(t,x(2,1:N),'Color','#EDB120','DisplayName','Flywheel Rate','LineWidth',2)
% Plot the flywheel rate bounds
plot(t,linspace(state_constraints2ct(2,1),state_constraints2ct(2,1),N), ...
    'b-.','DisplayName','Min Omega','LineWidth',2)
plot(t,linspace(state_constraints2ct(2,2),state_constraints2ct(2,2),N), ...
    'b-.','DisplayName','Max Omega','LineWidth',2)

% %plot(t,theta_f,'Color',	'#7E2F8E','DisplayName','Theta Flywheel','LineWidth',2)
% plot(t,x(2,1:N),'Color','#D95319','DisplayName','Flywheel Rate','LineWidth',2)
% %plot(t,x(3,1:N),'Color','#EDB120','DisplayName','Flywheel Rate Dot','LineWidth',2)
% plot(t,u(1:N),'Color',	'#7E2F8E','DisplayName','Theta Flywheel','LineWidth',2)
xlim([t(1), t(N)])
% ylim([-1 1])
% ylim([1.1*input_constraints2dt(1),1.1*input_constraints2ct(2)])
ylim([1.1*min(state_constraints2ct(2,1),input_constraints2ct(1)), 1.1*max(state_constraints2ct(2,2),input_constraints2ct(2))])
hold off
legend('Location','best')
grid on

%% -------------------------- Motor Model ------------------------------ %%
figure(8)
step(30*sys_mot4ct)

figure(9)
N = round(duration/Ts); % compute the number of time steps
t = linspace(0,duration,N);
u = zeros(size(B,2),length(t));
u(1:N) = 0.5*input_constraints4ct(2)+0.5*input_constraints4ct(2)*t/duration;
lsim(sys_mot4ct,u,t)
grid on

% ------------------------------- LQG --------------------------------- %%
B_dist = sysd_full3.B(:,2);
D_dist = sysd_full3.D(:,2);

Q = [100 0 0; 0 .01 0; 0 0 .01];
R = .01;
K = lqr(sys_ctrl2,Q,R);

Ad = sysd_ctrl2.A; Bd = sysd_ctrl2.B; Cd = sysd_ctrl2.C; Dd = sysd_ctrl2.D;
A = sys_ctrl2.A; B = sys_ctrl2.B; C = sys_ctrl2.C; D = sys_ctrl2.D;

cov_scale = (9.6*10^(-7)+3.7*10^(-7))/2;
Vd = 0.02*eye(size(Ad,1));
Vn = 0.001;

Kf = lqr(A',C',Vd,Vn)';

BF = [B Vd 0*B];

duration = 10;  % number of sec runtime
N = round(duration/Ts); % compute the number of time steps
t = linspace(0,duration,N);    % simulate over duration with N steps
r = zeros(1,N);

sysCL = ss(A-B*K,0*B,C-D*K,0*D);
sysCLd = c2d(sysCL,Ts);
Ad_CL = sysCLd.A; Bd_CL = sysCLd.B; Cd_CL = sysCLd.C; Dd_CL = sysCLd.D;

sysKF = ss(A-Kf*C, [B Kf], eye(size(A,1)), 0*[B Kf]);
sysKFd = c2d(sysKF,Ts);
Ad_KF = sysKFd.A; Bd_KF = sysKFd.B; Cd_KF = sysKFd.C; Dd_KF = sysKFd.D;

Bd_full = sysd_full3.B;   % full system B vector
Bd_dist = Bd_full(:,2);   % disturbance B vector

% discrete-time solution
x = zeros(size(Ad,1),length(t));
xtrue = zeros(size(Ad,1),length(t));
xe = zeros(size(Ad,1),length(t));
y = zeros(size(Cd,1),length(t));
u = zeros(size(Bd,1),length(t));
w = Vd*Vd*randn(3,size(t,2));
v = Vn*Vn*randn(size(t));
xtrue(:,1) = [pi/2; 0; 0];
x(:,1) = xtrue(:,1);
xe(:,1) = [0; 0; 0];
%u(:,k) = K*xe(:,1);
%y(:,1) = Cd*x(:,1) + Dd*u(1) + v(1);
for k = 1:length(t)
    x(:,k+1)  = Ad_CL*x(:,k) + Bd_dist*T_ext(k) + w(:,k);
    y(k)          = Cd_CL*x(:,k) + v(k);
    xtrue(:,k+1)  = Ad_CL*xtrue(:,k) + Bd_dist*T_ext(k);
    Kd = -(Ad_CL(1,:) - Ad(1,:))/Bd(1);
    u(k) = -Kd*xtrue(:,k);
    xe(:,k+1) = Ad_KF*xe(:,k) + Bd_KF*[u(k); y(k)'];  % update estimate using Kalman Filter
end

figure(10)
title('Discrete-Time LQG Solutions')
hold on
% plot(t,y,'o','MarkerEdgeColor','k','MarkerFaceColor','#D95319','DisplayName', ...
%     'DT Solution','LineWidth',1,'MarkerIndices',[1:(N/50):N])
plot(t,y,'b','DisplayName','Output Measured','LineWidth',1);
plot(t,xe(1,1:end-1),'k--','DisplayName','Estimated Theta')
plot(t,xtrue(1,1:end-1),'r','DisplayName','True Theta')
%ylim([1.1*min(y)-.01, 1.1*max(y)+.01])
%xlim([t(1), t(N)])
hold off
legend('Location','best')
grid on


N = length(t);

real_x = xe(1,:);


figure;
plot(1:1:N+1,xe(1,:),'LineWidth',3)
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

disp(min(find(real_x<=0.1571 )))
TimeToRise = min(find(real_x(100:end) <=0.1571 ))

Z_vec(TimeToRise:end)

RMSE = sqrt(mean((Z_vec(TimeToRise:end)  - real_x(TimeToRise:end)).^2))


figure;
plot(1:1:N+1,xe(2,:),'LineWidth',3)
grid on
title('State 2 vs time steps')
xlabel('Time Steps')
ylabel('State 2 Omega(Rad/sec)')

figure;

plot(1:1:N+1,xe(3,:),'LineWidth',3)
title('State 3 vs time steps')
xlabel('Time Steps')
ylabel('State 3 Omega Dot (Rad/sec^2)')
grid on



figure;
plot(1:1:length(u),u,'b','LineWidth',3)
title('input u vs time')
xlabel('Time steps')
ylabel('Input u')
grid on







