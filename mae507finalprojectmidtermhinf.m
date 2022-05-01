
clear all
close all
clc

% Bring in Data from Problem Statement

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
% Input controllable system matrix (using only control input (voltage))
B = [0        ;
     0        ;
     k/(L*Jf)];
C = [1 0 0];
D = 0;

olsys=ss(A,B,C,D);

n = size(A,1); %3
m = size(B,2); %1

% Solver selection and output suppression
OPTIONS = sdpsettings('solver','sedumi','verbose',0);
w = warning ('off','all');

disp('----------------------------------------------------------');
disp('BUILD THE 9 MATRIX TRACKING REPRESENTATION');

% Build the 9 matrix tracking Representation
A = A;
B1 = [zeros(n,m) B zeros(n,m)];
B2 = B;
C1 = [C;zeros(m,n)];
C2 = [zeros(m,n);C];
D11 = [-eye(m) D zeros(m,m);...
       zeros(m,m) zeros(m,m) zeros(m,m)];
D12 = [D;eye(m)];
D21 = [eye(m) zeros(m,m) zeros(m,m);...
       zeros(m,m) D eye(m)];
D22 = [zeros(m,m); D];

disp('The 9 matrix representation is shown below:');
A
B1
B2
C1
C2
D11
D12
D21
D22

% measure numbers of inputs and outputs 
eps=.00009;    % degree of strict positivity   
ns=size(A,1);   % number of states
nc=size(B2,2);  % number of actuators
nm=size(C2,1);  % number of sensors
nd=size(B1,2);  % number of external inputs
no=size(C1,1);  % number of regulated outputs

% Define Empty Controller
Ak=0;
Bk=0;
Ck=0;
Dk=0;

% Close the loop with Lower LFT
plant=ss(A,[B1 B2],[C1;C2],[D11 D12; D21 D22]);
controller=ss(Ak,Bk,Ck,Dk);
sys_cl=lft(plant,controller);
disp('The Closed Loop H-inf norm with K=0 is:');
disp(norm(sys_cl,Inf))
disp('The Closed Loop H2 norm with K=0 is:');
disp(norm(sys_cl))
disp('The H2 norm should be infinite since the feedthrough term D11 is non-zero');


%% SOLVING OPTIMAL OUTPUT FEEDBACK CONTROL
disp('----------------------------------------------------------');
disp('SOLVING OPTIMAL OUTPUT FEEDBACK CONTROL');

% Declare the variables
gamma=sdpvar(1);               % represents the bound on the H-infinity norm of the CL system.
X1=sdpvar(ns);
Y1=sdpvar(ns);
An=sdpvar(ns,ns,'full');
Cn=sdpvar(nc,ns,'full');
Dn=sdpvar(nc,nm,'full');
Bn=sdpvar(ns,nm,'full');

% declare constraints
F=[X1>=eps*eye(ns)];
F=[F;Y1>=eps*eye(ns)];
F=[F;[X1 eye(ns); eye(ns) Y1]>=0];
MAT=[A*Y1+Y1*A'+B2*Cn+Cn'*B2'  (A'+An+(B2*Dn*C2)')'        B1+B2*Dn*D21           (C1*Y1+D12*Cn)'; 
     A'+An+(B2*Dn*C2)'         X1*A+A'*X1+Bn*C2+C2'*Bn'    X1*B1+Bn*D21           (C1+D12*Dn*C2)'  ;
     (B1+B2*Dn*D21)'           (X1*B1+Bn*D21)'             -gamma*eye(nd)          (D11+D12*Dn*D21)'  ;
     C1*Y1+D12*Cn              C1+D12*Dn*C2                D11+D12*Dn*D21         -gamma*eye(no)];

F=[F;MAT<=0];

% Solve the LMI, minimizing gamma
optimize(F,gamma,OPTIONS);
% disp('The predicted H-inf gain is:');
% gamman=value(gamma);
% disp(gamman);

% retrieve decision variables
X1n=value(X1); 
Y1n=value(Y1); 
Ann=value(An);
Bnn=value(Bn);
Cnn=value(Cn);
Dnn=value(Dn);
temp1=[Ann Bnn; Cnn Dnn]-[X1n*A*Y1n zeros(ns,nm); zeros(nc,ns) zeros(nc,nm)];

% Choose X2, Y2, so that X2*Y2'=I-X1*Y1;
% Choose Y2=I, X2=I-X1Y1
Y2n=eye(ns);
X2n=eye(ns)-X1n*Y1n;

% Reverse variable substitution
temp2=inv([X2n X1n*B2;zeros(nc,ns) eye(nc)])*temp1*inv([Y2n' zeros(ns,nm); C2*Y1n eye(nm)]);
Ak2=temp2(1:ns,1:ns);
Bk2=temp2(1:ns,(ns+1):(ns+nm));
Ck2=temp2((ns+1):(ns+nc), 1:ns);
Dk2=temp2((ns+1):(ns+nc), (ns+1):(ns+nm));
Dk=inv(eye(nc)-Dk2*D22)*Dk2;
Bk=Bk2*(eye(nm)-D22*Dk);
Ck=(eye(nc)-Dk*D22)*Ck2;
Ak=Ak2-Bk*inv(eye(nm)-D22*Dk)*D22*Ck;

% Close the loop with Lower LFT
plant=ss(A,[B1 B2],[C1;C2],[D11 D12; D21 D22]);
controller=ss(Ak,Bk,Ck,Dk);
K=[Ak,Bk;...
    Ck,Dk];
disp('The reconstructed controller is:')
disp(K)
sys_cl=lft(plant,controller);
disp('The actual closed loop H-inf gain is:');
disp(norm(sys_cl,Inf));


Tf = 100;
Ts = 0.001;
tt = [0:Ts:Tf];
uu1 = (pi/2)*exp(-tt);
uu2 = 0*sin(1*tt); 
uu3 = 0*sin(tt);
uu = [uu1' uu2' uu3'];
X0=[pi/2,0,0,0,0,0];
[Y,T,X]=lsim(-sys_cl,uu,tt,X0);

figure();
hold on
grid on
plot(T,Y(:,1),'r-','LineWidth',1)
plot(T,Y(:,2),'b-','LineWidth',1)
legend('Error in Tracking','Plant Input')
title('H-infinity Control Trajectory Tracking')
xlabel('Time (seconds)')
ylabel('Theta (radians)')
hold off

figure();
hold on
grid on
plot(T,uu1,'k-','LineWidth',1)
plot(T,X(:,1),'r-','LineWidth',1)
plot(T,X(:,2),'b-','LineWidth',1)
plot(T,X(:,3),'g-','LineWidth',1)
legend('reference trajectory','state1','state2','state3')
title('H-infinity Control Trajectory Tracking')
xlabel('Time (seconds)')
ylabel('State Units')


real_x = X(:,1);

figure();
plot(tt,X(:,1),'LineWidth',2)
grid on
hold on
Z_vec = zeros(1,length(real_x));
plot(T,Z_vec,'LineWidth',3)
hold on
plot(T,(pi/2)*.10*ones(1,length(real_x)),'LineWidth',3)
hold on 
grid on
plot(T,-(pi/2)*.10*ones(1,length(real_x)),'LineWidth',3)
title('State 1: CubeSat Theta(rad) vs time steps')
legend('CubeSat Theta(rad)','Theta Goal(rad)','+10% Threshold(rad)','-10% Threshold(rad)')
xlabel('Time (seconds)')
ylabel('State 1: CubeSat Theta(rad) ')


disp(min(find(real_x<=0.1571 )));
TimeToRise = min(find(real_x <=0.1571 ));

Z_vec(TimeToRise:end)

RMSE = sqrt(mean((Z_vec(TimeToRise:end)  - real_x(TimeToRise:end)).^2))


figure();
plot(tt,X(:,2),'LineWidth',2)
grid on
title('State 2 vs time steps')
xlabel('Time (seconds)')
ylabel('State 2 Omega(Rad/sec)')

figure();
plot(tt,X(:,3),'LineWidth',2)
title('State 3 vs time steps')
xlabel('Time (seconds)')
ylabel('State 3 Omega Dot (Rad/sec^2)')
grid on

figure();
plot(tt,uu1,'r','LineWidth',2)
title('input u vs time')
xlabel('Time (seconds)')
ylabel('Input u')
grid on

