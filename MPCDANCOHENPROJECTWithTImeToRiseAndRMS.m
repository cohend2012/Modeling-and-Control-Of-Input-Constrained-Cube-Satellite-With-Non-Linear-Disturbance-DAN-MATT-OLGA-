
% MPC data
Q = [1000 0 0; 0 0.0001 0; 0 0 0.0001]
%N = [100,100,100]'
R = 0.0001
H =  eye(4)*1e6;

E = [1;0;0];

% Initial state
nx = 3; % Number of states
nu = 1; % Number of inputs


xgoal = [0;0;0];
x = [pi/2;0;0];% x0
%f = Ad*xgoal-xgoal;
%xprime = x-xgoal;
  
%z = [Ad f; 0 0 0 1]*[xprime;1];%+[B;0]*U(1);

%Bz = [Bd_ctrl;0];
%Az = [Ad f; 0 0 0 1];

N = 500;


% Initial state
x0 = [pi/2;0;0];


u = sdpvar(repmat(nu,1,N),repmat(1,1,N));
x = sdpvar(repmat(nx,1,N+1),repmat(1,1,N+1));

constraints = [];
objective = 0;%z{N}'*H*z{N};
for k = 1:N
 objective = objective +x{k}'*Q*x{k} + u{k}*R*u{k};
 constraints = [constraints, x{k+1} == Ad*x{k} + Bd_ctrl*u{k}];
 constraints = [constraints, -25 <= u{k}<= 25, [-pi/2;-180;-2000]<=x{k+1}<=[(pi/2)+0.1;180;2000]];
end
%ops = sdpsettings('verbose',2);

controller = optimizer(constraints, objective,sdpsettings('solver','gurobi'),x{1},[u{:}]);

xgoal = [0;0;0];
x = [pi/2;0;0];% x0
clf;
hold on
real_x = [pi/2];
real_x2 = [0];
real_x3 = [0];
implementedU = [];
real_z = [];
real_z2 = [];
%z = [A f; 0 0 0 1]*[xprime;1];
%disturbance = randn(1)*.001;

rng(0) % same seed any time
mu=(9.6*10^(-7)+3.7*10^(-7)+2.1*10^(-5)+1.5*10^(-6))/4;
sigma1=1*10^(-7);
disturbance=abs(sigma1*randn(1))+mu*sin(x(1))

figure(1)
for i = 1:5000
  xprime = x-xgoal;
  %f = A*xgoal-xgoal;
  U = controller{x};  
  stairs(i:i+length(U)-1,U,'r','LineWidth',3)
  x = Ad*x + Bd_ctrl*U(1)+ Bd(:,2)*disturbance; % foward simulation starting with x0
  % disturbance = randn(1)*.001;
  %z = [Ad f; 0 0 0 1]*z+[Bd_ctrl;0]*U(1);
  disturbance=abs(sigma1*randn(1))+mu*sin(x(1));
  
  %real_z = [real_z;z(1)];
  %real_z2 = [real_z2;z(2)];
  real_x = [real_x;x(1)];
  real_x2 = [real_x2;x(2)];
  real_x3 = [real_x3;x(3)];
  %pause(0.05)
  stairs(i:i+length(U)-1,U,'k')
  implementedU = [implementedU;U(1)];
end
%plotting 
hold on
stairs(implementedU,'b','LineWidth',3)
grid on
title('Input vs time steps')
xlabel('Time Steps')
ylabel('Input')



figure(2)
plot(real_x,'LineWidth',3)
grid on
hold on
Z_vec = zeros(1,length(real_x));
plot(Z_vec,'LineWidth',3)
hold on
plot((pi/2)*.10*ones(1,length(real_x)),'LineWidth',3)
hold on 
grid on
plot(-(pi/2)*.10*ones(1,length(real_x)),'LineWidth',3)
title('State 1: CubeSat Theta(rad) vs time steps')
legend('CubeSat Theta(rad)','Theta Goal(rad)','+10% Threshold(rad)','-10% Threshold(rad)')
xlabel('Time Steps')
ylabel('State 1: CubeSat Theta(rad) ')

disp(min(find(real_x <=0.1571 )))
TimeToRise = min(find(real_x <=0.1571 ))

Z_vec(TimeToRise:end)

RMSE = sqrt(mean((Z_vec(TimeToRise:end)  - real_x(TimeToRise:end)).^2))



figure(3)
plot(real_x2,'LineWidth',3)
title('State 2 vs time steps')
xlabel('Time Steps')
ylabel('State 2 Omega(Rad/sec)')
grid on


figure(4)
plot(real_x3,'LineWidth',3)
title('State 3 vs time steps')
xlabel('Time Steps')
ylabel('State 3 Omega Dot (Rad/sec^2)')
grid on



plot_3d_cube_DANCOHEN(real_x,'DanCohenTestingMPC.avi')

