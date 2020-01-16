%%% DELIVERABLE 3.1 %%%

set(0,'defaultfigurecolor',[1 1 1]);
Ts =1/5;
quad = Quad(Ts);
[xs,us] = quad.trim();
sys = quad.linearize(xs,us);
[sys_x, sys_y, sys_z, sys_yaw] = quad.decompose(sys,xs,us);

%% MPC for x

N_steps = 50;
mpc_x  = MPC_Control_x(sys_x, Ts);
x = [];
ux = [];
x0 = [0; 0; 0; 2];
x(:,1) = x0;
i = 1;
try
while i<N_steps 

ux(:,i) = mpc_x.get_u(x(:,i)); %Take the discretized matrices
x(:,i+1) = mpc_x.A*x(:,i) + mpc_x.B*ux(:,i);

i = i + 1;
end
catch
error('---> Initial state is outside the feasible set <---\n');
end

% Plotting the results
figure
hold on; grid on;
subplot(5,1,1)
hold on; grid on;
plot([0:Ts:(N_steps-1)*Ts],x(1,:),'-k','markersize',20,'linewidth',2);
ylabel('Vel pitch [rad/s]')

subplot(5,1,2)
hold on; grid on;
plot([0:Ts:(N_steps-1)*Ts],x(2,:),'-k','markersize',20,'linewidth',2);
ylabel('Pitch [rad]')

subplot(5,1,3)
hold on; grid on;
plot([0:Ts:(N_steps-1)*Ts],x(3,:),'-k','markersize',20,'linewidth',2);
ylabel('Vel x [m/s]')

subplot(5,1,4)
hold on; grid on;
plot([0:Ts:(N_steps-1)*Ts],x(4,:),'-k','markersize',20,'linewidth',2);
ylabel('x [m]')

subplot(5,1,5)
hold on; grid on;
stairs([0:Ts:(N_steps-2)*Ts],ux,'k','markersize',20,'linewidth',2);
ylabel('Input')
xlabel('time [s]')

%% MPC for y

mpc_y  = MPC_Control_y(sys_y, Ts);
y = [];
uy = [];
y0 = [0; 0; 0; 2];
y(:,1) = y0;
i = 1;
try
while i<N_steps 

uy(:,i) = mpc_y.get_u(y(:,i)); %Take the discretized matrices
y(:,i+1) = mpc_y.A*y(:,i) + mpc_y.B*uy(:,i);

i = i + 1;
end
catch
error('---> Initial state is outside the feasible set <---\n');
end

% Plotting the results

figure
hold on; grid on;

subplot(5,1,1)
hold on; grid on;
plot([0:Ts:(N_steps-1)*Ts],y(1,:),'-k','markersize',20,'linewidth',2);
ylabel('Vel roll [rad/s]')

subplot(5,1,2)
hold on; grid on;
plot([0:Ts:(N_steps-1)*Ts],y(2,:),'-k','markersize',20,'linewidth',2);
ylabel('Roll [rad]')

subplot(5,1,3)
hold on; grid on;
plot([0:Ts:(N_steps-1)*Ts],y(3,:),'-k','markersize',20,'linewidth',2);
ylabel('Vel y [m/s]')

subplot(5,1,4)
hold on; grid on;
plot([0:Ts:(N_steps-1)*Ts],y(4,:),'-k','markersize',20,'linewidth',2);
ylabel('y [m]')

subplot(5,1,5)
hold on; grid on;
stairs([0:Ts:(N_steps-2)*Ts],uy,'k','markersize',20,'linewidth',2);
ylabel('Input')
xlabel('time [s]')

%% MPC for z

mpc_z  = MPC_Control_z(sys_z, Ts);
z = [];
uz = [];
z0 = [0; 2];
z(:,1) = z0;
i = 1;
try
while i<N_steps 

uz(:,i) = mpc_z.get_u(z(:,i)); %Take the discretized matrices
z(:,i+1) = mpc_z.A*z(:,i) + mpc_z.B*uz(:,i);

i = i + 1;
end
catch
error('---> Initial state is outside the feasible set <---\n');
end

% Plotting the results

figure
hold on; grid on;

subplot(3,1,1)
hold on; grid on;
plot([0:Ts:(N_steps-1)*Ts],z(1,:),'-k','markersize',20,'linewidth',2);
ylabel('Vel z [m/s]')

subplot(3,1,2)
hold on; grid on;
plot([0:Ts:(N_steps-1)*Ts],z(2,:),'-k','markersize',20,'linewidth',2);
ylabel('z [m]')

subplot(3,1,3)
hold on; grid on;
stairs([0:Ts:(N_steps-2)*Ts],uz,'k','markersize',20,'linewidth',2);
ylabel('Input')
xlabel('time [s]')

%% MPC for yaw

mpc_yaw  = MPC_Control_yaw(sys_yaw, Ts);
yaw = [];
uyaw = [];
yaw0 = [0; 0.785];
yaw(:,1) = yaw0;
i = 1;
try
while i<N_steps 

uyaw(:,i) = mpc_yaw.get_u(yaw(:,i)); %Take the discretized matrices
yaw(:,i+1) = mpc_yaw.A*yaw(:,i) + mpc_yaw.B*uyaw(:,i);

i = i + 1;
end
catch
error('---> Initial state is outside the feasible set <---\n');
end

% Plotting the results

figure
hold on; grid on;

subplot(3,1,1)
hold on; grid on;
plot([0:Ts:(N_steps-1)*Ts],yaw(1,:),'-k','markersize',20,'linewidth',2);
ylabel('Vel yaw [rad/s]')

subplot(3,1,2)
hold on; grid on;
plot([0:Ts:(N_steps-1)*Ts],yaw(2,:),'-k','markersize',20,'linewidth',2);
ylabel('yaw [rad]')

subplot(3,1,3)
hold on; grid on;
stairs([0:Ts:(N_steps-2)*Ts],uyaw,'k','markersize',20,'linewidth',2);
ylabel('Input')
xlabel('time [s]')


