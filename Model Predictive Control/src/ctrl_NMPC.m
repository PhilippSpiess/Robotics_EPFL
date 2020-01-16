 function [ctrl, traj] = ctrl_NMPC(quad) 
 
 import casadi.*

 opti = casadi.Opti(); % Optimization problem 
 N = 40; % MPC horizon 
 
% −−−− decision variables −−−−−−−−−
X = opti.variable(12,N+1); % state trajectory variables
U = opti.variable(4, N); % control trajectory (throttle, brake)

X0 = opti.parameter(12,1);  % initial state
REF = opti.parameter(4,1); % reference position [x,y,z,yaw]

Us = [0.7007,0.7007,0.7007,0.7007];

%%% corresponding variables: %%%
% vel_roll   = X(1,:);
% vel_pitch = X(2,:);
% vel_yaw  = X(3,:);
% roll = X(4,:);
% pitch  = X(5,:);
% yaw = X(6,:);
% vel_x  = X(7,:);
% vel_y = X(8,:);
% vel_z  = X(9,:);
% x = X(10,:);
% y  = X(11,:);
% z = X(12,:);

weight_vel = 5;
weight_pos = 10;
weight_u = 2;

opti.minimize(...
    (X(1,:))*(X(1,:))' + ...
    (X(2,:))*(X(2,:))' + ...
    (X(3,:))*(X(3,:))' + ...
    (X(4,:))*(X(4,:))' + ...
    (X(5,:))*(X(5,:))' + ...
    (X(6,:)-REF(4))*(X(6,:)-REF(4))' + ...
    weight_vel*(X(7,:))*(X(7,:))' + ...
    weight_vel*(X(8,:))*(X(8,:))' + ...
    weight_vel*(X(9,:))*(X(9,:))' + ...
    weight_pos*(X(10,:)-REF(1))*(X(10,:)-REF(1))' + ...
    weight_pos*(X(11,:)-REF(2))*(X(11,:)-REF(2))' + ...
    weight_pos*(X(12,:)-REF(3))*(X(12,:)-REF(3))' + ...
    weight_u*(U(1,:)-Us(1))*(U(1,:)-Us(1))' + ...
    weight_u*(U(2,:)-Us(2))*(U(2,:)-Us(2))' + ... 
  	weight_u*(U(3,:)-Us(3))*(U(3,:)-Us(3))' + ... 
    weight_u*(U(4,:)-Us(4))*(U(4,:)-Us(4))');  

h = 0.2;
opti.subject_to(X(:,1)==X0);
for k=1:N 
    
   k1 = quad.f(X(:,k), U(:,k));
   k2 = quad.f(X(:,k)+h/2*k1, U(:,k));
   k3 = quad.f(X(:,k)+h/2*k2, U(:,k));
   k4 = quad.f(X(:,k)+h*k3,   U(:,k));
   
  opti.subject_to(X(:,k+1) == X(:,k) + h/6*(k1+2*k2+2*k3+k4));
 
end

% ---- input constraints -----------
opti.subject_to(0 <= U(1,:) <= 1.5); 
opti.subject_to(0 <= U(2,:) <= 1.5);
opti.subject_to(0 <= U(3,:) <= 1.5);
opti.subject_to(0 <= U(4,:) <= 1.5);% control is limited

% ---- boundary conditions - Not necessary here --------
% opti.subject_to(-0.035 <= X(4,:) <= 0.035); 
% opti.subject_to(-0.035 <= X(5,:) <= 0.035);

%%%%%%%%%%%%%%%%%%%%%%%%

ctrl = @(x,ref) eval_ctrl(x, ref, opti, X0, REF, X, U);
end

 
function u = eval_ctrl(x, ref, opti, X0, REF, X, U) 
% −−−− Set the initial state and reference −−−− 
opti.set_value(X0, x);
opti.set_value(REF, ref);

% −−−− Setup solver NLP −−−−−−
ops = struct('ipopt', struct('print_level',0, 'tol', 1e-3), 'print_time', false); 
opti.solver('ipopt', ops);

% −−−− Solve the optimization problem −−−−
sol = opti.solve();
assert(sol.stats.success == 1, 'Error computing optimal input');

u = opti.value(U(:,1));

% Use the current solution to speed up the next optimization
opti.set_initial(sol.value_variables());
opti.set_initial(opti.lam_g, sol.value(opti.lam_g)); 
end

