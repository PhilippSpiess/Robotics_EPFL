classdef MPC_Control_y < MPC_Control
  
  methods
    % Design a YALMIP optimizer object that takes a steady-state state
    % and input (xs, us) and returns a control input
    function ctrl_opt = setup_controller(mpc)

      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % INPUTS
      %   x(:,1) - initial state (estimate)
      %   xs, us - steady-state target
      % OUTPUTS
      %   u(:,1) - input to apply to the system
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      [n,m] = size(mpc.B);
      
      % Steady-state targets (Ignore this before Todo 3.2)
      xs = sdpvar(n, 1);
      us = sdpvar(m, 1);
      
       % SET THE HORIZON HERE
      N = 40;
      
      % Predicted state and input trajectories
      x = sdpvar(n, N);
      u = sdpvar(m, N-1);

      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
      
        %0 if tracking origin, 1 if tracking a reference
        tracking_state = 1;

        % Horizon and cost matrices
        Q = 10*eye(n);
        R = 100;

        % Constraints
        % u in U = { u | Mu <= mm } on input
        M = [1;-1]; 
        mm = [0.3; 0.3];
        % x in X = { x | Fx <= f } on angles
        F = [0 1 0 0; 0 -1 0 0];
        f = [0.035; 0.035];
        
        if tracking_state ==0

        % Compute LQR controller for unconstrained system
        [K,Qf,~] = dlqr(mpc.A,mpc.B,Q,R);
        % MATLAB defines K as -K, so invert its signal
        K = -K;

        % Compute maximal invariant set  
        Xf = polytope([F;M*K],[f;mm]);
        Acl = [mpc.A+mpc.B*K];
        while 1
        prevXf = Xf;
        [T,t] = double(Xf);
        preXf = polytope(T*Acl,t);
        Xf = intersect(Xf, preXf);
        if isequal(prevXf, Xf)
        break
        end
        end
        [Ff,ff] = double(Xf);

        % Visualizing the sets
        figure
        hold on; grid on;

        subplot(3,1,1)
        hold on; grid on;
        Xf.projection(1:2).plot();
        xlabel('vell roll [rad/s]')
        ylabel('roll [rad]')

        subplot(3,1,2)
        hold on; grid on;
        Xf.projection(2:3).plot();
        xlabel('roll [rad]')
        ylabel('vel y [m/s]')

        subplot(3,1,3)
        hold on; grid on;
        Xf.projection(3:4).plot();
        xlabel('vel y [m/s]')
        ylabel('y [m]')

        end

        % Defining the MPC controller

        con = (x(:,2) == mpc.A*x(:,1) + mpc.B*u(:,1)) + (M*u(:,1) <= mm);
        obj = (u(:,1)-us)'*R*(u(:,1)-us);
        for i = 2:N-1
        con = con + (x(:,i+1) == mpc.A*x(:,i) + mpc.B*u(:,i));
        con = con + (F*x(:,i) <= f) + (M*u(:,i) <= mm);
        obj = obj + (x(:,i)-xs)'*Q*(x(:,i)-xs) + (u(:,i)-us)'*R*(u(:,i)-us);
        end
        
        if tracking_state==0
        con = con + (Ff*x(:,N) <= ff);
        obj = obj + (x(:,N)-xs)'*Qf*(x(:,N)-xs);
        end
      
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
      
      ctrl_opt = optimizer(con, obj, sdpsettings('solver','gurobi'), ...
        {x(:,1), xs, us}, u(:,1));
    end
    
    
    % Design a YALMIP optimizer object that takes a position reference
    % and returns a feasible steady-state state and input (xs, us)
    function target_opt = setup_steady_state_target(mpc)

      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % INPUTS
      %   ref    - reference to track
      % OUTPUTS
      %   xs, us - steady-state target
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

      % Steady-state targets
      n = size(mpc.A,1);
      xs = sdpvar(n, 1);
      us = sdpvar;
      
      % Reference position (Ignore this before Todo 3.2)
      ref = sdpvar;            
            
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
      % You can use the matrices mpc.A, mpc.B, mpc.C and mpc.D
       con = [ -0.035 <= xs(2,1) <= 0.035 , -0.3 <= us <= 0.3 ,...
                        xs == mpc.A*xs + mpc.B*us    ,...
                        ref == mpc.C*xs + mpc.D ];

       obj   = us^2;
     
      
      
      % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE 
      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
      % Compute the steady-state target
      target_opt = optimizer(con, obj, sdpsettings('solver', 'gurobi'), ref, {xs, us});
      
    end
  end
end
