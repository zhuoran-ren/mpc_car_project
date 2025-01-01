classdef NmpcControl < handle

    properties
        % The NMPC problem
        opti

        % Problem parameters
        x0, ref, x0other

        % Most recent problem solution
        sol

        % The input that you want to apply to the system
        u0

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Add any variables you would like to read to debug here
        % and then store them in the NmpcControl function below.
        % e.g., you could place X here and then add obj.X = X
        % in the NmpcControl function below.
        % 
        % After solving the problem, you can then read these variables 
        % to debug via
        %   nmpc.sol.value(nmpc.X)
        % 
        X, U
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
    end

    methods
        function obj = NmpcControl(car, H)

            import casadi.*

            N_segs = ceil(H/car.Ts); % Horizon steps
            N = N_segs + 1;          % Last index in 1-based Matlab indexing

            nx = 4;
            nu = 2;

            % Define the NMPC optimization problem
            opti = casadi.Opti();
            
            % Parameters (symbolic)
            obj.x0 = opti.parameter(nx, 1);       % initial state
            obj.ref = opti.parameter(2, 1);       % target y, velocity
            obj.x0other = opti.parameter(nx, 1);  % initial state of other car

            % SET THIS VALUE TO BE YOUR CONTROL INPUT
            obj.u0 = opti.variable(nu, 1);

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            fd = @(x,u) RK4(x,u,car.Ts,@car.f);
            obj.X = opti.variable(nx, N+1); % State trajectory
            obj.U = opti.variable(nu, N); % Control trajectory
            
            % Define your problem using the opti object created above

            cost = 0;

            for k =1:N
                cost = cost + obj.U(1,k)*obj.U(1,k)';
                cost = cost + 2*obj.U(2,k)*obj.U(2,k)';
                cost = cost + 1*(obj.X(2,k)-obj.ref(1))*(obj.X(2,k)-obj.ref(1))';
                cost = cost + (obj.X(4,k)-obj.ref(2))*(obj.X(4,k)-obj.ref(2))'; 
            end
            
            % change this line accordingly
            opti.subject_to( obj.u0 == obj.U(:,1) );
            opti.subject_to( obj.X(:,1) == obj.x0);
            for k = 1:N
               
                opti.subject_to(obj.X(:,k+1) == fd(obj.X(:,k),obj.U(:,k)));
                opti.subject_to(-0.5<=obj.X(2,k)<=3.5);
                opti.subject_to(-0.0873<=obj.X(3,k)<=0.0873);
                opti.subject_to(-0.99<=obj.U(2,k)<=0.99);
                opti.subject_to(-0.5236<=obj.U(1,k)<=0.5236);

            end

            opti.minimize(cost)


            % Store the defined problem to solve in get_u
            obj.opti = opti;

            % Setup solver
            options = struct;
            options.ipopt.print_level = 0;
            options.print_time = 0;
            options.expand = true;
            obj.opti.solver('ipopt', options);
        end

        function u = get_u(obj, x0, ref, x0other)

            if nargin < 4
                x0other = zeros(4, 1);
            end

            % Compute solution from x0
            obj.solve(x0(1:4), ref, x0other(1:4));

            u = obj.sol.value(obj.u0);
        end

        function solve(obj, x0, ref, x0other)

            % Pass parameter values
            obj.opti.set_value(obj.x0, x0);
            obj.opti.set_value(obj.ref, ref);
            obj.opti.set_value(obj.x0other, x0other);

            obj.sol = obj.opti.solve();   % actual solve
            
            % Set warm start for next solve
            obj.opti.set_initial(obj.sol.value_variables());
            obj.opti.set_initial(obj.opti.lam_g, obj.sol.value(obj.opti.lam_g));
        end
    end
end
function [x_next] = RK4(X,U,h,f)
%
% Inputs : 
%    X, U current state and input
%    h    sample period
%    f    continuous time dynamics f(x,u)

   k1 = f(X,         U);
   k2 = f(X+h/2*k1, U);
   k3 = f(X+h/2*k2, U);
   k4 = f(X+h*k3,   U);
   x_next = X + h/6*(k1+2*k2+2*k3+k4);
end
