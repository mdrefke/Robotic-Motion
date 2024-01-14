function [handle, eom] = SymbolicSystemDynamics(system)
% Generates equations of motion for a RigidBodySystem
%
% @input system The RigidBodySystem object
% 
% @output handle A function handle which describes the dynamics numerically, in the form
%  [ [qdot; qddot], reactions ] = handle(t,[q;qdot],u)
% where u is a vector of control inputs (actuator efforts). If the system
% is autonomous (no control inputs), this handle can be passed directly to
% ode45 to simulate the system. If not, a controller in the form
%   u = controller(t,[q;qdot])
% must be used to create a function handle representing the closed-loop
% dynamics in the form
%  [ [qdot; qddot], reactions ] = handle(t,[q;qdot],controller(t,[q;qdot]))
% 
% @output eom Symbolic equations in terms of q, qdot, and qddot,
% control inputs, and reaction forces which describe the system's dynamics
% via the equation eom = 0

    n = numel(system.state_to_joint_index); % degrees of freedom

    %% Create symbolic variables for the state and accelerations
    q_sym = sym('q', [n,1], 'real');
    qdot_sym = sym('qdot', [n,1], 'real');
    qddot_sym = sym('qddot', [n,1], 'real');

    %% Formulate dynamics as a linear system we can solve
    
    % Generate symbolic equations of motion
    eom = system.NewtonEulerSystemEquations(q_sym, qdot_sym, qddot_sym) == 0;
    
    % Aggregate unknowns: both the accelerations and the reaction forces!
    reactions = system.AggregateUnknownReactions();
    unknowns = [qddot_sym; reactions];
    
    % reformulate the equations of motion into matrix form, linear in the unknowns
    [A,b] = equationsToMatrix(eom,unknowns);    

    %% Generate a function handle describing the dynamics
    
    % collect all control inputs to the system in a column vector 
    u = system.AggregateControlInputs();
    
    % create function handles for linear system
    A_ = matlabFunction(A,'Vars',{[q_sym ; qdot_sym]});
    b_ = matlabFunction(b,'Vars',{[q_sym ; qdot_sym],u});
    handle = @(t,x,u) dynamics(t,x,A_,b_,u);
 
end

function [xdot, reactions] = dynamics(t,x,A_,b_,u)
% This function is used to generate ode45-compatible handles for the
% numerical dynamics.
% 
% @input t The time in seconds (not used)
% @input x The current state of the system, [q;qdot]
% @input A_ A function handle in the form
%     A = A_(x)
% to generate the coefficient matrix for the linear system
% @input b_ A function handle in the form
%     b = b_(x,u)
% to generate the vector for the linear system
% @input u The current value of the applied control inputs 
% 
% @output xdot The current value of [qdot;qddot], the velocity and
% acceleration of the system determined by the dynamics as a function
% of state and input 
% @output reactions A vector of all unknown reaction forces, which you
% will solve for while computing the system's velocity and acceleration
% 
% Note that A_ and b_ will be generated by the above code and then sent
% to this function to create the function handle returned by
% SymbolicSystemDynamics. As shown above, the dynamics can be written
% as a linear system in the form
%     A(x) * [qddot; reactions] = b(x,u)
% which you will solve.
    
    if nargin < 4
        % in case the system has no inputs
        u = [];
    end
    
    %% YOUR CODE HERE
    
    % Hint: you will need to determine the lengths of the qddot and
    % reactions vectors. Since x = [q; qdot], this is possible!
    A = A_(x);
    b = b_(x);

    
    vec = A\b;
    numJ = length(x)/2;
    qdot = x((numJ+1):end);
    qddot = vec(1:numJ);


    reactions = vec((numJ+1):end);
    xdot = [qdot; qddot];
end