function [handle, eom] = SymbolicRigidBodyDynamics(system)
% Generates equations of motion for a RigidBodySystem modeling a single
% rigid body. The system may have additional joints and virtual bodies, but
% only the last body in the tree may have mass and inertia, while all others
% must be massless. Furthermore, the system must have exactly six non-Fixed
% joints which allow fully unconstrained motion, modeling a floating body.
%
% @input system The RigidBodySystem object
% 
% @output handle A function handle which describes the dynamics numerically, in
% the form:
%  [qdot; qddot] = handle(t,[q;qdot],u)
% where u is a vector of control inputs (actuator efforts). If the system
% is autonomous (no control inputs), this handle can be passed directly to
% ode45 to simulate the system. If not, a controller in the form
%   u = controller(t,[q;qdot])
% must be used to create a function handle representing the closed-loop
% dynamics in the form
%  [qdot; qddot] = handle(t,[q;qdot],controller(t,[q;qdot]))
% 
% @output eom Symbolic equations of motion in terms of q, qdot, and qddot
% which describe the system's dynamics via the differential equation eom = 0
%
% This function is already fully implemented for you.  You'll notice the
% majority of the work here is done by your NewtonEulerBodyEquations!

    %% Create symbolic variables for the time, state, and accelerations
    t_sym = sym('t','real');
    q_sym = sym('q', [6,1], 'real');
    qdot_sym = sym('qdot', [6,1], 'real');
    qddot_sym = sym('qddot', [6,1], 'real');

    %% Generate symbolic equations of motion
    % To get the equations of motion, all we need is your implementation of
    % NewtonEulerBodyEquations and the symbolic variables defined above.
    B = system.bodies(numel(system.child_bodies));
    eom = system.NewtonEulerBodyEquations(q_sym, qdot_sym, qddot_sym, B) == 0;
    
    %% Solve the equations of motion for the accelerations
    % Here, we are asking MATLAB to rewrite the above equations of motion in a
    % form that's linear in the accelerations:
    %
    %  eom == 0  <==>  A(q_sym,qdot_sym)*qddot_sym == b(q_sym,qdot_sym,u)
    % 
    % We use the MATLAB command equationsToMatrix to convert eom to matrix
    % form. Then, we solve for the accelerations, which for a fully determined
    % equation in the form A*x == b, the solution can be computed in MATLAB via
    % the command: inv(A)*b
    % 
    % Before solving the linear system, it can dramatically improve the
    % performance of your code to first run the simplify(...) function on
    % both A and b.
    [A,b] = equationsToMatrix(eom, qddot_sym);
    A = simplify(A, 'Steps', 20);
    b = simplify(b, 'Steps', 20);
    qddot_sol = A \ b;
    qddot_sol = simplify(qddot_sol, 'Steps', 20);
    
    %% Handle any control inputs
    % First, aggregate any control input variables due to applied forces.
    u = [];
    for force_torque = system.force_torques
        % is this applied force and torque due to an actuator we can control?
        if isfield(force_torque.params,'u')
            u = [u; force_torque.params.u];
        end
    end

    if isempty(u) % no control inputs: system dynamics are autonomous
       u = sym('none'); % this variable is only a placeholder
    end

    %% Generate a function handle describing the dynamics
    
    % note: if the sysyem is autonomous, we can use this handle directly with ode45
    handle = matlabFunction([qdot_sym; qddot_sol], 'Vars', ...
                            {t_sym, [q_sym ; qdot_sym], u});
    
end