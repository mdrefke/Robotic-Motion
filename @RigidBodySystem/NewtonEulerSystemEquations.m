function eqns = NewtonEulerSystemEquations(system, q, qdot, qddot)
% derives the equations of motion for a system of rigid bodies.
%
% @input system The RigidBodySystem being modeled
% @input q The vector of joint positions (should use the symbolic toolbox)
% @input qdot The vector of joint velocities (should use the symbolic toolbox)
% @input qddot The vector of joint accelerations (should use the symbolic toolbox)
%
% @output eqns A 6m x 1 vector, where eqns == 0 is the symbolic equations of 
% motion in terms of q, qdot, qddot, reaction force/torque components, and 
% any control input variables

    %% MD's CODE

    eqns = [];
    path = system.child_bodies;
    for i = 1:length(path)
        eqns = vertcat(eqns, NewtonEulerBodyEquations(system,q,qdot,qddot,system.bodies(i)));
    end
end