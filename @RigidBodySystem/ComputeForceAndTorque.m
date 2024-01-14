function [F_C_N, T_C_N] = ComputeForceAndTorque(system, force_torque, q, qdot)
% Computes the effect of a ForceTorque on its child body C. STRONGLY
% RECOMMENDED: read the documentation of the ForceTorque class before beginning
% this function!
%
% @input system The RigidBodySystem object
% @input force The ForceTorque object, storing data necessary to compute the
% force and torque applied on body C.
% @input q The vector of joint positions
% @input qdot The vector of joint velocities
%
% @output F_C_N The force applied on body C at G, in N coords
% @output T_C_N The torque applied on body C

N = system.GetInertialFrameN;

switch(force_torque.type)

    case ForceTorqueType.Gravity

        % This case is already implemented - you don't need to modify it!

        g = force_torque.params.g; % gravitational constant

        F_C_N = [0; 0; -force_torque.C.mass * g];
        T_C_N = [0; 0; 0]; % non-celestial gravity applies no torque

    case ForceTorqueType.Spring

        k = force_torque.params.k; % spring constant
        L = force_torque.params.L; % natural length of spring (length for
        % zero force)

        %% YOUR CODE HERE

        % Compute the force and torque applied on body C by a spring
        % stretched between point G (on body C) and point H (on body
        % P). The geometric description of these points is stored in
        % the ForceTorque object that is an argument of this function.
        % Consult the documentation of this class for more details.
        %
        % Note: you may assume the spring *total* length is always > 0
        % (that is, including the natural length at rest). You may also
        % assume that it is a "simple" spring i.e. there is a linear
        % relationship between displacement from rest and force
        % produced.

        R_H_G = RelativePointPosition(system, q, force_torque.C, force_torque.r_Co_G, force_torque.P, force_torque.r_Po_H);
        R_H_G = system.ChangeCoordinates(q, R_H_G, force_torque.P, N);
        len = sqrt(dot(R_H_G, R_H_G));
        displ = len-L; %take out natural length
        unitVec = R_H_G/len; %direction
        F_C_N = -k*displ*unitVec; %-kx vector
        T_C_N = [0;0;0]; %not relevent here

    case ForceTorqueType.JointConstraint
        F_C_N = force_torque.params.F_C_J;
        F_C_N = ChangeCoordinates(system,q,F_C_N,force_torque.P,N);
        T_C_N = force_torque.params.T_C_J;
        T_C_N = ChangeCoordinates(system,q,T_C_N,force_torque.P,N);

    case ForceTorqueType.ThrusterActuator
        axis = force_torque.params.axis;
        F_C_N = axis.*(-force_torque.params.u);
        F_C_N = ChangeCoordinates(system, q, F_C_N, force_torque.C, N);
        T_C_N = [0;0;0];

    case ForceTorqueType.JointActuator
        axis = force_torque.params.joint.axis;
        axis = ChangeCoordinates(system,q,axis,force_torque.C, N);
        if force_torque.params.joint.type == 'Rotation'
            F_C_N = zeros(3,1);
            T_C_N = axis.*force_torque.params.u;
        else
            F_C_N = axis.*force_torque.params.u;
            T_C_N = zeros(3,1);
        end

    case ForceTorqueType.Damper
        error('Not yet implemented.')

end

end
