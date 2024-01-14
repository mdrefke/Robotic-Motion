function [T, V] = ComputeEnergy(system, q, qdot)
% Computes the kinetic and potential energy of a system
%
% @input system The RigidBodySystem object
% @input q The vector of joint positions
% @input qdot The vector of joint velocities
%
% @output T The kinetic energy due to the motion of all rigid bodies
% @output V The potential energy due to applied conservative forces and torques
% (i.e. either spring forces or gravity forces)
%
% This function is already fully implemented for you.

    N = system.GetInertialFrameN;

    %% Kinetic Energy
    T = 0;
    for B = system.child_bodies
        if B.mass > 0
            w_N_B_N = BodyAngularVelocity(system, q, qdot, B, N);
            w_N_B_B = ChangeCoordinates(system,q,w_N_B_N,N,B);
            v_Bcm_N = PointVelocity(system, q, qdot, B, B.r_Bo_Bcm, N);

            % compute kinetic energy of just this body
            Ti = 1/2 * B.mass * (v_Bcm_N.' * v_Bcm_N) + ...
                 1/2 * w_N_B_B.' * B.I_B_Bcm * w_N_B_B;
            T = T + Ti;
        end
    end

    %% Potential Energy
    V = 0;
    for force_torque = system.force_torques
        switch force_torque.type
            case ForceTorqueType.Gravity
                g = force_torque.params.g;
                B = force_torque.C;
                r_No_Bcm_N = system.RelativePointPosition(q, B, B.r_Bo_Bcm, ...
                                                          N, zeros(3,1));
                
                % Compute potential energy due to just this force.
                Vi = B.mass * g * r_No_Bcm_N(3);
                
            case ForceTorqueType.Spring
                k = force_torque.params.k; % spring constant
                L = force_torque.params.L; % natural length of spring (length
                                           % corresponding to zero force)

                % Compute relative position vector.
                r_G_H_P = system.RelativePointPosition(q, force_torque.C, ...
                              force_torque.r_Co_G, force_torque.P, ...
                              force_torque.r_Po_H);
                stretch = norm(r_G_H_P) - L;

                % Compute potential energy due to just this force.
                Vi =  1/2 * k * stretch^2;

            otherwise
                Vi = 0; % Nonconservative forces and torques do not contribute
                        % to energy.
        end
        V = V + Vi;
    end

end