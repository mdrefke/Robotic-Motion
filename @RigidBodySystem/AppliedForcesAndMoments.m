function [F_B_N, M_Bcm_N] = AppliedForcesAndMoments(system, q, qdot, B)
% Compute the total force and total moment about the center of mass due to
% applied forces and moments
% 
% @input system The RigidBodySystem object
% @input q The vector of joint positions
% @input qdot The vector of joint velocities
% @input B The body B whose total applied forces and moments are being computed
%
% @output F_B_N the total applied force on body B, in N coords
% @output M_Bcm_N the total net moment on body B about its center of mass,
% expressed in N coords, due to the combination of:
%  - applied pure torques 
%  - the moment of applied forces about the center of mass, which arise due
%  to the points of application of said forces

    F_B_N = zeros(3,1);
    M_Bcm_N = zeros(3,1);
    N = system.GetInertialFrameN;
    
    for i=1:length(system.force_torques)
        force_torque = system.force_torques(i);
        
        % Call the point on B where the force is applied "E"
        % E will either by G on the child body or H on the parent body
        % See ForceTorque.m for force/torque definitions.
        if isequal(B, force_torque.C)
          % B = C, child body in ForceTorque.m
          % Force is applied at point G=E
          
          [Fi_B_N, Ti_B_N] = ComputeForceAndTorque(system, force_torque, q, qdot);
          
          % vector from center of mass to point of application E=G, in B coords
          r_Bcm_E_B = force_torque.r_Co_G - force_torque.C.r_Bo_Bcm;

        elseif isequal(B, force_torque.P)
          % B = P, parent body in ForceTorque.m
          % Force is applied at point H=E

          [Fi_C_N, Ti_C_N] = ComputeForceAndTorque(system, force_torque, q, qdot);

          % Force and torque is not on this body, but it is FROM this body,
          % so Newton's 3rd Law says there is an equal and opposite reaction
          Fi_B_N = -Fi_C_N;
          Ti_B_N = -Ti_C_N;

          % vector from center of mass to point of application E=H, in B coords
          r_Bcm_E_B = force_torque.r_Po_H - force_torque.P.r_Bo_Bcm;

        else
          % this force does not act on or from body B
          continue
        end
        
        %% YOUR CODE HERE
        
        % Edit this section so that the function will add up the
        % contributions of all applied forces and torques acting on this
        % body. You will need to make use of:
        % - the force Fi_B_N applied to the body at point E
        % - the torque Ti_B_N applied to the body
        % - the position vector r_Bcm_E_B from the body center of mass to E
        frame = system.GetInertialFrameN;
        F_B_N = F_B_N + Fi_B_N;
        coords = ChangeCoordinates(system, q, r_Bcm_E_B, B, frame);
        M_Bcm_N = M_Bcm_N + Ti_B_N + cross(coords, Fi_B_N);

    end

end