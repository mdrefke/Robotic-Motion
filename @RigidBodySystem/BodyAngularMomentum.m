function [H_A_P_B] = BodyAngularMomentum(system, q, qdot, A, B, r_Ao_P)
% Computes the angular momentum of body A about P (a point fixed in A), in 
% frame B, expressed in B's coordinates
%
% @input system The RigidBodySystem object
% @input q The current system joint configuration
% @input qdot The current system joint velocity
% @input A The Body that angular momentum will be found for
% @input B The frame that the angular momentum will be expressed in
% @input r_Ao_P The vector from the origin of body A to point P, in A
% coordinates
% 
% @output H_A_P_B The angular momentum of Body A about P in B expressed in frame B

%% MD's CODE:

I_A_P_A = ChangeInertiaOrigin(system, A, r_Ao_P);
I_A_P_B = ChangeInertiaCoordinates(system, q, I_A_P_A, A, B);
w_B_A = BodyAngularVelocity(system, q, qdot, A, B);


r_P_Acm = A.r_Bo_Bcm - r_Ao_P;
r_P_Acm = ChangeCoordinates(system, q, r_P_Acm, A, B);
H_A_P_B = I_A_P_B * w_B_A + cross(r_P_Acm, ...
    A.mass*PointVelocity(system, q, qdot, A, r_Ao_P, B));

end

