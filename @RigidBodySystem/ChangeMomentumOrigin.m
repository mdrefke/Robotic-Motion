function [H_A_P_B] = ChangeMomentumOrigin(system, q, qdot, A, B, H_A_Ao_B, r_Ao_P_B )
% Changes the origin of angular momentum of A about Ao in B (H_A_Ao_B)
% be about P (H_A_P_B)
%
% @input system The RigidBodySystem object
% @input q The current system configuration
% @input qdot The joint velocity
% @input A The body associated with the angular momentum
% @input B The frame that the angular momentum is in
% @input H_A_Ao_B Angular momentum of A in B, about Ao, expressed in B coordinates
% @input r_Ao_P_B Position vector from Ao to P, expressed in B coordinates
%
% @output H_A_P_B Angular momentum of A in B, about P, expressed in B coordinates

%% MD's CODE:

H_A_P_B = H_A_Ao_B + cross(-r_Ao_P_B, BodyLinearMomentum(system,q,qdot,A,B));

end

