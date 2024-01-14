function [L_A_B] = BodyLinearMomentum(system, q, qdot, A, B)
% Computes the linear momentum of Body A in body B, in B's coordinates
%
% @input system The RigidBodySystem object
% @input q The current system joint configuration
% @input qdot The current system joint velocity
% @input A The A body object
% @input B The B body object

% @output L_A_B The linear momentum of Body A in frame B, expressed in B's
% coordinates

%% YOUR CODE HERE

L_A_B = A.mass * PointVelocity(system, q, qdot, A, [0 0 0]', B);

end

