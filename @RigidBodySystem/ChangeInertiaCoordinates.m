function [I_S_P_B] = ChangeInertiaCoordinates(system, q, I_S_P_A, A, B)
% Changes the inertia matrix for system S from A's coordinates to B's
% coordinates.
%
% @input system The RigidBodySystem object
% @input q The current system configuration
% @input I_S_P_A Inertia of S around point P in A's coordinates
% @input A The body whose coordinates are used to express I_S_P_A
% @input B The body whose coordinates will be used to express I_S_P_B

% @output I_S_P_B The inertia of S around P in B coordinates

%% YOUR CODE HERE

I_S_P_B = RelativeRotationMatrix(system, q, A, B)*I_S_P_A*RelativeRotationMatrix(system, q, A, B)';

end

