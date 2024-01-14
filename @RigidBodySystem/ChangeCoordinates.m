function x_B = ChangeCoordinates(system, q, x_A, A, B)
% Express the vector x in B's coordinates, given an expression in A's
% coordinates. Note that x_A and x_B represent the same vector, just in
% different coordinates.
% 
% @input system The RigidBodySystem object
% @input q The vector of joint positions
% @input x_A A representation of x in A's coordinates
% @input A The Body A object
% @input B The Body B object

%% MD's CODE:
R_B_A = RelativeRotationMatrix(system, q, A, B);
x_B = R_B_A * x_A;

end