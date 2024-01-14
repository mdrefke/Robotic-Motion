function alpha_B_A = BodyAngularAcceleration(system, q, qdot, qddot, A, B)
% Compute alpha_B_A the angular acceleration of body A in body B, in B coordinates
% 
% @input system The RigidBodySystem object
% @input q The vector of joint positions
% @input qdot The vector of joint velocities
% @input qddot The vector of joint accelerations
% @input A The Body A object
% @input B The Body B object

[path, directions] = FindPath(system, B, A);

% Initialize to zero (C = B)
C = B;
w_B_C = zeros(3,1);
alpha_B_C = zeros(3,1);

for i=1:length(path)-1
  % The old child is the new parent
  w_B_P = w_B_C;
  alpha_B_P = alpha_B_C;
  P = C;
  
  if directions(i) < 0
    joint_index = path(i);
  else
    joint_index = path(i+1);
  end
  
  % The new child is the next body on the path
  C = system.bodies(path(i+1));

  %% MD's CODE:
  
  [~, w_JP_JC] = system.JointChildVelocity(system.joints(joint_index), ...
      qdot, system.joint_to_state_index(joint_index));
  [~, alpha_JP_JC] = system.JointChildAcceleration(system.joints(joint_index), ...
      qddot, system.joint_to_state_index(joint_index));

  % Change coordinates so all terms are in B coordinates
  % Update w_B_C and alpha_B_C

  if directions(i) > 0
      newCoordsW = ChangeCoordinates(system, q, w_JP_JC, P, B);
      newCoordsA = ChangeCoordinates(system, q, alpha_JP_JC, P, B);
      crossA = cross(w_B_P, newCoordsW);
      w_B_C = w_B_P + newCoordsW;
      alpha_B_C = alpha_B_P + newCoordsA + crossA;
  else
      newCoordsW = ChangeCoordinates(system, q, -w_JP_JC, C, B);
      newCoordsA = ChangeCoordinates(system, q, -alpha_JP_JC, C, B);
      crossA = cross(w_B_P, newCoordsW);
      w_B_C = w_B_P + newCoordsW;
      alpha_B_C = alpha_B_P + newCoordsA + crossA;
  end
end

% The last child is A   
alpha_B_A = alpha_B_C;
end
