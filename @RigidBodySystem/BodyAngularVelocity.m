function w_B_A = BodyAngularVelocity(system, q, qdot, A, B)
% Compute w_B_A the angular velocity of body A in body B, in B coordinates
% 
% @input system The RigidBodySystem object
% @input q The vector of joint positions
% @input qdot The vector of joint velocities
% @input A The Body A object
% @input B The Body B object

[path, directions] = FindPath(system, B, A);

% Initialize to zero, C=B
C = B;
w_B_C = zeros(3,1);

for i=1:length(path)-1
  % The old child is the new parent
  w_B_P = w_B_C;
  P = C;
  
  % Transformation across current joint
  % Determine the relevant joint, which depends if the path is going
  % forward across the joint (joint parent to child), or backward
  % (joint child to parent).
  if directions(i) < 0
    joint_index = path(i);
  else
    joint_index = path(i+1);
  end
  
  % The new child is the next body on the path
  C = system.bodies(path(i+1));

  % Compute w_JP_JC, the angular velocity of the child relative to the parent across the
  % current joint
  [~, w_JP_JC] = system.JointChildVelocity(system.joints(joint_index), qdot, ...
      system.joint_to_state_index(joint_index));

  %% YOUR CODE GOES HERE
  % Check whether we are going forward or backwards across the joint
  % HINT: relate JC, JP to C and P and keep track of coordinates!
  
  % REMEMBER, in this function, P and C refer to the parent and child in
  % the PATH, while JP and JC refer to the parent and child in the joint
  % (and therefore in the tree underlying the rigid body system).

  if directions(i) > 0
      w_B_C = w_B_P + ChangeCoordinates(system, q, w_JP_JC, P, B);
      % JP=P, JC=C
  else
      w_B_C = w_B_P + ChangeCoordinates(system, q, -w_JP_JC, C, B);
    % JP=C, JC=P
  end
  
  % Update w_B_C to get the angular velocity of child C in B, in B
  % coordinates
end

% The last child is A   
w_B_A = w_B_C;
end

