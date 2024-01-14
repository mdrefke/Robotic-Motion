function R_B_A = RelativeRotationMatrix(system, q, A, B)
% Compute R_B_A, the rotation matrix expressing B in A.
% This requires determining the path from B to A, then successively computing
% the rotation matrix expressing B in various vector bases along the path.
%
% @input system The RigidBodySystem object
% @input q The vector of joint positions
% @input A The Body A object
% @input B The Body B object

% Determine the path from B to A. See the function FindPath in 
% RigidBodySystem.m for a description of the outputs.
[path, directions] = FindPath(system, B, A);

% Now, travel along the given path from start to finish.

% Call `C` the current child body along the path, initializing C = B, so R_B_C
% is the identity matrix.
R_B_C = eye(3);

for i=1:length(path)-1
  % Inside this loop, calculate the "new" R_B_C, where the child "C" becomes the
  % next frame along the path.

  % The old child is the new parent
  R_B_P = R_B_C;
 
  % Transformation across current joint
  % Determine the relevant joint, which depends if the path is going forward
  % across the joint (joint parent to child), or backward (joint child to
  % parent).
  if directions(i) < 0
    joint_index = path(i);
  else
    joint_index = path(i+1);
  end

  % Call JointTransformation with the given joint and index into `q`
  [~, R_P_C] = system.JointTransformation(system.joints(joint_index), q, ...
                                    system.joint_to_state_index(joint_index));
    
  %% MD's CODE:
  % Calculate R_B_C given the matrices computed above
  
  %%moving backwards means you need to use the transpose
  if directions(i) < 0
    R_B_C = R_B_P * R_P_C.';
  
  %can use normal transformation when moving forward
  else
    R_B_C = R_B_P * R_P_C;

  end
end

% A is the last child
R_B_A = R_B_C;

end

