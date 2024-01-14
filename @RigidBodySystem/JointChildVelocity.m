function [v_P_Co, w_P_C] = JointChildVelocity(system,joint, qdot, velocity_index)
% Compute v_P_Co, the linear velocity of child origin Co relative to
% parent P (in P coordinates), and w_P_C, the angular velocity of
% child C relative to parent P (in P coordinates), as a function of the
% joint velocity

% @param joint The Joint object
% @param qdot The first derivative of the configuration vector
%              (typically, vector of joint velocities)
% @param velocity_index The index into `qdot` that corresponds to 
%        this particular joint. For a rotational joint,
%        qdot(velocity_index) is the joint velocity.
% @return v_P_Co The linear velocity of Co in P, expressed in P coordinates
% @return w_P_C The angular velocity of C in P, in P coordinates

%% MD's CODE
  switch(joint.type)
    case JointType.Fixed
      w_P_C = zeros(3,1);
      v_P_Co = zeros(3,1);
    case JointType.Rotation 
      v_P_Co = zeros(3,1);
      %% YOUR CODE GOES HERE
      % compute the child's ANGULAR velocity
      w_P_C = joint.R_P_J * qdot(velocity_index) * joint.axis;
    case JointType.Translation
      w_P_C = zeros(3,1);
      v_P_Co = joint.R_P_J * qdot(velocity_index) * joint.axis;
  end
end
