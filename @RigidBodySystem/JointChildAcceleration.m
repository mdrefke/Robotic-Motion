function [a_P_Co, alpha_P_C] = JointChildAcceleration(system, joint, qddot, acceleration_index)
% Compute a_P_Co, the linear acceleration of child origin Co relative to
% parent P (in P coordinates), and alpha_P_C, the angular acceleration of
% child C relative to parent P (in P coordinates), as a function of the
% joint acceleration.

% @param joint The Joint object
% @param qddot The second derivative of the configuration vector
%              (typically, vector of joint accelerations)
% @param acceleration_index The index into `qddot` that corresponds to 
%        this particular joint. For a rotational joint,
%        qddot(acceleration_index) is the joint acceleration.
% @return a_P_Co The linear acceleration of Co in P, expressed in P coordinates
% @return alpha_P_C The angular acceleration of C in P, in P coordinates

%% MD's CODE:
  switch(joint.type)
    case JointType.Fixed
    a_P_Co = zeros(3,1);
    alpha_P_C = zeros(3,1);
    case JointType.Rotation
      a_P_Co = zeros(3,1);

      % Calculate the child's angular acceleration
      alpha_P_C = joint.R_P_J * qddot(acceleration_index) * joint.axis;  
        
   
    case JointType.Translation
     alpha_P_C = zeros(3,1);
     a_P_Co = joint.R_P_J * qddot(acceleration_index) * joint.axis;
  end
end
