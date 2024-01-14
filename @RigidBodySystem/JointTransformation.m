function [r_Po_Co, R_P_C] = JointTransformation(system, joint, q, ...
                                                configuration_index)
% Compute the transformation across a joint J, from parent P to child C.
% Note that this method must compute the transformation from P to J, and
% then J to C.
%
% @param joint The Joint object
% @param q The configuration vector (typically, vector of joint angles)
% @param configuration_index The index into `q` that corresponds to this
%        particular joint. For a rotational joint, q(configuration_index)
%        is the joint angle.
% @return r_Po_Co The position vector from Po to Co, expressed in P coordinates
% @return R_P_C The rotation matrix from P to C


  r_Po_Co = zeros(3,1); % implement later (not for HW2)!

switch(joint.type)
  case JointType.Fixed
    % Implement later (not for HW2)!
    r_Po_Co = joint.r_Po_Jo;
    R_P_C = joint.R_P_J;

    
  case JointType.Rotation
    % Extract the angle of this particuluar joint
    position = q(configuration_index);
    r_Po_Co = joint.r_Po_Jo;      
   
    if joint.axis == [1;0;0]
        R_J_C = [1,0,0; 0, cos(position), -sin(position); 0, sin(position), cos(position)];
        R_P_C = joint.R_P_J * R_J_C;

    elseif joint.axis == [0;1;0]
        R_J_C = [cos(position),0,sin(position); 0,1,0; -sin(position), 0, cos(position)]; 
        R_P_C = joint.R_P_J * R_J_C;
    else
        R_J_C = [cos(position), -sin(position),0; sin(position), cos(position), 0; 0,0,1];
        R_P_C = joint.R_P_J * R_J_C;
    end


  case JointType.Translation
    % Implement later (not for HW2)!
    R_P_C = joint.R_P_J;
    r_Po_Co = joint.r_Po_Jo + joint.axis*q(configuration_index);
end

  
end
