function v_B_Q = PointVelocity(system, q, qdot, A, r_Ao_Q, B)
% Compute v_B_Q the velocity of point Q in body B, in B coordinates
% Assumes Q is fixed in body A
% 
% @input system The RigidBodySystem object
% @param q The configuration vector (typically, vector of joint angles)
% @param qdot The first derivative of the configuration vector
%              (typically, vector of joint velocities)
% @input A the body A
% @input r_Ao_Q The position of Q from Ao, expressed in A coordinates
% @input B the body B

[path, directions] = FindPath(system, B, A);

%% YOUR CODE HERE

% Implement the function. You may find it very helpful to base the
% structure of your solution on your implementation of
% BodyAngularVelocity from HW3.  Some helpful hints:
% * Using `path`, in a "for loop", find the velocity of points along the
%   path in B. If `C` is a body on the path, find `v_B_Co`.
% * Pay close attention to the vector basis used to express vectors. You
%   may wish to document this carefully in your code.
% * You'll find it helpful to make use of the functions you've written in
%   previous weeks. For instance, JointTransformation, JointChildVelocity,
%   BodyAngularVelocity, and ChangeCoordinates.
% * Just like with previous functions, keep track of the joint Parent/Child
%   and the current Parent/Child in your for loop.


C = B; %begin with B as child so that when u work through you cover everything
v_B_C = zeros(3,1); %adapted from BodyAngularVelocity

% r_Bo_Co = zeros(3,1); %will build up position vector using RelativePointPosition

for i = 1:length(path)-1

    %shift through series of objects, shift through velocities
    P = C; 
    v_B_P = v_B_C;
    
    C = system.bodies(path(i+1)); %same as relative point position start, in last run through, A is child


    
    if directions(i) < 0 %looking to either side of the present body
        joint_index = path(i);
        [v_P_C, ~] = system.JointChildVelocity(system.joints(joint_index), ...
            qdot, system.joint_to_state_index(joint_index));
        v_P_C = -v_P_C; %neg direction means neg velocity vector
        [r_Po_Co, ~] = system.JointTransformation(system.joints(joint_index), ...
            q, system.joint_to_state_index(joint_index));
        r_Po_Co = -r_Po_Co; %neg direction means neg position vector
        P = C; %move in other direction
    else 
        joint_index = path(i+1);
        [v_P_C, ~] = system.JointChildVelocity(system.joints(joint_index), ...
            qdot, system.joint_to_state_index(joint_index));
        [r_Po_Co, ~] = system.JointTransformation(system.joints(joint_index), ...
            q, system.joint_to_state_index(joint_index));
    end
    r_Po_Co = ChangeCoordinates(system, q, r_Po_Co, P, B); %write r_Po_Co in B's coordinates
    v_P_C = ChangeCoordinates(system, q, v_P_C, P, B); %same process


    %to do golden rule, need to use BodyAngularVelocity to find angular vel
    w_B_P = BodyAngularVelocity(system, q, qdot, P, B);
    %do golden rule
    v_B_C = v_B_P + v_P_C + cross(w_B_P, r_Po_Co);
end


w_B_A = BodyAngularVelocity(system, q, qdot, A, B);
r_Ao_Q = ChangeCoordinates(system, q, r_Ao_Q, A, B);
%do final golden rule using r_Ao_Q from inputs of function
v_B_Q = v_B_C + cross(w_B_A, r_Ao_Q);

end