function a_B_Q = PointAcceleration(system, q, qdot, qddot, A, r_Ao_Q, B)
% Compute a_B_Q the acceleration of point Q in body B, in B coordinates
% Assumes Q is fixed in body A
% 
% @input system The RigidBodySystem object
% @param q The configuration vector (typically, vector of joint angles)
% @param qdot The first derivative of the configuration vector
%              (typically, vector of joint velocities)
% @param qddot The second derivative of the configuration vector
%              (typically, vector of joint accelerations)
% @input A the index of body A
% @input r_Ao_Q The position of Q from Ao, expressed in A coordinates
% @input B the index of body B

[path, directions] = FindPath(system, B, A);

%% YOUR CODE HERE

% Implement the function. You may find it very helpful to base the
% structure of your solution on your implementation of
% BodyAngularAcceleration from HW3.  Some helpful hints:
% * Using `path`, in a "for loop", find the acceleration of points along
%   the path in B. If `C` is a body on the path, find `a_B_Co`.
% * Pay close attention to the vector basis used to express vectors. You
%   may wish to document this carefully in your code.
% * You'll find it helpful to make use of the functions you've written in
%   previous weeks. For instance, JointTransformation, JointChildVelocity,
%   JointChildAcceleration, BodyAngularVelocity, BodyAngularAcceleration,
%   and ChangeCoordinates.
% * Just like with previous functions, keep track of the joint Parent/Child
%   and the current Parent/Child in your for loop.


C = B; %begin with B as child so that when u work through you cover everything
v_B_C = zeros(3,1); %adapted from BodyAngularVelocity
a_B_C = zeros(3,1); %extrapolated

for i = 1:length(path)-1
    %shift parents and children (same start as the other files)
    P = C;
    v_B_P = v_B_C;
    a_B_P = a_B_C;

    C = system.bodies(path(i+1));

    if directions(i) < 0

        %at this point its effectively the same process as PointVelocity
        joint_index = path(i);
        [r_Po_Co, ~] = system.JointTransformation(system.joints(joint_index), ...
            q, system.joint_to_state_index(joint_index));
        r_Po_Co = -r_Po_Co;
        [v_P_C, ~] = system.JointChildVelocity(system.joints(joint_index), ...
            qdot, system.joint_to_state_index(joint_index));
        v_P_C = -v_P_C;
        [a_P_C, ~] = system.JointChildAcceleration(system.joints(joint_index), ...
            qddot, system.joint_to_state_index(joint_index));
        a_P_C = -a_P_C;
        P = C;
        
    else 
        joint_index = path(i+1);
        [r_Po_Co, ~] = system.JointTransformation(system.joints(joint_index), ...
            q, system.joint_to_state_index(joint_index));
        [v_P_C, ~] = system.JointChildVelocity(system.joints(joint_index), ...
            qdot, system.joint_to_state_index(joint_index));
        [a_P_C, ~] = system.JointChildAcceleration(system.joints(joint_index), ...
            qddot, system.joint_to_state_index(joint_index));

    end

    %change everything to being written in B
    r_Po_Co = ChangeCoordinates(system, q, r_Po_Co, P, B);
    v_P_C = ChangeCoordinates(system, qdot, v_P_C, P, B);
    a_P_C = ChangeCoordinates(system, qddot, a_P_C, P, B);

    %%find angular values to be used in golden rule crosses
    w_B_P = BodyAngularVelocity(system, q, qdot, P, B);
    alpha_P_B = BodyAngularAcceleration(system, q, qdot, qddot, P, B);
    a_B_C = a_B_P + cross(alpha_P_B, r_Po_Co) + cross(w_B_P, cross(w_B_P, r_Po_Co)) + a_P_C + cross(2*w_B_P, v_P_C);
    
end

%finding values for final cross(using A instead of P this time)
r_Ao_Q = ChangeCoordinates(system, q, r_Ao_Q, A, B);
w_B_A = BodyAngularVelocity(system, q, qdot, A, B);
alpha_B_A = BodyAngularAcceleration(system, q, qdot, qddot, A, B);

%final cross
a_B_Q = a_B_C + cross(alpha_B_A,r_Ao_Q) + cross(w_B_A,cross(w_B_A,r_Ao_Q));


end