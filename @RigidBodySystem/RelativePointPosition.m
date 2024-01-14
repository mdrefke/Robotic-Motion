function r_Q_P_B = RelativePointPosition(system, q, A, r_Ao_P, B, r_Bo_Q)
% Compute r_Q_P_B the position vector from Q to P, in B coordinates
% Assumes P is fixed in A and Q is fixed in B
% 
% @input system The RigidBodySystem object
% @param q The configuration vector (typically, vector of joint angles)
% @input A The Body A object
% @input r_Ao_P The position of P from Ao, expressed in A coordinates
% @input B The Body B object
% @input r_Bo_Q The position of Q from Bo, expressed in B coordinates

[path, directions] = FindPath(system, B, A);%b is start, a is goal

%% YOUR CODE HERE

% Implement the function. You may find it very helpful to base the
% structure of your solution on your implementation of
% RelativeRotationMatrix from HW2. Some helpful hints:
% * Pay close attention to the vector basis used to express vectors. You
%   may wish to document this carefully in your code.
% * You'll need to use your JointTransformation function!
% * Just like with previous functions, keep track of the joint Parent/Child
%   and the current Parent/Child in your for loop.

r_Bo_Co = zeros(3,1);  %there is no r yet since we haven't gone anywhere
C = B; %specify that B is the first child object


for i = 1:length(path)-1 %walk through whole path
    r_Bo_Po = r_Bo_Co;
    P = C; %old child is new parent (initially parent is B, our start)
    C = system.bodies(path(i+1)); %new child is one downstream in the path

    if directions(i) < 0
        joint_index = path(i); %%taken from relativeRotation
    else
        joint_index = path(i+1);
    end
    [r_JP_JC, ~] = system.JointTransformation(system.joints(joint_index), q, ...
        system.joint_to_state_index(joint_index)); %position vect between bodies in p's coordinates
        
    if directions(i) < 0
        r_JP_JC = -r_JP_JC; % neg direction means -r
        P = C; %old child is new parent %%%%%%%%%%%%%
    end
    
    % need r_Po_Co in parent's coordinates, not B's
    r_Po_Co = r_JP_JC;
    r_Po_Co = ChangeCoordinates(system, q, r_Po_Co, P, B); %now in B's coordinates
    r_Bo_Co = r_Bo_Po + r_Po_Co; %tracks total travel
end

r_Bo_Ao = r_Bo_Co; %now that all the position vectors have been added, we know we have reached A
r_Ao_P = ChangeCoordinates(system, q, r_Ao_P, A, B); %now in B's coordinates

r_Q_P_B  = r_Bo_Ao + r_Ao_P - r_Bo_Q; %add it all up

end

