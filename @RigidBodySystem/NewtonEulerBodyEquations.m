function eqn = NewtonEulerBodyEquations(system, q, qdot, qddot, B)
% Compute the Newton-Euler equations for a single rigid body. 
%
% @input B The Body object
% @input q The vector of joint positions
% @input qdot The vector of joint velocities
% @input qddot The vector of joint accelerations
%
% @output eqn A 6x1 vector `eqn` where `eqn=0` is satisfied when the
% equations of motion hold. For instance, to capture linear motion, the first
% three elements `eqn` are
%   eqn(1:3) = F - m*a (for appropriate F and a)
% where F is an applied force vector.  Similarly, the last three elements are
%   eqn(4:6) = M - I*alpha (for appropriate M and alpha)
% where M is an applied moment vector.

%% MD's CODE

% 1. Get the force and moment on the body due to applied forces and moments

frame = system.GetInertialFrameN;
[F_B_N, M_Bcm_N] = AppliedForcesAndMoments(system,q,qdot,B);

% 2. Get the linear acceleration, angular velocity, and angular acceleration of
% B's CoM in N, expressed in N coordinates.
a_Bcm_N = PointAcceleration(system,q,qdot,qddot,B,B.r_Bo_Bcm,frame);
w_Bcm_N = BodyAngularVelocity(system,q,qdot,B,frame);
alpha_Bcm_N = BodyAngularAcceleration(system,q,qdot,qddot,B,frame);

% 3. Convert the angular velocity and angular acceleration of B in N to B
% coordinates.
w_Bcm_B = ChangeCoordinates(system,q,w_Bcm_N,frame,B);
alpha_Bcm_B = ChangeCoordinates(system,q,alpha_Bcm_N,frame,B);


% 4. Compute the linear dynamics in N coordinates (F - ma).
LinDyn = F_B_N - B.mass*a_Bcm_N;


% 5. Compute the rotational dynamics in B coordinates (M - I alpha).
M_Bcm_N = ChangeCoordinates(system, q, M_Bcm_N, frame, B);
RotDyn = M_Bcm_N - B.I_B_Bcm * alpha_Bcm_B - cross(w_Bcm_B, B.I_B_Bcm*w_Bcm_B);


% 6. Combine the linear and rotational dynamics into one output vector.
eqn = [LinDyn, RotDyn];

end