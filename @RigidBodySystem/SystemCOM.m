function [r_Bo_Scm] = SystemCOM(system, q, B)
% Compute the center of mass of the system from point Bo, expressed in 
% B's coordinates
%
% @input system The RigidBodySystem oject
% @input q The current system configuration
% @input B Frame B
%
% @output r_Bo_Scm Center of mass of the system from point Bo, expressed in 
% B's coordinates

% Note: system.child_bodies is the list of all bodies (see Body.m for a
% list of Body properties)

%r_Bo_Scm is a vertical vector of length 3 written in B's coords

moment = 0;
for i = 0:length(system.child_bodies)
    bod = system.bodies(i);
    r_Bo_bodCM_bod = RelativePointPosition(system, q, bod, bod.r_Bo_Bcm, B, [0;0;0]);
    moment = moment + bod.mass*r_Bo_bodCM_bod;
end

mass = SystemMass(system);
r_Bo_Scm = moment/mass;


end
