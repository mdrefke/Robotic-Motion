function [I_B_P] = ChangeInertiaOrigin(system,B, r_Bcm_P)
% changes origin of inertia of body B from Bcm to arbitrary point P
%
% @input system A RigidBodySystem object
% @input B The body described by the inertia tensor
% @input r_Bcm_P Vector from Bcm to point P, in B coordinates
% @output I_B_P Inertia of B about point P, in B coordinates

%% MD's CODE:
I_B_P = B.I_B_Bcm + B.mass*(dot(r_Bcm_P,r_Bcm_P)*eye(3) - r_Bcm_P*r_Bcm_P');

end

