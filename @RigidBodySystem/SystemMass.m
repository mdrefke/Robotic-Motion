function [m_S] = SystemMass(system)
% Compute the total mass of the system m_S
% @input system The RigidBodySystem oject
%
% @output m_S The total mass of the system

% Note: system.child_bodies is the list of all bodies (see Body.m for a
% list of Body properties)

m_S = 0;
for i = 1:length(system.child_bodies)
    body = system.child_bodies(i);
    m_S = m_S + body.mass; %j keep adding masses
end







