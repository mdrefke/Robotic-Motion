function system = AddJointConstraints(system)
% This function adds ForceTorque objects to the system which model the
% constraint forces arising at all Joint's in the system. When called on an
% existing system, the function deletes all existing ForceTorques of type
% JointConstraint, and then adds new ForceTorques for each joint, where
% each ForceTorque has type JointConstraint and params which define the
% reaction forces that arise at that joint in accordance with its type.
% Note that reaction components should only be included in components
% which are constrained by the joint, not those that are free!
%
% @input system The RigidBodySystem object

% get rid of all stale JointConstraint ForceTorque's so we don't
% duplicate if the function is called twice
if ~isempty(system.force_torques)
    mask = [system.force_torques.type] ~= ForceTorqueType.JointConstraint;
    system.force_torques = system.force_torques(mask);
end

% iterate through all the joints in the system
for i = 1:numel(system.joints)
    C = system.bodies(system.joint_child_bodies(i));
    P = system.bodies(system.joint_parent_bodies(i));
    joint = system.joints(i);

    % Only one JointConstraint will exist between any two bodies,
    % since there is only one joint connecting any two bodies.
    % Therefore, a unique name for the reaction force and torque components
    % can be constructed using the parent and child body names.
    name = ['_reaction_' P.name '_' C.name '_'];

    %% MD's CODE:

    % determine which components have reaction forces/torques depending on
    % joint type (no reaction component should act along a degree of freedom
    % which is free to move)
    switch joint.type
        case JointType.Fixed
            % nothing is free to move, so all components are nonzero in
            % the reaction force and torque.
            nF = 3; % 3 nonzero components in reaction force
            nT = 3; % 3 nonzero components in reaction torque
            F = sym(['F' name],[nF 1],'real');
            T = sym(['T' name],[nT 1],'real');

            F_C_J = F;
            T_C_J = T;

        case JointType.Translation

            nF = 2;
            nT = 3;
            F = sym(['F' name],[nF 1],'real');
            T = sym(['T' name],[nT 1],'real');

            T_C_J = T;

            if joint.axis == [1;0;0]
                F_C_J = [0;F];
            elseif joint.axis == [0;0;1]
                F_C_J = [F;0];
            else
                F_C_J = [F(1);0;F(2)];
            end


        case JointType.Rotation

            nF = 3;
            nT = 2;
            F = sym(['F' name],[nF 1],'real');
            T = sym(['T' name],[nT 1],'real');

            F_C_J = F;

            if joint.axis == [1;0;0]
                T_C_J = [0;T];
            elseif joint.axis == [0;0;1]
                T_C_J = [T;0];
            else
                T_C_J = [T(1); 0; T(2)];
            end

    end

    %% END OF MD's CODE

    r_Co_G = [0;0;0]; % point it's applied ON is origin of the Joint's child frame Co
    r_Po_H = joint.r_Po_Jo; % point it's applied FROM is origin of the Joint's frame Jo
    params = struct(...
        'joint',joint,... Joint object corresponding to this constraint
        'F_C_J',F_C_J,... [3x1] reaction force in J frame of that joint
        'T_C_J',T_C_J ... [3x1] reaction torque in J frame of that joint
        );
    reaction = ForceTorque(ForceTorqueType.JointConstraint, C, r_Co_G, P, r_Po_H, params );
    system = system.AddForceTorque(reaction);

end

end
