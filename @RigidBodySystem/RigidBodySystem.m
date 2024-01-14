classdef RigidBodySystem
  %RIGIDBODYSYSTEM A system made of multiple, connected rigid bodies.
  %  A system can be thought of as a tree of bodies, with joints connecting
  %  parent bodies to their children. Bodies may have mass, or be massless.
  %  The system stores the list of all bodies and their relationships,
  %  along with a notion of a "state" which represents a vector of
  %  positions and velocities of all mobile joints. For example, a double
  %  pendulum might connect
  %
  %  World -> Body B -> Body C
  %
  %  via two rotational joints, where the angles/velocities of those joints
  %  would be stored in a state.
  %
  %  The world (N frame) is implicitly body 0.
  %
  %  For the purposes of MEAM 211, it is not necessary to understand
  %  precisely how this class operates. It is largely an organizational
  %  block, for keeping track of the various elements in the system.
  %  Your code will interact with this class through the method
  %    [obj, new_ind] = AddBody(obj, body, joint, parent)
  %  which is used to create the system, through
  %    [path, directions] = FindPath(obj, body1, body2)
  %  which identifies a path between two bodies or frames, and through the
  %  visualization methods Draw and Animate.

  properties
    joints         % Array of `n` joints
    child_bodies   % Aray of `n` bodies
    body_parents   % Index to the parent of a given body
    body_children  % List of child indices (a cell array of vectors)
    joint_parent_bodies  % The parent body of the given joint
    joint_child_bodies   % The child body of the given joint
    world_children       % List of all children of the world (0th body)
    joint_to_state_index % Map from joint index to state index. -1 if a fixed joint
    state_to_joint_index % Map from state index to joint index
    body_name_to_index   % containers.Map from name to index
    
    % graph and graph_A are for internal use only. Note that to include
    % the world body, all indices are offset by 1
    graph_A              % Connectivity matrix for all joints
    graph                % MATLAB graph object for joint connections
    
    N                    % The inertial frame

    visualizer           % The visualizer object
    
    force_torques        % array of applied Force objects
    
  end
  
  methods
     
    function obj = RigidBodySystem(visualizer)
      obj.joints = Joint.empty;
      obj.child_bodies = Body.empty;
      obj.body_parents = [];
      obj.body_children = {};
      obj.joint_parent_bodies = [];
      obj.joint_child_bodies = [];
      obj.world_children = [];
      obj.graph_A = [0];
      obj.graph;
      obj.joint_to_state_index = [];
      obj.state_to_joint_index = [];
      obj.body_name_to_index = containers.Map;
      obj.visualizer = visualizer;
      
      obj.N = Body('N', 0, zeros(3), zeros(3,1));
      obj.body_name_to_index(obj.N.name) = 0;
      obj.force_torques = [];
      
    end
    
    function N = GetInertialFrameN(obj)
      N = obj.N;
    end
    
    function B = bodies(obj, i)
      if i == 0
        B = obj.N;
      else
        B = obj.child_bodies(i);
      end
    end

    function obj = ParseURDF(obj, urdf, visual_options)
        warning('off', 'robotics:robotmanip:joint:ResettingHomePosition');

        if nargin < 3
            visual_options = DefaultVisualOptions();
        end
        
        % Use MATLAB Robotics library to parse the robot
        robot = importrobot(urdf);

        % Unfortunately, MATLAB gives pretty poor information on visual
        % elements, so we'll have to do that ourselves
        root = xmlread(urdf);
        for i=1:robot.NumBodies
            b = robot.Bodies{i};
            I_B_Bcm = [b.Inertia(1), b.Inertia(6), b.Inertia(5);
                       b.Inertia(6), b.Inertia(2), b.Inertia(4);
                       b.Inertia(5), b.Inertia(4), b.Inertia(3)];
            vis = ParseVisual(root, obj.visualizer, b.Name, visual_options); 
           
            body = Body(b.Name, b.Mass, I_B_Bcm, b.CenterOfMass', vis);
            j = b.Joint;
            switch j.Type
                case 'revolute'
                    type = JointType.Rotation;
                case 'prismatic'
                    type = JointType.Translation;
                case 'fixed'
                    type = JointType.Fixed;
            end
            
            % Child to joint must be identity
            assert(all(j.ChildToJointTransform == eye(4), 'all'))

            % TODO: confirm that sign convention is correct.
            R_P_J = j.JointToParentTransform(1:3, 1:3);
            r_Po_Jo = j.JointToParentTransform(1:3, 4);
            joint = Joint(type, j.JointAxis', r_Po_Jo, R_P_J);
            if isequal(b.Parent.Name,robot.BaseName)
                parent_ind = 0;
            else
                parent_ind = obj.body_name_to_index(b.Parent.Name);
            end
            parent = obj.bodies(parent_ind);
            obj = obj.AddBody(body, joint, parent);
        end
        obj = obj.AddAllJointActuators();
    end
   
    function obj = AddBody(obj, body, joint, parent)
      % The primary function to construct systems. Adds a new body/joint to
      % the system.
      % @param body The (child) body object to add
      % @param joint The joint to add. 
      % @param parent The parent body object.
      
      % Ensure that the body has a unique name
      if isKey(obj.body_name_to_index, body.name)
        error('All bodies must have a unique name.');
      end

      parent_ind = obj.body_name_to_index(parent.name);

      new_ind = length(obj.child_bodies) + 1;
      obj.child_bodies(new_ind) = body;
      obj.joints(new_ind) = joint;

      obj.body_parents(new_ind) = parent_ind;
      if parent_ind > 0
        obj.body_children{parent_ind} = [obj.body_children{parent_ind}, new_ind];
      else
        obj.world_children = [obj.world_children, new_ind];
      end
      obj.joint_parent_bodies(new_ind) = parent_ind;
      obj.joint_child_bodies(new_ind) = new_ind;
      
      obj.graph_A(parent_ind + 1, new_ind + 1) = 1;
      obj.graph_A(new_ind + 1, parent_ind + 1) = 1;
      obj.graph = graph(obj.graph_A);
      obj.body_children{new_ind} = [];
      
      % If this joint can move, create associated state index
      if joint.type ~= JointType.Fixed
        if isempty(obj.state_to_joint_index)
          new_state_index = 1;
        else
          new_state_index = max(obj.joint_to_state_index) + 1;
        end
        
        obj.joint_to_state_index(new_ind) = new_state_index;
        obj.state_to_joint_index(new_state_index) = new_ind;
        
      else
        obj.joint_to_state_index(new_ind) = -1;
      end
      
      obj.body_name_to_index(body.name) = new_ind;
    end
   
    function obj = AddForceTorque(obj, force)
        % Add an applied force to the system of rigid bodies.
        %
        % @input force A Force object 
        obj.force_torques = [obj.force_torques, force];
                
    end

    function obj = AddGravity(obj, g)
        % Turns on gravity for all bodies in the RigidBodySystem. Deletes
        % old gravity forces
        %
        % @input system The RigidBodySystem object
        % @input g The acceleration due to gravity, defaults to 9.81 m/s^2

        if nargin < 2 % default gravity
           g = 9.81; 
        end

        % get rid of all stale Gravity ForceTorque's
        if ~isempty(obj.force_torques)
            mask = [obj.force_torques.type] ~= ForceTorqueType.Gravity;
            obj.force_torques = obj.force_torques(mask);
        end
        
        for B = obj.child_bodies
            gravity = ForceTorque(ForceTorqueType.Gravity, B, B.r_Bo_Bcm, obj.N, zeros(3,1), struct('g',g));
            obj = obj.AddForceTorque(gravity);
        end
    end
    
    function obj = AddJointActuator(obj,joint_index)
        % Adds an actuator with control input at the joint specified which
        % will apply a force (Translation) or torque (Rotation) along the axis
        % of the joint
        % 
        % @input system The RigidBodySystem object
        % @input joint_index The index of the joint the actuator controls
                
        C = obj.bodies(obj.joint_child_bodies(joint_index));
        P = obj.bodies(obj.joint_parent_bodies(joint_index));
        joint = obj.joints(joint_index);

        if joint.type == JointType.Fixed
           error('Cannot add JointActuator to Fixed joint') 
        end
        
        state_index = obj.joint_to_state_index(joint_index);
        u = sym(['tau' num2str(state_index)], 'real');
        
        r_Co_G = [0;0;0];
        params = struct('joint',joint,'u',u);
        actuator = ForceTorque(ForceTorqueType.JointActuator, C, r_Co_G, P, joint.r_Po_Jo, params );

        obj = obj.AddForceTorque(actuator);
        
    end

    function obj = AddAllJointActuators(obj)
        % Adds JointActuators on all non-Fixed joints
        % 
        % @input system The RigidBodySystem object
        
        % get rid of stale JointActuator ForceTorque's
        if ~isempty(obj.force_torques)
            mask = [obj.force_torques.type] ~= ForceTorqueType.JointActuator;
            obj.force_torques = obj.force_torques(mask);
        end

        for i = 1:numel(obj.joints)
            if obj.joints(i).type ~= JointType.Fixed
                obj = AddJointActuator(obj,i);
            end
        end

    end
    
    function u = AggregateControlInputs(obj)
        % Collects all control inputs due to applied variable forces 
        % 
        % @input obj The RigidBodySystem object
        % 
        % @output u A column vector of symbolic variables containing all
        % control inputs to the system
        
        u = sym([]);
        for force_torque = obj.force_torques
            % is this applied force and torque due to an actuator we can control?
            if isfield(force_torque.params,'u')
                u = [u; force_torque.params.u];
            end
        end
    end
    
    function reactions = AggregateUnknownReactions(obj)
        % Collects all unknown constraint force and torque coefficients 
        % in the system. A system consisting of multiple rigid bodies will 
        % have constraint forces and torques which arise at each joint, 
        % in a different manner depending on the type of joint. 
        % These symbolic variables are created by AddJointConstraints(...)
        % 
        % @input obj The RigidBodySystem object
        % 
        % @output reactions A column vector of symbolic variables 
        % containing all reaction force and torque coefficients which 
        % appear in the system
        
        reactions = [];
        for force_torque=obj.force_torques
           if force_torque.type == ForceTorqueType.JointConstraint
               components = symvar([
                   force_torque.params.F_C_J;
                   force_torque.params.T_C_J
                ]).';
               reactions = [ reactions; components];
           end
        end
    
    end
    
    function [path_ind, directions] = FindPath(obj, body1, body2)
      % Determine the path between body1 and body2, specified by indices
      % @param body1 The start body
      % @param body2 The end body
      % @return path A list of the bodies along the chain. The first
      % element of the path will be body1, and the last body2
      % @return directions A vector corresponding to the joints along the
      % path. Thus, this vector is 1 shorter than the path vector.
      % directions is a vector of signs (+1 or -1), indicating whether
      % traversing the path goes in the +1 "normal direction", from parent to
      % child, or in the opposite direction, from child to parent.
      
      body1_ind = obj.body_name_to_index(body1.name);
      body2_ind = obj.body_name_to_index(body2.name);
      
      if body1_ind < 0 || body2_ind < 0 || ...
          body1_ind > length(obj.child_bodies) || body2_ind > length(obj.child_bodies)
        error('RigidBodySystem.FindPath body indices must be between 0 and the number of bodies, inclusive')
      end

      % Find the list of joint indices that connect body1 to body2.
      path_obj = shortestpathtree(obj.graph, body1_ind + 1, body2_ind + 1, 'OutputForm', 'cell');
      
      path_ind = path_obj{1} - 1;
      
      % To compute directions, note that parents are always added before
      % their children
      directions = 2*(path_ind(2:end) > path_ind(1:end-1)) - 1;
    end    
    
    function Draw(obj, q)
        % Draw the current state of the system
        % 
        % @input q The vector of joint positions
        for i=1:length(obj.child_bodies)
          B = obj.child_bodies(i);
          r_No_Bo = RelativePointPosition(obj, q, B, zeros(3,1), obj.N, zeros(3,1));
          R_N_B = RelativeRotationMatrix(obj, q, B, obj.N);
          B.Draw(r_No_Bo, R_N_B);
        end
    
    end
    
    function Animate(obj,ts,qs,qds,qdds,speed, varargin)
        % Animate the system based on sampled configurations over time
        %
        % @input ts A 1xN vector of the sampled times in seconds
        % @input qs A MxN array of configuration samples, where each column
        % is the value of q at that time step
        % @input axes Limits for the axis(...) command
        % @input speed A realtime factor for animation, so .5 would play half
        % as fast, 2 would play twice as fast
        % @input callbacks An optional function handle in the form
        %   callback = @(system,q,qd,qdd,t) ...
        % which draws any additional content for the animation, to be
        % called after system.Draw(q)
        i = 1; t = 0;
        tic

        if nargin < 4
            qds = [];
        end
        if nargin < 5
            qdds = [];
        end
        if nargin < 6
            speed = 1;
        end
                
        while ~isempty(i) % samples remain
          % configuration
          q = qs(:,i);
          
          % velocity
          qd_size = size(qds);
          if(qd_size(2) < i)
              qd = 0;
          else
              qd = qds(:,i);
          end
          
          % acceleration
          qdd_size = size(qdds);
          if(qdd_size(2) < i)
              qdd = 0;
          else
              qdd = qdds(:,i);
          end

          obj.Draw(q);
          if nargin > 5 
            for i = 1:length(varargin)
                varargin{i}(obj,q,qd,qdd,t);
            end
          end
          drawnow

          i = find(ts > toc * speed,1,'first'); % first sample past the current toc
          t = ts(i);
        end
    end
    
    function Graph(~, ts, qs, graphtype, qds)
        % Graph system values based on sampled configurations over time
        %
        % @input ts A 1xN vector of the sampled times in seconds
        % @input qs A MxN array of configuration samples, where each column
        % is the value of q at that time step
        % @input graphtype Determines which values (joint values: 'qs',
        % joint velocities 'qds', all:'a') are plotted
        % @input qds An optional MxN array of joint velocities, where each column is
        % the value of q dot at that time step. If not provided, qds will be
        % interpolated from qs
        
        %Get the number of joints
        [M,~]=size(qs);
        
         figure(2)
         clf
        %Graph the apropriate vales
        if graphtype == "qs"
            for i = 1:M
                subplot(M, 1, i); plot(ts,qs(i,:),'LineWidth',2);
                grid on
            end
            han = axes(figure(2),'visible','off');
            han.Title.Visible = 'on';
            han.XLabel.Visible = 'on';
            han.YLabel.Visible = 'on';
            xlabel(han,'Time in seconds');
            ylabel(han,'Joint configuration, radians or m');
            title(han,'Joint configuration vs time');
            
        elseif graphtype == "qds" 
            if nargin < 5
                qds = (qs-[zeros(M,1),qs(:,1:end-1)])./(ts-[0,ts(1:end-1)]);                
            end

            for i = 1:M
                subplot(M, 1, i); plot(ts,qds(i,:),'LineWidth',2);
                grid on
            end
            han = axes(figure(2),'visible','off');
            han.Title.Visible = 'on';
            han.XLabel.Visible = 'on';
            han.YLabel.Visible = 'on';
            xlabel(han,'Time in seconds');
            ylabel(han,'Joint velocity, radians/s or m/s');
            title(han,'Joint velocity vs time');
            
        elseif graphtype == "a"
            if nargin < 5
                qds = (qs-[zeros(M,1),qs(:,1:end-1)])./(ts-[0,ts(1:end-1)]);                
            end
            
            for i = 1:M
                subplot(M, 2, (i*2)-1); plot(ts,qs(i,:),'LineWidth',2);
                if i == 1
                    title('Joint configurations');
                end
                grid on
                xlim([ts(1),ts(end)])
                
                subplot(M, 2, (i*2)); plot(ts,qds(i,:),'LineWidth',2);
                if i == 1
                    title('Joint velocities');
                elseif i == idivide(M,int8(2),'ceil')
                    ylabel('Joint velocity, radians/s or m/s');
                end
                grid on
                xlim([ts(1),ts(end)])
            end
            
            han = axes(figure(2),'visible','off');
            han.Title.Visible = 'on';
            han.XLabel.Visible = 'on';
            han.YLabel.Visible = 'on';
            xlabel(han,'Time in seconds');
            ylabel(han,'Joint configuration, radians or m');            
        end      
    end
    
  end
  
end