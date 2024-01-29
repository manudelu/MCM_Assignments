%% Modelling and Control of Manipulator assignment 3 - Exercise 2 and 3: Inverse Kinematic Control
addpath('include')
model = load("panda.mat"); % don't worry about eventual warnings!
% Simulation Parameters
ts = 0.5;
t_start = 0.0;
t_end = 30.0;
t = t_start:ts:t_end;

% Initial Joints configuration
q_init = [0.0167305,-0.762614,-0.0207622,-2.34352,-0.0305686,1.53975,0.753872]';
% Joint limits
qmin = [-2.8973;-1.7628;-2.8973;-3.0718;-2.8973;-0.0175;-2.8973];
qmax = [2.8973;1.7628;2.8973;-0.0698;2.8973;3.7525;2.8973];
% Function that gives the transformation from <base> to <e-e>, given a
% configuration of the manipulator
bTe = getTransform(model.franka,[q_init',0,0],'panda_link7');%DO NOT EDIT 

bOg = [0.55, -0.3, 0.2]';
theta = pi/6;
phi = -44.98;

% Switch between the two cases (with and without the tool frame)
tool = false; % change to true for using the tool
if tool == true
    eRt = [cos(phi)  -sin(phi)  0; sin(phi)  cos(phi)  0; 0  0  1];
    eTt = [eRt [0, 0, 0.2104]'; 0 0 0 1];
    bTt = bTe(1:3,1:3)*eTt(1:3,1:3);
    tRgt = [cos(theta)  0  sin(theta); 0  1  0; -sin(theta)  0  cos(theta)]; 
    bRgt = bTt(1:3,1:3)*tRgt;
    bTgt = [bRgt bOg; 0 0 0 1];  % if controlling the tool frame
else
    eRge = [cos(theta) 0 sin(theta); 0  1  0; -sin(theta)  0  cos(theta)]; 
    bRge = bTe(1:3,1:3)*eRge;
    bTge = [bRge bOg; 0 0 0 1];  % if controlling the ee frame
end   

% Control Proportional Gain 
angular_gain = 0.2;
linear_gain = 0.2;
% Preallocation variables
x_dot = zeros(6,1);
lin_err = zeros(3,1);
ang_err = zeros(3,1); 
% Start the inverse kinematic control  
q = q_init;

%% Simulation Loop
for i = t
    
    if tool == true %compute the error between the tool frame and goal frame
        
        % Computing transformation matrix from base to end effector 
        bTe = getTransform(model.franka,[q',0,0],'panda_link7'); %DO NOT EDIT
        bTt = bTe*eTt;
        tmp = geometricJacobian(model.franka,[q',0,0],'panda_link7'); %DO NOT EDIT
        bJe = tmp(1:6,1:7); %DO NOT EDIT

        % Compute error at each step
        lin_err = bTgt(1:3,4) - bTt(1:3,4);
        bRg = bTt(1:3,1:3);
        [theta, h] = ComputeInverseAngleAxis(bRg'*bRgt);
        ang_err = bRg*h'*theta; 

        % Compute the reference velocities
        w_t = angular_gain * ang_err;
        v_t = linear_gain * lin_err;
        x_dot = [w_t; v_t];

        % Rigid body transformation
        skew_matrix =[0 -eTt(3,4) eTt(2,4) ; eTt(3,4) 0 -eTt(1,4) ; -eTt(2,4) eTt(1,4) 0]; 
        eeTt = [eye(3) zeros(3,3); skew_matrix eye(3)];
        % Calculation of the Jacobian matrix from base to rigid-tool with rbt
        bJt = eeTt * bJe;
    
        % Compute desired joint velocities 
        q_dot = pinv(bJt)*x_dot;
        
    else % compute the error between the e-e frame and goal frame
        % Computing transformation matrix from base to end effector 
        bTe = getTransform(model.franka,[q',0,0],'panda_link7'); %DO NOT EDIT
        % Computing end effector jacobian w.r.t. base
        tmp = geometricJacobian(model.franka,[q',0,0],'panda_link7'); %DO NOT EDIT
        bJe = tmp(1:6,1:7); %DO NOT EDIT

        % Compute error at each step
        lin_err = bTge(1:3,4) - bTe(1:3,4);
        bRe = bTe(1:3,1:3);
        [theta, h] = ComputeInverseAngleAxis(bRe'*bRge);
        ang_err = bRe*h'*theta; 

        % Compute the reference velocities
        w_e = angular_gain * ang_err;
        v_e = linear_gain * lin_err;
        x_dot = [w_e; v_e];
    
        % Compute desired joint velocities 
        q_dot = pinv(bJe)*x_dot;
    end

    %% Simulate the robot - implement the function KinematicSimulation()
    q = KinematicSimulation(q(1:7), q_dot,ts, qmin, qmax);
    
    % DO NOT EDIT - plot the robot moving
    %switch visuals to off for seeing only the frames
    show(model.franka,[q',0,0],'visuals','on');
    hold on
    if tool == true
        %set the window size of the figure to "full-screen" for a better visualization
        plot3(bTe(1,4),bTe(2,4),bTe(3,4),'go','LineWidth',15);
        plot3(bOg(1),bOg(2),bOg(3),'ro','LineWidth',5);
    else
        plot3(bTe(1,4),bTe(2,4),bTe(3,4),'go','LineWidth',15);
        plot3(bOg(1),bOg(2),bOg(3),'ro','LineWidth',5);
    end
    drawnow
    if(norm(x_dot) < 0.001)
        disp('REACHED THE REQUESTED GOAL POSITION')
        break
    end
    % refreshing the window every loop-1.
    if i < t_end 
        % function that clean the window.
        cla();
    end
end
