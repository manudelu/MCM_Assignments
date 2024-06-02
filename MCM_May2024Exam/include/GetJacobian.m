%% Delucchi Manuel S4803977
function J = GetJacobian(biTei, bTe, jointType)
%% GetJacobian function
% Function returning the end effector jacobian for a manipulator which current
% configuration is described by bTei.
%
% Inputs:
% - biTei: vector of matrices containing the transformation matrices from
% joint i-1 to joint i for the current configuration.
% - bTe: current transformation matrix from base to the end effector.
% - jointType: vector identifying the joint type, 0 for revolute, 1 for
% prismatic
%
% Output:
% - J: end-effector jacobian matrix

    % Get the number of joints in the manipulator's structure
    n = length(jointType);
    
    % Initialize the Jacobian matrix
    J = zeros(6, n);
    
    % Iterate through each joint to calculate the Jacobian columns
    for i = 1:n
        % Get the transformation matrix that describes the pose of the joint 
        % <i> w.r.t. the base of the manipulator
        bTi = GetTransformationWrtBase(biTei, i);

        % Initialize the i-th column of the Jacobian
        h = zeros(6, 1);

        % Extract the unit vector indicating the direction of the i-th joint's axis
        % All frames are oriented such that the z-axis points along joint axes 
        k_i = bTi(1:3, 3);
        
        % Compute the distance vector between the end-effector and joint <i>
        r_e0 = bTe(1:3, 4);      % position of the end-eff w.r.t. the base
        r_i0 = bTi(1:3, 4);      % position of <i> w.r.t. the base
        r_ei = r_e0 - r_i0;      % distance between end-eff and <i>
        
        % Compute the Jacobian column based on joint type
        if(jointType(i) == 0) 
            % For a revolute joint
            h = [k_i ; cross(k_i, r_ei)]; 
            J(:,i) = h;
        
        elseif(jointType(i) == 1)       
            % For a prismatic joint
            h = [0 ; 0 ; 0 ; k_i];
            J(:,i) = h;
        end
    end
end
