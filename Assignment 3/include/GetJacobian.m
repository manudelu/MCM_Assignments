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
    n = size(jointType, 1);
    
    % Initialize the Jacobian
    J = zeros(6, n);
    
    for i = 1:n
        % Get the transformation matrix that describes the pose of the joint 
        % <i> w.r.t. the base of the manipulator
        bTi = GetTransformationWrtBase(biTei, i);
        
        % Fill the Jacobian's columns

        % Initialize the i-th column of the Jacobian
        h = zeros(6, 1);

        % Unit vector that indicates the direction of the i-th joint's axis,
        % since all the frames have been assigned s.t. the z axis points in the 
        % direction of the joints' axes it holds 
        k_i = bTi(1:3, 3);
        
        % Compute the distance vector between the end-effector and joint <i>
        r_e0 = bTe(1:3, 4);      % position of the end-eff w.r.t. the base
        r_i0 = bTi(1:3, 4);      % position of <i> w.r.t. the base
        r_ei = r_e0 - r_i0;      % distance between end-eff and <i>
        
        if(jointType == 0) 
            % revolute joint
            h = [k_i ; cross(k_i, r_ei)]; 
            J(:,i) = h;
        
        elseif(jointType == 1)       
            % prismatic joint
            h = [0 ; 0 ; 0 ; k_i];
            J(:,i) = h;
        end
    end
end
