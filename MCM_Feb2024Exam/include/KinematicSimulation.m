function [q] = KinematicSimulation(q, q_dot, ts, q_min, q_max)
%% Kinematic Simulation function
%
% Inputs
% - q current robot configuration
% - q_dot joints velocity
% - ts sample time
% - q_min lower joints bound
% - q_max upper joints bound
%
% Outputs
% - q new joint configuration

    % Update the joint configuration based on the velocities and time step
    % (basic forward kinematics integration step)
    q = q + q_dot*ts;
    
    % Saturate the joint angles to ensure they stay within the defined limits
    for i=1:length(q)
        if q(i) < q_min(i)
            q(i) = q_min(i);
        elseif q(i) > q_max(i)
            q(i) = q_max(i);
        end
    end

end