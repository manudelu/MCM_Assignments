function [rot_matrix] = quatToRot(q0,q1,q2,q3)
% quatToRot convert a quaternion into a rotation matrix
    %Covert a quaternion into a full three-dimensional rotation matrix.
 
    %Input
    %:param Q: A 4 element array representing the quaternion (q0,q1,q2,q3) 
 
    %Output
    %return: A 3x3 element matrix representing the full 3D rotation matrix. 

    %First row of the rotation matrix
    r11 = 2*(q0^2 + q1^2) - 1;
    r12 = 2*(q1*q2 + q0*q3);
    r13 = 2*(q1*q3 - q0*q2);
    %Second row of the rotation matrix
    r21 = 2*(q1*q2 - q0*q3);
    r22 = 2*(q0^2 + q2^2) -1;
    r23 = 2*(q2*q3 + q0*q1);
    %Third row of the rotation matrix
    r31 = 2*(q1*q3 + q0*q2);
    r32 = 2*(q2*q3 - q0*q1);
    r33 = 2*(q0^2 + q3^2) - 1;
    
    %3x3 rotation matrix
    rot_matrix = [r11 r12 r13; 
                  r21 r22 r23; 
                  r31 r32 r33];

end