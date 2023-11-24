function iTj_q = DirectGeometry(qi, iTj, linkType)
% DirectGeometry Function 
% inputs: 
% qi : current joint position;
% iTj is the constant transformation between the base of the link <i>
% and its follower frame <j>; 
% jointType :0 for revolute, 1 for prismatic

% output :
% iTj_q : transformation between the base of the joint <i> and its follower frame taking 
% into account the actual rotation/traslation of the joint

if linkType == 0 % rotational
    Rz = [cos(qi) -sin(qi) 0; sin(qi) cos(qi) 0; 0 0 1];
    iRj = iTj(1:3, 1:3)*Rz;
    iTj_q = [iRj iTj(1:3, 4); 0 0 0 1];

elseif linkType == 1 % prismatic
    
    % due to the config of the robot there isn't any prismatic joint
    disp('Erros, due to the config of the robot there isnt any prismatic joint');

end

end