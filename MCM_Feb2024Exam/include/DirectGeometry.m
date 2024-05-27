function iTj_qi = DirectGeometry(qi, iTj, jointType)
% DirectGeometry Function 
% inputs: 
% qi : current link position;
% iTj is the constant transformation between the base of the link <i>
% and its follower frame <j>;
% jointType :0 for revolute, 1 for prismatic

% output :
% iTj_qi : transformation between the base of the joint <i> and its follower frame taking 
% into account the actual rotation/translation of the joint qi

if jointType == 0 % rotational
    Rz = [cos(qi) -sin(qi) 0; sin(qi) cos(qi) 0; 0 0 1];
    iRj = iTj(1:3, 1:3)*Rz;
    iTj_qi = [iRj iTj(1:3, 4); 0 0 0 1];
elseif jointType == 1 % prismatic
    r = iTj(1:3,4) + iTj(1:3,3)*qi; 
    iTj_qi = [iTj(1:3,1:3) r; 0 0 0 1];
end

end