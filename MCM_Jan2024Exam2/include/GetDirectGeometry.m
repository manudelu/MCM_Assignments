function [iTj_q] = GetDirectGeometry(q, iTj, jointType)
%%% GetDirectGeometryFunction

% Inputs: 
% q : links current position ; 
% geom_model : vector of matrices containing the transformation matrices from link
% i to link j
% jointType: vector of size numberOfLinks identiying the joint type, 0 for revolute, 1 for
% prismatic.

% Outputs :
% biTei vector of matrices containing the transformation matrices 
% from link i to link j for the input q. 
% The size of geom_model is equal to (4,4,numberOfLinks)

numberOfLinks = length(jointType);

for i = 1:numberOfLinks
    iTj_q(:,:,i) = DirectGeometry(q(i),iTj(:,:,i),jointType(i));
end

end