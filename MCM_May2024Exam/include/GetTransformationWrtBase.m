%% Delucchi Manuel S4803977
function [bTi] = GetTransformationWrtBase(biTei, linkNumber)
%%% GetTransformatioWrtBase function
%
% Inputs :
% biTei vector of matrices containing the transformation matrices
% from joint i to joint i +1 for a given q.
% The size of biTei is equal to (4,4,numberOfLinks)
% linkNumber for which computing the transformation matrix

% outputs
% bTi : transformation matrix from the manipulator base to the ith joint in
% the configuration identified by biTei.

bTi = eye(4,4);

for i = 1:linkNumber
    bTi = bTi*biTei(:,:,i);
end

end