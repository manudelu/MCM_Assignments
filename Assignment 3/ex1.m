%% Modelling and Control of Manipulator assignment 3 - Exercise 1: Jacobian matrix
clc;
clear;
close("all");
addpath('include');

% The same model of assignment 2
geom_model = BuildTree();
numberOfLinks = size(geom_model,3); % number of manipulator's links.
linkType = zeros(numberOfLinks,1); % specify two possible link type: Rotational, Prismatic.
bTi = zeros(4,4,numberOfLinks);% Trasformation matrix i-th link w.r.t. base

% Initial joint configuration 
q1 = [1.8,1.8,1.8,1.8,1.8,1.8,1.8];
q2 = [0.3, 1.4, 0.1, 2.0, 0, 1.3, 0];
q3 = [0, 0.1, 0.2, 0.3, 0.4, 0.5, 0];
q4 = [1, 1, 1, 1, 1, 1, 1];
q = [q1', q2', q3', q4'];

for n=1:4
    % Compute direct geometry
    biTei = GetDirectGeometry(q(:,n), geom_model, linkType);
    % Compute the transformation w.r.t. the base
    bTe = GetTransformationWrtBase(biTei, numberOfLinks);
    % Compute the Jacobian matrices 
    J(:,:,n) = GetJacobian(biTei, bTe, linkType); 
end

J(:,:,:)
