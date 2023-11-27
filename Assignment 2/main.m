%% Modelling and Control of Manipulator assignment 2: Manipulator Geometry and Direct kinematic
clc;
clear;
close all;
addpath('include');

%% 1.
% You will need to define all the model matrices, and fill the so called iTj matrices inside BuildTree() function 
% Be careful to define the z-axis coinciding with the joint rotation axis,
% and such that the positive rotation is the same as showed in the CAD model you received.
geom_model = BuildTree();

% Useful initizializations
numberOfLinks = 7;                    % number of manipulator's links.
linkType = [0,0,0,0,0,0,0];           % boolean that specifies two possible link types: Rotational, Prismatic.
bri= zeros(3,numberOfLinks);          % Basic vector of i-th link w.r.t. base
bTi = zeros(4,4,numberOfLinks);       % Trasformation matrix i-th link w.r.t. base

iTj = zeros(4,4,1);
% Initial joint configuration 
q1 = [0,0,0,0,0,0,0];
q2 = [0,0,0,0,0,pi/2,0];
q3 = [0,pi/2,0,-pi/2,0,0,0];
q4 = [pi/4,pi/2,-pi/8,-pi/2,pi/4,2*pi/3,0];

%%
% Q1.1 and Q1.2
biTei1 = GetDirectGeometry(q1, geom_model, linkType);
biTei2 = GetDirectGeometry(q2, geom_model, linkType);
biTei3 = GetDirectGeometry(q3, geom_model, linkType);
biTei4 = GetDirectGeometry(q4, geom_model, linkType);

%%
% Q1.3
linkNumber_i = 1;
linkNumber_j = 4;
iTj = GetFrameWrtFrame(linkNumber_i, linkNumber_j, biTei1)

linkNumber = 3;
bTin = GetTransformationWrtBase(biTei1, linkNumber)

bri = GetBasicVectorWrtBase(biTei1, linkNumber)
%%
% Q1.4
% Hint: use plot3() and line() matlab functions. 
qi1 = q1;
qi2 = q3;
qi3 = [1.3,0.1,0.1,1,0.2,0.3,1.3];
qi4 = [3,0.5,0,0.1,0.8,0.7,0.3];
qi5 = [1.2,0.2,0.7,2.1,1,0,0.9];
qi6 = q1;
qi = [qi1', qi2', qi3', qi4', qi5', qi6'];

qf1 = q4;
qf2 = q1;
qf3 = [2,2,2,2,2,2,2];
qf4 = [2,3,4,2,1,3,2];
qf5 = [2.3,1,0.4,1.7,1,0.3,1.5];
qf6 = [0,pi/2,0,-pi/2,0,pi/4,0];
qf = [qf1', qf2', qf3', qf4', qf5', qf6'];

numberOfSteps = 100; 

for n = 1:6
    PlotMovement(numberOfSteps, qi(:,n), qf(:,n), geom_model, linkType, n)
end