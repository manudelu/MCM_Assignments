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
qi = [qi1', qi2', qi3'];

qf1 = q4;
qf2 = q1;
qf3 = [2,2,2,2,2,2,2];
qf = [qf1', qf2', qf3'];

numberOfSteps = 100; 

%for n = 1:6
%    PlotMotion(numberOfSteps, qi(:,n), qf(:,n), geom_model, linkType, n)
%end

%%
% Q1.5

qii = [0,0,0,0,0,0,0];
qfi = qii;
qf4 = [pi/2,0,0,0,0,0,0];
qf5 = [0,pi/2,0,0,0,0,0];
qf6 = [0,0,pi/2,0,0,0,0];
qf7 = [0,0,0,pi/2,0,0,0];
qf8 = [0,0,0,0,pi/2,0,0];
qf9 = [0,0,0,0,0,pi/2,0];
qf10 = [0,0,0,0,0,0,pi/2];

for i = 1:numberOfLinks
    qfi(:, i) = pi/2;
    qfi(:, i-1) = 0;
    qff(:, i) = qfi';
end

qff = [qf4', qf5', qf6', qf7', qf8', qf9', qf10'];

for n = 1:numberOfLinks
    PlotMotion(numberOfSteps, qii, qff(:,n), geom_model, linkType, n)
end

