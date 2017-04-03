% Use optimization to find center of mass and velocity

% globals
global markerData
global numSamples

load markerData

numFrames = 100; % number of frames (including initial frame)

% set options for fminunc()
% options = optimset();
options = optimset('MaxFunEvals',1000000);

% p0 is the intitial parameter vector
p0 = zeros(1,6);

% do optimization
[answer,fval,exitflag]=fminunc(@criterion,p0,options);

