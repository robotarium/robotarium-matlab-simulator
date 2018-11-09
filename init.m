% This script initializes the Robotarium simulator, adding the required
% paths to the MATLAB instance.  You need to run this before doing anything
% in the simulator.  Also, DO NOT SUBMIT THIS FILE WITH YOUR EXPERIMENT OR
% CALL IT IN YOUR SCRIPTS.  All of these utilities will be automatically
% included in your experiment!

path = genpath('utilities');
if(isempty(path))
    disp('WARNING: Cannot find utilities directory.  This script should be run from the base directory.')
else
    addpath(path)
end

path = genpath('patch_generation');
if(isempty(path))
    disp('WARNING: Cannot find patch_generation directory.  This script should be run from the base directory.')
    
else
    addpath(path)
end

addpath('./')