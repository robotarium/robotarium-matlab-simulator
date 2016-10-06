% This script initializes the Robotarium simulator, adding the required
% paths to the MATLAB instance.

%Get path to simulator
display('Initializing MATLAB simulator')

paths = {'utilities/transformations', ...
    'utilities/barrier_certificates', 'utilities/misc', ...
    'utilities/graph', 'utilities/controllers', 'patch_generation'}; 

path_local_sim_init = pwd;
warned = 0;

for i = 1:length(paths) 
   addpath(strcat(path_local_sim_init, '/', paths{i})); 
   if(exist(paths{i}) ~= 7) 
      warning('The path %s was not correctly added.  Make sure that you are in the directory of the simulator!', paths{i}); 
      warned = 1;
   end
end

% If robot patches haven't been created already... 
% if(warned==0 && ~exist(strcat(path_local_sim_init, '/', 'patch_generation/patches.mat')))
%    % Create GRITSbot patches 
%    display('First time executing this script, generating patches!')
%    tic
%    patches = gritsbot_patch(100);
%    toc
%    save('patch_generation/patches.mat', 'patches')
%    display('Finished generating patches!')
% end

if(warned == 0) 
   addpath(pwd);
   display('MATLAB simulator initialized successfully!')
end

    
