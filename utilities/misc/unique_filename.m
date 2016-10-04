%% unique_filename 
% Returns a timestamped filename with *.mat appended 
%% Example Usage 
%   filename = unique_filename('filename');
function [ filePath ] = unique_filename(file_name)
    date = datetime('now');
    filePath = strcat(file_name, '_', num2str(date.Month), '_', num2str(date.Day), '_', num2str(date.Year), '_', num2str(date.Hour), '_', num2str(date.Minute), '_', num2str(date.Second), '.mat');
end

