%% unique_filename 
% Returns a timestamped filename with *.mat appended 
%% Example Usage 
%   filename = unique_filename('filename');
function [ filePath ] = unique_filename(file_name)
    date = datetime('now');
    filePath = strcat(file_name, '--', num2str(date.Month), '-', num2str(date.Day), '-', num2str(date.Year), '--', num2str(date.Hour), ':', num2str(date.Minute), ':', num2str(date.Second), '.mat');
end

