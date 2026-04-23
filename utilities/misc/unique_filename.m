function filePath = unique_filename(file_name)
% UNIQUE_FILENAME  Append a timestamp to a filename and add a .mat extension.
%
%   filePath = unique_filename(file_name)
%
%   Input
%   -----
%   file_name : base filename string (no extension required)
%
%   Output
%   ------
%   filePath : string of the form '<file_name>_YYYY_MM_DD_HH_MM_SS.mat'
%
%   Example
%   -------
%       path = unique_filename('experiment');
%       % -> 'experiment_2026_03_23_14_05_09.mat'

    t = datetime('now', 'Format', 'yyyy_MM_dd_HH_mm_ss');
    filePath = sprintf('%s_%s.mat', file_name, char(t));
end