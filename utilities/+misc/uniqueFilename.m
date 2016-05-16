function [ filePath ] = uniqueFilename(fileName)
%UNIQUEFILENAME Returns a time-stamped file name
%  Appends the current time information to the passed argument

    date = datetime('now');
    filePath = strcat(fileName, '--', num2str(date.Month), '-', num2str(date.Day), '-', num2str(date.Year), '--', num2str(date.Hour), ':', num2str(date.Minute), ':', num2str(date.Second), '.mat');

end

