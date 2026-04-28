%% NetCDF Reader
% Author: Henry Hong
% Contact: hmhh@uw.edu
% Last Updated: April 9th, 2026

function allData = nc_file_reader_function(folder, varNames, maxFiles, startIndex)
%% READ_NC_FOLDER Reads multiple NetCDF files and variables
%
% allData = read_nc_folder(folder, varNames, maxFiles, startIndex)
%
% Inputs:
%   folder     - path to folder containing .nc files ([] to select manually)
%   varNames   - cell array of variable names
%   maxFiles   - (optional) maximum number of files to read
%   startIndex - (optional) file index to start from (default = 1)
%
% Output:
%   allData    - cell array of structs (one per file)

    % Folder
    if nargin < 1 || isempty(folder)
        folder = uigetdir;
    end

    if nargin < 3 || isempty(maxFiles)
        maxFiles = Inf;
    end

    if nargin < 4 || isempty(startIndex)
        startIndex = 1;
    end

    % Get files
    files = dir(fullfile(folder, '*.nc'));
    totalFiles = length(files);

    % Validate start index
    if startIndex > totalFiles
        error('startIndex exceeds number of files');
    end

    % Determine end index correctly
    endIndex = min(startIndex + maxFiles - 1, totalFiles);

    numFiles = endIndex - startIndex + 1;

    % Preallocate
    allData = cell(numFiles,1);

    % Loop from startIndex
    idx = 1;
    for k = startIndex:endIndex
        filename = fullfile(files(k).folder, files(k).name);

        S = struct();

        for v = 1:length(varNames)
            varName = varNames{v};

            try
                S.(varName) = ncread(filename, varName);
            catch
                warning(['Could not read variable: ', varName, ...
                         ' in file: ', files(k).name]);
                S.(varName) = [];
            end
        end

        % Metadata
        S.filename = files(k).name;
        S.folder   = files(k).folder;

        allData{idx} = S;
        idx = idx + 1;
    end
end