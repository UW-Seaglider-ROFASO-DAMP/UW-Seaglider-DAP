%% NetCDF to Excel Worksheet
% Author: Henry Hong
% Contact: hmhh@uw.edu
% Last Updated: April 9th, 2026
% IMPORTANT:
% For script to work, the nc_files folders must be downloaded on computer.
% In Student-Seaglider-Center Github
% raw_data > nc_files

%% Clear
clear; clc

%% Toggle Variable Export (USER EDIT)
% Lowest one set to true will be runned, cannot run simultaneously.

export_SelectedVars = true; % reads data from selected variables

export_EngVars = false; % reads data from eng files

export_gcVars = false; % reads data from log files

export_allVars = false; % reads data from all variables

%% Toggle Excel Export (USER EDIT)

runExcelExport = true;

runCSVExport = false;

%% Directory
% Prompt for folder first
folder = uigetdir;
files = dir(fullfile(folder, '*.nc'));
sampleFile = fullfile(files(1).folder, files(1).name);
info = ncinfo(sampleFile);
if isempty(files)
    error('No .nc files found in selected folder');
end

allVars = {info.Variables.Name};

%% Variable Selector (USER EDIT THIS)
% Edit List for selected variables to read
if export_SelectedVars
    vars = {'log_C_PITCH',...
        'eng_pitchCtl'};
end

%% Variable Selector (AUTO from NetCDF)

if export_EngVars   
    % Select only variables starting with 'eng'
    vars = allVars(startsWith(allVars, 'eng'));
end

if export_gcVars   
    % Select only variables starting with 'gc'
    vars = allVars(startsWith(allVars, 'gc'));
end

if export_allVars
    vars = {info.Variables.Name};  % all variable names
end

%% nc_file_reader function (USER EDIT THIS)

data = nc_file_reader_function(folder,vars,[],[]);  % will prompt folder

%% Excel Import
if runExcelExport
    [file, path] = uiputfile('*.xlsx', 'Save Excel File As');
    Excel_Sheet_Title = fullfile(path, file);
    
    for k = 1:length(data)
        S = data{k};
    
        % Find max length among variables
        maxLen = 0;
        for v = 1:length(vars)
            varName = vars{v};
            if isfield(S, varName)
                maxLen = max(maxLen, length(S.(varName)));
            end
        end
    
        % Build table with padded columns
        tblData = table();
        for v = 1:length(vars)
            varName = vars{v};
            if isfield(S, varName)
                col = S.(varName);
                if length(col) < maxLen
                    col(end+1:maxLen,1) = NaN;  % pad with NaN
                end
                tblData.(varName) = col;
            else
                tblData.(varName) = NaN(maxLen,1);
            end
        end
    
        % Info to show once
        info = {'Filename', S.filename; 'Folder', S.folder};
    
        % Additional information if needed, commented out for now
        % Write file/folder info at top
        % writecell(info, Excel_Sheet_Title, 'Sheet', ['File_', num2str(k)], 'Range', 'A1');
        % If uncommented, change write table to write 'A3' below
    
        % Write table
        writetable(tblData, Excel_Sheet_Title, 'Sheet', ['File_', num2str(k)], 'Range', 'A1', 'WriteRowNames', false);
    end
end

%% CSV Export with base filename
if runCSVExport
    % Ask user for folder and base filename
    folderPath = uigetdir([], 'Select folder to save CSV files');
    if isequal(folderPath,0)
        disp('User canceled CSV export.');
    else
        prompt = {'Enter base filename (no extension):'};
        dlgtitle = 'Base CSV Filename';
        dims = [1 50];
        definput = {'MyData'};
        answer = inputdlg(prompt, dlgtitle, dims, definput);
        if isempty(answer)
            disp('User canceled base filename input.');
        else
            baseName = answer{1};

            for k = 1:length(data)
                S = data{k};

                % Generate sequential filename
                csvFileName = fullfile(folderPath, sprintf('%s_%d.csv', baseName, k));

                % Find max length among variables
                maxLen = 0;
                for v = 1:length(vars)
                    varName = vars{v};
                    if isfield(S, varName)
                        maxLen = max(maxLen, length(S.(varName)));
                    end
                end

                % Build table with padded columns
                tblData = table();
                for v = 1:length(vars)
                    varName = vars{v};
                    if isfield(S, varName)
                        col = S.(varName);
                        if length(col) < maxLen
                            col(end+1:maxLen,1) = NaN;  % pad with NaN
                        end
                        tblData.(varName) = col;
                    else
                        tblData.(varName) = NaN(maxLen,1);
                    end
                end

                % Write CSV
                writetable(tblData, csvFileName, 'WriteRowNames', false);
            end
        end
    end
end