function Mission_Data_Matrix = Create_Mission_Data_Matrix(Log_Matrix, NC_Matrix)
% CREATE_MISSION_DATA_MATRIX
%
% Combines important Log file parameters and NC file data into one matrix.
%
% INPUTS:
%   Log_Matrix - output from Log_File_Unpacker
%                Column 1 = names
%                Column 2 = values
%
%   NC_Matrix  - output from NC_File_Unpacker
%                Row 1 = variable names
%                Rows 2:end = data
%
% OUTPUT:
%   Mission_Data_Matrix
%       Row 1 = variable names
%       Rows 2:end = numeric mission data

%% =========================================================
% SELECT IMPORTANT LOG PARAMETERS HERE
% ==========================================================

logNames = { ...
    '$C_PITCH', ...
    '$C_ROLL_DIVE', ...
    '$C_ROLL_CLIMB', ...
    '$MHEAD_RNG_PITCHd_Wd', ...
    '$C_VBD', ...
    };

%% =========================================================
% SELECT IMPORTANT NC VARIABLES HERE
% ==========================================================

ncNames = { ...
    'eng_elaps_t_0000', ...
    'eng_elaps_t', ...
    'eng_depth', ...
    'eng_head', ...
    'eng_pitchAng', ...
    'eng_rollAng', ...
    'eng_pitchCtl', ...
    'eng_rollCtl', ...
    'eng_vbdCC', ...
    'temperature', ...
    'pressure', ...
    'density', ...
    };

%% =========================================================
% EXTRACT NC DATA
% ==========================================================

NC_headers = NC_Matrix(1,:);
NC_data = cell2mat(NC_Matrix(2:end,:));

missionHeaders = {};
missionData = [];

for i = 1:length(ncNames)

    idx = strcmp(NC_headers, ncNames{i});

    if any(idx)

        missionHeaders{end+1} = ncNames{i};
        missionData(:,end+1) = NC_data(:,idx);

    else

        warning('NC variable "%s" not found. Skipping.', ncNames{i});

    end

end

%% =========================================================
% EXTRACT LOG PARAMETERS AND REPEAT DOWN ROWS
% ==========================================================

numRows = size(missionData,1);

for i = 1:length(logNames)

    idx = strcmp(Log_Matrix(:,1), logNames{i});

    if any(idx)

        rowNum = find(idx, 1);

        % Take ONLY the first value after the variable name
        value = Log_Matrix{rowNum, 2};

        % Convert to numeric if needed
        if ischar(value) || isstring(value)

            numericValue = str2double(value);

        else

            numericValue = value;

        end

        if isnan(numericValue)

            warning('Log parameter "%s" is not numeric. Skipping.', logNames{i});
            continue;

        end

        % Repeat log value down entire column
        repeatedColumn = numericValue * ones(numRows,1);

        missionHeaders{end+1} = logNames{i};
        missionData(:,end+1) = repeatedColumn;

    else

        warning('Log parameter "%s" not found. Skipping.', logNames{i});

    end

end

%% =========================================================
% FILL MISSING DATA
% ==========================================================
%
% Rules:
%
% 1) If NaNs occur at the START of a column:
%       Fill with first recorded value
%
% 2) If NaNs occur AFTER valid data:
%       Fill with previous recorded value
%
% Example:
%
%   [NaN NaN 5 6 NaN 8 NaN]
%
% becomes
%
%   [5 5 5 6 6 8 8]
%

[numRows, numCols] = size(missionData);

for col = 1:numCols

    % Extract one column
    dataCol = missionData(:,col);

    % -----------------------------------------------------
    % FIND FIRST VALID VALUE
    % -----------------------------------------------------

    firstValidIdx = find(~isnan(dataCol), 1, 'first');

    % If entire column is NaN, skip it
    if isempty(firstValidIdx)

        warning('Column %d contains only NaNs. Skipping.', col);
        continue;

    end

    % -----------------------------------------------------
    % FILL LEADING NaNs
    % -----------------------------------------------------

    dataCol(1:firstValidIdx-1) = dataCol(firstValidIdx);

    % -----------------------------------------------------
    % FILL REMAINING NaNs
    % -----------------------------------------------------

    for row = firstValidIdx+1:numRows

        if isnan(dataCol(row))

            dataCol(row) = dataCol(row-1);

        end

    end

    % Store corrected column back
    missionData(:,col) = dataCol;

end

%% =========================================================
% BUILD FINAL MISSION DATA MATRIX
% ==========================================================

Mission_Data_Matrix = [missionHeaders; num2cell(missionData)];

%% =========================================================
% DISPLAY SUMMARY
% ==========================================================

fprintf('\n====================================\n');
fprintf('Mission Data Matrix Created\n');
fprintf('====================================\n');

fprintf('Rows: %d numeric rows + 1 header row\n', numRows);
fprintf('Columns: %d\n', length(missionHeaders));

fprintf('\nMission Data Columns:\n');

for i = 1:length(missionHeaders)

    fprintf('%2d : %s\n', i, missionHeaders{i});

end

fprintf('====================================\n');