function Diagnose = missioncompare_comparator(DT_Output, MI_Output)
% missioncompare
% Seaglider Mission-Sim Comparator for Diagnoser
% Author: Dante Weerasooriya
% Date: April 2026
%
% GOAL: 
%   Output the error percentage for comparator

%% 1. Extract variable names from the first row of each input
varNames_DT = string(DT_Output(1, :));   % DT variable names (row 1)
varNames_MI = string(MI_Output(1, :));   % MI variable names (row 1)

%% 2. Basic consistency checks: same number of columns and same variable names
if size(DT_Output, 2) ~= size(MI_Output, 2)
    % If the number of columns (variables) differ, abort with an error
    error('MissionCompare:VariableCountMismatch', ...
          'Digital Twin and Mission data have different numbers of variables.');
end

if ~isequal(varNames_DT, varNames_MI)
    % If the variable names do not match exactly, abort with an error
    error('MissionCompare:VariableNameMismatch', ...
          'Variable names do not match between DT and MI data.');
end

%% 3. Extract time vectors (assumed to be in column 1, rows 2:end)
time_DT = cell2mat(DT_Output(2:end, 1)); % DT time vector
time_MI = cell2mat(MI_Output(2:end, 1)); % MI time vector

% Check that DT and MI have the same number of time samples
if size(time_DT, 1) ~= size(time_MI, 1)
    error('MissionCompare:TimeMismatch', ...
          'Digital Twin and Mission data have different numbers of time samples.');
end

% Use DT time as the reference time vector
time = time_DT;

% Total mission duration and start time (used for trimming)
T  = time(end) - time(1);   % total time span
t0 = time(1);               % start time

%% 4. Build DT and MI structs: one field per variable
DT = struct();   % will hold DT data as DT.<varName>
MI = struct();   % will hold MI data as MI.<varName>

for i = 1:size(DT_Output, 2)
    % Convert variable name to char for struct field access
    varNameChar = char(varNames_DT(i));

    % Extract column i (rows 2:end) and convert from cell to numeric
    DT.(varNameChar) = cell2mat(DT_Output(2:end, i));
    MI.(varNameChar) = cell2mat(MI_Output(2:end, i));
end

%% 5. Initialize Diagnose struct to store all results
Diagnose = struct();
Diagnose.difference    = struct();     % percentage difference for comparison

%% 6. Define per-variable percent difference tolerances
%    - Pitch, Roll, Heading: 8.8%
%    - Pitch rate, Roll rate, Heading rate: 4.4%
%    - Others: default tolerance (e.g., 2%)
toleranceMap = containers.Map( ...
    {'PitchAngle','RollAngle','Heading','PitchAngleRate','RollAngleRate','HeadingRate'}, ...
    [6.6,    6.6,   6.6,      3.3,         3.3,         3.3] ...
);

defaultTolerance = 2.0;   % fallback tolerance if variable not in map

%% 7. Main loop through variables
% (skip time column, which is column 1)
for i = 2:length(varNames_DT)
    % Get variable name as string and char
    varName     = varNames_DT(i);
    varNameChar = char(varName);

    % Extract raw DT and MI data for this variable
    DT_raw = DT.(varNameChar);
    MI_raw = MI.(varNameChar);

    %  7.1 Remove samples where either DT or MI is zero
    %     - This avoids division-by-zero in percent difference
    %     - Also removes clearly invalid data points
    validMask = (DT_raw ~= 0) & (MI_raw ~= 0);  % true where both are non-zero

    % Count how many samples were removed
    numRemoved = sum(~validMask);

    % Store in Diagnose struct
    Diagnose.RemovedZeros.(varNameChar) = numRemoved;

    DT_clean = DT_raw(validMask);   % keep only valid DT samples
    MI_clean = MI_raw(validMask);   % keep only valid MI samples
    t_clean  = time(validMask);     % keep corresponding times

    % If no valid samples remain, record NaNs/empty and skip this variable
    if isempty(DT_clean) || isempty(MI_clean)
        Diagnose.Diff.(varNameChar)         = NaN;
        Diagnose.Count.(varNameChar)        = 0;
        Diagnose.Excess.(varNameChar)       = NaN;
        Diagnose.WindowScores.(varNameChar) = [];
        Diagnose.Excursions.(varNameChar)   = [];
        Diagnose.Plots.(varNameChar)        = '';
        continue;   % move to next variable
    end

    % 7.2 Trim to middle 60% of mission:
    %     - Remove first 20% and last 20% of time
    %     - This avoids startup and end-of-dive transients
    trimMask = (t_clean > t0 + 0.2*T) & (t_clean < t0 + 0.8*T);

    DT_trim = DT_clean(trimMask);   % trimmed DT data
    MI_trim = MI_clean(trimMask);   % trimmed MI data
    t_trim  = t_clean(trimMask);    % trimmed time
   

    % If trimming removed everything, record NaNs/empty and skip
    if isempty(DT_trim) || isempty(MI_trim)
        Diagnose.Diff.(varNameChar)         = NaN;
        Diagnose.Count.(varNameChar)        = 0;
        Diagnose.Excess.(varNameChar)       = NaN;
        Diagnose.WindowScores.(varNameChar) = [];
        Diagnose.Excursions.(varNameChar)   = [];
        Diagnose.Plots.(varNameChar)        = '';
        continue;
    end

    % 7.3 Remove NaNs (just in case any appear)
    nanMask = isnan(DT_trim) | isnan(MI_trim);  % true where either is NaN

    DT_trim(nanMask) = [];   % remove NaNs from DT
    MI_trim(nanMask) = [];   % remove NaNs from MI
    t_trim(nanMask)  = [];   % remove corresponding times

    % If everything got removed, skip this variable
    if isempty(DT_trim) || isempty(MI_trim)
        Diagnose.Diff.(varNameChar)         = NaN;
        Diagnose.Count.(varNameChar)        = 0;
        Diagnose.Excess.(varNameChar)       = NaN;
        Diagnose.WindowScores.(varNameChar) = [];
        Diagnose.Excursions.(varNameChar)   = [];
        Diagnose.Plots.(varNameChar)        = '';
        continue;
    end

    % 7.4 Determine variable-specific tolerance (percent difference)
    if isKey(toleranceMap, varNameChar)
        tolerance = toleranceMap(varNameChar);  % use mapped tolerance
    else
        tolerance = defaultTolerance;           % fallback tolerance
    end

    % 7.5 Compute percent difference:
    %     E = |MI - DT| / |DT| * 100
    %     - DT is the reference
    %     - Safe because zeros were removed earlier
    E = abs((MI_trim - DT_trim) ./ DT_trim) * 100;   % percent difference array

    % Saved difference for Comparator
    Diagnose.difference.(varNameChar) = E;
end
   