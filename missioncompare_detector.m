function Diagnose = missioncompare_detector(DT_Output, MI_Output)
% missioncompare
% Seaglider Mission-Sim Comparator for Diagnoser
% Author: Dante Weerasooriya
% Date: April 2026
% GOAL: Updated for new binary-window detector
%   Compare Digital Twin (DT) and Mission (MI) outputs variable-by-variable,
%   compute percent difference, and evaluate performance using a windowed
%   binary error method:
%
%       • Convert each sample to an error (1) if percent difference > tolerance,
%         otherwise 0.
%       • Divide the mission into fixed-size windows (e.g., 20 samples).
%       • For each window, compute the error rate:
%             windowErrorRate = (# of errors) / (window size)
%       • A window is flagged if its error rate exceeds 40%.
%       • A variable FAILS only if a flagged window occurs AND it is the last
%         window or all following windows are also flagged.
%   This method detects sustained deviations while ignoring brief spikes.

%% 1. Extract variable names from the first row of each input
varNames_DT = string(DT_Output(1, :));   % DT variable names (row 1)
varNames_MI = string(MI_Output(1, :));   % MI variable names (row 1)

%% 1.1 Extract time vectors (needed before normalization)
time_DT = cell2mat(DT_Output(2:end, 1));
time_MI = cell2mat(MI_Output(2:end, 1));

%% 1.2 Normalize DT and MI to the same time length
len_DT = length(time_DT);
len_MI = length(time_MI);
minLen = min(len_DT, len_MI);

% Trim time vectors
time_DT = time_DT(1:minLen);
time_MI = time_MI(1:minLen);

% Trim the entire DT_Output and MI_Output matrices (keep header row)
DT_Output = DT_Output([1, 2:minLen+1], :);
MI_Output = MI_Output([1, 2:minLen+1], :);

% fprintf('\n=== TIME LENGTH NORMALIZATION ===\n');
% fprintf('DT length: %d samples\n', len_DT);
% fprintf('MI length: %d samples\n', len_MI);
% fprintf('Shortest length: %d samples\n', minLen);
% fprintf('Both datasets trimmed to %d samples.\n', minLen);
% fprintf('---------------------------------------------\n');

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
Diagnose.VariableNames = varNames_DT;  % store variable names
Diagnose.Diff          = struct();     % final Diff metric per variable
Diagnose.Failures      = struct();     % variables that fail threshold
Diagnose.Plots         = struct();     % file paths to saved plots
Diagnose.Excursions    = struct();     % time intervals of excursions
Diagnose.Count         = struct();     % count of samples above tolerance
Diagnose.Excess        = struct();     % total excess difference (sum over mission)
Diagnose.WindowScores  = struct();     % per-window scores (vector per variable)
Diagnose.RemovedZeros  = struct();     % number of zero-value samples removed
Diagnose.difference    = struct();     % percentage difference for comparison

%% 6. Define per-variable percent difference tolerances
%    - Pitch, Roll, Heading: 8.8%
%    - Pitch rate, Roll rate, Heading rate: 4.4%
%    - Others: default tolerance (e.g., 2%)
toleranceMap = containers.Map( ...
    {'eng_pitchAng','eng_rollAng','eng_head','depth'}, ...
    [7.5,    187.5,   13.5,         14.5] ...
);

defaultTolerance = 2.0;   % fallback tolerance if variable not in map

%% 7. Define window size and failure threshold
windowSize       = 20;   % number of samples per window
failureThreshold = 40;   % if ANY window score > 40 → variable fails

%% 8. Create folder for plots (if it does not already exist)
saveFolder = fullfile(pwd, 'MS_Comparator_Plots');  % folder in current directory
if ~exist(saveFolder, 'dir')
    mkdir(saveFolder);   % create folder if missing
end

%% 9. Main loop through variables
% (skip time column, which is column 1)
for i = 2:length(varNames_DT)
    % Get variable name as string and char
    varName     = varNames_DT(i);
    varNameChar = char(varName);

    % Extract raw DT and MI data for this variable
    DT_raw = DT.(varNameChar);
    MI_raw = MI.(varNameChar);

    %  9.1 Remove samples where either DT or MI is zero
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

    % 9.2 Trim to middle 60% of mission:
    %     - Remove first 20% and last 20% of time
    %     - This avoids startup and end-of-dive transients
    trimMask = (t_clean > t0 + 0.2*T) & (t_clean < t0 + 0.8*T);

    DT_trim = DT_clean(trimMask);   % trimmed DT data
    MI_trim = MI_clean(trimMask);   % trimmed MI data
    t_trim  = t_clean(trimMask);    % trimmed time

    % % 9.X TRIMMING VERIFICATION (TIMESTAMPS)
    % % Prove that trimming removed the first and last 20% of the mission.
    % % We print:
    % %   - RAW time range (before zero removal)
    % %   - CLEAN time range (after zero removal)
    % %   - TRIMMED time range (middle 60%)
    % 
    % % RAW time range (from original DT_raw and time vector)
    % raw_time_start = time(find(validMask,1,'first'));
    % raw_time_end   = time(find(validMask,1,'last'));
    % 
    % % CLEAN time range (after zero removal)
    % clean_time_start = t_clean(1);
    % clean_time_end   = t_clean(end);
    % 
    % % TRIMMED time range (after removing first/last 20%)
    % trim_time_start = t_trim(1);
    % trim_time_end   = t_trim(end);
    % 
    % fprintf('\n=== TRIMMING CHECK: %s ===\n', varNameChar);
    % fprintf('RAW time range:    %.3f  -->  %.3f sec\n', raw_time_start, raw_time_end);
    % fprintf('CLEAN time range:  %.3f  -->  %.3f sec\n', clean_time_start, clean_time_end);
    % fprintf('TRIMMED (60%%):     %.3f  -->  %.3f sec\n', trim_time_start, trim_time_end);
    % 
    % % Expected trimming boundaries
    % expected_start = t0 + 0.2*T;
    % expected_end   = t0 + 0.8*T;
    % 
    % fprintf('Expected trim:     %.3f  -->  %.3f sec\n', expected_start, expected_end);
    % fprintf('---------------------------------------------\n');


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

    % 9.3 Remove NaNs (just in case any appear)
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

    % 9.4 Determine variable-specific tolerance (percent difference)
    if isKey(toleranceMap, varNameChar)
        tolerance = toleranceMap(varNameChar);  % use mapped tolerance
    else
        tolerance = defaultTolerance;           % fallback tolerance
    end

    % 9.5 Compute percent difference:
    %     E = |MI - DT| / |DT| * 100
    %     - DT is the reference
    %     - Safe because zeros were removed earlier
    E = abs((MI_trim - DT_trim) ./ DT_trim) * 100;   % percent difference array

    % Saved difference for Comparator
    Diagnose.difference.(varNameChar) = E;

    % 9.6 Excursion mask: where difference exceeds tolerance
    excursionMask = E > tolerance;   % true where difference is above tolerance

    % Count how many samples are above tolerance
    Diagnose.Count.(varNameChar) = sum(excursionMask);

    % 9.7 Excess difference (difference - tolerance, but not below 0)
    %     - This measures how far above the tolerance each sample is
    excessdifference = max(E - tolerance, 0);   % negative values clipped to 0

    % Total excess difference over the entire trimmed mission (for reference)
    Diagnose.Excess.(varNameChar) = sum(excessdifference);

    % 9.8 NEW WINDOWED DETECTOR (binary error + 40% rule)
    % New method:
    %   1. Convert each sample into a binary error:
    %        - 1 if percent difference E > tolerance
    %        - 0 otherwise
    %   2. Break into windows of size 'windowSize'
    %   3. For each window:
    %        windowErrorRate = (# of 1s) / (# of samples in window)
    %   4. A window is "flagged" if windowErrorRate > 40%
    %   5. A variable FAILS only if:
    %        - A flagged window occurs AND
    %        - It is the LAST window OR all windows after it are also flagged
    %      (prevents false positives from isolated spikes)

    % Step 1: Convert percent-difference array into binary error mask
    errorMask = E > tolerance;   % logical vector: 1 = above tolerance, 0 = OK

    numSamples = length(errorMask);
    numWindows = ceil(numSamples / windowSize);   % include partial last window

    % Preallocate arrays
    windowErrorRates = zeros(numWindows,1);   % stores % of errors per window
    windowFlags      = false(numWindows,1);   % true if window > 40% errors

    for w = 1:numWindows
        % Compute window boundaries
        idxStart = (w-1)*windowSize + 1;
        idxEnd   = min(w*windowSize, numSamples);   % clamp to end for partial window

        % Extract binary errors for this window
        windowErrors = errorMask(idxStart:idxEnd);

        % Compute error percentage for this window
        windowErrorRates(w) = (sum(windowErrors) / length(windowErrors)) * 100;

        % Flag window if > 40% of samples exceed tolerance
        windowFlags(w) = windowErrorRates(w) > failureThreshold;
    end
    
    % Store window error rates (for debugging, plotting, etc.)
    Diagnose.WindowScores.(varNameChar) = windowErrorRates;

    % Store failed window information
    Diagnose.WindowFailCount.(varNameChar) = sum(windowFlags);

    Diagnose.TotalWindows.(varNameChar) = numWindows;

    Diagnose.FailedWindowIndices.(varNameChar) = find(windowFlags);

    % 9.9 NEW FAILURE LOGIC (run-to-end rule)
    % A variable FAILS if:
    %   - A window exceeds 40% error rate AND
    %   - That window is the LAST window OR
    %   - ALL windows after it also exceed 40%
    % This prevents false alarms from isolated spikes.
    
    fail = false;
    
    for w = 1:numWindows
        if windowFlags(w)
            % If this window is flagged, check the run-to-end condition
            if w == numWindows || all(windowFlags(w:end))
                fail = true;
                break;   % no need to check further windows
            end
        end
    end
    
    % For reporting, store Diff as the maximum window error rate
    Diagnose.Diff.(varNameChar) = max(windowErrorRates);
    
    % 9.10 Record failure if detected
    if fail
        Diagnose.Failures.(varNameChar) = max(windowErrorRates);
    end
    
    % 9.11 Record excursion time intervals (continuous segments)
    %       - Uses excursionMask (based on E > tolerance)
    %       - Groups consecutive excursion samples into intervals
    idx = find(excursionMask);   % indices where difference > tolerance
    intervals = [];              % will store [startTime endTime] rows
    
    if ~isempty(idx)
        d      = diff(idx);                      % differences between indices
        breaks = [0; find(d > 1); length(idx)];  % segment boundaries

        for b = 1:length(breaks)-1
            seg = idx(breaks(b)+1 : breaks(b+1));   % indices in this segment
            intervals = [intervals; ...
                         t_trim(seg(1)) t_trim(seg(end))]; 
        end
    end

    Diagnose.Excursions.(varNameChar) = intervals;   % store intervals

    % 9.12 Plot DT vs MI with tolerance bands (hidden figure)
    fig = figure('Visible','off');   % create invisible figure

    % Ensure column vectors for plotting
    t_plot  = t_trim(:);
    DT_plot = DT_trim(:);
    MI_plot = MI_trim(:);

    % Plot DT: blue line with square markers
    plot(t_plot, DT_plot, 'b-s', 'LineWidth', 1.5, ...
        'MarkerSize', 3, 'MarkerFaceColor', 'w', 'MarkerEdgeColor', 'b');
    hold on;

    % Plot MI: red line with circle markers
    plot(t_plot, MI_plot, 'r-o', 'LineWidth', 1.5, ...
        'MarkerSize', 2, 'MarkerFaceColor', 'w', 'MarkerEdgeColor', 'r');

    % Compute tolerance bands: DT ± tolerance%
    lower = DT_plot * (1 - tolerance/100);   % lower bound
    upper = DT_plot * (1 + tolerance/100);   % upper bound

    % Plot tolerance bands as dashed black lines
    plot(t_plot, lower, 'k--', 'LineWidth', 1);
    plot(t_plot, upper, 'k--', 'LineWidth', 1);

    % Label axes and add legend
    xlabel('Time (minutes)');
    ylabel(varNameChar);
    legend('DT','MI', ...
           sprintf('-%g%%',tolerance), sprintf('+%g%%',tolerance), ...
           'Location','best');

    % Title and grid
    title(['DT vs MI: ' varNameChar]);
    grid on;

    % Save plot to file
    filePath = fullfile(saveFolder, varNameChar + ".png");
    saveas(fig, filePath);   % save as PNG
    close(fig);              % close figure to avoid clutter

    % Store file path in Diagnose struct
    Diagnose.Plots.(varNameChar) = filePath;
end

%% 10. Summary output to command window

fprintf('\n============================================================\n');
fprintf('MISSION COMPARATOR SUMMARY\n');
fprintf('Window-based failure threshold: %.2f\n', failureThreshold);
fprintf('============================================================\n');

checkedVars = varNames_DT(2:end);

fprintf('%-20s %-10s %-15s %-30s\n', ...
    'Variable', 'Result', 'Diff Score', 'Failed Windows');

fprintf('--------------------------------------------------------------------------------\n');

for i = 1:length(checkedVars)

    varNameChar = char(checkedVars(i));

    % Diff score
    if isfield(Diagnose.Diff, varNameChar)

        diffScore = Diagnose.Diff.(varNameChar);

    else

        diffScore = NaN;

    end

    % PASS / FAIL
    if isfield(Diagnose, 'Failures') && ...
       isfield(Diagnose.Failures, varNameChar)

        resultText = 'FAIL';

    else

        resultText = 'PASS';

    end

    % Failed window indices
    if isfield(Diagnose, 'FailedWindowIndices') && ...
       isfield(Diagnose.FailedWindowIndices, varNameChar)

        failedIdx = Diagnose.FailedWindowIndices.(varNameChar);

        if isempty(failedIdx)

            windowText = 'None';

        else

            windowText = sprintf('%d, ', failedIdx);
            windowText = windowText(1:end-2);

        end

    else

        windowText = 'None';

    end

    fprintf('%-20s %-10s %-15.2f %-30s\n', ...
        varNameChar, resultText, diffScore, windowText);

end

fprintf('--------------------------------------------------------------------------------\n\n');

end