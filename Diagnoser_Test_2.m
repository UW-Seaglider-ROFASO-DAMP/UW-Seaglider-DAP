%% SEAGLIDER MASTER DIAGNOSER SYSTEM
% Architecture: Comparator_Windowed(Trigger) -> 
% comparator_average (Verification) -> Isolator (Diagnosis)

clear; clc;

% 1. DATA INGESTION
% Load your Unknown Mission Data (MI)
MI_Data = missioncompare_unpack_eng_to_dive_data('p1940008.eng');

% Load your "Digital Twin Library" (The Suspects)
DT_Lib.Nominal = missioncompare_unpack_eng_to_dive_data('p1940005.eng');
DT_Lib.Rudder  = missioncompare_unpack_eng_to_dive_data('p1940008.eng');
DT_Lib.Wing    = missioncompare_unpack_eng_to_dive_data('p1940011.eng');

% --- START DIAGNOSIS PIPELINE ---

%% STEP 1: INITIAL DETECTION 
% We use the Nominal case as our baseline healthy model
fprintf('Executing Tier 1: Windowed Fault Detection...\n');
detection_result = missioncompare_version_4(DT_Lib.Nominal, MI_Data);

% Check if Tier 1 flags a fault
isFaultDetected = ~isempty(fieldnames(detection_result.Failures));

if ~isFaultDetected
    fprintf('DIAGNOSIS: Glider is HEALTHY (No sustained window failures).\n');
else
    fprintf('FAULT DETECTED! Proceeding to Tier 2 Verification...\n');
    
    %% STEP 2: STATISTICAL VERIFICATION 
    % We verify if the fault exceeds the "Natural Noise Floor" (25% duration @ 40% error)
    verification_score = missionIsolate_version_holland_4(DT_Lib.Nominal, MI_Data);
    
    %  FinalValue > 25% 
    if verification_score.FinalValue < 25
        fprintf('WARNING: Fault detected by windows, but below statistical noise floor (%.2f%%).\n', ...
                verification_score.FinalValue);
        fprintf('Potential Transient Event. Proceeding with caution...\n');
    else
        fprintf('VERIFIED: Fault exceeds statistical noise floor (%.2f%%).\n', ...
                verification_score.FinalValue);
    end
    
    %% STEP 3: FAULT ISOLATION (Ranking the suspects)
    fprintf('Executing Tier 3: Fault Isolation and Ranking...\n');
    
    % Run comparator against the rest of the library to find the match
    out_N = detection_result; % Already ran this
    out_R = missioncompare_version_4(DT_Lib.Rudder, MI_Data);
    out_W = missioncompare_version_4(DT_Lib.Wing, MI_Data);
    
    % The Isolator determines which one is the "Closest % Match"
    [RankingTable, BestMatch] = D_fault_isolator(out_N, out_R, out_W);
    
    % FINAL REPORTING
    fprintf('\n============================================================\n');
    fprintf('FINAL DAMAGE ASSESSMENT REPORT\n');
    fprintf('============================================================\n');
    fprintf('Primary Diagnosis: %s\n', BestMatch);
    fprintf('Confidence Metric (Error): %.2f%%\n', RankingTable.Percent_Match_Error(1));
    disp(RankingTable);
    fprintf('============================================================\n');
end