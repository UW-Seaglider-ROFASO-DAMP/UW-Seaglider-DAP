%% OP prelim Fault isolator code V3
    %from our ms/mf doc
    %needs to utilize comparator data for tabulation and ranking metrics
    %sort % difference for each fault case iteration into table 
    %Ranks % difference foreach fault case iteration
    % resents the known mission fault case as closest % match
    %V2 changes:
    %realized i can just make the original modifications to
    %missioncompare_comparator in here
    %1. example code in the code after you run missioncompare_comparator:
    %1.1 out0 = missioncompare_comparator(DT_0_Output, MI_Data);
    %    etc
    %    ResultsLibrary = {out0, out50, out100,etc};
    %    CaseNames      = {'0% Rudder Case', '50% Rudder Case', '100% Rudder Case',etc};

    %V3 changes:
    % changed results library and case names to varargin so i don't need to
    % worry about case names. Used Gemini to help with implementing this
    % change. 
    %

    %V4 changes:
    %fixed naming convention bug

    %V5 changes:
    %fixed typo
    %added helper function at the end (if empty function to make sure it
    %works in the code

    %V6 Changes:
    %figured out how to replace helper function with if/else imbedded in
    %isolator
function [RankedTable, BestMatch] = D_fault_isolator(varargin)
    %% How to Use
    %Inputs:
    %varargin: Variable Argument Input: allows us to use as many cases as
    %we want
    %ex: [table, diagnosis] = fault_isolator(score_N, score_50W,score_50R):
    %outputs a best match and table.

    %outputs:
    %RankedTable = autosorted lowest to highest diff 
    
    %BestMatch = lowest percent diff case

    %% 1. Generate scores array

    %find # of tests we are comparing and generates array
    numLibraryCases = nargin;
    finalScores = zeros(numLibraryCases, 1);
    caseNames = cell(numLibraryCases, 1);

    %% 2. Fill scores array w/ (for loop?)

    for i = 1:numLibraryCases
        %finds variable names so we can slap em in the table by reading the
        %text screen used, and generates a generic name incase the variable
        %has no name (otherwise it kept throwing errors)
        varName = inputname(i);
        if isempty(varName)
            caseNames{i} = sprintf('Case_%d', i);
        else
            caseNames{i} = varName;
        end

        %extracts comparator results by getting ith result from the
        %Diagnose structure in the comparator, uses .difference to find the
        %percent error, and lists out every sensor used.
        Resultcur = varargin{i};
        compStruct = Resultcur.difference;
        vars = fieldnames(compStruct);

        %find total error score from all the comparator variables
        error_sum = 0;
        count = 0;
        for v = 1:numel(vars)
            arr = compStruct.(vars{v});
            mean_error = mean(arr,'omitnan');
            if ~isnan(mean_error)
                error_sum = error_sum + mean_error; %the fact that matlab does not have += makes me cry
                count = count + 1; %again, += 
            end
        end
        %global score for the fault case.  the lower it is the closer to
        %mission data the digital twin is. 
        finalScores(i) = error_sum / count; 
        
    end   
    %% 3. Make a Table out of our scores

    %use table function w/ case names
    RankedTable = table(caseNames,finalScores,'VariableNames', {'Digital_Twin_Library_Case', 'Percent_Match_Error'});

    %% 4. Rank table


    %sort table in ascending order
    RankedTable = sortrows(RankedTable, 'Percent_Match_Error', 'ascend');

    %% 5. Best Match

    %picks the topmost row as best match

    BestMatch = RankedTable.Digital_Twin_Library_Case{1};
end









