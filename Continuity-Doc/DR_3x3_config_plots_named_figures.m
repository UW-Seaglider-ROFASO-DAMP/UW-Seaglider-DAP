% Basic UWAL 3x3 Wind Tunnel DR Script
% What it does: Weight tare, Shindo's blockage, moment transfer
% What it doesn't do: Wall corrections, angularity corrections, 
%                     additional tares, bring joy
% All values are still in the body frame
% Plotting done (poorly) by chatgpt
% Good luck everybody else — KW

%% Boring Initial Stuff
clear;close;clc

% 3x3 properties
C_A = 0.9144^2; % 3x3 cross sectional area in m^2
B = 0.9144; % test section is 1m across :), 3ft = 0.9144 m
BMC = 4.905e-2; % balance moment center in m, from conversation with miguel

%% Somebody please look at some cad for me

% From SG Balance Sheet (CB location) 
MMx = (180-95.752) * 0.17; % cm, converted to model size
MMy = 0;
MMz = 0;

% From WT Model CAD (cg location in CM from BALANCE MOUNT), optional
CGx = 0+BMC; % Is calculated later, not necessary now
CGy = 0;
CGz = 0;

% Model properties
span = 0.17; % wing span m
area = 0.01405; % reference area (total area in x-y plane) m^2
area_wing = 0.005429; % wing area m^2


%% Reading data 

% Reading data .csv (will add when i get a .csv)
data = readmatrix('alldata.csv'); % Reading file

run = data(:,1); % all run numbers for each data point
tp = data(:,2); % test point numbers for each data point, per run
code = data(:,3); % test point code, used for removing woz's for now
Q_A = data(:,4); % uncorrected dynamic pressure
% temp = data(:,5); % temperature, unused for now
% pres = data(:,6); % pressure, unused for now
L = data(:,14); % lift (lbf)
D = data(:,15); % drag (lbf)
M_pitch = data(:,16); % pitch moment (lbf-in)
M_yaw = data(:,17); % yaw moment (lbf-in)
Y = data(:,18); % side force (lbf)
M_roll = data(:,19); %roll moment
alpha = data(:,20); % aoa, from run log
beta = data(:,21); % sideslip, from run log

%% Unit Conversions (maybe add switch to US units later?)

% SI
Q_A = Q_A * 47.880208; % Psf to Pa
L = L * 4.44822; % Lbf to N
D = D * 4.44822; % Lbf to N
Y = Y * 4.44822; % Lbf to N
M_pitch = M_pitch * 0.112985; % Lbf-in to N-m
M_roll = M_roll * 0.112985; % Lbf-in to N-m
M_yaw = M_yaw * 0.112985; % Lbf-in to N-m
MMx = MMx * 0.01; % cm to m
MMy = MMy * 0.01; % cm to m
MMz = MMz * 0.01; % cm to m
CGx = CGx * 0.01 + BMC; % cm to m, adding in distance to BMC
CGy = CGy * 0.01; % cm to m
CGz = CGz * 0.01; % cm to m

%% Trimming & Inverse for UD Runs

% Trimming WOZ's
for i = length(L):-1:1
    if code(i) == 1
    L(i) = [];
    D(i) = [];
    Y(i) = [];
    M_pitch(i) = [];
    M_roll(i) = [];
    M_yaw(i) = [];
    Q_A(i) = [];
    alpha(i) = [];
    beta(i) = [];
    run(i) = [];
    tp(i) = [];
    end
end


%% Weight Tares-

% For this method we assume you do the following for a weight tare:
% 1) Install model
% 2) Wind off zero at alpha = 0
% 3) Take test point at max alpha

% WT Run #s
wtruns = [1 3 5 7 9 11 14 16 18]; % from run log

% Weight averaged between lift and drag readings, where:
% L = w (1 - cosd(alpha) )
% D = w sind(alpha)
for i = 1:length(run)
    if ismember(run(i),wtruns)
        weight(i) =abs ( 0.5 * ( L(i) / (1 - cosd(alpha(i) ) ) + ...
                            D(i) / sind( alpha(i) ) ) );
    else
        weight(i) = weight(i-1);
    end
end

% Trimming the strut tare weight tare
weight = weight(1,18:end);

% Taking mean for scalar weight for all damage modes. <5g was taken off for
% wing mode
weight = mean(weight);

% Finding CGx from weight (remove if we get CGx from CAD
for i = 1:length(run)
    if ismember(run(i),wtruns)
        CGx(i) = abs( M_pitch(i) / (weight * (1 - cosd(alpha(i)) ) ) );
    end
end
CGx = CGx(1,18:end); % trimming strut tare stuff
for i = length(CGx):-1:1 % trimming 0's from CGx
    if CGx(i) == 0
        CGx(i) = [];
    end
end
CGx = mean(CGx); % Setting CGx as mean of the values

% Weight tare
for i = 18:length(L) % starting at tp 18 which is first wind on model tp
    if ismember(run(i),wtruns)
        continue
    else
        L(i) = L(i) - weight * (1 - cosd(alpha(i)));
        D(i) = D(i) - weight * sind(alpha(i));
        M_pitch(i) = M_pitch(i) - weight * CGx * (1 - cosd(alpha(i)));
    end
end

% Weight tare for strut only (first non-trimmed tp is strut tare)
w_strut = abs ( 0.5 * ( L(1) / (1 - cosd(alpha(1) ) ) + ...
                            D(1) / sind( alpha(1) ) ) );
% Strut CG location
CGx_strut = abs(M_pitch(1) / (w_strut * (1 - cosd(alpha(1)))));

% Applying weight tare to strut tare run
for i = 2:17 % rows 2:17 are the wind-on strut-only tare sweep
    if ismember(run(i),wtruns)
        continue
    else
        L(i) = L(i) - w_strut * (1 - cosd(alpha(i)));
        D(i) = D(i) - w_strut * sind(alpha(i));
        M_pitch(i) = M_pitch(i) - w_strut * CGx_strut * (1 - cosd(alpha(i)));
    end
end

%% Strut Tares

% Tare values
tareAlpha = alpha(2:17);
tareBeta  = beta(2:17);
tareQ = Q_A(2:17);
tareL = L(2:17);
tareD = D(2:17);
tareY = Y(2:17);
tareMpitch = M_pitch(2:17);
tareMroll = M_roll(2:17);
tareMyaw = M_yaw(2:17);

% Angle tolerance
angleTol = 1e-8;

for i = 18:length(run)

    % Skip weight-tare points.
    if ismember(run(i), wtruns)
        continue
    end

    a = alpha(i);
    b = beta(i);

    % First try exact alpha/beta match.
    k = find(abs(tareAlpha - a) < angleTol & ...
             abs(tareBeta  - b) < angleTol, 1);

    % This strut tare set only has beta >= 0.
    % For beta < 0, mirror the positive-beta tare.
    % Even in beta: L, D, M_pitch.
    % Odd in beta: Y, M_roll, M_yaw.
    oddBetaSign = 1;
    if isempty(k)
        k = find(abs(tareAlpha - a) < angleTol & ...
                 abs(tareBeta  - abs(b)) < angleTol, 1);

        if ~isempty(k) && b < 0
            oddBetaSign = -1;
        end
    end

    % Scale tare by dynamic pressure.
    qScale = Q_A(i) / tareQ(k);

    % Direct subtraction for even beta terms
    L(i) = L(i) - qScale * tareL(k);
    D(i) = D(i) - qScale * tareD(k);
    M_pitch(i) = M_pitch(i) - qScale * tareMpitch(k);

    % Direct subtraction with beta symmetry for odd beta terms
    Y(i) = Y(i) - qScale * oddBetaSign * tareY(k);
    M_roll(i) = M_roll(i) - qScale * oddBetaSign * tareMroll(k);
    M_yaw(i) = M_yaw(i) - qScale * oddBetaSign * tareMyaw(k);
end


% Remove non-model points before moment transfer, blockage correction, coefficients,
% and plotting. This drops the strut-only tare sweep and all weight-tare points.
dropRows = false(size(run));
dropRows(1:17) = true;
dropRows(ismember(run, wtruns)) = true;

L(dropRows) = [];
D(dropRows) = [];
Y(dropRows) = [];
M_pitch(dropRows) = [];
M_roll(dropRows) = [];
M_yaw(dropRows) = [];
Q_A(dropRows) = [];
alpha(dropRows) = [];
beta(dropRows) = [];
run(dropRows) = [];
tp(dropRows) = [];

%% UD Runs
% Upside down runs are upside down! Multiply the ud affected guys by -1
udruns = [5 6 9 10 14 15 18 19]; % from run log
for i = 1:length(run) 
    if ismember(run(i),udruns)
            L(i) = L(i)*-1;
            Y(i) = Y(i)*-1;
            M_pitch(i) = M_pitch(i)*-1;
            % M_roll is unchanged for a 180 deg roll about the sting/drag axis.
            M_yaw(i) = M_yaw(i)*-1;
            alpha(i) = alpha(i)*-1;
            beta(i) = beta(i)*-1;
    end
end
%% Moment Transfers

M_mmc = M_pitch + MMx .* L - MMz .* D;
R_mmc = M_roll + MMy .* L - MMz .* Y;
N_mmc = M_yaw + MMx .* Y - MMy .* D;

%% Blockage Corrections

% Uncorrected coefs (for Shindo's)
Cl_u = L./(area.*Q_A);
Cd_u = D./(area.*Q_A);

% Del_w approx equation from Barlow (1999) Fig 10.3
delw = 0.8 +  0.03 * (2*span/B) + 0.1 * (2*span/B)^2;

% Blockage factor
eps = area_wing./C_A .* (Cd_u - Cl_u.^2 .* ( 1/(pi*span^2/area_wing) - ...
                                                 delw * area_wing/C_A));

% Corrected dynamic pressure
Q_C = Q_A .* (1 + eps).^2;

% "Final" Coefs (no wall or upflow or anything fancy)
Cl = L ./(area.*Q_C); % lift
Cd = D ./(area.*Q_C); % drag
Cy = Y ./(area.*Q_C); % side

CM = M_mmc ./ (area.*span.*Q_C); % pitch
CR = R_mmc ./ (area.*span.*Q_C); % roll
CN = N_mmc ./ (area.*span.*Q_C); % yaw


%% Coefficient Plots by Damage Configuration
% One figure per damage configuration. Each figure combines the right-side-up
% and upside-down model data for that configuration, after the UD sign/angle
% transformation above. Tare and weight-tare rows have already been removed.
%
% Within each coefficient subplot, each beta value is plotted as a separate
% curve versus alpha.

plotCoefSweeps = true;
saveCoefPlots = false;          % Set true to export PNG files
coefPlotDir = 'coef_config_plots';

% Configuration/run map from run log. These are model wind-on runs only;
% weight tares and strut tares are already removed from the arrays above.
configMap = struct([]);
configMap(1).name = 'Nominal';
configMap(1).rightSideUpRuns = [4];
configMap(1).upsideDownRuns  = [6];

configMap(2).name = '50% Rudder';
configMap(2).rightSideUpRuns = [8];
configMap(2).upsideDownRuns  = [10];

configMap(3).name = '50% Wing';
configMap(3).rightSideUpRuns = [12 13];
configMap(3).upsideDownRuns  = [15];

configMap(4).name = '25% Wing';
configMap(4).rightSideUpRuns = [17];
configMap(4).upsideDownRuns  = [19];

if plotCoefSweeps
    coefData = {Cl, Cd, Cy, CM, CR, CN};
    coefLabels = {'C_L', 'C_D', 'C_Y', 'C_m', 'C_l', 'C_n'};
    coefTitles = {'Lift', 'Drag', 'Side Force', ...
                  'Pitch Moment', 'Roll Moment', 'Yaw Moment'};

    if saveCoefPlots && ~exist(coefPlotDir, 'dir')
        mkdir(coefPlotDir);
    end

    for gg = 1:length(configMap)
        cfg = configMap(gg);
        cfgRuns = [cfg.rightSideUpRuns cfg.upsideDownRuns];
        cfgMask = ismember(run, cfgRuns);

        if nnz(cfgMask) < 2
            warning('Skipping %s: no model data found after trimming.', cfg.name);
            continue
        end

        figName = sprintf('%s Damage Mode', cfg.name);
        figure('Name', figName, ...
               'NumberTitle', 'off', ...
               'Color', 'w');

        betaVals = unique(round(beta(cfgMask), 8));

        for cc = 1:length(coefData)
            ax = subplot(3, 2, cc);
            hold(ax, 'on');
            grid(ax, 'on');
            box(ax, 'on');

            for bb = 1:length(betaVals)
                thisBeta = betaVals(bb);
                idx = cfgMask & abs(beta - thisBeta) < 1e-8;

                if nnz(idx) < 1
                    continue
                end

                aPlot = alpha(idx);
                cPlot = coefData{cc}(idx);

                [aPlot, sortIdx] = sort(aPlot);
                cPlot = cPlot(sortIdx);

                plot(ax, aPlot, cPlot, '-o', ...
                     'LineWidth', 1.2, ...
                     'MarkerSize', 4, ...
                     'DisplayName', sprintf('\\beta = %g^\\circ', thisBeta));
            end

            xlabel(ax, '\alpha (deg)');
            ylabel(ax, coefLabels{cc});
            title(ax, coefTitles{cc});
            legend(ax, 'show', 'Location', 'best');
        end

        titleText = sprintf('%s Damage Mode: coefficients vs alpha by beta', cfg.name);
        if exist('sgtitle', 'file')
            sgtitle(titleText, 'Interpreter', 'none');
        else
            annotation('textbox', [0 0.96 1 0.04], ...
                       'String', titleText, ...
                       'EdgeColor', 'none', ...
                       'HorizontalAlignment', 'center', ...
                       'Interpreter', 'none');
        end

        if saveCoefPlots
            safeName = lower(regexprep(cfg.name, '[^A-Za-z0-9]+', '_'));
            safeName = sprintf('%s_coef_vs_alpha_by_beta.png', safeName);
            if exist('exportgraphics', 'file')
                exportgraphics(gcf, fullfile(coefPlotDir, safeName), 'Resolution', 300);
            else
                saveas(gcf, fullfile(coefPlotDir, safeName));
            end
        end
    end
end
