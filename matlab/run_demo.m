function run_demo

matlabDir = fileparts(mfilename('fullpath'));
root = fileparts(matlabDir);

cfgPath   = fullfile(root,'config','experiment.json');
sfunDir   = fullfile(root,'cpp','simulink');
modelPath = fullfile(matlabDir,'intercept_system.slx');
figDir    = fullfile(root,'results','figures');

addpath(matlabDir);
addpath(sfunDir);

if ~isfile(cfgPath)
    error('Missing configuration file: %s', cfgPath);
end

% Build MEX on first run if needed.
mexPath = fullfile(sfunDir, ['guidance_sfunction.' mexext]);
if ~isfile(mexPath)
    fprintf('S-function MEX not found. Building it now...\n');
    oldDir = pwd;
    cleanupDir = onCleanup(@() cd(oldDir));
    cd(sfunDir);
    try
        build_sfunction;
    catch ME
        error(['Could not build the S-function.\n' ...
               'Run "mex -setup C++" once, then run run_demo again.\n\n%s'], ...
               ME.message);
    end
    clear cleanupDir
    cd(oldDir);
end

% Build Simulink model on first run if needed.
if ~isfile(modelPath)
    fprintf('Simulink model not found. Building it now...\n');
    build_models;
end

if ~exist(figDir,'dir')
    mkdir(figDir);
end

% Preserve the original JSON exactly.
originalText = fileread(cfgPath);
restoreConfig = onCleanup(@() writeText(cfgPath, originalText));

originalCfg = jsondecode(originalText);
hitRadius = originalCfg.simulation.intercept_radius_m;

% Predictive run.
fprintf('\nRunning predictive guidance...\n');
writeMode(cfgPath, originalText, 'predictive');
outPred = run_simulink;

% PN run.
fprintf('Running PN guidance...\n');
writeMode(cfgPath, originalText, 'pn');
outPN = run_simulink;

% Restore original config before any further processing.
writeText(cfgPath, originalText);

% Extract state histories.
IP = stateMatrix(outPred.interceptor_state);
TP = stateMatrix(outPred.target_state);
IN = stateMatrix(outPN.interceptor_state);
TN = stateMatrix(outPN.target_state);

tP = outPred.interceptor_state.time(:);
tN = outPN.interceptor_state.time(:);

RP = hypot(TP(:,1)-IP(:,1), TP(:,2)-IP(:,2));
RN = hypot(TN(:,1)-IN(:,1), TN(:,2)-IN(:,2));

[dPred, kPred] = min(RP);
[dPN,   kPN]   = min(RN);

tPred = tP(kPred);
tPN   = tN(kPN);

predResult = "MISS";
if dPred <= hitRadius
    predResult = "HIT";
end

pnResult = "MISS";
if dPN <= hitRadius
    pnResult = "HIT";
end

fprintf('\n============================================\n');
fprintf('Simulink guidance comparison\n');
fprintf('Hit radius: %.3f m\n', hitRadius);
fprintf('Predictive: %.3f m at %.3f s  [%s]\n', dPred, tPred, predResult);
fprintf('PN:         %.3f m at %.3f s  [%s]\n', dPN, tPN, pnResult);
fprintf('============================================\n\n');

plotEnd = max(tPred, tPN) + 1.0;

% Figure 1: trajectories.
f1 = figure('Name','PN vs Predictive Trajectories');
iP = tP <= plotEnd;
iN = tN <= plotEnd;

plot(IP(iP,1), IP(iP,2), 'LineWidth', 1.5);
hold on;
plot(IN(iN,1), IN(iN,2), 'LineWidth', 1.5);
plot(TP(iP,1), TP(iP,2), 'LineWidth', 1.5);
plot(IP(kPred,1), IP(kPred,2), 'o', 'MarkerSize', 7);
plot(IN(kPN,1), IN(kPN,2), 'o', 'MarkerSize', 7);

axis equal;
grid on;
legend('Predictive','PN','Target','Predictive CA','PN CA','Location','best');
xlabel('x (m)');
ylabel('y (m)');
title('PN vs Predictive Engagement Trajectories');

saveas(f1, fullfile(figDir,'simulink_trajectories.png'));

% Figure 2: range.
f2 = figure('Name','PN vs Predictive Range');

plot(tP(iP), RP(iP), 'LineWidth', 1.5);
hold on;
plot(tN(iN), RN(iN), 'LineWidth', 1.5);
yline(hitRadius, '--', 'Hit radius');
plot(tPred, dPred, 'o', 'MarkerSize', 7);
plot(tPN,   dPN,   'o', 'MarkerSize', 7);

grid on;
xlabel('Time (s)');
ylabel('Range (m)');
title('PN vs Predictive Range');
legend('Predictive','PN','Hit radius','Predictive minimum','PN minimum','Location','best');

saveas(f2, fullfile(figDir,'simulink_range.png'));

% Figure 3: commanded acceleration.
uPred = scalarLog(outPred.guidance_command);
uPN   = scalarLog(outPN.guidance_command);

tcP = outPred.guidance_command.time(:);
tcN = outPN.guidance_command.time(:);

jP = tcP <= plotEnd;
jN = tcN <= plotEnd;

f3 = figure('Name','PN vs Predictive Commands');

plot(tcP(jP), uPred(jP), 'LineWidth', 1.5);
hold on;
plot(tcN(jN), uPN(jN), 'LineWidth', 1.5);

grid on;
xlabel('Time (s)');
ylabel('Commanded lateral acceleration (m/s^2)');
title('PN vs Predictive Guidance Commands');
legend('Predictive','PN','Location','best');

saveas(f3, fullfile(figDir,'simulink_commands.png'));

% Figure 4: actual actuator acceleration.
aPred = scalarLog(outPred.actual_accel);
aPN   = scalarLog(outPN.actual_accel);

taP = outPred.actual_accel.time(:);
taN = outPN.actual_accel.time(:);

qP = taP <= plotEnd;
qN = taN <= plotEnd;

f4 = figure('Name','PN vs Predictive Actual Acceleration');

plot(taP(qP), aPred(qP), 'LineWidth', 1.5);
hold on;
plot(taN(qN), aPN(qN), 'LineWidth', 1.5);

grid on;
xlabel('Time (s)');
ylabel('Actual lateral acceleration (m/s^2)');
title('PN vs Predictive Actuator Response');
legend('Predictive','PN','Location','best');

saveas(f4, fullfile(figDir,'simulink_actual_accel.png'));

fprintf('Saved figures:\n');
fprintf('  %s\n', fullfile(figDir,'simulink_trajectories.png'));
fprintf('  %s\n', fullfile(figDir,'simulink_range.png'));
fprintf('  %s\n', fullfile(figDir,'simulink_commands.png'));
fprintf('  %s\n', fullfile(figDir,'simulink_actual_accel.png'));

% Keep outputs available for inspection.
assignin('base','outPred',outPred);
assignin('base','outPN',outPN);
assignin('base','dPred',dPred);
assignin('base','dPN',dPN);

clear restoreConfig
end

function writeMode(cfgPath, originalText, mode)
cfg = jsondecode(originalText);
cfg.guidance.mode = mode;
writeText(cfgPath, jsonencode(cfg));
end

function writeText(path, text)
fid = fopen(path,'w');
if fid < 0
    error('Could not write %s', path);
end
cleanup = onCleanup(@() fclose(fid));
fwrite(fid, text, 'char');
end

function X = stateMatrix(logData)
X = squeeze(logData.signals.values);
n = numel(logData.time);

if size(X,1) ~= n && size(X,2) == n
    X = X.';
end

if size(X,1) ~= n
    error('Unexpected state log dimensions.');
end
end

function v = scalarLog(logData)
v = squeeze(logData.signals.values);
v = v(:);
end
