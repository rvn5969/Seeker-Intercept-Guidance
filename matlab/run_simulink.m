function out = run_simulink
% Build if needed, then run the configured closed-loop model.
root = fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root,'cpp','simulink'));
model = fullfile(root,'matlab','intercept_system.slx');
if ~isfile(fullfile(root,'cpp','simulink',['guidance_sfunction.' mexext]))
    error('Build the C++ block first: cd cpp/simulink; build_sfunction');
end
if ~isfile(model), build_models; end
load_system(model);
out = sim('intercept_system','ReturnWorkspaceOutputs','on');
fprintf('Simulation complete. Logs: interceptor_state, target_state, guidance_command, actual_accel.\n');
end
