function build_models
% Build one compact closed-loop Simulink model from experiment.json.
root = fileparts(fileparts(mfilename('fullpath')));
cfgPath = fullfile(root,'config','experiment.json');
cfg = jsondecode(fileread(cfgPath));
addpath(fullfile(root,'cpp','simulink'));

m = 'intercept_system';
if bdIsLoaded(m), close_system(m,0); end
new_system(m); open_system(m);

VI = cfg.interceptor.speed_mps; VT = cfg.target.speed_mps;
dtS = 1/cfg.seeker.update_rate_hz;
ref = cfg.simulink_reference;
lat = ref.latency_s;
xI0 = cfg.interceptor.initial_position_m;
xT0 = [ref.target_initial_downrange_m, ref.target_initial_lateral_m];
psiT0 = cfg.target.initial_heading_center_rad + ref.target_initial_heading_offset_rad;
aT = ref.target_lateral_accel_mps2;
rng('shuffle');
rangeSeed = seedValue(ref.range_noise_seed); bearingSeed = seedValue(ref.bearing_noise_seed); dropoutSeed = seedValue(ref.dropout_seed);

% Interceptor truth.
matlabFunctionBlock([m '/InterceptorDynamics'],[590 180 760 260], ...
    sprintf('function dx=f(a,x)\nV=%.17g; dx=[V*cos(x(3));V*sin(x(3));a/max(V,1e-9)];\nend',VI));
add_block('simulink/Continuous/Integrator',[m '/InterceptorState'],'Position',[800 190 845 250], ...
    'InitialCondition',sprintf('[%.17g;%.17g;%.17g]',xI0(1),xI0(2),cfg.interceptor.initial_heading_rad));
add_line(m,'InterceptorDynamics/1','InterceptorState/1'); add_line(m,'InterceptorState/1','InterceptorDynamics/2');

% Target truth.
matlabFunctionBlock([m '/TargetDynamics'],[590 370 760 450], ...
    sprintf('function dx=f(x)\nV=%.17g; a=%.17g; dx=[V*cos(x(3));V*sin(x(3));a/max(V,1e-9)];\nend',VT,aT));
add_block('simulink/Continuous/Integrator',[m '/TargetState'],'Position',[800 380 845 440], ...
    'InitialCondition',sprintf('[%.17g;%.17g;%.17g]',xT0(1),xT0(2),psiT0));
add_line(m,'TargetDynamics/1','TargetState/1'); add_line(m,'TargetState/1','TargetDynamics/1');

% Geometry and seeker sampling.
matlabFunctionBlock([m '/Geometry'],[900 245 1070 340], ...
    sprintf(['function y=f(xI,xT)\n' ...
    'r=xT(1:2)-xI(1:2); y=[hypot(r(1),r(2));atan2(r(2),r(1));xI(3)];\nend']));
add_line(m,'InterceptorState/1','Geometry/1'); add_line(m,'TargetState/1','Geometry/2');
add_block('simulink/Signal Routing/Demux',[m '/GeometrySplit'],'Position',[1110 245 1115 340],'Outputs','3');
add_line(m,'Geometry/1','GeometrySplit/1');
add_block('simulink/Sources/Clock',[m '/Clock'],'Position',[900 80 930 100]);

names = {'RangeSample','BearingSample','TimeSample'}; sources = {'GeometrySplit/1','GeometrySplit/2','Clock/1'};
for k=1:3
    add_block('simulink/Discrete/Zero-Order Hold',[m '/' names{k}],'Position',[1170 80+75*k 1230 110+75*k], ...
        'SampleTime',num2str(dtS,17));
    add_line(m,sources{k},[names{k} '/1']);
end

% Sampled sensor noise.
add_block('simulink/Sources/Random Number',[m '/RangeNoise'],'Position',[1170 390 1230 420], ...
    'Mean','0','Variance',num2str(cfg.seeker.range_sigma_m^2,17),'SampleTime',num2str(dtS,17),'Seed',num2str(rangeSeed));
add_block('simulink/Sources/Random Number',[m '/BearingNoise'],'Position',[1170 445 1230 475], ...
    'Mean','0','Variance',num2str(cfg.seeker.bearing_sigma_rad^2,17),'SampleTime',num2str(dtS,17),'Seed',num2str(bearingSeed));
add_block('simulink/Math Operations/Add',[m '/RangeAdd'],'Position',[1280 175 1310 205]);
add_block('simulink/Math Operations/Add',[m '/BearingAdd'],'Position',[1280 250 1310 280]);
add_line(m,'RangeSample/1','RangeAdd/1'); add_line(m,'RangeNoise/1','RangeAdd/2');
add_line(m,'BearingSample/1','BearingAdd/1'); add_line(m,'BearingNoise/1','BearingAdd/2');

% Dropout and fixed nominal latency for the Simulink reference run.
add_block('simulink/Sources/Uniform Random Number',[m '/DropoutDraw'],'Position',[1280 390 1350 420], ...
    'Minimum','0','Maximum','1','SampleTime',num2str(dtS,17),'Seed',num2str(dropoutSeed));
signals = {'TimeSample/1','RangeAdd/1','BearingAdd/1','DropoutDraw/1'};
delays = {'TimeDelay','RangeDelay','BearingDelay','DropoutDelay'};
ypos = [95 180 255 390];
for k=1:4
    add_block('simulink/Continuous/Transport Delay',[m '/' delays{k}],'Position',[1510 ypos(k) 1580 ypos(k)+30], ...
        'DelayTime',num2str(lat,17));
    add_line(m,signals{k},[delays{k} '/1']);
end
add_block('simulink/Logic and Bit Operations/Compare To Constant',[m '/Valid'],'Position',[1600 385 1670 425], ...
    'relop','>=','const',num2str(cfg.seeker.dropout_probability,17));
add_block('simulink/Signal Attributes/Data Type Conversion',[m '/ValidDouble'],'Position',[1700 385 1770 425], ...
    'OutDataTypeStr','double');
add_line(m,'DropoutDelay/1','Valid/1'); add_line(m,'Valid/1','ValidDouble/1');

% Current ownship navigation data + delayed seeker packet.
add_block('simulink/Sources/Constant',[m '/InterceptorSpeed'],'Position',[1510 470 1580 500],'Value',num2str(VI,17));
add_block('simulink/Signal Routing/Mux',[m '/GuidanceInput'],'Position',[1810 145 1815 455],'Inputs','7');
add_line(m,'TimeDelay/1','GuidanceInput/1'); add_line(m,'RangeDelay/1','GuidanceInput/2'); add_line(m,'BearingDelay/1','GuidanceInput/3');
add_line(m,'InterceptorSpeed/1','GuidanceInput/4'); add_line(m,'GeometrySplit/3','GuidanceInput/5');

% Actuator and C++ guidance close the loop.
add_block('simulink/User-Defined Functions/S-Function',[m '/Guidance'],'Position',[1870 230 1990 290]);
set_param([m '/Guidance'],'FunctionName','guidance_sfunction','Parameters',quotePath(cfgPath));
add_block('simulink/Signal Routing/Demux',[m '/GuidanceSplit'],'Position',[2030 225 2035 295],'Outputs','2');
add_line(m,'GuidanceInput/1','Guidance/1'); add_line(m,'Guidance/1','GuidanceSplit/1');
add_block('simulink/Discontinuities/Saturation',[m '/AccelLimit'],'Position',[2080 190 2150 230], ...
    'UpperLimit',num2str(cfg.interceptor.max_lateral_accel_mps2,17),'LowerLimit',num2str(-cfg.interceptor.max_lateral_accel_mps2,17));
add_block('simulink/Continuous/Transfer Fcn',[m '/ActuatorLag'],'Position',[2200 185 2290 235], ...
    'Numerator','1','Denominator',sprintf('[%.17g 1]',cfg.interceptor.actuator_tau_s));
add_line(m,'GuidanceSplit/1','AccelLimit/1'); add_line(m,'AccelLimit/1','ActuatorLag/1');
add_line(m,'ActuatorLag/1','InterceptorDynamics/1'); add_line(m,'ActuatorLag/1','GuidanceInput/6'); add_line(m,'ValidDouble/1','GuidanceInput/7');

% Logs for validation/plots.
logBlock(m,'InterceptorLog','InterceptorState/1',[900 520 1000 550],'interceptor_state');
logBlock(m,'TargetLog','TargetState/1',[900 570 1000 600],'target_state');
logBlock(m,'CommandLog','GuidanceSplit/1',[2080 330 2180 360],'guidance_command');
logBlock(m,'AccelLog','ActuatorLag/1',[2200 330 2300 360],'actual_accel');

set_param(m,'StopTime',num2str(cfg.simulation.max_time_s,17),'Solver','ode4','SolverType','Fixed-step', ...
    'FixedStep',num2str(cfg.simulation.truth_dt_s,17));
save_system(m,fullfile(root,'matlab',[m '.slx']));
fprintf('Created matlab/%s.slx from config/experiment.json\n',m);
end


function matlabFunctionBlock(path,pos,script)
% MATLAB Function blocks store code in a Stateflow EMChart object.
add_block('simulink/User-Defined Functions/MATLAB Function',path,'Position',pos);
chart = sfroot().find('-isa','Stateflow.EMChart','Path',path);
if isempty(chart), error('Could not configure MATLAB Function block: %s',path); end
chart(1).Script = script;
end

function seed=seedValue(value)
if ischar(value) || isstring(value)
    if string(value) ~= "random", error('Seed must be numeric or random'); end
    seed = randi(2^31-1);
else
    seed = value;
end
end

function q=quotePath(path)
q=['''' strrep(path,'''','''''') ''''];
end

function logBlock(m,name,src,pos,var)
add_block('simulink/Sinks/To Workspace',[m '/' name],'Position',pos,'VariableName',var,'SaveFormat','Structure With Time');
add_line(m,src,[name '/1']);
end
