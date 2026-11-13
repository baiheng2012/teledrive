function build_pid_model()
% 搭建 pid_ctrl.slx：8 个输入 -> MATLAB Function 块 -> 1 个输出
mdl = 'pid_ctrl';
if bdIsLoaded(mdl), close_system(mdl, 0); end
if exist([mdl '.slx'], 'file'), delete([mdl '.slx']); end
new_system(mdl);
open_system(mdl);

% ---- MATLAB Function 块 ----
blk = [mdl '/PID'];
add_block('simulink/User-Defined Functions/MATLAB Function', blk);

% 用 Stateflow API 把 pid_step.m 的内容写进块里
rt = sfroot;
chart = rt.find('-isa', 'Stateflow.EMChart', 'Path', blk);
chart.Script = fileread('pid_step.m');

% ---- 8 个输入端口，名字与函数参数一致 ----
names = {'err','meas','kp','ki','kd','omin','omax','alpha'};
for i = 1:numel(names)
    add_block('simulink/Sources/In1', [mdl '/' names{i}], 'Port', num2str(i));
end
add_block('simulink/Sinks/Out1', [mdl '/out']);

% ---- 连线 ----
for i = 1:numel(names)
    add_line(mdl, [names{i} '/1'], ['PID/' num2str(i)]);
end
add_line(mdl, 'PID/1', 'out/1');

% ---- 求解器：离散定步长 0.02 s；全部 double ----
set_param(mdl, 'SolverType', 'Fixed-step', 'Solver', 'FixedStepDiscrete', ...
    'FixedStep', '0.02', 'StopTime', '10');

try, Simulink.BlockDiagram.arrangeSystem(mdl); catch, end
save_system(mdl, fullfile(pwd, [mdl '.slx']));
fprintf('saved %s.slx\n', mdl);
end
