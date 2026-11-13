function build_diffdrive_model()
% 搭建差速驱动 Simulink 模型 diffdrive_sim.slx。
% 结构：指令 -> Rate Limiter（加速度限制）-> 积分得到位姿
%   x' = v*cos(th),  y' = v*sin(th),  th' = w
%mdl = 'diffdrive_sim';
%if bdIsLoaded(mdl), close_system(mdl, 0); end
%new_system(mdl);
mdl = 'diffdrive_sim';
if bdIsLoaded(mdl), close_system(mdl, 0); end
% 删掉上次生成的文件，避免 new_system 发现同名文件而发出 shadowing 警告
if exist([mdl '.slx'], 'file'), delete([mdl '.slx']); end
new_system(mdl);


% ---- 输入：从工作区读取 [t value] 矩阵，不插值（零阶保持），与 C++ 的"每步常值指令"一致 ----
add_block('simulink/Sources/From Workspace', [mdl '/v_cmd'], ...
    'VariableName', 'v_in', 'Interpolate', 'off', 'OutputAfterFinalValue', 'Holding final value');
add_block('simulink/Sources/From Workspace', [mdl '/w_cmd'], ...
    'VariableName', 'w_in', 'Interpolate', 'off', 'OutputAfterFinalValue', 'Holding final value');

% ---- 加速度限制：上升 = 加速，下降 = 减速（输入非负时与 C++ 的判断等价）----
%add_block('simulink/Discontinuities/Rate Limiter', [mdl '/RL_v'], ...
%    'RisingSlewLimit', '1.0', 'FallingSlewLimit', '-2.0');
%add_block('simulink/Discontinuities/Rate Limiter', [mdl '/RL_w'], ...
%    'RisingSlewLimit', '3.0', 'FallingSlewLimit', '-6.0');

add_block('simulink/Discontinuities/Rate Limiter', [mdl '/RL_v'], ...
    'RisingSlewLimit', '1.0', 'FallingSlewLimit', '-2.0', ...
    'InitialCondition', '0');
add_block('simulink/Discontinuities/Rate Limiter', [mdl '/RL_w'], ...
    'RisingSlewLimit', '3.0', 'FallingSlewLimit', '-6.0', ...
    'InitialCondition', '0');



% ---- 运动学 ----
add_block('simulink/Continuous/Integrator', [mdl '/int_th']);
add_block('simulink/Math Operations/Trigonometric Function', [mdl '/cosT'], 'Operator', 'cos');
add_block('simulink/Math Operations/Trigonometric Function', [mdl '/sinT'], 'Operator', 'sin');
add_block('simulink/Math Operations/Product', [mdl '/prod_x']);
add_block('simulink/Math Operations/Product', [mdl '/prod_y']);
add_block('simulink/Continuous/Integrator', [mdl '/int_x']);
add_block('simulink/Continuous/Integrator', [mdl '/int_y']);

% ---- 输出：5 路信号合并后写入工作区，列顺序 [x y th v w] ----
add_block('simulink/Signal Routing/Mux', [mdl '/mux'], 'Inputs', '5');
add_block('simulink/Sinks/To Workspace', [mdl '/out'], ...
    'VariableName', 'y_out', 'SaveFormat', 'Array');

% ---- 连线 ----
L = @(a, b) add_line(mdl, a, b, 'autorouting', 'on');
L('v_cmd/1', 'RL_v/1');   L('w_cmd/1', 'RL_w/1');
L('RL_w/1',  'int_th/1');
L('int_th/1','cosT/1');   L('int_th/1','sinT/1');
L('RL_v/1',  'prod_x/1'); L('cosT/1', 'prod_x/2');
L('RL_v/1',  'prod_y/1'); L('sinT/1', 'prod_y/2');
L('prod_x/1','int_x/1');  L('prod_y/1','int_y/1');
L('int_x/1', 'mux/1');    L('int_y/1', 'mux/2');   L('int_th/1','mux/3');
L('RL_v/1',  'mux/4');    L('RL_w/1',  'mux/5');
L('mux/1',   'out/1');

% ---- 求解器：定步长 ode4，步长 1 ms，仿真 10 s ----
set_param(mdl, 'SolverType', 'Fixed-step', 'Solver', 'ode4', ...
    'FixedStep', '0.001', 'StopTime', '10');

try, Simulink.BlockDiagram.arrangeSystem(mdl); catch, end   % 自动整理布局（旧版本没有则跳过）
save_system(mdl, fullfile(pwd, [mdl '.slx']));
fprintf('saved %s.slx\n', mdl);
end
