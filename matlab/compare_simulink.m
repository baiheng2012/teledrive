% 对比 C++ 与 Simulink。先运行 build_diffdrive_model 生成模型。
T = readtable(fullfile('..', 'docs', 'trace_open.csv'));

% From Workspace 要求的格式：[时间 值] 两列矩阵（必须在 base 工作区）
%v_in = [T.t, T.v_cmd];
%w_in = [T.t, T.w_cmd];
%assignin('base', 'v_in', v_in);
%assignin('base', 'w_in', w_in);

tt = (0:0.001:10)';
v_cmd = interp1(T.t, T.v_cmd, tt, 'previous', 'extrap');
w_cmd = interp1(T.t, T.w_cmd, tt, 'previous', 'extrap');
v_cmd(1) = 0;  w_cmd(1) = 0;     % t=0 输入为 0，Rate Limiter 首个输出即为 0
assignin('base', 'v_in', [tt, v_cmd]);
assignin('base', 'w_in', [tt, w_cmd]);


out = sim('diffdrive_sim');                 % 返回 out.tout 与 out.y_out
Y = interp1(out.tout, out.y_out, T.t);      % 取到 CSV 的时间点上：[x y th v w]

angdiff = @(a, b) atan2(sin(a - b), cos(a - b));
pos_err = hypot(Y(:,1) - T.x, Y(:,2) - T.y);
hdg_err = abs(angdiff(Y(:,3), T.heading));

fprintf('max position error : %.4f m\n',   max(pos_err));
fprintf('final position err : %.4f m\n',   pos_err(end));
fprintf('max heading error  : %.4f rad\n', max(hdg_err));

fig = figure('Position', [100 100 1000 420]);
subplot(1,2,1);
plot(T.x, T.y, 'b-', 'LineWidth', 3); hold on;
plot(Y(:,1), Y(:,2), 'r--', 'LineWidth', 1.2);
axis equal; grid on; xlabel('x (m)'); ylabel('y (m)');
legend('C++ (discrete, 20 ms)', 'Simulink (ode4, 1 ms)'); title('Trajectory');
subplot(1,2,2);
plot(T.t, pos_err * 100); grid on;
xlabel('time (s)'); ylabel('position error (cm)'); title('C++ vs Simulink');
exportgraphics(fig, fullfile('..', 'docs', 'simulink_vs_cpp.png'), 'Resolution', 150);
