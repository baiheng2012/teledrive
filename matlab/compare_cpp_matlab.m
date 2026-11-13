% 对比 C++ 仿真与 MATLAB 参考实现。运行前先在 WSL 里生成 docs/trace_open.csv。
T = readtable(fullfile('..', 'docs', 'trace_open.csv'));
dt = T.t(2) - T.t(1);

X = diffdrive_ref(T.v_cmd, T.w_cmd, dt);

% 角度差必须取最短弧：179 度与 -179 度只差 2 度
angdiff = @(a, b) atan2(sin(a - b), cos(a - b));

pos_err = hypot(X(:,1) - T.x, X(:,2) - T.y);
hdg_err = abs(angdiff(X(:,3), T.heading));
v_err   = abs(X(:,4) - T.v);

fprintf('max position error : %.3e m\n',   max(pos_err));
fprintf('max heading error  : %.3e rad\n', max(hdg_err));
fprintf('max speed error    : %.3e m/s\n', max(v_err));
assert(max(pos_err) < 1e-6 && max(hdg_err) < 1e-6, '与 C++ 不一致');

% 图：轨迹重合 + 误差
fig = figure('Position', [100 100 1000 420]);
subplot(1,2,1);
plot(T.x, T.y, 'b-', 'LineWidth', 3); hold on;
plot(X(:,1), X(:,2), 'r--', 'LineWidth', 1.2);
axis equal; grid on; xlabel('x (m)'); ylabel('y (m)');
legend('C++', 'MATLAB'); title('Trajectory');
subplot(1,2,2);
semilogy(T.t, max(pos_err, 1e-16)); grid on;
xlabel('time (s)'); ylabel('position error (m)'); title('C++ vs MATLAB');
exportgraphics(fig, fullfile('..', 'docs', 'matlab_vs_cpp.png'), 'Resolution', 150);
