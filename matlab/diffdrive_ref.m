function X = diffdrive_ref(v_cmd, w_cmd, dt)
% DIFFDRIVE_REF  差速驱动模型，与 C++ DiffDriveModel 同一算法（加速度限制 + 中点法积分）。
%   输入：指令序列（列向量，长度 n），步长 dt
%   输出：n x 5 矩阵，列为 [x y heading v w]；第 i 行是"施加第 i 条指令之前"的状态，
%         与 C++ 的 CSV 约定一致（最后一条指令不施加，只记录最终状态）。

% ---- 与 C++ VehicleParams 默认值一致的常量 ----
max_v = 2.0;   max_w = 4.0;          % 物理速度上限
acc_v = 1.0;   dec_v = 2.0;          % 线加速度 / 减速度 (m/s^2)
acc_w = 3.0;   dec_w = 6.0;          % 角加速度 / 减速度 (rad/s^2)

n = numel(v_cmd);
X = zeros(n, 5);
s = zeros(1, 5);                     % [x y th v w]

for i = 1:n
    X(i, :) = s;
    if i == n, break; end            % 最后一行只记录状态

    % 1) 目标速度限制在物理上限内
    vt = min(max(v_cmd(i), -max_v), max_v);
    wt = min(max(w_cmd(i), -max_w), max_w);

    % 2) 速度向目标靠近，受加速度限制
    s(4) = slew(s(4), vt, acc_v, dec_v, dt);
    s(5) = slew(s(5), wt, acc_w, dec_w, dt);

    % 3) 中点法积分：用步内朝向的中间值推进位置
    mid  = s(3) + 0.5 * s(5) * dt;
    s(1) = s(1) + s(4) * cos(mid) * dt;
    s(2) = s(2) + s(4) * sin(mid) * dt;
    s(3) = wrap(s(3) + s(5) * dt);
end
end

function c = slew(c, target, acc, dec, dt)
% 变化率限制：速度绝对值变小或反向时用减速度，否则用加速度（与 C++ 相同的判断）
reducing = (target * c < 0) || (abs(target) < abs(c));
if reducing, m = dec * dt; else, m = acc * dt; end
c = c + min(max(target - c, -m), m);
end

function a = wrap(a)
% 把角度规范到 [-pi, pi)
a = a - 2*pi * floor((a + pi) / (2*pi));
end
