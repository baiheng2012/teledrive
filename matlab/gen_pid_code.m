% gen_pid_code.m：用 Embedded Coder 从 pid_ctrl.slx 生成 C++ 代码
mdl = 'pid_ctrl';
load_system(mdl);

% 目标：ERT（Embedded Real-Time，即 Embedded Coder）+ C++ 语言
set_param(mdl, 'SystemTargetFile', 'ert.tlc');
set_param(mdl, 'TargetLang', 'C++');
set_param(mdl, 'GenCodeOnly', 'on');           % 只生成代码，不编译
% 选一个工具链，让 MATLAB 不再用只支持 C 的 LCC 做兼容性检查。
% 只生成代码时，并不会真的调用它来编译。
set_param(mdl, 'Toolchain', 'Automatically locate an installed toolchain');
set_param(mdl, 'GenCodeOnly', 'on');   % 再设一次，确保生效
set_param(mdl, 'GenerateReport', 'on');        % 生成 HTML 报告，便于检查
set_param(mdl, 'SupportNonFinite', 'on');      % 允许 NaN/Inf 相关代码
set_param(mdl, 'GenerateSampleERTMain', 'off');% 不生成示例 main，我们自己集成

% 下面两项的取值依版本而异：先查可选值，避免报错
% 想要生成一个 C++ 类，优先 C++ class 接口；不支持则退回 Reusable function
try
    set_param(mdl, 'CodeInterfacePackaging', 'C++ class');
catch
    warning('此版本不支持 C++ class 接口，改用 Reusable function');
    set_param(mdl, 'CodeInterfacePackaging', 'Reusable function');
end

slbuild(mdl);
fprintf('代码已生成到 %s_ert_rtw\n', mdl);
