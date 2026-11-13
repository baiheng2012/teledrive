function out = pid_step(err, meas, kp, ki, kd, omin, omax, alpha)
% 与 C++ Pid::update 同一算法，dt 固定为 0.02
dt = 0.02;
persistent integ prev dfilt first
if isempty(integ), integ = 0; prev = 0; dfilt = 0; first = true; end

dterm = 0;
if ~first
    raw = -(meas - prev) / dt;
    dfilt = dfilt + alpha * (raw - dfilt);
    dterm = kd * dfilt;
end
prev = meas;  first = false;

p = kp * err;
cand = integ + err * dt;
unsat = p + ki * cand + dterm;
out = min(max(unsat, omin), omax);

satHigh = unsat > omax && err > 0;
satLow  = unsat < omin && err < 0;
if ~(satHigh || satLow)
    integ = cand;
end
end