# TeleDrive

A C++17 tele-operation framework with a safety state machine, heartbeat/command watchdog,
and a simulated differential-drive vehicle. Built as a foundation for operator-assist
and semi-autonomous driving features.

## Demo

![TeleDrive demo](docs/demo.gif)

*The operator console (top) drives the simulated vehicle (bottom). E-stop latches a fault; suspending the
operator process triggers the watchdog and the vehicle brakes automatically.*

## Semi-autonomous mode: pure pursuit

Press `m` after arming to let the vehicle follow a preset path. The path follower runs
*behind* the safety layer: arm switch, command watchdog, E-stop and lidar speed limiting
all still apply, and any manual driving key immediately takes control back.

![Auto mode demo](docs/demo2.gif)

*Live demo: `e` then `m` starts the run; pressing a driving key hands control back at once.*

Offline simulation of the same controller on a square path. The vehicle cuts slightly
inside each corner, which is inherent to pure pursuit (a larger lookahead cuts more,
a smaller one tracks tighter but oscillates more):

![Pure pursuit](docs/pure_pursuit.png)


## Operator assist: heading hold

With a constant yaw disturbance (simulating a slipping wheel), the vehicle drifts into a
circle without assistance; the PID heading hold keeps it on a straight line.

![Heading hold](docs/heading_hold.png)


## Features

- Binary UDP protocol with versioning, sequence numbers and CRC-16
- Transport and clock abstractions (`ITransport`, `IClock`) for deterministic testing
- Safety state machine: Idle / Armed / Driving / Fault, latched faults, E-stop priority
- Command watchdog (200 ms), command limiting, NaN/Inf rejection
- Differential-drive simulation with acceleration limits and battery model
- Replay/duplicate rejection with sequence wrap-around handling
- CI (GitHub Actions) and Docker build, 60+ unit and end-to-end tests

## Build and run

```bash
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure

# terminal 1
./build/apps/vehicle_sim
# terminal 2
./build/apps/operator_cli
```

Keys: `e` arm/disarm, `w`/`s` speed, `a`/`d` turn, `space` zero, `x` E-stop, `r` reset, `q` quit.

## Known limitations

- No authentication or encryption on the link
- Keyboard input cannot detect key release, so an arm toggle replaces a true deadman switch
- Status line assumes a terminal at least 80 columns wide
