# TeleDrive
[![CI](https://github.com/baiheng2012/teledrive/actions/workflows/ci.yml/badge.svg)](https://github.com/baiheng2012/teledrive/actions/workflows/ci.yml)
C++17 · CMake · GoogleTest · OpenCV · SocketCAN · MATLAB/Simulink · Docker · GitHub Actions

A C++17 tele-operation framework with a safety state machine, command watchdog, and a
simulated differential-drive vehicle. It is built as a foundation for operator-assist and
semi-autonomous driving features.

The design is motivated by remotely operated mining vehicles, where an operator controls
a machine from outside the hazardous area (line-of-sight, tele-remote and tele-operation
modes) and a lost link, a dropped command or a bad input must never leave the machine
moving. TeleDrive models that problem on a simulated vehicle: latched faults, E-stop
priority, a 200 ms command watchdog, obstacle-aware speed limiting and a path-following
mode that all sit behind one safety layer, over UDP or CAN, with live onboard video.

This is a software simulation. It has not been run on real vehicle hardware, and the
vehicle parameters are illustrative rather than calibrated to any real machine.

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

## Vision: simulated onboard camera

The vehicle renders its surroundings (path, obstacles, lidar returns, HUD) with OpenCV,
compresses each frame to JPEG and streams it to the operator over a *separate* UDP
channel (control port + 1), fragmented into MTU-sized packets. The operator console
shows the live picture in a window together with frame rate, jitter and an estimated
latency.

![Vision demo](docs/vision_demo.png)

Design decisions:

- **Control plane and data plane are separate.** Video uses its own socket and its own
  protocol (different magic number), so video congestion cannot delay the 50 Hz command
  stream or the watchdog. A test shows the vehicle behaves bit-identically with video on,
  off, or with every video packet lost.
- **Newest frame wins.** A frame is split into up to 255 fragments (≤ 1200 bytes each,
  CRC-protected). The reassembler works on one frame at a time and drops an incomplete
  frame as soon as a newer one starts; there are no retransmissions, because a late
  frame is worth less than a lost one.
- **Untrusted input.** Fragment counts, indices, lengths and CRCs are validated;
  fragments of one frame that contradict each other are rejected; the receiver only
  accepts datagrams from the vehicle address.
- **Hello handshake.** The operator sends a small hello datagram once per second so the
  vehicle learns where to send video, and recovers automatically if the vehicle restarts.
- **Latency without clock sync.** The two machines' clocks are not synchronized, so the
  capture timestamp is never subtracted from local time. Jitter is the mean of
  |arrival interval − capture interval|; estimated latency ≈ RTT/2 + jitter.

Measured on the development machine: rendering about 0.4 ms, JPEG encoding about 3.4 ms
(max 3.3 ms in a 40 s run), roughly 20 KB per frame (26x compression), 17 packets per
frame, about 190 KB/s at 10 fps. Rendering and encoding run synchronously inside the
20 ms control cycle.

OpenCV is an optional dependency: without it the vision module is skipped and the rest
of the project builds normally. CI and Docker require it so the vision tests cannot be
skipped silently.

Limitations: the hello handshake is unauthenticated, latency is an estimate rather than
a measurement, and the image is a synthetic top-down render, not a real camera.

## Cross-validation with MATLAB/Simulink

The C++ vehicle model was validated against two independent references on the same
open-loop scenario (commands exported from C++ to CSV, 10 s, 20 ms steps).

| Reference | Method | Max position error | Max heading error |
|---|---|---|---|
| MATLAB re-implementation | same discrete algorithm | 7e-10 m | 6e-10 rad |
| Simulink model | continuous integration (ode4, 1 ms) | 1.0 cm | 0.4° |

The MATLAB script agrees to print precision, which rules out implementation errors.
The Simulink residual comes from the integration scheme: the C++ model integrates
position with the end-of-step velocity, adding about 0.5·a·dt² per step during
acceleration (about 1 cm over the 1 s ramp). The Simulink inputs are delayed by 1 ms
to give the rate limiters a zero initial output, which also contributes to the residual.

![C++ vs Simulink](docs/simulink_vs_cpp.png)

## Model-based design: Simulink-generated PID

The PID core was designed in Simulink (`matlab/pid_ctrl.slx`, built by
`matlab/build_pid_model.m`), and C++ was generated with Embedded Coder
(`matlab/gen_pid_code.m`). The generated class lives in `libs/control/generated/`,
so building the project does not require MATLAB.

`HeadingHold` depends on an `IPidCore` interface with two implementations: the
hand-written `Pid` and the generated `GeneratedPid` (a thin hand-written adapter).
Verification:

- Randomized equivalence tests: both implementations agree to ~1e-9 on 5000-step
  random sequences, including heavy saturation (anti-windup)
- The closed-loop disturbance-rejection test runs against both cores and gives the
  same result
- Adapter behaviour (reset, invalid inputs) tested separately: a `reset()` bug in the
  adapter was caught this way, so generated code does not remove the need for tests

Limitations: the model fixes dt = 0.02 s (the 50 Hz control period), so the generated
core is only equivalent to the hand-written one at that rate; input validation (NaN,
bad dt) lives in the hand-written adapter, outside the model.

## CAN transport

`CanTransport` implements the same `ITransport` interface over Linux SocketCAN, so
`VehicleNode` and `OperatorSession` run on a CAN bus without modification (tested on the
virtual interface `vcan0`). Protocol frames (14 to 28 bytes) are split into classic CAN
frames with a 2-byte fragment header. The 11-bit ID encodes
`[priority:3][destination:4][source:4]`, so hardware arbitration favours E-stop frames
over commands, heartbeats and telemetry. Reassembly is per source node, drops an
incomplete message when a newer one starts, and the protocol-level CRC still validates
every message. A mock bus allows testing without a CAN interface; real-bus tests are
skipped when `vcan0` is missing.

Estimated bus load at 500 kbit/s is about 7% (50 Hz commands, 10 Hz telemetry, 5 Hz
heartbeat); this is a calculation, not a measurement. Video stays on UDP.
Limitations: classic CAN only (no CAN FD or ISO-TP), no real hardware tested, static
node ids.The real-bus tests also run in CI on a virtual `vcan0` interface.


## Quality

- **Sanitizers**: the full suite (270+ tests) runs under AddressSanitizer and UBSan in CI
  (`-fno-sanitize-recover`, so any undefined behaviour fails the build)
- **Property/fuzz tests** with fixed seeds: random bytes are never accepted; any
  single-byte corruption of a valid frame is rejected; a lossy, duplicated, reordered
  video stream never delivers a corrupt or out-of-order frame; 20,000 garbage datagrams
  never move the vehicle
- **Coverage**: about 93% of lines (97% of functions), measured with gcov/gcovr over the
  hand-written libraries; Simulink-generated code is excluded because it is verified by
  equivalence tests. Uncovered code is mostly the real-time loop helper and defensive
  paths such as failed system calls
- **CI**: GitHub Actions runs a regular build and a sanitizer build on every push

## Performance

Release build, WSL2 on a desktop CPU, general-purpose (non-real-time) Linux kernel;
`apps/bench` reproduces these numbers.

| Measurement | p50 | p99 | max |
|---|---|---|---|
| `VehicleNode::tick`, assist off | 0.40 us | 1.10 us | 86.4 us |
| `VehicleNode::tick`, lidar assist on | 1.40 us | 2.50 us | 61.3 us |
| UDP loopback round trip | 3.5 us | 13.2 us | 226.6 us |

The control cycle budget is 20 ms (50 Hz); a full cycle including lidar ray casting
uses well under 0.1% of it at p99. Protocol encode + decode takes about 500 ns per
message. Video rendering and JPEG encoding (about 4 ms) run in the same cycle every
100 ms. Occasional maxima of tens of microseconds come from OS scheduling; this is not
a hard real-time guarantee. The loopback figure measures syscall overhead only, not
network latency.

## Docker

The image is built in two stages (compile and test, then a runtime image that contains
only `vehicle_sim` and runs as a non-root user). The compiler, sources and tests are not
in the final image. Its size (about 650 MB vs 710 MB single-stage) is dominated by the
distribution's OpenCV runtime dependencies.

```bash
docker build -t teledrive:slim .
docker run --rm -p 9001:9001/udp -p 9002:9002/udp teledrive:slim
```

Run `operator_cli` on the host (it needs a display) with `--vehicle 127.0.0.1:9001`.


## Features

- Binary UDP protocol with versioning, sequence numbers and CRC-16
- Transport and clock abstractions (`ITransport`, `IClock`) for deterministic testing
- Safety state machine: Idle / Armed / Driving / Fault, latched faults, E-stop priority
- Command watchdog (200 ms), command limiting, NaN/Inf rejection
- Differential-drive simulation with acceleration limits and battery model
- Replay/duplicate rejection with sequence wrap-around handling
- CI (GitHub Actions) and Docker build, about 270 automated tests
- CAN transport(SocketCAN)


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
- No authentication or encryption on the control link; the video hello handshake can be
  spoofed by anyone on the same network
- Keyboard input cannot detect key release, so an arm toggle replaces a true deadman switch
- Steering input does not self-center on the keyboard (angular velocity persists until zeroed)
- Lidar covers only the front; reversing is not protected; obstacles are static circles
- Latency shown in the video window is an estimate (RTT/2 + jitter), not a measurement
- Decoded image size is not limited before JPEG decompression
- Status lines assume a terminal at least 80 columns wide
- The Simulink-generated PID is equivalent only at dt = 0.02 s
- CAN: classic frames only, tested on virtual bus
