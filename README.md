# Robotics Sim Controller

```
                         +------------------------------+
                         | Virtual Differential Drive   |
                         |        Robot Plant           |
                         +------------------------------+
                                ^                 |
                                | sensor models   | state integration
                                |                 v
+-----------------+       +------------+    +-------------+     +---------------------+
| Profile Generator|----->| Kinematics |--->| PID Control |----->| Motor & Slip Model |
+-----------------+       +------------+    +-------------+     +---------------------+
                                               ^
                                               |
                                         gains (JSON)

```

```
            +----------------- Control Loop -----------------+
            |                                                |
     v_ref, ω_ref --> split --> PID --> PWM --> plant --> sensors --> feedback

```

A C++17 simulation of a two-wheel differential drive robot with per-wheel PID velocity control, configurable noise/drift, and Python-based telemetry plotting. The layout models embedded control structure for straightforward porting to MCUs such as STM32 or ESP32.

## Features

- 100 Hz simulated control loop with differential-drive kinematics and configurable slip.
- Wheel encoders with Gaussian noise, IMU yaw with bias drift and noise.
- Header-only PID with anti-windup clamping, gains loaded from `config/gains.json`.
- Telemetry streamed to stdout and persisted at `out/telemetry.csv`.
- Python 3.11 visualization renders trajectory and time-series plots (`out/plots/`).
- GoogleTest unit tests for PID behaviour and kinematics utility.
- GitHub Actions workflow builds, tests, runs a sample sim, and ensures plot generation.

## Build & Run

```bash
mkdir build
cd build
cmake ..
cmake --build .
```

Example run (streams CSV rows to stdout and writes the same to `out/telemetry.csv`):

```bash
./robotics-sim --duration 20 --v 0.3 --w 0.2 --slip 0.05 --noise 0.01 --drift 0.002 --profile step
```

Profiling waveforms:

```bash
./robotics-sim --duration 15 --v 0.4 --w 0.3 --profile triangle --csv out/triangle.csv
./robotics-sim --duration 12 --v 0.25 --w 0.4 --profile square --slip 0.02 --noise 0.02
```

## Visualization

Install Python dependencies:

```bash
python3 -m pip install -r sim/requirements.txt
```

Generate plots:

```bash
python3 sim/visualize.py out/telemetry.csv --output-dir out/plots --headless
```

Outputs:

- `out/plots/trajectory.png` — top-down XY path.
- `out/plots/signals.png` — linear speed tracking, yaw vs IMU, and PWM control effort.
- `docs/system_diagram.png` — static overview image (lightweight placeholder).

## Configuration

`config/gains.json` controls PID gains and output limits independently per wheel:

```json
{
  "left": { "kp": 2.0, "ki": 0.5, "kd": 0.05, "output_min": -1.0, "output_max": 1.0 },
  "right": { "kp": 2.0, "ki": 0.5, "kd": 0.05, "output_min": -1.0, "output_max": 1.0 }
}
```

Modify during tuning, rebuild, or re-run the simulator without recompiling.

## PID Tuning Tips

- Start with proportional-only (`ki = kd = 0`) to ensure correct directionality.
- Increase `kp` until oscillations appear, then back off ~20%.
- Add `ki` to eliminate steady-state error; increase slowly to avoid integrator windup.
- Use `kd` to tame overshoot, particularly for aggressive profiles (triangle/square).
- Verify the controller saturates cleanly via the telemetry PWM plot; rescale output limits for MCU PWM ranges when porting.

## Porting Notes

- Control loop already runs at 100 Hz simulated; replace the timing loop with hardware timers.
- Telemetry interface writes plain CSV; on STM32/ESP32 redirect to UART or logging middleware.
- PID and kinematics are header-only / freestanding; isolate `SensorSuite` to wrap hardware drivers.
- Keep the `TelemetryRow` layout for quick integration with existing visualization tooling.

## Continuous Integration

GitHub Actions workflow (`.github/workflows/ci.yml`) builds the project on Ubuntu, runs unit tests, executes a 5 s simulation, and ensures `sim/visualize.py` produces both trajectory and signal plots. This mirrors the recommended local validation steps.

Enjoy experimenting with the virtual robot and tailoring the controller for future embedded deployments!