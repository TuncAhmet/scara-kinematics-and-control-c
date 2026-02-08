# SCARA Robot Kinematics & Closed-Loop Control

A C implementation of SCARA robot forward/inverse kinematics using Denavit-Hartenberg parameters, PID-based motion control with trajectory generation, real-time simulation, and a Python visualization dashboard.

## Features

- **Forward Kinematics (FK)**: Compute end-effector pose (x, y, z, yaw) from joint states using D-H transformation matrices
- **Inverse Kinematics (IK)**: Compute joint targets from desired pose with elbow-up/down solutions
- **PID Control**: Per-joint PID controllers with anti-windup and rate limiting
- **Trajectory Generation**: Trapezoidal velocity profiles for smooth motion
- **Real-time Simulation**: 500 Hz deterministic control loop with first-order motor dynamics
- **Python Dashboard**: 2D/3D visualization with live telemetry

## SCARA Configuration

Classic 4-DOF SCARA robot:
- θ₁: Base rotation (revolute)
- θ₂: Elbow rotation (revolute)
- d₃: Vertical extension (prismatic)
- θ₄: Wrist rotation (revolute)

## Building

```bash
# Build main executable
make

# Run tests
make test

# Clean build artifacts
make clean
```

## Running

```bash
# Start C simulation server
./build/scara_sim.exe

# In another terminal, start Python UI
cd python_ui
pip install -r requirements.txt
python main.py
```

## Project Structure

```
├── include/scara/    # Header files
├── src/              # C source files
├── tests/            # Unity test framework & tests
├── python_ui/        # Python dashboard & visualization
├── Makefile          # Build system
└── README.md
```

## License

MIT License
