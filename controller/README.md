# Robot Soccer Controller

This is the central control software for the robot, running on a Raspberry Pi.
It is responsible for:
- vision (detections of field, ball and other robots),
- sending actuator commands over serial
- performing closed loop control to determine actuator commands based on a target value
- computing target values based on a soccer strategy

## Infrastructure
Dependencies are managed by the `uv` python package manager. This package manager itself
is installed via a Nix flake.

To run the flake shell, run inside of this folder:
```bash
nix develop
```

The `uv` package manager will create its own `.venv` directory by default.
However, Raspberry Pi specific dependencies such as `picamera2` depend on the
versions libraries installed by `apt`. Thus, it is necessary to create a custom
venv with `--system-site-packages` to include the python bindings to these
libraries. `uv` will then manage the activated virtual environment.
**Ensure the Nix DevShell is activated when creating the venv to avoid version conflicts**
```bash
python -m venv --system-site-packages .venv
```

Some dependencies (eg: opencv-python) require built dependencies better installed from apt:
```bash
sudo apt install python3-opencv
```
