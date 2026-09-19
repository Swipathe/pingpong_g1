# Nexus Online Probe

This directory is stage 1 of the online mocap pipeline. It only verifies that
Vicon Nexus live data can be read from the Vicon DataStream SDK. It does not
publish LCM and does not drive RobotBridge.

## Prerequisites

The Vicon DataStream SDK is expected at:

```bash
/root/Hitter/RobotBridge/vicon_datastream_sdk
```

Load the SDK runtime library path before running C++ DataStream tools:

```bash
cd /root/Hitter/RobotBridge
source vicon_datastream_sdk/setup_env.sh
```

Vicon DataStream SDK 1.13 Linux binaries are built for Ubuntu 24.04 / glibc
2.39 or newer. If linking or running fails with `GLIBC_2.38` or
`GLIBCXX_3.4.32`, use a newer host, or use an older Vicon SDK that matches the
OS.

## C++ Probe

The Linux SDK package contains C/C++ libraries. Build the C++ probe on the
machine where it will run:

```bash
cd /root/Hitter/RobotBridge/deploy
source ../vicon_datastream_sdk/setup_env.sh
bash mocap_bridge/build_cpp_probe.sh
```

List live Nexus subjects, markers, and segments:

```bash
./mocap_bridge/bin/nexus_probe_cpp --host <nexus_ip>:801 --list
```

Stream a ball marker only:

```bash
./mocap_bridge/bin/nexus_probe_cpp \
  --host <nexus_ip>:801 \
  --ball-subject Ball \
  --ball-marker Ball \
  --print-hz 20
```

Stream a ball marker and robot base subject:

```bash
./mocap_bridge/bin/nexus_probe_cpp \
  --host <nexus_ip>:801 \
  --ball-subject Ball \
  --ball-marker Ball \
  --base-subject G1Base \
  --print-hz 20 \
  --csv logs/nexus_probe.csv
```

If the base segment is not specified, the probe asks Nexus for the root segment
of `--base-subject`.

## Optional Python Probe

The Python probe requires a Python binding that provides:

```python
from vicon_dssdk import ViconDataStream
```

The Vicon Linux SDK archive used here does not expose this module directly. Use
the C++ probe unless the target machine already has an official Python binding.

Python list command:

```bash
cd /root/Hitter/RobotBridge/deploy
python -m mocap_bridge.nexus_probe --host <nexus_ip>:801 --list
```

Python stream command:

```bash
python -m mocap_bridge.nexus_probe \
  --host <nexus_ip>:801 \
  --ball-subject Ball \
  --ball-marker Ball \
  --print-hz 20
```

## Notes

Vicon translations are reported in millimeters. The console also prints meters
for quick checking. The CSV stores both millimeters and meters for the ball.

Expected next stages:

1. Add planner-world calibration and ball velocity filtering.
2. Publish the filtered ball state over LCM.
3. Subscribe to that LCM state in MuJoCo and RealWorld simulators.
