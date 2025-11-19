# pyrobotiq

`pyrobotiq` is a small Python library that provides a simple socket-based interface
to Robotiq grippers connected to a Universal Robots (UR) controller via the
Robotiq URCap.

It talks to the UR controller over TCP using the Robotiq ASCII `GET/SET` protocol
on the URCap’s internal server port (typically `63352`).

## Features

- Connect to a UR controller over TCP
- Activate / reset the Robotiq gripper
- Move to a raw position `0–255`
- Move using a convenient `0–100 %` interface (`0%` open, `100%` closed)
- Read status, object status and fault codes

## Installation

Once published, you’ll be able to install via:

```bash
pip install pyrobotiq
```
