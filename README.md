# Asyncio Gazebo

| Requirements | Tests |
|---|---|
| [![python](https://img.shields.io/pypi/pyversions/asyncio_for_robotics?logo=python&logoColor=white&label=Python&color=%20blue)](https://pypi.org/project/asyncio_for_robotics/)<br>[![mit](https://img.shields.io/badge/License-MIT-gold)](https://opensource.org/license/mit) | `3.10`, `3.11`, `3.12`, `3.13`<br>`ubuntu`, `windows`, `macos`<br>[![Python](https://github.com/2lian/asyncio-gazebo/actions/workflows/python-pixi.yml/badge.svg)](https://github.com/2lian/asyncio-gazebo/actions/workflows/python-pixi.yml) |

Asyncio interface to gazebo transport subscribers!
Based on [`asyncio_for_robotics`](https://github.com/2lian/asyncio-for-robotics)

### Code Sample

This library is an [Asyncio for Robotics](https://github.com/2lian/asyncio-for-robotics) subscriber, for Gazebo transport. Refer to the detailed docs of *Asyncio for Robotics* to use the subscriber and asyncio.

```python
sub = asyncio_gazebo.Sub(Pose_V, "/world/diff_drive/dynamic_pose/info")

# Get a new message
await sub.wait_for_new()

# Continuously process all incoming messages
async for msg in sub.listen_reliable():
    print(msg)

# and many more ...
```

# Installation

First, you need to [install Gazebo transport](https://gazebosim.org/api/transport/15/installation.html). Then install with pip:

```bash
pip install asyncio_gazebo
```

### [Example code](./asyncio_gazebo/example.py)

```bash
pixi run python -m asyncio_gazebo.example
```
