# Asyncio Gazebo

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

You need to [install Gazebo transport](https://gazebosim.org/api/transport/15/installation.html).

```bash
pip install https://github.com/2lian/asyncio-gazebo
```

### [Example code](./asyncio_gazebo/example.py)

```bash
pixi run python -m asyncio_gazebo.example
```
