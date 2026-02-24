# asyncio-gazebo
asyncio executor to interface with gazebo transport... But right now it's more of a transform experiment.

## Important prerequisite

This is because the wheel of `transforms-py` on pypi is broken.

```bash
# inside this repo
git clone https://github.com/art-e-fact/transforms-py.git
```

## Python example

```bash
pixi run example
```

## Gazebo sim

```bash
pixi run sim
```

## Move the robot in sim

```bash
pixi run move
```

# Result

```console
world  ->  vehicle_blue
Transform(position=[-4.52007568  6.86254757  0.32499999], rotation=[-1.86669122e-10 -2.85378094e-10 -6.50232916e-01  7.59734924e-01])

vehicle_green  ->  vehicle_blue
Transform(position=[-4.52007533e+00  8.86254757e+00  5.54573092e-06], rotation=[-3.55647033e-07  4.15036707e-07 -6.50232916e-01  7.59734924e-01])
```

# Example explanation

```python
async def async_main():
    # Sub to get raw "Pose_V" data from gazebo transport
    raw_sub = GzSub(Pose_V, "/world/diff_drive/dynamic_pose/info")
    # Sub to convert each payload to a list of transforms instead
    # So this tf_stream is no longer specific to gazebo
    tf_stream: afor.BaseSub[List[TransformNode]] = afor.ConverterSub(
        raw_sub, lambda msg: [TransformNode.from_gz(k) for k in msg.pose]
    )
    # Sub that also incorporates a buffer and graph of TF
    # this sub has the .subscribe_to_tf(from, to) method. This will return a
    # subscriber generating the requested transform. 
    main_sub = TfBufferSub(tf_stream)
    # Sub only triggering on updates to tf related to the transform from green to blue.
    tf_sub = tf_buffer_sub.subscribe_to_tf("vehicle_green", "vehicle_blue")
    async for k in tf_sub.listen_reliable():
        print(tf_sub.parent, " -> ", tf_sub.child)
        print(k)
        print()
```
