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
