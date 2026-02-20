from transforms_py import hello, hello_t
from transforms_py._core import PyRegistry, hello_from_bin


def main():
    print(hello())
    print(hello_t())
    print(hello_from_bin())

    registry = PyRegistry()
    registry.add_transform(
        8.756692593259174e-11,
        2.0000000001634226,
        0.32500718901591097,
        -2.5258998287146937e-10,
        1.5503755677123035e-10,
        -2.592567331066336e-12,
        1.0,
        10,
        "world",
        "vehicle_blue",
    )
    transform = registry.get_transform("world", "vehicle_blue", 10)
    print(transform)


if __name__ == "__main__":
    main()
