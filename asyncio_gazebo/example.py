import asyncio
import logging
import time
from contextlib import suppress
from typing import List, Optional, TypeVar

import asyncio_for_robotics as afor
import numpy as np
import posetree
import transforms_py._core as tp
from asyncio_for_robotics.core._logger import setup_logger
from gz.msgs.clock_pb2 import Clock
from gz.msgs.pose_v_pb2 import Pose_V
from gz.transport import Node, ProtoMsg, _transport

_MsgType = TypeVar("_MsgType")

setup_logger("./")
logger = logging.getLogger(__name__)


class TransformsPoseTree(posetree.CustomFramePoseTree):
    """My implementation of PoseTree to integrate with MyTransformManager"""

    def __init__(self, registry: tp.PyRegistry):
        super().__init__()
        self._registry = registry

    def _get_transform(
        self, parent_frame: str, child_frame: str, timestamp: Optional[float] = None
    ) -> posetree.Transform:
        if timestamp is None:
            timestamp = time.time()
        transform_data = self._registry.get_transform(
            parent_frame, child_frame, int(timestamp * 1e9)
        )
        if transform_data is None:
            raise KeyError(
                f"No transform found from {parent_frame} to {child_frame} at time {timestamp}"
            )
        tx, ty, tz, qx, qy, qz, qw, _ts, _parent, _child = transform_data
        return posetree.Transform.from_position_and_quaternion(
            [tx, ty, tz], [qx, qy, qz, qw]
        )


class GzSub(afor.BaseSub[_MsgType]):
    def __init__(
        self,
        msg_type: ProtoMsg,
        topic: str,
        options: _transport.SubscribeOptions = _transport.SubscribeOptions(),
    ) -> None:
        self.n = Node()
        self.o = self.n.subscribe(msg_type, topic, self.input_data, options)
        self.msg_type = msg_type
        self.topic = topic
        super().__init__()

    @property
    def name(self) -> str:
        return f"GzSub-{self.topic}"


class TfBufferSub(afor.BaseSub[List[str]]):
    _worlds_end = 2**128 - 1

    def __init__(self, topic: str) -> None:
        self.raw_sub = GzSub(Pose_V, topic)
        self.reg = tp.PyRegistry()
        self.tpt = TransformsPoseTree(self.reg)
        super().__init__()

        self._loop_task = asyncio.create_task(self._loop())

    async def _loop(self):
        async for msg in self.raw_sub.listen_reliable(queue_size=0):
            for pero in msg.pose:
                self.reg.add_transform(
                    x=pero.position.x,
                    y=pero.position.y,
                    z=pero.position.z,
                    qx=pero.orientation.x,
                    qy=pero.orientation.y,
                    qz=pero.orientation.z,
                    qw=pero.orientation.w,
                    timestamp=time.time_ns(),
                    parent="world",
                    child=pero.name,
                )
                self.reg.add_transform(
                    x=pero.position.x,
                    y=pero.position.y,
                    z=pero.position.z,
                    qx=pero.orientation.x,
                    qy=pero.orientation.y,
                    qz=pero.orientation.z,
                    qw=pero.orientation.w,
                    timestamp=self._worlds_end,
                    parent="world",
                    child=pero.name,
                )
            self._input_data_asyncio([p.name for p in msg.pose])

    def close(self):
        self._loop_task.cancel()
        self.raw_sub.close()
        super().close()

    def subscribe_to_tf(self, parent: str, child: str):
        return TfSub(parent, child, self)


class TfSub(afor.BaseSub[posetree.Transform]):
    def __init__(self, parent: str, child: str, buffer_sub: TfBufferSub) -> None:
        self.parent = parent
        self.child = child
        self.buffer_sub = buffer_sub
        super().__init__()
        self._loop_task = asyncio.create_task(self._loop())

    async def _loop(self):
        prev = None
        async for _ in self.buffer_sub.listen_reliable(queue_size=0):
            tf = self.buffer_sub.tpt.get_transform(self.parent, self.child, time.time())
            if prev is None:
                self._input_data_asyncio(tf)
                prev = tf
                continue
            identical = tf.almost_equal(prev, atol=1e-15)
            # identical = np.all(tf.position == prev.position) and np.all(
            #     tf.rotation.as_matrix == prev.rotation.as_matrix
            # )
            if identical:
                continue
            self._input_data_asyncio(tf)
            prev = tf

    def close(self):
        self._loop_task.cancel()
        self.buffer_sub.close()
        super().close()


async def async_main():
    main_sub = TfBufferSub("/world/diff_drive/dynamic_pose/info")
    tf_sub = main_sub.subscribe_to_tf("vehicle_blue", "vehicle_green")
    async for k in tf_sub.listen_reliable():
        print(k)


if __name__ == "__main__":
    with suppress(KeyboardInterrupt):
        asyncio.run(async_main())
