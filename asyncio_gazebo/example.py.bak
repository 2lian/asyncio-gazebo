import asyncio
import dataclasses
import logging
import time
from contextlib import suppress
from pprint import pprint
from typing import List, Optional, Self, Set, TypeVar

import asyncio_for_robotics as afor
import networkx
import numpy as np
import posetree
import quaternion
import spatialmath
import transforms_py._core as tp
from asyncio_for_robotics.core._logger import setup_logger
from gz.msgs.clock_pb2 import Clock
from gz.msgs.pose_v_pb2 import Pose_V
from gz.transport import Node, ProtoMsg, _transport

_MsgType = TypeVar("_MsgType")

setup_logger("./")
logger = logging.getLogger(__name__)


@dataclasses.dataclass
class TransformNode:
    name: str
    tf: posetree.Transform = dataclasses.field(
        default_factory=lambda *_: posetree.Transform.identity()
    )
    timestamp: int = 0
    parent: str = "world"

    @classmethod
    def from_gz(cls, pose_gz) -> Self:
        return cls(
            name=pose_gz.name,
            parent="world",
            tf=posetree.Transform.from_position_and_quaternion(
                [
                    pose_gz.position.x,
                    pose_gz.position.y,
                    pose_gz.position.z,
                ],
                [
                    pose_gz.orientation.x,
                    pose_gz.orientation.y,
                    pose_gz.orientation.z,
                    pose_gz.orientation.w,
                ],
            ),
            timestamp=time.time_ns(),
        )

    def to_transpy(
        self,
    ) -> tuple[float, float, float, float, float, float, float, int, str, str]:
        return (
            self.tf.x,
            self.tf.y,
            self.tf.z,
            self.tf.rotation.as_quat()[0],  # x
            self.tf.rotation.as_quat()[1],  # y
            self.tf.rotation.as_quat()[2],  # z
            self.tf.rotation.as_quat()[3],  # w
            self.timestamp,
            self.parent,
            self.name,
        )


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
        self._n = Node()
        self._o = self._n.subscribe(msg_type, topic, self.input_data, options)
        self.msg_type = msg_type
        self.topic = topic
        super().__init__()

    @property
    def name(self) -> str:
        return f"GzSub-{self.topic}"


class TfBufferSub(afor.BaseSub[List[TransformNode]]):
    _worlds_end = 2**128 - 1

    def __init__(self, tf_raw_sub: afor.BaseSub[List[TransformNode]]) -> None:
        self.raw_sub = tf_raw_sub
        self.reg = tp.PyRegistry()
        self.tpt = TransformsPoseTree(self.reg)
        self.live_graph: networkx.Graph = networkx.Graph(name="TF graph")
        super().__init__()

        self._loop_task = asyncio.create_task(self._loop())

    async def _loop(self):
        async for msg in self.raw_sub.listen_reliable(queue_size=0):
            for pero in msg:
                self.live_graph.add_edge(pero.parent, pero.name)
                transpy = pero.to_transpy()
                self.reg.add_transform(*transpy)
                transpy = list(transpy)
                transpy[7] = self._worlds_end
                self.reg.add_transform(*transpy)
            self._input_data_asyncio(msg)

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
        path: Set[str] = set()
        prev_graph = self.buffer_sub.live_graph.copy()
        async for tfs in self.buffer_sub.listen_reliable(queue_size=0):
            graph_update = False
            if prev_graph.edges != self.buffer_sub.live_graph.edges:
                prev_graph = self.buffer_sub.live_graph.copy()
                graph_update = True
            if graph_update or len(path) == 0:
                try:
                    path: Set[str] = set(
                        networkx.shortest_path(
                            self.buffer_sub.live_graph, self.parent, self.child
                        )
                    )
                except networkx.NodeNotFound:
                    continue
                except networkx.NetworkXNoPath:
                    continue
            updated_tfs = {k.name for k in tfs}
            update_on_path = len(updated_tfs & path) > 0
            if not update_on_path:
                continue
            tf = self.buffer_sub.tpt.get_transform(self.parent, self.child, time.time())
            self._input_data_asyncio(tf)

    def close(self):
        self._loop_task.cancel()
        self.buffer_sub.close()
        super().close()


async def world_to_blue(tf_buffer_sub: TfBufferSub):
    tf_sub = tf_buffer_sub.subscribe_to_tf("world", "vehicle_blue")
    async for k in tf_sub.listen_reliable():
        print(tf_sub.parent, " -> ", tf_sub.child)
        print(k)
        print()


async def green_to_blue(tf_buffer_sub: TfBufferSub):
    tf_sub = tf_buffer_sub.subscribe_to_tf("vehicle_green", "vehicle_blue")
    async for k in tf_sub.listen_reliable():
        print(tf_sub.parent, " -> ", tf_sub.child)
        print(k)
        print()


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
    # sub generatin the requested transform. The sub only trigger on updates to
    # tf related to the requested transform.
    main_sub = TfBufferSub(tf_stream)
    try:
        async with asyncio.TaskGroup() as tg:
            tg.create_task(world_to_blue(main_sub))
            tg.create_task(green_to_blue(main_sub))
    finally:
        main_sub.close()


if __name__ == "__main__":
    with suppress(KeyboardInterrupt):
        asyncio.run(async_main())
