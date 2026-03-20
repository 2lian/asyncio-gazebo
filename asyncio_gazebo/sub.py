import logging
from typing import Optional, TypeVar

import asyncio_for_robotics as afor
from gz.transport import Node, ProtoMsg, _transport

from .session import auto_session

logger = logging.getLogger("asyncio_for_robotics." + __name__)


class Sub(afor.BaseSub[ProtoMsg]):
    def __init__(
        self,
        msg_type: ProtoMsg,
        topic: str,
        options: _transport.SubscribeOptions = _transport.SubscribeOptions(),
        session: Optional[Node] = None,
    ) -> None:
        """Implementation of a Gazebo Transport subscriber.

        Refere to the base class (BaseSub) for details.

        Args:
            msg_type: Gazebo message type
            topic: Topic to listen to
            options: additional transport options
            session: The Gazebo node to use (if not provided, will use the
                                             auto_session to create/get one)
        """
        self.session: Node = self._resolve_session(session)
        self.msg_type: ProtoMsg = msg_type
        self.topic: str = topic
        self.options: _transport.SubscribeOptions = options
        self._o: None = self._resolve_sub(
            self.msg_type, self.topic, self.options, self.session
        )
        super().__init__()

    @property
    def name(self) -> str:
        return f"GzSub-{self.topic}"

    def _resolve_session(self, session: Optional[Node]) -> Node:
        """Called at __init__ to get the Node.

        Usefull to overide in a child class and change the Node behavior.
        """
        return auto_session(session)

    def _gz_data_ingress(self, msg: ProtoMsg):
        if self._closed.is_set():
            return
        self.input_data(msg)

    def _resolve_sub(
        self,
        msg_type: ProtoMsg,
        topic: str,
        options: _transport.SubscribeOptions,
        session: Node,
    ) -> None:
        """Called at __init__ to create the subscriber.

        Usefull to overide in a child class and change the Subscription behavior.
        """
        return session.subscribe(msg_type, topic, self._gz_data_ingress, options)

    def close(self):
        if self._closed.is_set():
            return
        del self._o
        return super().close()
