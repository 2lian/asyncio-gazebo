import asyncio
import logging
import time
from typing import Any, AsyncGenerator, Callable, Generator, Optional, Union

import pytest
from asyncio_for_robotics.core import BaseSub
from asyncio_for_robotics.core.sub import ConverterSub
from base_tests import (
    test_freshness,
    test_listen_one_by_one,
    test_listen_too_fast,
    test_loop_cancellation,
    test_reliable_extremely_fast,
    test_reliable_one_by_one,
    test_reliable_too_fast,
    test_wait_cancellation,
    test_wait_for_value,
    test_wait_new,
    test_wait_next,
)
from gz.msgs.stringmsg_pb2 import StringMsg
from gz.transport import Node

from asyncio_gazebo import Sub, auto_session

logger = logging.getLogger("asyncio_for_robotics.test")


@pytest.fixture(scope="function", autouse=True)
def session() -> Generator[Node, Any, Any]:
    ses = auto_session()
    yield ses
    del ses


@pytest.fixture(scope="function")
def pub(session) -> Generator[Callable[[str], None], Any, Any]:
    pub_topic = "test/something"
    logger.debug("Creating PUB-%s", pub_topic)
    p = auto_session().advertise(pub_topic, StringMsg)

    def pub_func(input: str):
        msg = StringMsg()
        msg.data = input
        p.publish(msg)

    yield pub_func
    del p

@pytest.fixture
async def sub(session) -> AsyncGenerator[BaseSub[str], Any]:
    inner_sub = Sub(StringMsg, "test/something")
    s: ConverterSub[str] = ConverterSub(inner_sub, lambda msg: msg.data)
    yield s
    s.close()
