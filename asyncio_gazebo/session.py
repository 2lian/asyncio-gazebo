import logging
from typing import Optional

from gz.transport import Node

GLOBAL_SESSION: Optional[Node] = None

logger = logging.getLogger("asyncio_for_robotics."+__name__)


def set_auto_session(session: Optional[Node] = None) -> None:
    """Set the global shared session instance (Gazebo transport node).

    If called with a session, it replaces the current global session.
    If called with None, the global session is unset (but not close!).

    Args:
        session: Gazebo transport node to set as default
    """
    global GLOBAL_SESSION
    GLOBAL_SESSION = session


def auto_session(session: Optional[Node] = None) -> Node:
    """Returns the passed session.
    If None, returns a singleton session shared with every other None call of
    this function."""
    global GLOBAL_SESSION
    if GLOBAL_SESSION is not None:
        return GLOBAL_SESSION

    logger.info("Global gazebo transport node: Starting")
    ses = Node()
    set_auto_session(ses)
    logger.info("Global gazebo transport node: Running")
    return ses
