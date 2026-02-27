import asyncio
from contextlib import suppress

from asyncio_for_robotics.core.sub import ConverterSub
from gz.msgs.stringmsg_pb2 import StringMsg

from asyncio_gazebo import Sub, auto_session

topic = "hello/world"


async def listen():
    sub = Sub(StringMsg, topic)
    async for msg in sub.listen_reliable():
        print(f"I heard: {msg.data}")


async def send():
    publisher = auto_session().advertise(topic, StringMsg)
    count = 0
    while 1:
        msg = StringMsg()
        msg.data = f"Hello World! {count}"
        print(f"I sent: {msg.data}, onto topic: {topic}")
        count += 1
        publisher.publish(msg)
        await asyncio.sleep(1)


async def main():
    async with asyncio.TaskGroup() as tg:
        tg.create_task(listen())
        tg.create_task(send())


if __name__ == "__main__":
    with suppress(KeyboardInterrupt):
        asyncio.run(main())
