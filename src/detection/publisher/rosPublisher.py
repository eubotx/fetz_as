import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import asyncio
import signal

class AsyncPublisher(Node):
    def __init__(self):
        super().__init__('async_publisher')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        self.count = 0
        self._running = True

    async def publish_loop(self):
        try:
            while self._running and rclpy.ok():
                msg = String()
                msg.data = f'Hello from async ROS 2! Count: {self.count}'
                self.publisher_.publish(msg)
                self.get_logger().info(f'Publishing: "{msg.data}"')
                self.count += 1
                await asyncio.sleep(1.0)
        except asyncio.CancelledError:
            self.get_logger().info('Publish loop cancelled, shutting down.')

    def stop(self):
        self._running = False

async def main():
    rclpy.init()
    node = AsyncPublisher()

    publish_task = asyncio.create_task(node.publish_loop())

    # Define shutdown handler
    def shutdown_handler():
        node.get_logger().info('Shutdown signal received.')
        node.stop()
        publish_task.cancel()

    # Handle Ctrl+C gracefully
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, shutdown_handler)

    try:
        await rclpy.spin_until_future_complete(node, asyncio.Future())
    except asyncio.CancelledError:
        pass
    finally:
        await publish_task
        node.destroy_node()
        rclpy.shutdown()

class RosPublisher:
    def __init__(self):
        try:
            asyncio.run(main())
        except KeyboardInterrupt:
            pass

    def publish(self, data: dict):
        print(f'Need to connect this properly! Publishing: {data}')
