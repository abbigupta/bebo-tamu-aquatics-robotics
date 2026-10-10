"""Linux 1-Wire DS18B20 temperature driver."""

from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature


def read_temperature(path: Path) -> float:
    lines = path.read_text(encoding='utf-8').strip().splitlines()
    if len(lines) < 2 or not lines[0].endswith('YES') or 't=' not in lines[1]:
        raise ValueError('invalid DS18B20 reading')
    return int(lines[1].split('t=', 1)[1]) / 1000.0


class Ds18b20Driver(Node):
    """Publish water temperature from a Linux 1-Wire device."""

    def __init__(self) -> None:
        super().__init__('ds18b20_driver')
        self.declare_parameter('device_id', '28-000000230f20')
        device = str(self.get_parameter('device_id').value)
        self.path = Path('/sys/bus/w1/devices') / device / 'w1_slave'
        self.publisher = self.create_publisher(
            Temperature, '/asv/environment/water_temperature', 10
        )
        self.create_timer(1.0, self._read)

    def _read(self) -> None:
        try:
            value = read_temperature(self.path)
        except (OSError, ValueError) as error:
            self.get_logger().warning(str(error), throttle_duration_sec=5.0)
            return
        message = Temperature()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'base_link'
        message.temperature = value
        message.variance = 0.0625
        self.publisher.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Ds18b20Driver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
