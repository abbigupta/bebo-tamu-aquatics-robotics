"""TFmini Plus serial driver publishing sensor_msgs/Range."""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range


def decode_frame(frame: bytes) -> tuple[float, int, float]:
    if len(frame) != 9 or frame[:2] != b'\x59\x59':
        raise ValueError('invalid TFmini frame header or length')
    if sum(frame[:8]) & 0xff != frame[8]:
        raise ValueError('invalid TFmini checksum')
    distance_cm = frame[2] | frame[3] << 8
    strength = frame[4] | frame[5] << 8
    temperature = (frame[6] | frame[7] << 8) / 8.0 - 256.0
    return distance_cm / 100.0, strength, temperature


class TfminiDriver(Node):
    """Decode TFmini serial frames and publish front range data."""

    def __init__(self) -> None:
        super().__init__('tfmini_driver')
        self.declare_parameter('port', '/dev/ttyAMA3')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('frame_id', 'lidar_link')
        import serial
        self.serial = serial.Serial(
            str(self.get_parameter('port').value),
            int(self.get_parameter('baud').value), timeout=0.05,
        )
        self.buffer = bytearray()
        self.publisher = self.create_publisher(Range, '/asv/range/front', 10)
        self.create_timer(0.02, self._read)

    def _read(self) -> None:
        self.buffer.extend(self.serial.read(self.serial.in_waiting or 1))
        while len(self.buffer) >= 9:
            index = self.buffer.find(b'\x59\x59')
            if index < 0:
                self.buffer.clear()
                return
            del self.buffer[:index]
            if len(self.buffer) < 9:
                return
            frame = bytes(self.buffer[:9])
            del self.buffer[:9]
            try:
                distance, _, _ = decode_frame(frame)
            except ValueError:
                continue
            message = Range()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = str(self.get_parameter('frame_id').value)
            message.radiation_type = Range.INFRARED
            message.field_of_view = 0.035
            message.min_range = 0.10
            message.max_range = 12.0
            message.range = distance
            self.publisher.publish(message)

    def destroy_node(self):
        self.serial.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TfminiDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
