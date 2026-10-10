"""BME280 I2C environmental sensor driver."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import FluidPressure, RelativeHumidity, Temperature


class Bme280Driver(Node):
    """Publish air temperature, humidity, and pressure readings."""

    def __init__(self) -> None:
        super().__init__('bme280_driver')
        self.declare_parameter('address', 0x76)
        import board
        import adafruit_bme280.basic as adafruit_bme280
        self.sensor = adafruit_bme280.Adafruit_BME280_I2C(
            board.I2C(), address=int(self.get_parameter('address').value)
        )
        self.temperature = self.create_publisher(
            Temperature, '/asv/environment/air_temperature', 10
        )
        self.humidity = self.create_publisher(
            RelativeHumidity, '/asv/environment/relative_humidity', 10
        )
        self.pressure = self.create_publisher(
            FluidPressure, '/asv/environment/air_pressure', 10
        )
        self.create_timer(1.0, self._read)

    def _read(self) -> None:
        stamp = self.get_clock().now().to_msg()
        temperature = Temperature()
        temperature.header.stamp = stamp
        temperature.header.frame_id = 'base_link'
        temperature.temperature = float(self.sensor.temperature)
        humidity = RelativeHumidity()
        humidity.header = temperature.header
        humidity.relative_humidity = float(self.sensor.relative_humidity) / 100.0
        pressure = FluidPressure()
        pressure.header = temperature.header
        pressure.fluid_pressure = float(self.sensor.pressure) * 100.0
        self.temperature.publish(temperature)
        self.humidity.publish(humidity)
        self.pressure.publish(pressure)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Bme280Driver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
