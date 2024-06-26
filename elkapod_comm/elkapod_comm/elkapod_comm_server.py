import sys
import rclpy
from rclpy.node import Node, ParameterDescriptor, IntegerRange
import spidev
from sensor_msgs.msg import Temperature
from elkapod_msgs.msg import LegFrames
from .hexapod_protocol import one_leg_frame, split_to_integer_and_float_parts, FrameType, FRAME_LENGTHS_TRANSMIT, FRAME_LENGTHS_RECEIVE, INFO_FRAME_LIST, check_info_frame
from .hexapod_protocol import __version__ as HexapodProtocolVersion
from .hexapod_protocol_exceptions import MalformedHexapodInfoFrameError, IncompatibleHexapodProtocolVersionError, OldHexapodProtocolVersionWarning, NewHexapodProtocolVersionWarning


class ElkapodCommServer(Node):
    def __init__(self):
        super().__init__("elkapod_comm_server")
        self.declare_parameter("spi_speed", value=1000000,
                               descriptor=ParameterDescriptor(
                                   description="Operating frequency of SPI peripherial in Hz",
                                   integer_range=[IntegerRange(from_value=1000000, to_value=15000000, step=0)]))
        
        # Setup SPI connection
        self._spi = spidev.SpiDev()
        try:
            self._spi.open(0, 0)
        except Exception:
            self.get_logger().fatal("Cannot open SPI connection to Hexapod Hardware Controller! Aborting...")
            sys.exit(1)

        spi_speed = self.get_parameter("spi_speed").value
        self._spi.max_speed_hz = spi_speed
        self._spi.mode = 0b00


        # Control subscriptions
        self._elkapod_leg_subscription = self.create_subscription(
            LegFrames,
            "elkapod_comm_server_leg_frames",
            self._leg_frame_callback,
            10
        )


        # Sensor & Info publishers
        self._info_timer_period = 30
        self._adc_timer_period = 0.1
        self._temperature_timer_period = 1


        self._get_info_timer = self.create_timer(self._info_timer_period, self.get_hhc_info)
        self._get_temperature_timer = self.create_timer(self._temperature_timer_period, self.get_temperature_info)
        self._mcu_temperature_publisher = self.create_publisher(Temperature, "/elkapod_mcu_temp", 10)


        # Check Hardware Controler
        self.get_hhc_info()

    def _leg_frame_callback(self, msg):
        leg_frames = msg.leg_frames
        for frame in leg_frames:
            leg_id = frame.leg_nb
            servo_op_codes = frame.servo_op_codes.tolist()
            angles = frame.servo_angles.tolist()

            # Translation from kinematics ranges to servo ranges
            angles[0] += 90
            angles[1] += 90
            angles[2] *= -1

            angle_int_parts = []
            angle_float_parts = []
            for angle in angles:
                int_part, float_part = split_to_integer_and_float_parts(angle)
                angle_int_parts.append(int_part)
                angle_float_parts.append(float_part)

            message2 = one_leg_frame(leg_id, servo_op_codes, angle_int_parts, angle_float_parts)
            self._send_data(FRAME_LENGTHS_TRANSMIT[FrameType.ONE_LEG], message2)

    def get_hhc_info(self):
        self._send_data(FRAME_LENGTHS_TRANSMIT[FrameType.INFO], INFO_FRAME_LIST)
        data = self._spi.readbytes(FRAME_LENGTHS_RECEIVE[FrameType.INFO])

        try:
            check_info_frame(data)
            self.get_logger().info(f"Connection with Hardware Controller stable, compatible Hexapod Protocol version -> {HexapodProtocolVersion}")
        except (IncompatibleHexapodProtocolVersionError, MalformedHexapodInfoFrameError) as e:
            self.get_logger().error(str(e))
        except (OldHexapodProtocolVersionWarning, NewHexapodProtocolVersionWarning) as w:
            self.get_logger().warning(str(w))

    def get_adc_info(self):
        self._send_data(FRAME_LENGTHS_TRANSMIT[FrameType.ADC], [FRAME_LENGTHS_TRANSMIT[FrameType.ADC], FrameType.ADC.value])
        data = self._spi.readbytes(FRAME_LENGTHS_RECEIVE[FrameType.ADC])
        
        raw_voltage = self._convert_bytes_to_value(data[3:])
        voltage = (raw_voltage / 4096)*3.3

        print(f"Voltage: {voltage} V")

    def get_temperature_info(self):
        self._send_data(FRAME_LENGTHS_TRANSMIT[FrameType.TEMPERATURE], [FRAME_LENGTHS_TRANSMIT[FrameType.TEMPERATURE], FrameType.TEMPERATURE.value])
        data = self._spi.readbytes(FRAME_LENGTHS_RECEIVE[FrameType.TEMPERATURE])

        temperature = self._convert_bytes_to_value(data[2:]) / 1000


        temperature_msg = Temperature()
        temperature_msg.header.stamp = self.get_clock().now().to_msg()
        temperature_msg.temperature = temperature
        temperature_msg.variance = 0.0                # for now variance not specified

        self._mcu_temperature_publisher.publish(temperature_msg)

    def _send_data(self, nbytes: int, bytes_to_be_send: list):
        # Send how many bytes the MCU should expect to be sent
        self._spi.writebytes2([nbytes])
        self._spi.writebytes2(bytes_to_be_send)

    @staticmethod
    def _convert_bytes_to_value(datas: list) -> float:
        value = 0
        for i, data in enumerate(datas):
            value |= data << (len(datas) - i - 1) * 8
        return value


def main(args=None):
    rclpy.init(args=args)
    driver = ElkapodCommServer()

    rclpy.spin(driver)
    driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
