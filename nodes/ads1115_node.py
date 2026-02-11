#!/usr/bin/env python3
"""ROS2 node that reads ADS1115 over I2C and publishes std_msgs/Float32 per channel."""

import time

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float32
from std_srvs.srv import Trigger

from ads1115_adc.ads1115_driver import ADS1115Driver, FakeADS1115Driver


class ADS1115AdcNode(Node):
    def __init__(self):
        super().__init__('ads1115_adc_node')

        # ── Declare parameters ────────────────────────────────────
        self.declare_parameter('fake_mode', True)
        self.declare_parameter('i2c_bus', 1)
        self.declare_parameter('device_address', 0x48)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('active_channels', [0, 1, 2, 3])
        self.declare_parameter('pga_gain', 2)
        self.declare_parameter('data_rate', 4)

        # ── Read parameters ───────────────────────────────────────
        self.fake_mode    = self.get_parameter('fake_mode').value
        self.bus_num      = self.get_parameter('i2c_bus').value
        self.address      = self.get_parameter('device_address').value
        rate              = self.get_parameter('publish_rate').value
        self.channels     = list(self.get_parameter('active_channels').value)
        self.pga_gain     = self.get_parameter('pga_gain').value
        self.data_rate    = self.get_parameter('data_rate').value

        # ── Voltage offset bias per channel (set by calibration) ──
        self.channel_bias = {ch: 0.0 for ch in range(4)}

        # ── Initialise driver ─────────────────────────────────────
        self._init_driver()

        # ── Publishers (one per active channel) ───────────────────
        self.pubs = {}
        for ch in self.channels:
            topic = f'adc/channel{ch}'
            self.pubs[ch] = self.create_publisher(Float32, topic, 10)

        self.timer = self.create_timer(1.0 / rate, self._timer_cb)
        ch_str = ', '.join(str(c) for c in self.channels)
        self.get_logger().info(
            f'Publishing std_msgs/Float32 on adc/channel[{ch_str}] '
            f'@ {rate} Hz')

        # ── Services ──────────────────────────────────────────────
        self.create_service(Trigger, 'adc/calibrate', self._calibrate_cb)
        self.create_service(Trigger, 'adc/reset', self._reset_cb)
        self.get_logger().info(
            'Services: "adc/calibrate", "adc/reset"')

        # ── Parameter change callback ─────────────────────────────
        self.add_on_set_parameters_callback(self._on_param_change)

    # ── Driver init helper ───────────────────────────────────────
    def _init_driver(self):
        if self.fake_mode:
            self.driver = FakeADS1115Driver()
            self.get_logger().info(
                'FAKE MODE enabled — generating random ADC voltage data')
        else:
            try:
                self.driver = ADS1115Driver(
                    self.bus_num, self.address,
                    self.pga_gain, self.data_rate)
                self.get_logger().info(
                    f'ADS1115 initialised  bus={self.bus_num}  '
                    f'addr=0x{self.address:02X}')
            except Exception as e:
                self.get_logger().fatal(f'Failed to open ADS1115: {e}')
                raise

    # ── Timer callback ───────────────────────────────────────────
    def _timer_cb(self):
        for ch in self.channels:
            try:
                voltage = self.driver.read_channel(ch)
            except OSError as e:
                self.get_logger().warn(
                    f'I2C read error (ch{ch}): {e}',
                    throttle_duration_sec=2.0)
                continue

            voltage -= self.channel_bias.get(ch, 0.0)

            msg = Float32()
            msg.data = voltage
            self.pubs[ch].publish(msg)

    # ── Service: /adc/calibrate ──────────────────────────────────
    def _calibrate_cb(self, request, response):
        if self.fake_mode:
            response.success = True
            response.message = 'Calibration complete (fake)'
            self.get_logger().info('Calibration requested in fake mode — skipped')
            return response

        self.get_logger().info(
            'Calibrating ADC — collecting data for 2 seconds...')
        samples = {ch: [] for ch in self.channels}
        end_time = time.monotonic() + 2.0
        while time.monotonic() < end_time:
            for ch in self.channels:
                try:
                    v = self.driver.read_channel(ch)
                    samples[ch].append(v)
                except OSError:
                    pass
            time.sleep(0.05)

        total = 0
        for ch in self.channels:
            if samples[ch]:
                self.channel_bias[ch] = sum(samples[ch]) / len(samples[ch])
                total += len(samples[ch])

        response.success = True
        bias_str = ', '.join(
            f'ch{ch}={self.channel_bias[ch]:.4f}V' for ch in self.channels)
        response.message = (
            f'Calibration complete — {total} samples, bias=[{bias_str}]')
        self.get_logger().info(response.message)
        return response

    # ── Service: /adc/reset ──────────────────────────────────────
    def _reset_cb(self, request, response):
        self.channel_bias = {ch: 0.0 for ch in range(4)}
        self.driver.close()
        self._init_driver()

        response.success = True
        response.message = 'Sensor reset complete'
        self.get_logger().info(response.message)
        return response

    # ── Runtime parameter change ─────────────────────────────────
    def _on_param_change(self, params):
        for param in params:
            if param.name == 'publish_rate':
                new_rate = param.value
                if new_rate <= 0.0:
                    return SetParametersResult(
                        successful=False,
                        reason='publish_rate must be > 0')
                self.timer.cancel()
                self.timer = self.create_timer(1.0 / new_rate, self._timer_cb)
                self.get_logger().info(f'publish_rate changed to {new_rate} Hz')
        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    node = ADS1115AdcNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.driver.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
