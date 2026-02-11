# Copyright 2025 The ads1115_adc Authors
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file or at
# https://opensource.org/licenses/MIT.
"""Launch test for ADS1115 ADC sensor node."""

import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.markers
from launch_testing_ros import WaitForTopics
import pytest
from rcl_interfaces.srv import SetParameters
import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import Float32
from std_srvs.srv import Trigger


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """Launch ADS1115 node in fake_mode for testing."""
    node = launch_ros.actions.Node(
        package='ads1115_adc',
        executable='ads1115_node.py',
        name='ads1115_adc_node',
        parameters=[{
            'fake_mode': True,
            'publish_rate': 50.0,
            'active_channels': [0, 1, 2, 3],
        }],
    )
    return launch.LaunchDescription([
        node,
        launch_testing.actions.ReadyToTest(),
    ]), {'sensor_node': node}


class TestADS1115Topics(unittest.TestCase):
    """Verify all 4 ADC channel topics publish valid data."""

    def test_all_channels_published(self):
        """All 4 channel topics should receive messages."""
        topic_list = [
            ('adc/channel0', Float32),
            ('adc/channel1', Float32),
            ('adc/channel2', Float32),
            ('adc/channel3', Float32),
        ]
        with WaitForTopics(topic_list, timeout=10.0) as wait:
            self.assertEqual(
                wait.topics_received(),
                {'adc/channel0', 'adc/channel1',
                 'adc/channel2', 'adc/channel3'})

    def test_voltage_range(self):
        """Voltage on channel0 should be in reasonable range."""
        topic_list = [('adc/channel0', Float32)]
        with WaitForTopics(
            topic_list, timeout=10.0, messages_received_buffer_length=5
        ) as wait:
            msgs = wait.received_messages('adc/channel0')
            self.assertGreater(len(msgs), 0)
            for msg in msgs:
                self.assertGreaterEqual(msg.data, -10.0)
                self.assertLessEqual(msg.data, 10.0)


class TestADS1115Services(unittest.TestCase):
    """Verify calibrate and reset services."""

    @classmethod
    def setUpClass(cls):
        """Set up ROS2 context for service tests."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Shut down ROS2 context."""
        rclpy.shutdown()

    def setUp(self):
        """Create test node for service calls."""
        self.node = rclpy.create_node('test_ads1115_services')

    def tearDown(self):
        """Destroy test node."""
        self.node.destroy_node()

    def test_calibrate_service(self):
        """Calibrate should return success in fake mode."""
        client = self.node.create_client(Trigger, 'adc/calibrate')
        self.assertTrue(client.wait_for_service(timeout_sec=10.0))
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        self.assertTrue(future.result().success)
        self.assertIn('fake', future.result().message.lower())
        self.node.destroy_client(client)

    def test_reset_service(self):
        """Reset should return success."""
        client = self.node.create_client(Trigger, 'adc/reset')
        self.assertTrue(client.wait_for_service(timeout_sec=10.0))
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        self.assertTrue(future.result().success)
        self.assertIn('reset complete', future.result().message.lower())
        self.node.destroy_client(client)


class TestADS1115Parameters(unittest.TestCase):
    """Verify runtime parameter changes."""

    @classmethod
    def setUpClass(cls):
        """Set up ROS2 context for parameter tests."""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Shut down ROS2 context."""
        rclpy.shutdown()

    def setUp(self):
        """Create test node for parameter service calls."""
        self.node = rclpy.create_node('test_ads1115_params')

    def tearDown(self):
        """Destroy test node."""
        self.node.destroy_node()

    def test_change_publish_rate(self):
        """Publish_rate should be changeable at runtime."""
        client = self.node.create_client(
            SetParameters, 'ads1115_adc_node/set_parameters')
        self.assertTrue(client.wait_for_service(timeout_sec=10.0))
        request = SetParameters.Request()
        request.parameters = [
            Parameter('publish_rate', value=20.0).to_parameter_msg(),
        ]
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
        self.assertTrue(future.result().results[0].successful)
        self.node.destroy_client(client)


@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    """Verify clean shutdown."""

    def test_exit_code(self, proc_info):
        """Node should exit cleanly."""
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=[0, -2, -15])
