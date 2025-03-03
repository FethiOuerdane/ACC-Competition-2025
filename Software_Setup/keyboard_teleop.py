#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from qcar2_interfaces.msg import MotorCommands, BooleanLeds
from pynput import keyboard
from sensor_msgs.msg import Image
from hal.products.mats import SDCSRoadMap
from cv_bridge import CvBridge
import cv2

class CommandPublisher(Node):
    def __init__(self):
        super().__init__('keyboard_publisher')
        self.command_publisher_ = self.create_publisher(MotorCommands, 'qcar2_motor_speed_cmd', 10)
        self.led_publisher_ = self.create_publisher(BooleanLeds, 'qcar2_led_cmd', 10)
        self.image_subscriber_ = self.create_subscription(
            Image, 
            'camera/color_image', 
            self.image_callback, 
            10
        )
        self.bridge = CvBridge()
        self.throttle = 0.0
        self.steering = 0.0
        self.led_values = [False] * 16  # All LEDs off initially

        # Set up the keyboard listener
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()
        nodeSequence = [10,2,4, 20,22,10]
        roadmap = SDCSRoadMap(leftHandTraffic=False)
        waypointSequence = roadmap.generate_path(nodeSequence)
        initialPose = roadmap.get_node_pose(nodeSequence[0]).squeeze()
        print(waypointSequence)
        # Timer to periodically publish the command
        self.timer_ = self.create_timer(0.1, self.timer_callback)

    def on_press(self, key):
        try:
            if key.char == 'w':
                self.throttle = 0.015  # Forward movement
                self.set_led(8, True)  # Turn on front headlights
                self.set_led(9, True)  # Turn on middle headlights
            elif key.char == 's':
                self.throttle = -0.015  # Reverse movement
                self.set_led(5, True)  # Turn on reverse lights
                self.set_led(8, False)  # Turn off front headlights
                self.set_led(9, False)  # Turn off middle headlights
            elif key.char == 'a':
                self.steering = 0.3  # Turn left
                self.set_led(12, True)  # Left turn signal
                self.set_led(14, False)  # Right turn signal off
            elif key.char == 'd':
                self.steering = -0.3  # Turn right
                self.set_led(12, False)  # Left turn signal off
                self.set_led(14, True)  # Right turn signal
            elif key.char == 'q':
                self.throttle = 0
                self.steering = 0
                self.set_all_leds(False)  # Turn off all LEDs when stopping
        except AttributeError:
            pass

    def on_release(self, key):
        if key == keyboard.Key.esc:
            # Stop the listener when the ESC key is pressed
            self.listener.stop()
        else:
            # Reset throttle and steering to 0 when the key is released
            if key.char in ['w', 's']:
                self.throttle = 0.0
                self.set_led(5, False)  # Turn off reverse lights
            elif key.char in ['a', 'd']:
                self.steering = 0.0
                self.set_led(12, False)  # Turn off left turn signal
                self.set_led(14, False)  # Turn off right turn signal

    def set_led(self, led_index, state):
        """Set a specific LED by its index to the desired state (True/False)."""
        if 0 <= led_index < len(self.led_values):
            self.led_values[led_index] = state
            self.publish_leds()

    def set_all_leds(self, state):
        """Set all LEDs to the same state (True/False)."""
        self.led_values = [state] * 16  # Set all LED states at once
        self.publish_leds()

    def publish_leds(self):
        """Publish the LED state to the appropriate topic."""
        led_command = BooleanLeds()
        led_command.led_names = [
            "left_outside_brake_light", "left_inside_brake_light", "right_inside_brake_light", "right_outside_brake_light",
            "left_reverse_light", "right_reverse_light", "left_rear_signal", "right_rear_signal",
            "left_outside_headlight", "left_middle_headlight", "left_inside_headlight", "right_inside_headlight",
            "right_middle_headlight", "right_outside_headlight", "left_front_signal", "right_front_signal"
        ]
        led_command.values = self.led_values
        self.led_publisher_.publish(led_command)

    def timer_callback(self):
        # Ensure throttle and steering are within valid float ranges
        self.steering = float(self.steering)
        self.throttle = float(self.throttle)

        # Clamp the values to ensure they are within valid ranges
        self.steering = max(-1.0, min(1.0, self.steering))  # Steering should be between -1.0 and 1.0
        self.throttle = max(-0.2, min(0.2, self.throttle))  # Throttle should be between -0.2 and 0.2

        # Create the motor command message
        motor_command = MotorCommands()
        motor_command.motor_names = ['steering_angle', 'motor_throttle']
        motor_command.values = [self.steering, self.throttle]  # Values should be floats
        self.command_publisher_.publish(motor_command)

    def image_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Check if the image has valid dimensions
            if cv_image is None or cv_image.size == 0:
                # self.get_logger().warn("Received an invalid image")
                return

            # Display the image
            cv2.imshow("Camera Feed", cv_image)
            cv2.waitKey(1)  # Display frame for 1ms to show the image
        except Exception as e:
            self.get_logger().error(f"Error in image callback: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = CommandPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
