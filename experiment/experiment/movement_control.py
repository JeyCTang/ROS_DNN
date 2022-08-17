# Package: experiment
# Filename: movement_control.py

import rclpy
from geometry_msgs.msg import Twist
from math import pi
from rclpy.node import Node
from project_interfaces.srv import RecordSwitch


class MoveController(Node):
    def __init__(self):
        """
        This node is going to control the movement of the robot, it will turn the robot
        90 deg/1.5707 rad counter-clockwise
        """
        # Initiate the Node class constructor and give it a name
        super().__init__('movement_controller')
        # Create the volicity publisher
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        self.client = self.create_client(RecordSwitch, 'record_switch')
        while not self.client.wait_for_service(timeout_sec=0.5):
            self.get_logger().info('service not available, waiting....')
        self.get_logger().info("Service 'record_switch' is connected!")

        self.client_futures = []

    def send_request(self, cmd_code):
        req = RecordSwitch.Request()
        req.switcher = cmd_code
        future = self.client.call_async(req)
        return future

    def move(self):
        # The angle of rad we are going to turn
        relative_angle = float(90 * pi / 180)

        # Initiate Twist object, it is the message type we are going to publish
        msg = Twist()

        # We won't use linear components, we will just let the robot turning on z axis by 90 deg
        msg.linear.x = float(0)
        msg.linear.y = float(0)
        msg.linear.z = float(0)
        msg.angular.x = float(0)
        msg.angular.y = float(0)

        # The robot will turn 4 times, when starting the turning, the client will send the reqeust to
        # save to let it start recording
        self.get_logger().info("Sending start command to result saver...")
        cmd_code = True
        future = self.send_request(cmd_code)
        self.client_futures.append(future)
        
        # Turning process
        for i in range(0, 4):
            self.get_logger().info(f'The {i+1} time detection')
            if i % 2 == 0:
                angular_speed = float(0.182)
                msg.angular.z = angular_speed
            else:
                angular_speed = -float(0.182)
                msg.angular.z = angular_speed

            # Setting the current time for distance calculation
            t0 = self.get_clock().now().to_msg().sec
            current_angle = float(0.0)

            while (current_angle < relative_angle):
                self.velocity_publisher.publish(msg)
                t1 = self.get_clock().now().to_msg().sec
                current_angle = abs(angular_speed) * (t1-t0)
            
            # Forcing the robot to stop
            msg.angular.z = float(0.0)
            self.velocity_publisher.publish(msg)

        # After turning the client will send the req to let the saver stop recording
        self.get_logger().info("Sending stop command to result saver...")
        cmd_code = False
        future = self.send_request(cmd_code)
        self.client_futures.append(future)

        self.get_logger().info("Done!")
        

def main():
    # Initiate the ros client library
    rclpy.init()

    # Create the node
    movement_controller = MoveController()

    # Start to move the robot
    movement_controller.move()

    # Destory the node after the movement
    movement_controller.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__=='__main__':
    main()
            


