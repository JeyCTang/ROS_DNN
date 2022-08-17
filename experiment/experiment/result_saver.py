# Package: experiment
# Filename: result_saver.py

"""
This node is going to save the detection result as json format and store on our disk with .csv file.

The node will process the service request from movement_controller node, so it will start and stop recording the result
The node will subscribe the message published by the detection node, and process the message as json format. Finally, it will receive the result.
"""
import rclpy
from rclpy.node import Node
from project_interfaces.srv import RecordSwitch
from project_interfaces.msg import DetectResults

import pandas as pd 
import json


class resultSaver(Node):

    def __init__(self):
        super().__init__("result_saver")
        # Create service to receive the signal to start recording
        self.srv = self.create_service(RecordSwitch, 'record_switch', self.service_callback)
        self.get_logger().info("The server is created, waiting for request from control node...")
        # Create subscriber to receive the detect result
        # self.sub = self.create_subscription(DetectResults, '/detect_result', self.sub_callback, 10)
        # self.sub    # Prevent unuseful subscription
        self.categories = []
        self.scores = []

        # self.declare_parameter('robot_position', '(0, -4)')
        # self.declare_parameter('item_name', 'bus')

    def sub_callback(self, msg):
        self.get_logger().info(f"Get category: {msg.category}, corresponding score: {msg.score}")
        self.categories.append(msg.category)
        self.scores.append(float(msg.score))


    def service_callback(self, request, response):
        if request.switcher:
            self.get_logger().info(f"Start recording the result")

            # Create subscriber to receive the detect result
            self.sub = self.create_subscription(DetectResults, '/detect_result', self.sub_callback, 10)
            self.sub    # Prevent unuseful subscription
        elif request.switcher == False:
            self.get_logger().info(f"Stop recording the result")
            # TODO: After stopping recording, save the results as csv
            results_dic = {"category": self.categories, "score": self.scores}
            df = pd.DataFrame(data=results_dic)
            df.to_csv('results_performance.csv')
            # After saving results, destory the subscriber
            self.sub.destroy()

        return response


def main():
    rclpy.init()

    service = resultSaver()

    rclpy.spin(service)

    rclpy.shutdown()

if __name__=='__main__':
    main()