#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import subprocess
import os
from datetime import datetime


class MapSaverService(Node):
    def __init__(self):
        super().__init__('map_saver_service')
        self.declare_parameter('map_save_dir', os.path.expanduser('~/maps'))
        self.map_dir = self.get_parameter('map_save_dir').value
        os.makedirs(self.map_dir, exist_ok=True)

        self.srv = self.create_service(Trigger, '/explore/save', self.save_callback)
        self.get_logger().info(f'MapSaverService ready. Maps -> {self.map_dir}')

    def save_callback(self, request, response):
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        map_path  = os.path.join(self.map_dir, f'map_{timestamp}')
        cmd = ['ros2', 'run', 'nav2_map_server', 'map_saver_cli',
               '-f', map_path, '--ros-args', '-p', 'use_sim_time:=true']
        try:
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=15)
            if result.returncode == 0:
                response.success = True
                response.message = f'Map saved: {map_path}.pgm'
                self.get_logger().info(response.message)
            else:
                response.success = False
                response.message = f'Save failed: {result.stderr}'
        except subprocess.TimeoutExpired:
            response.success = False
            response.message = 'Map saver timeout!'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MapSaverService()
    rclpy.spin(node)


if __name__ == '__main__':
    main()