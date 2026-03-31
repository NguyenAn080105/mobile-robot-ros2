#!/usr/bin/env python3
"""
Đọc packet từ STM32 qua UART, tính odometry, publish /odom bằng Threading
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
import serial
import struct
import math
import threading
import queue
from typing import Optional

# ── Packet format (10 bytes, little-endian) ────────────────────────────────
PACKET_SIZE   = 8
START_FRAME   = 0xABCD
PACKET_FORMAT = '<HHHH'   # start(u16), omegaL(i16), omegaR(i16), checksum
OMEGA_SCALE   = 100.0     # counts → rad/s

# ── Robot parameters ───────────────────────────────────────────────────────
WHEEL_RADIUS  = 0.08255     # m  (từ URDF)
WHEEL_BASE    = 0.42109     # m  (wheel_separation)

class WheelOdomNode(Node):
    def __init__(self):
        super().__init__('wheel_odom_node')

        # 1. Parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('publish_tf', False) # EKF sẽ lo việc này
        
        port = self.get_parameter('serial_port').value
        baud = self.get_parameter('baud_rate').value
        self.publish_tf_flag = self.get_parameter('publish_tf').value

        # 2. Odometry Variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

        # 3. ROS Publishers & TF
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        if self.publish_tf_flag:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # 4. Setup Serial
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            self.get_logger().info(f"Connected to STM32 on {port} at {baud} baud.")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to connect to STM32: {e}")
            raise e

        # 5. Threading & Queue Setup
        self._pkt_queue = queue.Queue(maxsize=50)
        self._stop_event = threading.Event()
        
        # Khởi chạy Thread đọc Serial (Daemon = True để tự tắt khi node chết)
        self._serial_thread = threading.Thread(target=self._serial_reader_loop, daemon=True)
        self._serial_thread.start()

        # 6. ROS Timer (chỉ để lấy data từ Queue và publish) - 100Hz = 0.01s
        self.create_timer(0.01, self._process_queue)

    def _serial_reader_loop(self):
        """Thread riêng rẽ: Đọc và parse dữ liệu Serial liên tục, không block ROS."""
        buffer = bytearray()
        
        while not self._stop_event.is_set():
            if not self.ser.is_open:
                break
                
            try:
                # Đọc tất cả data đang có trong buffer của OS
                waiting = self.ser.in_waiting or 1
                data = self.ser.read(waiting)
                if data:
                    buffer.extend(data)
                    
                    # Tìm và parse packet
                    while len(buffer) >= PACKET_SIZE:
                        if buffer[0] == (START_FRAME & 0xFF) and buffer[1] == ((START_FRAME >> 8) & 0xFF):
                            packet = buffer[:PACKET_SIZE]
                            
                            try:
                                _, wR_raw, wL_raw, _ = struct.unpack(PACKET_FORMAT, packet)
                                
                                wL_raw = wL_raw - 32768
                                wR_raw = wR_raw - 32768

                                # Convert sang rad/s
                                omega_L = wL_raw / OMEGA_SCALE
                                omega_R = wR_raw / OMEGA_SCALE
                                
                                pkt_data = {'omega_L': omega_L, 'omega_R': omega_R}
                                
                                # Đẩy vào Queue an toàn
                                if self._pkt_queue.full():
                                    self._pkt_queue.get_nowait() # Bỏ packet cũ nhất chống trễ
                                self._pkt_queue.put_nowait(pkt_data)
                                
                            except Exception as e:
                                self.get_logger().warn(f"Packet parse error: {e}")
                                
                            buffer = buffer[PACKET_SIZE:] # Cắt bỏ packet đã xử lý
                        else:
                            buffer.pop(0) # Trượt buffer 1 byte nếu không tìm thấy Header
            except Exception as e:
                self.get_logger().error(f"Serial read error: {e}")
                break

    def _process_queue(self):
        """ROS Timer callback @100Hz: RẤT NHANH, KHÔNG BLOCK."""
        while not self._pkt_queue.empty():
            try:
                pkt = self._pkt_queue.get_nowait()
                self._update_odom(pkt['omega_L'], pkt['omega_R'])
            except queue.Empty:
                break

    def _update_odom(self, omega_L, omega_R):
        """Tính toán tịnh tiến (v) và quay (w) từ vận tốc góc của 2 bánh."""
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        if dt <= 0:
            return

        # Tính v và w cho differential drive
        v_L = omega_L * WHEEL_RADIUS
        v_R = omega_R * WHEEL_RADIUS
        
        v = (v_R + v_L) / 2.0
        w = (v_R - v_L) / WHEEL_BASE

        # Cập nhật vị trí x, y, theta
        delta_x = v * math.cos(self.theta) * dt
        delta_y = v * math.sin(self.theta) * dt
        delta_theta = w * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta

        self._publish_odom_and_tf(v, w, current_time)

    def _publish_odom_and_tf(self, v, w, current_time):
        """Publish message lên /odom và TF nếu được phép."""
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)

        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w

        self.odom_pub.publish(odom)

        # Chú ý: Vì bạn dùng robot_localization (EKF) nên publish_tf nên bằng False
        if self.publish_tf_flag:
            tf = TransformStamped()
            tf.header = odom.header
            tf.child_frame_id = 'base_footprint'
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(tf)

    def destroy_node(self):
        """Cleanup đúng cách khi tắt Node."""
        self.get_logger().info("Shutting down Wheel Odom Node...")
        self._stop_event.set()
        self._serial_thread.join(timeout=2.0)
        
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
            
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = WheelOdomNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()