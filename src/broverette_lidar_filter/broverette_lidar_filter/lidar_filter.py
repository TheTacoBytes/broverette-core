# # ONLY FLIP
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import LaserScan

# class LidarFilter(Node):
#     def __init__(self):
#         super().__init__('lidar_filter')
#         self.subscription = self.create_subscription(
#             LaserScan,
#             '/scan',
#             self.lidar_callback,
#             10
#         )
#         self.publisher = self.create_publisher(
#             LaserScan,
#             '/scan_filter',
#             10
#         )

#     def lidar_callback(self, msg: LaserScan):
#         # Create new LaserScan message
#         rotated_scan = LaserScan()

#         # Copy header and timing/angle metadata from the original
#         rotated_scan.header = msg.header
#         # If you'd rather put the current timestamp:
#         # rotated_scan.header.stamp = self.get_clock().now().to_msg()

#         rotated_scan.angle_min = msg.angle_min
#         rotated_scan.angle_max = msg.angle_max
#         rotated_scan.angle_increment = msg.angle_increment
#         rotated_scan.time_increment = msg.time_increment
#         rotated_scan.scan_time = msg.scan_time
#         rotated_scan.range_min = msg.range_min
#         rotated_scan.range_max = msg.range_max

#         # Number of points in the scan
#         n = len(msg.ranges)

#         # Rotate each array by 180 degrees
#         offset = n // 2  # half the array length
#         rotated_scan.ranges = [0.0]*n
#         rotated_scan.intensities = [0.0]*n

#         for i in range(n):
#             rotated_scan.ranges[i] = msg.ranges[(i + offset) % n]
#             rotated_scan.intensities[i] = msg.intensities[(i + offset) % n]

#         # Publish the rotated scan
#         self.publisher.publish(rotated_scan)

# def main(args=None):
#     rclpy.init(args=args)
#     node = LidarFilter()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


# # RIGHT SIDE
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import LaserScan

# class LidarFilter(Node):
#     def __init__(self):
#         super().__init__('lidar_filter')
#         self.subscription = self.create_subscription(
#             LaserScan,
#             '/scan',
#             self.lidar_callback,
#             10
#         )
#         self.publisher = self.create_publisher(
#             LaserScan,
#             '/scan_filter',
#             10
#         )

#     def lidar_callback(self, msg: LaserScan):
#         # 1) Make a new LaserScan message
#         back_scan = LaserScan()

#         # -- Copy header & timing/angle metadata
#         back_scan.header = msg.header
#         # You could stamp with now():
#         # back_scan.header.stamp = self.get_clock().now().to_msg()

#         back_scan.angle_min = msg.angle_min
#         back_scan.angle_max = msg.angle_max
#         back_scan.angle_increment = msg.angle_increment
#         back_scan.time_increment = msg.time_increment
#         back_scan.scan_time = msg.scan_time
#         back_scan.range_min = msg.range_min
#         back_scan.range_max = msg.range_max

#         n = len(msg.ranges)
#         if n == 0:
#             self.get_logger().warn("Received empty LaserScan!")
#             return

#         # 2) Rotate the scan by 180 degrees (half the array length)
#         offset = n // 2
#         rotated_ranges = [0.0] * n
#         rotated_intensities = [0.0] * n

#         for i in range(n):
#             rotated_ranges[i] = msg.ranges[(i + offset) % n]
#             rotated_intensities[i] = msg.intensities[(i + offset) % n]

#         # 3) Keep only the "back half" after rotation.
#         #    Because we rotated, the original "back" ends up in the first half of rotated_*.
#         half_n = n // 2

#         # Slice out the first half: indices [0 .. half_n-1]
#         # That corresponds to the old "back" portion.
#         back_scan.ranges = rotated_ranges[0:half_n]
#         back_scan.intensities = rotated_intensities[0:half_n]

#         # 4) Adjust angle_max so that we correctly publish a 180° window (half of 360°).
#         original_span = (msg.angle_max - msg.angle_min)  # ~ 2*pi
#         half_span = original_span / 2.0  # 180 degrees
#         # We'll keep angle_min the same, set angle_max = angle_min + half_span
#         back_scan.angle_max = back_scan.angle_min + half_span

#         # 5) Publish
#         self.publisher.publish(back_scan)

# def main(args=None):
#     rclpy.init(args=args)
#     node = LidarFilter()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()


import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LidarFilter(Node):
    def __init__(self):
        super().__init__('lidar_filter')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )
        self.publisher = self.create_publisher(
            LaserScan,
            '/scan_filter',
            10
        )

    def lidar_callback(self, msg: LaserScan):
        # Create new LaserScan message
        rotated_scan = LaserScan()

        # Copy header and timing/angle metadata from the original
        rotated_scan.header = msg.header
        # If you'd rather put the current timestamp:
        # rotated_scan.header.stamp = self.get_clock().now().to_msg()

        rotated_scan.angle_min = msg.angle_min
        rotated_scan.angle_max = msg.angle_max
        rotated_scan.angle_increment = msg.angle_increment
        rotated_scan.time_increment = msg.time_increment
        rotated_scan.scan_time = msg.scan_time
        rotated_scan.range_min = msg.range_min
        rotated_scan.range_max = msg.range_max

        # Number of points in the scan
        n = len(msg.ranges)

        # Rotate each array by 180 degrees
        offset = n // 2  # half the array length
        rotated_scan.ranges = [0.0]*n
        rotated_scan.intensities = [0.0]*n

        for i in range(n):
            rotated_scan.ranges[i] = msg.ranges[(i + offset) % n]
            rotated_scan.intensities[i] = msg.intensities[(i + offset) % n]

        # --------------------------------------------------------------------
        # APPEND THESE LINES to keep only the "front" 180° around 0
        # ( i.e. angles from -pi/2 to +pi/2 ), dropping the "back" 180°.
        # --------------------------------------------------------------------

        # 1) We'll slice out the middle half of the data:
        start_i = n // 4         # index corresponding roughly to -pi/2
        end_i   = 3 * n // 4     # index corresponding roughly to +pi/2

        rotated_scan.ranges      = rotated_scan.ranges[start_i:end_i]
        rotated_scan.intensities = rotated_scan.intensities[start_i:end_i]

        # 2) Update the angle_min, angle_max to reflect only front 180°
        rotated_scan.angle_min = -math.pi / 2
        rotated_scan.angle_max =  math.pi / 2
        
        # 3) Recompute angle_increment to match the new array size exactly
        new_n = len(rotated_scan.ranges)
        # Avoid division by zero in pathological cases
        if new_n > 1:
            rotated_scan.angle_increment = (rotated_scan.angle_max - rotated_scan.angle_min) / (new_n - 1)

        # Publish the final "front-only" scan
        self.publisher.publish(rotated_scan)

def main(args=None):
    rclpy.init(args=args)
    node = LidarFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()





# # import rclpy
# # from rclpy.node import Node
# # from sensor_msgs.msg import LaserScan

# # class LidarFilter(Node):
# #     def __init__(self):
# #         super().__init__('lidar_filter')
# #         self.subscription = self.create_subscription(
# #             LaserScan,
# #             '/scan_raw',  # Input topic
# #             self.lidar_callback,
# #             10
# #         )
# #         self.publisher = self.create_publisher(
# #             LaserScan,
# #             '/scan',  # Output topic
# #             10
# #         )

# #     def lidar_callback(self, msg):
# #         # Copy the incoming LaserScan message and set the desired rear 180° range
# #         filtered_scan = LaserScan()
# #         filtered_scan.header = msg.header
# #         filtered_scan.angle_min = 1.57   # Rear-left (90 degrees)
# #         filtered_scan.angle_max = -1.57  # Rear-right (270 degrees or -90 degrees)
# #         filtered_scan.angle_increment = msg.angle_increment
# #         filtered_scan.time_increment = msg.time_increment
# #         filtered_scan.scan_time = msg.scan_time
# #         filtered_scan.range_min = msg.range_min
# #         filtered_scan.range_max = msg.range_max

# #         total_points = len(msg.ranges)

# #         # Calculate start and end indices for rear 180°
# #         start_index = int((filtered_scan.angle_min - msg.angle_min) / msg.angle_increment)
# #         end_index = int((filtered_scan.angle_max - msg.angle_min) / msg.angle_increment)

# #         # Correct for wrap-around (if start_index or end_index goes out of bounds)
# #         start_index = start_index % total_points
# #         end_index = end_index % total_points

# #         # Rearrange indices to ensure rear 180° is selected
# #         if start_index < end_index:
# #             filtered_scan.ranges = msg.ranges[start_index:end_index]
# #             if msg.intensities:
# #                 filtered_scan.intensities = msg.intensities[start_index:end_index]
# #         else:
# #             filtered_scan.ranges = msg.ranges[start_index:] + msg.ranges[:end_index]
# #             if msg.intensities:
# #                 filtered_scan.intensities = msg.intensities[start_index:] + msg.intensities[:end_index]

# #         self.publisher.publish(filtered_scan)

# # def main(args=None):
# #     rclpy.init(args=args)
# #     node = LidarFilter()
# #     rclpy.spin(node)
# #     node.destroy_node()
# #     rclpy.shutdown()

# # if __name__ == '__main__':
# #     main()
