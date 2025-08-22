import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LidarFilter(Node):
    def __init__(self):
        super().__init__('lidar_filter')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan_raw',  # Input topic
            self.lidar_callback,
            10
        )
        self.publisher = self.create_publisher(
            LaserScan,
            '/scan',  # Output topic
            10
        )

    def lidar_callback(self, msg):
        # Create a new LaserScan message
        filtered_scan = LaserScan()
        filtered_scan.header = msg.header

        # Set metadata for the desired rear 180° range
        filtered_scan.angle_min = 1.57  # Rear-left (90 degrees)
        filtered_scan.angle_max = -1.57  # Rear-right (270 degrees or -90 degrees)
        filtered_scan.range_min = msg.range_min
        filtered_scan.range_max = msg.range_max

        # Calculate total points in the original scan
        total_points = len(msg.ranges)

        # Calculate start and end indices for the rear 180°
        start_index = int((filtered_scan.angle_min - msg.angle_min) / msg.angle_increment)
        end_index = int((filtered_scan.angle_max - msg.angle_min) / msg.angle_increment)

        # Handle wrap-around indices
        start_index = start_index % total_points
        end_index = end_index % total_points

        # Extract the rear 180° ranges
        if start_index < end_index:
            filtered_ranges = msg.ranges[start_index:end_index]
        else:
            filtered_ranges = msg.ranges[start_index:] + msg.ranges[:end_index]

        # Update angle_increment to match the filtered data
        filtered_scan.angle_increment = (filtered_scan.angle_max - filtered_scan.angle_min) / (len(filtered_ranges) - 1)

        # Assign filtered ranges
        filtered_scan.ranges = filtered_ranges

        # If intensities exist, assign them too
        if msg.intensities:
            if start_index < end_index:
                filtered_scan.intensities = msg.intensities[start_index:end_index]
            else:
                filtered_scan.intensities = msg.intensities[start_index:] + msg.intensities[:end_index]

        # Publish the filtered scan
        self.publisher.publish(filtered_scan)

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
