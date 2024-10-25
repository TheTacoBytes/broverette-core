#!/usr/bin/env python3

import board
import digitalio
import adafruit_ssd1306
from PIL import Image, ImageDraw, ImageFont
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import socket

class OLEDBatteryDisplay(Node):
    def __init__(self):
        super().__init__('oled_battery_display')

        # Subscribe to the /voltage topic
        self.subscription = self.create_subscription(
            Float32,
            '/voltage',
            self.battery_callback,
            10
        )
        self.subscription  # prevent unused variable warning

        # Initialize OLED display
        self.oled_reset = digitalio.DigitalInOut(board.D4)
        self.OLED_WIDTH = 128
        self.OLED_HEIGHT = 32
        self.i2c = board.I2C()
        self.disp = adafruit_ssd1306.SSD1306_I2C(
            self.OLED_WIDTH, self.OLED_HEIGHT, self.i2c, addr=0x3C, reset=self.oled_reset
        )
        self.disp.fill(0)
        self.disp.show()

        # Create blank image for drawing with mode '1' for 1-bit color
        self.image = Image.new('1', (self.OLED_WIDTH, self.OLED_HEIGHT))
        self.draw = ImageDraw.Draw(self.image)

        # Load default font
        self.font = ImageFont.load_default()

        # Set battery percentage as placeholder until callback receives data
        self.percentage = 0.0

        # Initialize IP address
        self.ip_address = self.get_ip_address()

        # Create a timer to periodically check and update IP address (every 10 seconds)
        self.ip_timer = self.create_timer(10.0, self.check_ip_update)

        # Update display initially
        self.update_display()

    def battery_callback(self, msg):
        # Update percentage when new data is received
        self.percentage = self.calculate_battery_percentage(msg.data)

        # Update the display with the new data
        self.update_display()

    def calculate_battery_percentage(self, voltage):
        min_voltage = 9.52
        max_voltage = 12.6

        # Ensure voltage does not go below min or above max
        voltage = max(min_voltage, min(voltage, max_voltage))

        percentage = ((voltage - min_voltage) / (max_voltage - min_voltage)) * 100
        return percentage

    def get_ip_address(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            # Doesn't have to be reachable
            s.connect(('10.254.254.254', 1))
            ip = s.getsockname()[0]
        except Exception:
            ip = 'No IP'
        finally:
            s.close()
        return ip

    def check_ip_update(self):
        # Periodically check for IP address change
        new_ip = self.get_ip_address()
        if new_ip != 'No IP':
            # Once we get a valid IP, update the display and stop further IP checks
            self.ip_address = new_ip
            self.get_logger().info(f'IP Address acquired: {self.ip_address}')
            self.update_display()
            # Stop checking the IP once we have a valid one
            self.ip_timer.cancel()

    def update_display(self):
        # Clear the display buffer
        self.draw.rectangle((0, 0, self.OLED_WIDTH, self.OLED_HEIGHT), outline=0, fill=0)

        # Draw IP address
        self.draw.text((0, 0), f"IP: {self.ip_address}", font=self.font, fill=255)

        # Draw battery icon
        padding = 2
        battery_width = self.OLED_WIDTH - 2 * padding
        battery_height = (self.OLED_HEIGHT // 2) - 2 * padding
        battery_y_start = self.OLED_HEIGHT // 2
        battery_outline = [padding, battery_y_start, battery_width + padding, battery_y_start + battery_height]

        nipple_width = 4
        nipple_height = battery_height // 3
        nipple_x_start = battery_outline[2]
        nipple_y_start = battery_y_start + (battery_height - nipple_height) // 2
        battery_nipple = [nipple_x_start, nipple_y_start, nipple_x_start + nipple_width, nipple_y_start + nipple_height]

        inner_width = int((battery_width - 2) * self.percentage / 100)
        inner_rectangle = [padding + 1, battery_y_start + 1, padding + 1 + inner_width, battery_y_start + battery_height - 1]

        self.draw.rectangle(battery_outline, outline=255, fill=0)
        self.draw.rectangle(battery_nipple, outline=255, fill=255)
        self.draw.rectangle(inner_rectangle, outline=255, fill=255)

        # Display image on OLED
        self.disp.image(self.image)
        self.disp.show()


def main(args=None):
    rclpy.init(args=args)
    oled_battery_display = OLEDBatteryDisplay()
    rclpy.spin(oled_battery_display)
    oled_battery_display.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()












# #!/usr/bin/env python3

# import board
# import digitalio
# import adafruit_ssd1306
# from PIL import Image, ImageDraw, ImageFont
# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Float32
# import socket

# class OLEDBatteryDisplay(Node):
#     def __init__(self):
#         super().__init__('oled_battery_display')

#         # Subscribe to the /voltage topic
#         self.subscription = self.create_subscription(
#             Float32,
#             '/voltage',
#             self.battery_callback,
#             10
#         )
#         self.subscription  # prevent unused variable warning

#         # Initialize OLED display
#         self.oled_reset = digitalio.DigitalInOut(board.D4)
#         self.OLED_WIDTH = 128
#         self.OLED_HEIGHT = 32
#         self.i2c = board.I2C()
#         self.disp = adafruit_ssd1306.SSD1306_I2C(
#             self.OLED_WIDTH, self.OLED_HEIGHT, self.i2c, addr=0x3C, reset=self.oled_reset
#         )
#         self.disp.fill(0)
#         self.disp.show()

#         # Create blank image for drawing with mode '1' for 1-bit color
#         self.image = Image.new('1', (self.OLED_WIDTH, self.OLED_HEIGHT))
#         self.draw = ImageDraw.Draw(self.image)

#         # Load default font
#         self.font = ImageFont.load_default()

#         # Set battery percentage as placeholder until callback receives data
#         self.percentage = 0.0

#         # Get IP address
#         self.ip_address = self.get_ip_address()

#         # Update display initially
#         self.update_display()

#     def battery_callback(self, msg):
#         # Update percentage when new data is received
#         self.percentage = self.calculate_battery_percentage(msg.data)

#         # Update the display with the new data
#         self.update_display()

#     def calculate_battery_percentage(self, voltage):
#         min_voltage = 9.52
#         max_voltage = 12.6

#         # Ensure voltage does not go below min or above max
#         voltage = max(min_voltage, min(voltage, max_voltage))

#         percentage = ((voltage - min_voltage) / (max_voltage - min_voltage)) * 100
#         return percentage

#     def get_ip_address(self):
#         s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#         try:
#             # Doesn't have to be reachable
#             s.connect(('10.254.254.254', 1))
#             ip = s.getsockname()[0]
#         except Exception:
#             ip = 'No IP'
#         finally:
#             s.close()
#         return ip

#     def update_display(self):
#         # Clear the display buffer
#         self.draw.rectangle((0, 0, self.OLED_WIDTH, self.OLED_HEIGHT), outline=0, fill=0)

#         # Draw IP address
#         self.draw.text((0, 0), f"IP: {self.ip_address}", font=self.font, fill=255)

#         # Draw battery icon
#         padding = 2
#         battery_width = self.OLED_WIDTH - 2 * padding
#         battery_height = (self.OLED_HEIGHT // 2) - 2 * padding
#         battery_y_start = self.OLED_HEIGHT // 2
#         battery_outline = [padding, battery_y_start, battery_width + padding, battery_y_start + battery_height]

#         nipple_width = 4
#         nipple_height = battery_height // 3
#         nipple_x_start = battery_outline[2]
#         nipple_y_start = battery_y_start + (battery_height - nipple_height) // 2
#         battery_nipple = [nipple_x_start, nipple_y_start, nipple_x_start + nipple_width, nipple_y_start + nipple_height]

#         inner_width = int((battery_width - 2) * self.percentage / 100)
#         inner_rectangle = [padding + 1, battery_y_start + 1, padding + 1 + inner_width, battery_y_start + battery_height - 1]

#         self.draw.rectangle(battery_outline, outline=255, fill=0)
#         self.draw.rectangle(battery_nipple, outline=255, fill=255)
#         self.draw.rectangle(inner_rectangle, outline=255, fill=255)

#         # Display image on OLED
#         self.disp.image(self.image)
#         self.disp.show()


# def main(args=None):
#     rclpy.init(args=args)
#     oled_battery_display = OLEDBatteryDisplay()
#     rclpy.spin(oled_battery_display)
#     oled_battery_display.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()
