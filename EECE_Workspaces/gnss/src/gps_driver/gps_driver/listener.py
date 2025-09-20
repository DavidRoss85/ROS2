#This is a listener built specifically to listen to Customgps.msg data on a channel and write it to a csv file

import rclpy
from rclpy.node import Node
from gps_interfaces.msg import Customgps

#Global Constants:
FILE_HEADER = "HEAD_FRAME,UTC_SEC,UTC_NANO,LAT,LON,ALT,UTME,UTMN,ZONE,LETTER,HDOP,STRING\n"
DEFAULT_FILENAME = "~/gps_data.csv"

class GPSListener(Node):
    def __init__(self,filename=DEFAULT_FILENAME):
        super().__init__('gps_recorder')

        self.filename = filename
        with open(filename, 'w') as file:
            file.write(FILE_HEADER)

        self.subscription = self.create_subscription(
            Customgps,
            'gps',
            self.listener_callback,
            10
        )
        self.subscription

    def listener_callback(self,message:Customgps):
        self.get_logger().info(f"Recording: {message.gpgga_read}")
        self.write_to_CSV(message)

    def write_to_CSV(self,message:Customgps):
        with open(self.filename,'a') as file:
            formatted_message = (
                f"{message.header.frame_id},"
                +f"{message.header.stamp.sec},"
                +f"{message.header.stamp.nanosec},"
                +f"{message.latitude},"
                +f"{message.longitude},"
                +f"{message.altitude},"
                +f"{message.utm_easting},"
                +f"{message.utm_northing},"
                +f"{message.zone},"
                +f"{message.letter},"
                +f"{message.hdop},"
                +f"{message.gpgga_read}\n"
            )
            file.write(formatted_message)


def main(args=None):
    rclpy.init(args=args)

    file_writer = GPSListener(DEFAULT_FILENAME)

    rclpy.spin(file_writer)

    file_writer.destroy_node()
    rclpy.shutdown

if __name__ == '__main__':
    main()