#This is a listener built specifically to listen to Customrtk.msg data on a channel and write it to a csv file

import rclpy
from rclpy.node import Node
from gps_interfaces.msg import Customrtk

#Global Constants:
FILE_HEADER = "HEAD_FRAME,UTC_SEC,UTC_NANO,LAT,LON,ALT,UTME,UTMN,ZONE,LETTER,HDOP,FIX_Q,STRING,STRING2,STRING3,STRING4,STRING5,STRING6,STRING7,STRING8,STRING9,STRING10,STRING11,STRING12,STRING13,STRING14,STRING15\n"
DEFAULT_FILENAME = "./gps_data/gps_data.csv"
DEFAULT_TOPIC = 'gps'

class GPSListener(Node):
    def __init__(self,filename=DEFAULT_FILENAME,topic=DEFAULT_TOPIC):
        super().__init__('gps_recorder')

        self.filename = filename
        self.__topic = topic
        with open(filename, 'w') as file:
            file.write(FILE_HEADER)

        self.subscription = self.create_subscription(
            Customrtk,
            self.__topic,
            self.listener_callback,
            10
        )
        self.get_logger().info(f"Recorder listening to {self.__topic}\n")
        self.subscription

    def listener_callback(self,message:Customrtk):
        self.get_logger().info(f"Recording: {message.gngga_read}")
        self.write_to_CSV(message)

    def write_to_CSV(self,message:Customrtk):
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
                +f"{message.fix_quality},"
                +f"{message.gngga_read.rstrip()}\n"
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