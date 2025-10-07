
#Tasks:
# Your complete driver should:

# Read in and parse a GPGGA string into latitude, longitude, UTC, and HDOP
# Convert latitude and longitude into UTM values using the UTM package specified in "software" above
# Publish lat/lon/UTM data in a custom message named Customgps.msg with the following fields and data types:
# header (type: Header)
# header.frame_id should be 'GPS1_Frame' (type: string)
# header.stamp.sec should be UTC from the GPS sensor converted to epoch time. Please do not use system time for UTC, but you will need to use system time to get epoch time to the current day. (type: uint32)
# header.stamp.nsec should be remaining nanoseconds (type: uint32). You may have a value of 0 for this, depending on your GPS unit.
# latitude (type: float64)
# longitude (type: float64)
# altitute (type: float64)
# utm_easting (type: float64)
# utm_northing (type: float64)
# zone (type: uint8)
# letter (type: string)
# hdop (type: float64)
# gpgga_read (type: string, this should be the actual string you read from the puck)
# Publish the custom message to topic named '/gps'


#imports:
import serial   #For reading data from the serial port
import rclpy    #ROS2 Dependency
from rclpy.node import Node     #Node Object
from gps_interfaces.msg import Customgps    #Cutom interface
from std_msgs.msg import String
from .decoders import GPGGAPositionData as PositionData


# Global Constants:

PUBLISH_TOPIC = 'gps'
SUB_TOPIC = 'gps2'
MAX_MSG = 10
FREQUENCY = 0.1  #seconds
BAUD_RATE = 4800
TIMEOUT = 0.1   #10Hz
DEFAULT_PORT = '/dev/ttyUSB0'

#*******************************************************************

#GPS Publishing Node class. Will grab data from the serial port and publish
class GPSPublisher(Node):
    def __init__(self, port_address=DEFAULT_PORT):
        super().__init__('gps_publisher')
        self.__pub_topic = PUBLISH_TOPIC
        self.__publisher = self.create_publisher(Customgps,self.__pub_topic,MAX_MSG)
        # self.__serial_port_address = port_address
        self.declare_parameter('port',port_address)
        self.__serial_port_address = self.get_parameter('port').get_parameter_value().string_value
        self.__serial_port = serial.Serial(self.__serial_port_address, BAUD_RATE,timeout=TIMEOUT)
        self.get_logger().info(f"Listening to {self.__serial_port_address}\nPosting to {self.__pub_topic}")
        self.__poll_serial_port()

#------------------------------------------

    def __poll_serial_port(self):
        while rclpy.ok():
            if self.__serial_port.in_waiting > 0:
                gpgga_read = self.__serial_port.readline().decode('utf-8')
                my_position = PositionData(gpgga_read)
                if my_position.is_ok():
                    self.publish_message(my_position)
#------------------------------------------

    def close_ports(self):
        self.__serial_port.close()
#------------------------------------------

    def publish_message(self, message:PositionData):
        msg = Customgps()
        msg.header.frame_id = message.get_header()
        msg.header.stamp.sec = int(message.get_utc())
        msg.header.stamp.nanosec = int((message.get_utc()%1)*1000000000)
        msg.latitude = message.get_lat()
        msg.longitude = message.get_lon()
        msg.altitude = message.get_alt()
        msg.utm_easting = message.get_utmE()
        msg.utm_northing = message.get_utmN()
        msg.zone = message.get_zone()
        msg.letter = message.get_zone_letter()
        msg.hdop = message.get_hdop()
        msg.gpgga_read = message.get_read_string()
        
        self.__publisher.publish(msg)
        self.get_logger().info(f"Publishing gps Data:\n{message}")

#*******************************************************************

#Listens to a ROS channel for a Customgps, uses the GPGGA data and publishes new Customgps message on a different channel
#(Useful for correcting inaccurate data recorded in a ROS bag)
class GPSListenPublisher(Node):
    def __init__(self, sub_topic=SUB_TOPIC, pub_topic=PUBLISH_TOPIC):
        super().__init__('gps_publisher')
        self.__pub_topic = pub_topic
        self.__sub_topic = sub_topic
        self.__publisher = self.create_publisher(Customgps,self.__pub_topic,MAX_MSG)
        self.__subscription = self.create_subscription(
            Customgps,
            self.__sub_topic,
            self.__echo_gps_from_topic,
            MAX_MSG
        )
        self.get_logger().info(f"Listening to {self.__sub_topic}\nPosting to {self.__pub_topic}")
        self.__subscription

#------------------------------------------
    
    def __echo_gps_from_topic(self, message:Customgps):
        self.get_logger().info(f"GPGGA message: {message.gpgga_read}")
        my_position = PositionData(message.gpgga_read)
        if my_position.is_ok():
            self.publish_message(my_position)


#------------------------------------------

    def publish_message(self, message:PositionData):
        msg = Customgps()
        msg.header.frame_id = message.get_header()
        msg.header.stamp.sec = int(message.get_utc())
        msg.header.stamp.nanosec = int((message.get_utc()%1)*1000000000)
        msg.latitude = message.get_lat()
        msg.longitude = message.get_lon()
        msg.altitude = message.get_alt()
        msg.utm_easting = message.get_utmE()
        msg.utm_northing = message.get_utmN()
        msg.zone = message.get_zone()
        msg.letter = message.get_zone_letter()
        msg.hdop = message.get_hdop()
        msg.gpgga_read = message.get_read_string()
        
        self.__publisher.publish(msg)
        self.get_logger().info(f"Publishing gps Data:\n{message}")
#------------------------------------------
    def close_ports(self):
        pass
#*******************************************************************
#*******************************************************************

def main(args=None):

    rclpy.init(args=args)

    gps_publisher = GPSPublisher(DEFAULT_PORT)
    # gps_publisher = GPSListenPublisher()
    rclpy.spin(gps_publisher)

    gps_publisher.close_ports()
    gps_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

