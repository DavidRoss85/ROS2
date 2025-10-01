
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
import  utm     #for calculating UTM (Easting Northing etc)
import time     #Time functions
import serial   #For reading data from the serial port
import rclpy    #ROS2 Dependency
from rclpy.node import Node     #Node Object
from gps_interfaces.msg import Customgps    #Cutom interface
from std_msgs.msg import String




# Global Constants:
SAMPLE_DATA = "$GPGGA,202530.00,5109.0262,N,11401.8407,W,5,40,0.5,1097.36,M,-17.00,M,18,TSTR*61" #Sample GPGGA String

GPGGA_HEADER = "$GPGGA" #Valid header

# -- Position of data in list:
HEADER_INDEX = 0
UTC_INDEX = 1
LAT_INDEX = 2
LAT_DIR_INDEX = 3
LON_INDEX = 4
LON_DIR_INDEX = 5
QUALITY_INDEX = 6
NUM_SATS_INDEX = 7
HDOP_INDEX = 8
ALT_INDEX = 9
ALT_UNIT_INDEX = 10
UND_INDEX = 11
UND_UNIT_INDEX = 12
AGE_INDEX = 13
STN_ID_INDEX = 14
CHK_INDEX = 15

PUBLISH_TOPIC = 'gps'
SUB_TOPIC = 'gps2'
MAX_MSG = 10
FREQUENCY = 0.1  #seconds
BAUD_RATE = 4800
TIMEOUT = 0.1   #10Hz
DEFAULT_PORT = '/dev/ttyUSB0'

HOUR_TO_SECOND_FACTOR = 3600
MINUTE_TO_SECOND_FACTOR = 60
#*******************************************************************
#*******************************************************************
#Position Data class... Automatically converts a GPGGA string on creation
class PositionData():
    def __init__(self, string_data):
        self.__header = "GPS1_Frame"
        self.__utc = 0.0
        self.__lat = 0.0
        self.__lon = 0.0
        self.__alt = 0.0
        self.__utmE = 0.0
        self.__utmN = 0.0
        self.__zone = 0
        self.__letter = ""
        self.__hdop = 0.0
        self.__read_string = string_data
        self.__ok = False
        self.translate_string(string_data)
#------------------------------------------

    #Verify string is a GPGGA format
    def is_GPGGA_string(self,input_string):
        if input_string.startswith(GPGGA_HEADER):
            return True
        return False
#------------------------------------------
    # Split String into parts:
    def split_GPGGA_string(self,input_string):
        string_list = input_string.split(",")
        for index, item in enumerate(string_list):
            if item == '' or item == None:
                string_list[index] = '0'
        return string_list
#------------------------------------------

    def convert_degMins_to_degDec(self,value):
        #Convert from DDmm.mmmm/DDDmm.mmmm to DD.dddd
        minute_value = float(value)%100
        degree_whole_value = (value-minute_value)/100
        degree_fraction_value = minute_value/60
        return degree_whole_value + degree_fraction_value
#------------------------------------------

    def negate_by_hemisphere(self, degrees, hemisphere):
        if hemisphere.upper() == "W" or hemisphere.upper() == "S":
            return degrees *-1
        return degrees
#------------------------------------------

    def convert_utc_string_to_seconds(self,time_in_UTC):
        converted_time_in_UTC = (float(time_in_UTC[0:2])*HOUR_TO_SECOND_FACTOR)+ (float(time_in_UTC[2:4])*MINUTE_TO_SECOND_FACTOR) + float(time_in_UTC[4:])
        return converted_time_in_UTC
#------------------------------------------

    def convert_UTC_to_epoch(self,time_in_UTC):
        now = time.localtime() #Current time
        today_start = time.struct_time((
            now.tm_year,
            now.tm_mon,
            now.tm_mday,
            0,0,0,
            now.tm_wday,
            now.tm_yday,
            now.tm_isdst
        ))
        
        epoch_in_secs = float(time.mktime(today_start)) + self.convert_utc_string_to_seconds(time_in_UTC)
        return epoch_in_secs
#------------------------------------------

    def assign_values_from_list(self, value_list):
        self.__utc = self.convert_UTC_to_epoch(value_list[UTC_INDEX])
        self.__lat = self.negate_by_hemisphere(
            self.convert_degMins_to_degDec(float(value_list[LAT_INDEX])),
            value_list[LAT_DIR_INDEX]
        )
        self.__lon = self.negate_by_hemisphere(
            self.convert_degMins_to_degDec(float(value_list[LON_INDEX])),
            value_list[LON_DIR_INDEX]
        )
        self.__alt = float(value_list[ALT_INDEX])
        self.__hdop = float(value_list[HDOP_INDEX])
        utm_values = utm.from_latlon(self.__lat,self.__lon) #Returns a tuple with UTM information
        self.__utmE, self.__utmN, self.__zone, self.__letter = utm_values #Unpack UTM information
#------------------------------------------

    def translate_string(self, string_value):
        if not self.is_GPGGA_string(string_value):
            self.__ok = False
            return
        self.assign_values_from_list(self.split_GPGGA_string(string_value))
        self.__ok = True
#------------------------------------------

    def __str__(self):
        return(f"Header: {self.__header}\n"
               +f"UTC: {self.__utc}\n"
              +f"LAT: {self.__lat}\n"
              +f"LON: {self.__lon}\n"
              +f"ALT: {self.__alt}\n"
              +f"HDOP: {self.__hdop}\n"
              +f"UTME: {self.__utmE}\n"
              +f"UTMN: {self.__utmN}\n"
              +f"ZONE: {self.__zone}\n"
              +f"LETTER: {self.__letter}\n"
              +f"STRING: {self.__read_string}\n"
              +f"OK?: {'Yes' if self.__ok else 'No'}"
              )
#------------------------------------------
    #Getters/Setters:
#------------------------------------------
    #setters:
    def set_header(self, value):
        self.__header = value
    #getters:
    def get_header(self):
        return self.__header
    def get_utc(self):
        return self.__utc
    def get_lat(self):
        return self.__lat
    def get_lon(self):
        return self.__lon
    def get_alt(self):
        return self.__alt
    def get_hdop(self):
        return self.__hdop
    def get_utmE(self):
        return self.__utmE
    def get_utmN(self):
        return self.__utmN
    def get_zone(self):
        return self.__zone
    def get_zone_letter(self):
        return self.__letter
    def get_read_string(self):
        return self.__read_string
    def is_ok(self):
        return self.__ok
#*******************************************************************

#GPS Publishing Node class. Will grab data from the serial port and publish
class GPSPublisher(Node):
    def __init__(self, port_address=DEFAULT_PORT):
        super().__init__('gps_publisher')
        self.__publisher = self.create_publisher(Customgps,PUBLISH_TOPIC,MAX_MSG)
        # self.__serial_port_address = port_address
        self.declare_parameter('port',port_address)
        self.__serial_port_address = self.get_parameter('port').get_parameter_value().string_value
        self.__serial_port = serial.Serial(self.__serial_port_address, BAUD_RATE,timeout=TIMEOUT)
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

