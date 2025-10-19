
#imports
import serial   #For reading data from the serial port
import rclpy    #ROS2 Dependency
from rclpy.node import Node     #Node Object
from vn_interfaces.msg import Vectornav    #Cutom interface
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
from .decoders import VNYMRPositonData as PoseData, TEST_STRING
# Global Constants:
PUB_TOPIC = "nav" #ROS2 topic to publish data on
MAX_MSG = 10    #ROS2 Maximum messages allowed at once
FREQUENCY = 0.1 #10Hz
BAUD_RATE = 115200  #Serial port baud
TIMEOUT = 0.1   #timeout rate for serial port
DEFAULT_PORT = "/dev/ttyUSB0"   #Default port to query


#*******************************************************************
#*******************************************************************
# IMU Publishing Node Class. Grabs data from the serial port an publish on specified ROS2 topic
class IMUPublisher(Node):
    #Constructor    
    def __init__(self, port_address=DEFAULT_PORT, topic=PUB_TOPIC):
        super().__init__('imu_publisher')
        self.__pub_topic = topic
        self.__publisher = self.create_publisher(Vectornav,self.__pub_topic,MAX_MSG)
        self.declare_parameter('port',port_address)
        self.__serial_port_address = self.get_parameter('port').get_parameter_value().string_value
        self.__serial_port = serial.Serial(self.__serial_port_address, BAUD_RATE,timeout=TIMEOUT)
        self.get_logger().info(f"Listening to {self.__serial_port_address}\nPosting to {self.__pub_topic}")
        self.__poll_serial_port()

        #Uncomment Only for Testing:
        # self.__timer = self.create_timer(1,self.__test_message)

    #------------------------------------------
    #This fuction is used for testing publisher
    def __test_message(self):
        p= PoseData(TEST_STRING)
        p.set_header(f"new_header - {p.get_time()}")
        self.publish_message(p)

    #------------------------------------------
    # Poll the serial port and publish message if valid
    def __poll_serial_port(self):
        while rclpy.ok():
            if self.__serial_port.in_waiting > 0:
                string_read = self.__serial_port.readline().decode('utf-8')
                my_pose = PoseData(string_read)
                if my_pose.is_ok():
                    self.__publish_message(my_pose)
    #------------------------------------------
    #Close open ports
    def close_ports(self):
        self.__serial_port.close()

    #------------------------------------------
    # Extract PoseData values and publish on topic
    def publish_message(self, message:PoseData):
        #Create IMU and MagneticField objects for holding data:
        imu = Imu()
        mag_field= MagneticField()

        # Get 2D list of vectors from message:
        vector_m = message.get_2Dvector_list()  
        # Unpack linear list:
        imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z = vector_m[1]
        # Unpack angular list:
        imu.angular_velocity.x, imu.angular_velocity.y,imu.angular_velocity.z = vector_m[2]
        # Unpack magnetic list:
        mag_field.magnetic_field.x, mag_field.magnetic_field.y, mag_field.magnetic_field.z = vector_m[3]
        # Unpack quaternion list:
        imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w = vector_m[4]

        # Create Vectornav message and publish:
        msg = Vectornav()
        msg.header.frame_id = message.get_header()
        msg.header.stamp.sec = int(message.get_time_in_seconds())
        msg.header.stamp.nanosec = int(message.get_remainder_nanoseconds())
        msg.imu = imu
        msg.mag_field = mag_field
        msg.raw_imu_data = message.get_raw_string()
        
        self.__publisher.publish(msg)
        self.get_logger().info(f"Publishing gps Data:\n{message}")

#*******************************************************************
#*******************************************************************

def main(args=None):
    print("Running\n")

    rclpy.init(args=args)
    imu_publisher = IMUPublisher(DEFAULT_PORT)
    rclpy.spin(imu_publisher)

    # imu_publisher.close_ports()
    imu_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()