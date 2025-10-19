"""
# This is a message to hold data from an IMU (Inertial Measurement Unit)
#
# Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec
#
# If the covariance of the measurement is known, it should be filled in (if all you know is the
# variance of each measurement, e.g. from the datasheet, just put those along the diagonal)
# A covariance matrix of all zeros will be interpreted as "covariance unknown", and to use the
# data a covariance will have to be assumed or gotten from some other source
#
# If you have no estimate for one of the data elements (e.g. your IMU doesn't produce an
# orientation estimate), please set element 0 of the associated covariance matrix to -1
# If you are interpreting this message, please check for a value of -1 in the first element of each
# covariance matrix, and disregard the associated estimate.


std_msgs/Header header
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id

geometry_msgs/Quaternion orientation
	float64 x 0
	float64 y 0
	float64 z 0
	float64 w 1
float64[9] orientation_covariance # Row major about x, y, z axes

geometry_msgs/Vector3 angular_velocity
	float64 x
	float64 y
	float64 z
float64[9] angular_velocity_covariance # Row major about x, y, z axes

geometry_msgs/Vector3 linear_acceleration
	float64 x
	float64 y
	float64 z
float64[9] linear_acceleration_covariance # Row major x, y z
"""
"""
# Measurement of the Magnetic Field vector at a specific location.
#
# If the covariance of the measurement is known, it should be filled in.
# If all you know is the variance of each measurement, e.g. from the datasheet,
# just put those along the diagonal.
# A covariance matrix of all zeros will be interpreted as "covariance unknown",
# and to use the data a covariance will have to be assumed or gotten from some
# other source.

std_msgs/Header header               # timestamp is the time the
	builtin_interfaces/Time stamp
		int32 sec
		uint32 nanosec
	string frame_id
    # field was measured
    # frame_id is the location and orientation
    # of the field measurement

geometry_msgs/Vector3 magnetic_field # x, y, and z components of the
	float64 x
	float64 y
	float64 z
    # field vector in Tesla
    # If your sensor does not output 3 axes,
    # put NaNs in the components not reported.

float64[9] magnetic_field_covariance       # Row major about x, y, z axes
                                           # 0 is interpreted as variance unknown

"""

#imports:
import time
import math
from tf_transformations import quaternion_from_euler # Converts euler to quaternion

#For importing directly from a ROS2 message
from vn_interfaces.msg import Vectornav
from geometry_msgs.msg import Quaternion
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField


# Global Constants:
SAMPLE_DATA = "$VNYMR,Yaw,Pitch,Roll,MagX,MagY,MagZ,AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ*CS" #Sample VNYMR string

VNYMR_HEADER = "$VNYMR"
TEST_STRING = "$VNYMR,-1.516,0.301,65.340,-0.169,-0.505,-0.603,0.007,-0.004,1.006,0.000,-0.001,0.000,*66"
# -- Position of data in list:
HEADER_INDEX = 0
YAW_INDEX = 1
PITCH_INDEX = 2
ROLL_INDEX = 3
MAGX_INDEX = 4
MAGY_INDEX = 5
MAGZ_INDEX = 6
ACCELX_INDEX = 7
ACCELY_INDEX = 8
ACCELZ_INDEX = 9
GYROX_INDEX = 10
GYROY_INDEX = 11
GYROZ_INDEX = 12
CHECKSUM_DELIMITER = "*"
VALUE_DELIMITER = ","

SECOND_TO_NANO_FACTOR = 1e9
GAUSS_TO_TESLA_CONVERSION = 1e-4

#*******************************************************************
#*******************************************************************
# Object to store a 3D Vector
class Vector3D():
    def __init__(self, x:float=0.0,y:float=0.0,z:float=0.0):
        self.x=x
        self.y=y
        self.z=z
    
    def get_vector_array(self)->list: #Retuns a list containing [x,y,z]
        v_list = [self.x,self.y,self.z]
        return v_list
    
    def set_vector_array(self,x:float=None,y:float=None,z:float=None):
        self.x = x if x!=None else self.x
        self.y = y if y!=None else self.y
        self.z = z if z!=None else self.z

#*******************************************************************
#*******************************************************************
# Object to store a 4D Vector
class Vector4D():
    def __init__(self, x:float=0.0,y:float=0.0,z:float=0.0,w:float=0.0):
        self.x=x
        self.y=y
        self.z=z
        self.w=w
    
    def get_vector_array(self)->list: #Retuns a list containing [x,y,z,w]
        v_list = [self.x,self.y,self.z,self.w]
        return v_list
    
    def set_vector_array(self,x:float=None,y:float=None,z:float=None,w:float=None):
        self.x = x if x!=None else self.x
        self.y = y if y!=None else self.y
        self.z = z if z!=None else self.z
        self.w = w if w!=None else self.w

#*******************************************************************
#*******************************************************************
# Position data for VectorNav IMU
class VNYMRPositonData():
    #Constructor    
    def __init__(
            self, string_data:str="",
            orientation_in_degrees:bool=True, angular_vel_in_degrees:bool=True, magnetic_in_gauss:bool=True, 
        ):
        self.__header = "imu1_frame"    #Header title
        self.__time = time.time_ns()  #For timestamps
        self.__quaternion = Vector4D(0,0,0,0)   #x,y,z,w
        self.__pose_angle = Vector3D(0,0,0)     #roll,pitch,yaw
        self.__linear_vel = Vector3D(0,0,0)     #velocity x,y,z
        self.__angular_vel = Vector3D(0,0,0)    #angular velocity x,y,z
        self.__magnetic_pose = Vector3D(0,0,0)  #magnetic values
        self.__string_data = string_data        #original string input
        self.__convert_orientation = orientation_in_degrees   #Trigger on whether to apply degrees to radian conversion on roll,pitch,yaw values
        self.__convert_angular = angular_vel_in_degrees   #Trigger on whether to apply degrees to radian conversion on angular values
        self.__convert_magnet = magnetic_in_gauss  #Trigger on whether to apply gauss to tesla conversion on incoming magnetic values
        self.__ok = False       #Indicator if conversion was ok
        self.__translate_string(string_data)

    #------------------------------------------
    #Verifies if the input string is valid:    
    def __is_valid_string(self, input_string:str)->bool:
        checksum_value = '' #Variable to store the checksum at the end of the string

        if input_string.startswith(VNYMR_HEADER) and CHECKSUM_DELIMITER in input_string:
            #Separate the checksum at the end of the string:
            string_and_checksum = input_string.split(CHECKSUM_DELIMITER)
            data_string = string_and_checksum[0][1:]
            checksum_value = string_and_checksum[1].rstrip()
            # Calculate checksum:
            new_checksum_value = 0
            for character in data_string:
                new_checksum_value ^= ord(character)

            new_checksum_string = f"{new_checksum_value:02X}"
            if new_checksum_string == checksum_value:
                return True
        return False
    
    #------------------------------------------
    #Splits string into separate values. Returns a list of values:
    def __split_encoded_string(self, input_string:str)->list:
        
        #Separate the checksum at the end of the string:
        string_and_checksum = input_string.split(CHECKSUM_DELIMITER)
        data_string = string_and_checksum[0][1:]

        string_list = data_string.split(VALUE_DELIMITER)   # Split string into values separated by comma
        
        for index, item in enumerate(string_list):    
            #Filter out blank values and replace with 0
            if item == '' or item == None or item.upper() == "NAN":
                string_list[index] = '0'
        
        return string_list

    #------------------------------------------
    # Takes a list of values and assigns them to members based on index
    def __assign_values_from_list(self, value_list:list):
        try:
            #Set pose:
            roll = float(value_list[ROLL_INDEX])
            pitch = float(value_list[PITCH_INDEX])
            yaw = float(value_list[YAW_INDEX])
            self.__pose_angle.set_vector_array(
                math.radians(roll) if self.__convert_orientation else roll,
                math.radians(pitch) if self.__convert_orientation else pitch,
                math.radians(yaw) if self.__convert_orientation else yaw,
            )
            # Set linear vel:
            self.__linear_vel.set_vector_array(
                float(value_list[ACCELX_INDEX]),
                float(value_list[ACCELY_INDEX]),
                float(value_list[ACCELZ_INDEX]),
            )
            # Set angular velocity:
            ax=float(value_list[GYROX_INDEX])
            ay=float(value_list[GYROY_INDEX])
            az=float(value_list[GYROZ_INDEX])
            self.__angular_vel.set_vector_array(
                math.radians(ax) if self.__convert_angular else ax,
                math.radians(ay) if self.__convert_angular else ay,
                math.radians(az) if self.__convert_angular else az,
            )
            # Set magnetic values:
            mx=float(value_list[MAGX_INDEX])
            my=float(value_list[MAGY_INDEX])
            mz=float(value_list[MAGZ_INDEX])
            self.__magnetic_pose.set_vector_array(
                (mx*GAUSS_TO_TESLA_CONVERSION) if self.__convert_magnet else mx,
                (my*GAUSS_TO_TESLA_CONVERSION) if self.__convert_magnet else my,
                (mz*GAUSS_TO_TESLA_CONVERSION) if self.__convert_magnet else mz
            )
            # Set quaternion:
            qx,qy,qz,qw = quaternion_from_euler(
                self.__pose_angle.x,
                self.__pose_angle.y,
                self.__pose_angle.z
            )
            self.__quaternion.set_vector_array(
                qx,qy,qz,qw
            )
        except:
            self.__ok = False
    #------------------------------------------
    # Takes a VNYMR string and stores values to self
    def __translate_string(self, string_value:str):
        if not self.__is_valid_string(string_value):
            self.__ok = False
            return
        self.__ok = True
        self.__assign_values_from_list(self.__split_encoded_string(string_value))

    #------------------------------------------
    #Imports from ROS2 msg
    def import_vectornav_msg(self, msg:Vectornav):
        imu = Imu()
        mag_field = MagneticField()
        head = String()

        # Try to Collect data from msg:
        try:
            head = msg.header.frame_id
            seconds = msg.header.stamp.sec
            nanosecs = msg.header.stamp.nanosec
            imu = msg.imu
            mag_field = msg.mag_field
        except:
            self.__ok = False
            return
        
        # Attempt to fetch the string data if available:
        try:
            raw_imu_data = msg.raw
        except:
            try:
                raw_imu_data = msg.raw_imu_data
            except:
                raw_imu_data = None

        # Exit if invalid string:
        if raw_imu_data is not None and not self.__is_valid_string(raw_imu_data):
            self.__ok = False
            return
        
        # Assign collected data to object members:
        self.__header = head
        self.__time = (seconds*SECOND_TO_NANO_FACTOR + nanosecs)
        self.__linear_vel.set_vector_array(
            imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z
        )
        self.__angular_vel.set_vector_array(
            imu.angular_velocity.x, imu.angular_velocity.y,imu.angular_velocity.z
        )
        self.__quaternion.set_vector_array(
            imu.orientation.x, imu.orientation.y, imu.orientation.z, imu.orientation.w
        )
        self.__magnetic_pose.set_vector_array(
            mag_field.magnetic_field.x, mag_field.magnetic_field.y, mag_field.magnetic_field.z
        )
        self.__string_data = "No string data" if raw_imu_data is None else raw_imu_data
        self.__ok = True
        
    #------------------------------------------
    #Returns printout
    def __str__(self):
        return (
            f"Header: {self.__header}\n" +
            f"Time: {self.get_time_in_seconds()}\n"+
            f"Nano time: {self.__time}\n"+
            f"Pose angle:\n" +
            f"\tRoll: {self.__pose_angle.x}\n"+
            f"\tPitch: {self.__pose_angle.y}\n"+
            f"\tYaw: {self.__pose_angle.z}\n"+
            f"Linear Velocity:\n"+
            f"\tx: {self.__linear_vel.x}\n"+
            f"\ty: {self.__linear_vel.y}\n"+
            f"\tz: {self.__linear_vel.z}\n"+
            f"Angular Velocity:\n"+
            f"\tx: {self.__angular_vel.x}\n"+
            f"\ty: {self.__angular_vel.y}\n"+
            f"\tz: {self.__angular_vel.z}\n"+
            f"Magnetic:\n"+
            f"\tx: {self.__magnetic_pose.x}\n"+
            f"\ty: {self.__magnetic_pose.y}\n"+
            f"\tz: {self.__magnetic_pose.z}\n"+
            f"Quaternion:\n"+
            f"\tx: {self.__quaternion.x}\n"+
            f"\ty: {self.__quaternion.y}\n"+
            f"\tz: {self.__quaternion.z}\n"+
            f"\tw: {self.__quaternion.w}\n"+
            f"String: {self.__string_data}\n"+
            f"Ok: {self.__ok}"
        )
    
    #------------------------------------------
    #Getters/Setters:
    #------------------------------------------
    #Setters:
    def set_header(self, value:str):
        self.__header = value
    def set_time(self, seconds:float):
        self.__time = seconds*SECOND_TO_NANO_FACTOR
    
    #Getters:
    def is_ok(self)->bool:
        return self.__ok
    def get_header(self)->str:
        return self.__header
    def get_raw_string(self)->str:
        return self.__string_data
    def get_quaternion(self)->Vector4D:
        return self.__quaternion
    def get_pose_angle(self)->Vector3D:
        return self.__pose_angle
    def get_angular_velocity(self)->Vector3D:
        return self.__angular_vel
    def get_linear_velocity(self)->Vector3D:
        return self.__linear_vel
    def get_magnetic_pose(self)->Vector3D:
        return self.__magnetic_pose
    
    def get_2Dvector_list(self)->list:
        value_list = [
            self.__pose_angle.get_vector_array(),
            self.__linear_vel.get_vector_array(),
            self.__angular_vel.get_vector_array(),
            self.__magnetic_pose.get_vector_array(),
            self.__quaternion.get_vector_array()
        ]
        return value_list
    
    def get_time_struct(self)->time.struct_time:
        return time.localtime(self.__time/SECOND_TO_NANO_FACTOR)
    
    def get_time_in_seconds(self)->float:
        return self.__time/SECOND_TO_NANO_FACTOR
    
    def get_time_in_nanoseconds(self)->int:
        return self.__time
    
    def get_remainder_nanoseconds(self)->int:
        seconds = int(self.__time/SECOND_TO_NANO_FACTOR)*SECOND_TO_NANO_FACTOR
        nanoseconds = self.__time - seconds
        return nanoseconds
#*******************************************************************
#*******************************************************************

def main():
    pass

if __name__ == "__main__":
    main()