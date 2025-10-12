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
# Position data for VectorNav IMU
class VNYMRPositonData():
    #Constructor    
    def __init__(self, string_data:str=""):
        self.__header = "imu1_frame"    #Header title
        self.__time = time.localtime()  #For timestamps
        self.__pose_angle = Vector3D(0,0,0)     #roll,pitch,yaw
        self.__linear_vel = Vector3D(0,0,0)     #velocity x,y,z
        self.__angular_vel = Vector3D(0,0,0)    #angular velocity x,y,z
        self.__magnetic_pose = Vector3D(0,0,0)  #magnetic values
        self.__string_data = string_data        #original string input
        self.__ok = False       #Indicator if conversion was ok
        self.__translate_string(string_data)

    #------------------------------------------
    #Verifies if the input string is valid:    
    def __is_valid_string(self, input_string:str)->bool:
        if input_string.startswith(VNYMR_HEADER):
            return True
        return False
    
    #------------------------------------------
    #Splits string into separate values. Returns a list of values:
    def __split_encoded_string(self, input_string:str)->list:
        string_list = input_string.split(",")
        for index, item in enumerate(string_list):
            if item == '' or item == None or item.upper() == "NAN":
                string_list[index] = '0'
        return string_list

    #------------------------------------------
    # Takes a list of values and assigns them to members based on index
    def __assign_values_from_list(self, value_list:list):

        #Set pose:
        self.__pose_angle.set_vector_array(
            float(value_list[ROLL_INDEX]),
            float(value_list[PITCH_INDEX]),
            float(value_list[YAW_INDEX])
        )
        #Set linear vel:
        self.__linear_vel.set_vector_array(
            float(value_list[ACCELX_INDEX]),
            float(value_list[ACCELY_INDEX]),
            float(value_list[ACCELZ_INDEX]),
        )
        #Set angular velocity:
        self.__angular_vel.set_vector_array(
            float(value_list[GYROX_INDEX]),
            float(value_list[GYROY_INDEX]),
            float(value_list[GYROZ_INDEX]),
        )
        #Set magnetic values:
        self.__magnetic_pose.set_vector_array(
            float(value_list[MAGX_INDEX]),
            float(value_list[MAGY_INDEX]),
            float(value_list[MAGZ_INDEX]),
        )

    #------------------------------------------
    # Takes a VNYMR string and stores values to self
    def __translate_string(self, string_value:str):
        if not self.__is_valid_string(string_value):
            self.__ok = False
            return
        self.__assign_values_from_list(self.__split_encoded_string(string_value))
        self.__ok = True

    #------------------------------------------
    #Returns printout
    def __str__(self):
        return (
            f"Header: {self.__header}\n" +
            f"Time: {self.get_time_in_seconds()}\n"
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
            f"String: {self.__string_data}\n"+
            f"Ok: {self.__ok}"
        )
    
    #------------------------------------------
    #Getters/Setters:
    #------------------------------------------
    #Setters:
    def set_header(self, value:str):
        self.__header = value
    
    #Getters:
    def is_ok(self)->bool:
        return self.__ok
    def get_header(self)->str:
        return self.__header
    def get_raw_string(self)->str:
        return self.__string_data
    def get_time(self)->time.struct_time:
        return self.__time
    def get_time_in_seconds(self)->float:
        return time.mktime(self.__time)
    def get_time_nanoseconds(self)->int:
        return (self.get_time_in_seconds()%1)*1000000000
    def get_pose_angle(self)->Vector3D:
        return self.__pose_angle
    def get_angular_velocity(self)->Vector3D:
        return self.__angular_vel
    def get_linear_velocity(self)->Vector3D:
        return self.__linear_vel
    def get_magnetic_pose(self)->Vector3D:
        return self.__magnetic_pose
    def get_vector_matrix(self)->list:
        value_matrix = [
            self.__pose_angle.get_vector_array(),
            self.__linear_vel.get_vector_array(),
            self.__angular_vel.get_vector_array(),
            self.__magnetic_pose.get_vector_array()
        ]
        return value_matrix
#*******************************************************************
#*******************************************************************

def main():
    pass

if __name__ == "__main__":
    main()