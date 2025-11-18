#ROS2 bag reading Code from https://pypi.org/project/rosbags/

from pathlib import Path

from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore
from vn_interfaces.msg import Vectornav    #Custom interface
from decoders import VNYMRPositonData as PoseData, Vector3D

FILE_HEADER = "HEADER,TIME_SECS,TIME_NANO,LIN_X,LIN_Y,LIN_Z,ANG_X,ANG_Y,ANG_Z,MAG_X,MAG_Y,MAG_Z,QUAT_X,QUAT_Y,QUAT_Z,QUAT_W,STRING,,,,,,,,,,,,,,,,\n"
bagpath = Path('bag_data/Lab5/west_seattle/imu_bag_11_7_2025_171412')
DEFAULT_FILENAME = "imu_data/imu_data.csv"
DEFAULT_TOPIC = '/imu'
OFFSET_SECONDS = 1759599824

def create_CSV(filename):
    with open(filename, 'w') as file:
        file.write(FILE_HEADER)

def write_to_CSV(filename, message:PoseData,separate_nanoseconds=False, offset_nanoseconds=OFFSET_SECONDS):
    lin=message.get_linear_velocity()
    ang=message.get_angular_velocity()
    mag=message.get_magnetic_pose()
    quat=message.get_quaternion()
    
    with open(filename,'a') as file:
        nanosecond_value = message.get_remainder_nanoseconds() if separate_nanoseconds else (message.get_time_in_nanoseconds()-offset_nanoseconds)
        formatted_message = (
            f"{message.get_header()},"+
            f"{int(message.get_time_in_seconds())},"+
            f"{nanosecond_value},"+
            f"{lin.x},{lin.y},{lin.z},"+
            f"{ang.x},{ang.y},{ang.z},"+
            f"{mag.x},{mag.y},{mag.z},"+
            f"{quat.x},{quat.y},{quat.z},{quat.w},"+
            f"{message.get_raw_string().rstrip()}"+
            "\n"
        )
        file.write(formatted_message)

def main():

    my_file = "imu_data/imu_data.csv"

    # Create a type store to use if the bag has no message definitions.
    typestore = get_typestore(Stores.ROS2_JAZZY)

    create_CSV(my_file) # Create file for writing with headers

    # Create reader instance and open for reading.
    nanosec_offset = None
    with AnyReader([bagpath], default_typestore=typestore) as reader:
        connections = [x for x in reader.connections if x.topic == DEFAULT_TOPIC]
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            msg = reader.deserialize(rawdata, connection.msgtype)
            pose = PoseData(
                msg.raw_imu_data,
                orientation_in_degrees=True,
                angular_vel_in_degrees=False,
                magnetic_in_gauss=True
            )
            pose2=PoseData()
            pose2.import_vectornav_msg(msg)
            pose.set_time(msg.header.stamp.sec+(msg.header.stamp.nanosec/1e9))
            if pose.is_ok():
                if nanosec_offset == None:
                    nanosec_offset = pose.get_time_in_nanoseconds()
                    print(f"Time offset set to {nanosec_offset} nanoseconds\n")
                write_to_CSV(my_file,pose,offset_nanoseconds=nanosec_offset)
                print(f"write: {pose.get_raw_string()}\n")
            else:
                print(f"*******INVALID STRING... SKIPPING************")
            # print(f"{msg}\n********COMPARISON************")
            # print(f"Magn... Raw: {pose.get_magnetic_pose().x}, Given: {pose2.get_magnetic_pose().x}")
            # print(f"Pose... Raw: {pose.get_pose_angle().x}, Given: {pose2.get_pose_angle().x}")
            # print(f"Angu... Raw: {pose.get_angular_velocity().x}, Given: {pose2.get_angular_velocity().x}")
            # print(f"Quat... Raw: {pose.get_quaternion().x}, Given: {pose2.get_quaternion().x}")
            # print(f"\n")
            # break

if __name__ == '__main__':
    main()