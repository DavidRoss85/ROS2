#Code from https://pypi.org/project/rosbags/

from pathlib import Path

from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore
from vn_interfaces.msg import Vectornav    #Custom interface
from decoders import VNYMRPositonData as PoseData#, Vector3D

FILE_HEADER = "HEADER,TIME_SECS,TIME_NANO,LIN_X,LIN_Y,LIN_Z,ANG_X,ANG_Y,ANG_Z,MAG_X,MAG_Y,MAG_Z,STRING,,,,,,,,,,,,,,,,"
DEFAULT_TOPIC = '/nav'
bagpath = Path('bag_data/imu_bag_4')

def create_CSV(filename):
    with open(filename, 'w') as file:
        file.write(FILE_HEADER)

def write_to_CSV(filename, message:PoseData):
    lin=message.get_linear_velocity()
    ang=message.get_angular_velocity()
    mag=message.get_magnetic_pose()
    
    with open(filename,'a') as file:
        formatted_message = (
            f"{message.get_header()},"+
            f"{int(message.get_time_in_seconds())},"+
            f"{message.get_remainder_nanoseconds()},"+
            f"{lin.x},{lin.y},{lin.z},"+
            f"{ang.x},{ang.y},{ang.z},"+
            f"{mag.x},{mag.y},{mag.z},"+
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
    with AnyReader([bagpath], default_typestore=typestore) as reader:
        connections = [x for x in reader.connections if x.topic == DEFAULT_TOPIC]
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            msg = reader.deserialize(rawdata, connection.msgtype)
            pose = PoseData(msg.raw)
            pose.set_time(msg.header.stamp.sec+(msg.header.stamp.nanosec/1e9))
            if pose.is_ok():
                write_to_CSV(my_file,pose)
                print(f"write: {pose.get_raw_string()}\n")
            else:
                print(f"*******INVALID STRING... SKIPPING************")


if __name__ == '__main__':
    main()