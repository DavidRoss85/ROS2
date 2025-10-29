#ROS2 bag reading Code from https://pypi.org/project/rosbags/

from pathlib import Path

from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore
from gps_interfaces.msg import Customgps

FILE_HEADER = "HEAD_FRAME,UTC_SEC,UTC_NANO,LAT,LON,ALT,UTME,UTMN,ZONE,LETTER,HDOP,FIX_Q,STRING,STRING2,STRING3,STRING4,STRING5,STRING6,STRING7,STRING8,STRING9,STRING10,STRING11,STRING12,STRING13,STRING14,STRING15\n"
bagpath = Path('bag_data/gps_bag_10_28_2025_155749')
DEFAULT_FILENAME = "./gps_data/gps_data.csv"
DEFAULT_TOPIC = "/gps"


def create_CSV(filename):
    with open(filename, 'w') as file:
        file.write(FILE_HEADER)

def write_to_CSV(filename, message:Customgps):

    
    with open(filename,'a') as file:
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
                # +f"{message.fix_quality},"
                +f"{message.gpgga_read.rstrip()}\n"
            )
            file.write(formatted_message)

def main():

    my_file = DEFAULT_FILENAME

    # Create a type store to use if the bag has no message definitions.
    typestore = get_typestore(Stores.ROS2_JAZZY)

    create_CSV(my_file) # Create file for writing with headers

    # Create reader instance and open for reading.
    with AnyReader([bagpath], default_typestore=typestore) as reader:
        connections = [x for x in reader.connections if x.topic == DEFAULT_TOPIC]
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            msg = reader.deserialize(rawdata, connection.msgtype)
            write_to_CSV(my_file,msg)
            print(f"write: {msg.gpgga_read}\n")
   

if __name__ == '__main__':
    main()