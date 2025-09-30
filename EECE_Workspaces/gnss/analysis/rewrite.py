import csv
import time

import os


CWD = os.getcwd()
OUTDOOR_FILE =  'gps_data/stationary_open_2.csv'
INDOOR_FILE =  'gps_data/stationary_indoors_2.csv'
WALKING_FILE = 'gps_data/walking_outdoors.csv'

OUTDOOR_FULL_PATH = os.path.join(CWD,OUTDOOR_FILE)
OCCLUDED_FULL_PATH = os.path.join(CWD,INDOOR_FILE)
WALKING_FULL_PATH = os.path.join(CWD,WALKING_FILE)


HOUR_TO_SECOND_FACTOR = 3600
MINUTE_TO_SECOND_FACTOR = 60
SECONDS_IN_A_DAY = 86400

def convert_utc_string_to_seconds(time_in_UTC: str) -> int:
    """Convert UTC string (hhmmss.sss) into seconds since midnight."""
    hours = int(time_in_UTC[0:2]) * HOUR_TO_SECOND_FACTOR
    minutes = int(time_in_UTC[2:4]) * MINUTE_TO_SECOND_FACTOR
    seconds = float(time_in_UTC[4:])  # may have decimals
    return int(hours + minutes + seconds)

def base_epoch_for_today_utc() -> int:
    """Return epoch timestamp for today's midnight in UTC (not local)."""
    now = time.gmtime()  # Current UTC time
    today_start = time.struct_time((
        now.tm_year,
        now.tm_mon,
        now.tm_mday,
        0, 0, 0,
        now.tm_wday,
        now.tm_yday,
        now.tm_isdst
    ))
    return int(time.mktime(today_start))

def process_csv(input_file: str, output_file: str):
    with open(input_file, mode='r', newline='', encoding='utf-8') as infile:
        reader = csv.DictReader(infile)
        fieldnames = reader.fieldnames

        with open(output_file, mode='w', newline='', encoding='utf-8') as outfile:
            writer = csv.DictWriter(outfile, fieldnames=fieldnames)
            writer.writeheader()

            day_offset = 0
            prev_seconds = None
            base_epoch = base_epoch_for_today_utc()

            for row in reader:
                utc_string = row.get("STRING2")

                if utc_string:
                    try:
                        current_seconds = convert_utc_string_to_seconds(utc_string)

                        # Detect rollover
                        if prev_seconds is not None and current_seconds < prev_seconds:
                            day_offset += SECONDS_IN_A_DAY

                        # Continuous epoch time as int
                        epoch_time = base_epoch + current_seconds + day_offset
                        row["UTC_SEC"] = str(int(epoch_time))

                        prev_seconds = current_seconds
                    except Exception as e:
                        print(f"Error converting {utc_string}: {e}")

                writer.writerow(row)

if __name__ == "__main__":
    process_csv(OCCLUDED_FULL_PATH, "Corrected_CSV.csv")

