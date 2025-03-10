import os
import sqlite3
import pandas as pd
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

# Hardcode the paths to your ROSBAG files here
ROSBAGS = [
    "/home/desiree/ASRL/Thesis/BuddySystemDatasets/FINAL_EXPERIMENTS/feb23Repeats/dome/rosbag2_2025_02_25-18_29_53/rosbag2_2025_02_25-18_29_53_0.db3"
]

OUTPUT_DIR = "extracted_gps_data"

def extract_oem7_raw_from_ros2_bag(bag_file, output_dir):
    """Extract NovAtel OEM7 raw GNSS data from a ROS 2 .db3 bag file and save as CSV."""
    
    print(f"Processing {bag_file} (ROS 2 format)...")
    os.makedirs(output_dir, exist_ok=True)

    # Locate the actual .db3 file inside the directory
    db_path = os.path.join(bag_file, "rosbag2_2025_02_25-18_29_53_0.db3")  # Adjust if needed
    if not os.path.exists(db_path):
        db_path = bag_file  # If the user provides the .db3 file directly

    # Connect to the ROS 2 bag database
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()

    # Ensure the topic `/novatel/oem7/oem7raw` exists
    topic_name = "/novatel/oem7/oem7raw"
    cursor.execute("SELECT name FROM topics WHERE name=?", (topic_name,))
    topic_exists = cursor.fetchone()
    
    if not topic_exists:
        print(f" Error: Topic {topic_name} not found in the bag!")
        conn.close()
        return

    print(f"Extracting OEM7 raw data from {topic_name}...")

    # Query and deserialize `Oem7Raw` messages
    cursor.execute(f"""
        SELECT timestamp, data FROM messages 
        WHERE topic_id = (SELECT id FROM topics WHERE name='{topic_name}')
    """)
    rows = cursor.fetchall()

    # Process and save the extracted data
    gps_data = []
    for row in rows:
        timestamp, raw_data = row
        msg = deserialize_message(raw_data, get_message("novatel_oem7_msgs/msg/Oem7Raw"))
        gps_data.append([timestamp, msg.raw_data.hex()])  # Convert binary to hex

    conn.close()

    # Save to CSV
    csv_path = os.path.join(output_dir, "oem7_raw.csv")
    df = pd.DataFrame(gps_data, columns=["Timestamp", "RawData"])
    df.to_csv(csv_path, index=False)
    
    print(f"Saved OEM7 raw data to {csv_path}\n")


# Run extraction for all hardcoded bags
if __name__ == "__main__":
    for bag_path in ROSBAGS:
        extract_oem7_raw_from_ros2_bag(bag_path, OUTPUT_DIR)
