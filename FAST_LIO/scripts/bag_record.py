import subprocess
import signal
import os

def start_rosbag_record(bag_name, topics):
    command = ["ros2", "bag", "record", "-o", bag_name]
    command.extend(topics)
    return subprocess.Popen(command)

def main():
    # 指定rosbag的名称和要记录的话题
    bag_name = "bag/mapping_12_22"
    topics = ["/livox/lidar", "/livox/imu"]  

    # 启动rosbag记录
    rosbag_process = start_rosbag_record(bag_name, topics)

    try:
        print("Recording. Press Ctrl+C to stop.")
        rosbag_process.wait()
    except KeyboardInterrupt:
        print("Stopping rosbag record...")
        rosbag_process.send_signal(signal.SIGINT)

if __name__ == "__main__":
    main()
