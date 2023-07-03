import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
import rosbag

def convert_rosbag_to_video(rosbag_path, video_path, topic):
    bridge = CvBridge()

    # Open the bag file
    bag = rosbag.Bag(rosbag_path, "r")

    # Get the image topics from the bag
    image_topics = []
    for topic, msg, t in bag.read_messages():
        if msg._type == 'sensor_msgs/CompressedImage':
            image_topics.append(topic)

    # Set up video writer
    first_image = True
    video_writer = None

    # Loop through the bag and convert compressed images to video frames
    for topic, msg, t in bag.read_messages(topics=[topic]):
        if msg._type == 'sensor_msgs/CompressedImage':
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            # Create video writer on the first image
            if first_image:
                height, width, _ = cv_image.shape
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                video_writer = cv2.VideoWriter(video_path, fourcc, 30, (width, height))
                first_image = False

            # Write frame to the video
            video_writer.write(cv_image)

    # Release the video writer and close the bag
    video_writer.release()
    bag.close()

if __name__ == "__main__":
    rospy.init_node('rosbag_to_video')

    # Set the path to your rosbag file
    rosbag_path = "/data/mid360/bag/cvlabDSR/0605/narrow_motion_bf.bag"

    # Set the desired path and filename for the output video
    video_path = "/data/mid360/bag/cvlabDSR/0605/narrow_motion_bf.mp4"

    topic = "/cam0/compressed"

    # Convert the rosbag to video
    convert_rosbag_to_video(rosbag_path, video_path, topic)
