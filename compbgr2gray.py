import rosbag
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image

bag = rosbag.Bag('/data/r3live/degenerate_seq_00.bag')

output_bag = rosbag.Bag('degenerate_seq_00_gary.bag', 'w')

bridge = CvBridge()
def process_compressed_image(image_msg):
    np_arr = np.fromstring(image_msg.data, np.uint8)
    cv_img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    gray_img = cv2.cvtColor(cv_img, cv2.COLOR_RGB2GRAY)

    gray_img_msg = bridge.cv2_to_imgmsg(gray_img, encoding="mono8")
    gray_img_msg.header = image_msg.header
    output_bag.write('/camera/image_gray', gray_img_msg, t=image_msg.header.stamp)

def process_image(image_msg):
    cv_img = bridge.imgmsg_to_cv2(image_msg, desired_encoding="passthrough")

    gray_img = cv2.cvtColor(cv_img, cv2.COLOR_RGB2GRAY)

    gray_img_msg = bridge.cv2_to_imgmsg(gray_img, encoding="mono8")
    gray_img_msg.header = image_msg.header
    output_bag.write('/camera/image_gray', gray_img_msg, t=image_msg.header.stamp)

for topic, msg, t in bag.read_messages():
    if topic == '/camera/image_color/compressed':
        process_compressed_image(msg)
    if topic == '/camera/image_color':
        process_image(msg)
    if topic == '/livox/imu':
        output_bag.write(topic, msg, t=msg.header.stamp)
    if topic == '/livox/lidar':
        output_bag.write(topic, msg, t=msg.header.stamp)
    if topic == '/livox_pcl0':
        output_bag.write(topic, msg, t=msg.header.stamp)

bag.close()
output_bag.close()
