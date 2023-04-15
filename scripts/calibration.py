import cv2
import numpy as np
import rosbag
from cv_bridge import CvBridge

def read_images_from_rosbag(rosbag_file, topic):
    images = []
    bridge = CvBridge()
    with rosbag.Bag(rosbag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=[topic]):
            img = bridge.imgmsg_to_cv2(msg, "bgr8")
            images.append(img)
    return images

def get_camera_calibration(images, chessboard_size):
    objpoints = [] # 3D points in real world space
    imgpoints = [] # 2D points in image plane
    gray_images = []
    for img in images:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray_images.append(gray)
        ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)
        if ret == True:
            objp = np.zeros((chessboard_size[0]*chessboard_size[1],3), np.float32)
            objp[:,:2] = np.mgrid[0:chessboard_size[0],0:chessboard_size[1]].T.reshape(-1,2)
            objpoints.append(objp)
            imgpoints.append(corners)
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray_images[0].shape[::-1], None, None)
    return ret, mtx, dist, rvecs, tvecs

rosbag_file = "/data/livox/bag/calib_02.bag"
topic = "/cam0/image_raw"
chessboard_size = (6, 6)

images = read_images_from_rosbag(rosbag_file, topic)

ret, mtx, dist, rvecs, tvecs = get_camera_calibration(images, chessboard_size)

print("ret:", ret)
print("camera matrix:", mtx)
print("distortion coefficients:", dist)