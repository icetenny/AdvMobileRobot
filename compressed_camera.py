#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header
import cv2
from std_msgs.msg  import Float32
import time

def publish_image():

    rospy.init_node('compressed_image_publisher')
    image_pub = rospy.Publisher('/camera_frame', CompressedImage, queue_size=10)
    cam_index = rospy.get_param("/camera_index", default=0)
    print("opening cam:", cam_index)
    cap = cv2.VideoCapture(cam_index)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            rospy.logerr("Error reading image frame")
            break

        # Compress the image using OpenCV
        _, img_data = cv2.imencode('.jpg', frame)
        img_data = img_data.tobytes()

        # Create a CompressedImage message
        compressed_msg = CompressedImage()
        compressed_msg.format = 'jpeg'
        compressed_msg.data = img_data

        # Publish the compressed image
        image_pub.publish(compressed_msg)
        print("Image Small GO")

        rate.sleep()


print("YES MAE")

if __name__ == '__main__':
	try:
		publish_image()
	except rospy.ROSInterruptException:
		pass
