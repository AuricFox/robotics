# Samuel Kitzerw, kitze012
# Extra Cedit Node

'''
* Face detection
    - Write a node to draw bounding boxes around faces on images received from some topic the node is
    subscribed to
* Video Player
    - Write a node to play a video through an image topic at the correct framerate
'''
from tokenize import String
import rospy
import cv2
from sensor_msgs.msg import Image
import numpy as np 
from cv_bridge import CvBridge

def callback(data):
    # Converts image from ROS to cv2
    img = bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
    # Convert image to grayscale
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    faces = face_cascade.detectMultiScale(img_gray, 1.1, 4) # Detects faces in image
    eyes = eye_cascade.detectMultiScale(img_gray, 1.1, 4)   # Detects eyes in image

    for(x,y,w,h) in faces:                                  # Draw faces on image
        cv2.rectangle(img, (x,y), (x+w, y+h), (255,0,0), 2)

    for(x,y,w,h) in eyes:                                   # Draw eyes on image
        cv2.rectangle(img, (x,y), (x+w, y+h), (255,0,0), 2)

    cv2.imshow("image grayscale", img)                      # Show Image
    cv2.waitKey(2)

def main():
    rospy.init_node("image_listerner", anonymous=True)      # Create new node
    rospy.Subscriber("usb_cam/image_raw", Image, callback=callback) # Add a listener node
    rospy.spin()

if __name__ == "__main__":
    bridge = CvBridge() # Bridge between cv and ROS
    face_cascade = cv2.CascadeClassifier("/catkin_ws/src/scripts/haarcascade_face.xml")
    eye_cascade = cv2.CascadeClassifier("/catkin_ws/src/scripts/haarcascade_eye.xml")
    main()