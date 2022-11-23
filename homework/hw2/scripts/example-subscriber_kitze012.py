# Samuel Kitzerw, kitze012
# Subscriber Node

'''
This node will contain three subscribers:
* /chatter: String subscriber
    - This subscriber’s callback function will print a message using rospy.loginfo() containing
    the string received on the /chatter topic in a format similar to below
    - “/CHATTER – str: str_from_chatter”
* /tf: Turtlebot3 Gazebo Simulation Robot Current Pose
    - This subscriber’s callback function will print a message using rospy.logdebug() containing
    the translation x and rotation w values in a similar format to below
    - “/TF -- translation x: XX.XXXX rotation w: WW.WWWW”
* /imu: Turtlebot Gazebo Simulation Acceleration:
    - This subscriber’s callback function will print a message using rospy.logdebug() containing
    the robot’s x acceleration in a similar format to below
    - “/IMU – acceleration x: XX.XXXX”
'''

import rospy
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import Imu

def callback(msg):
    rospy.loginfo("Recieved {}".format(msg.data))


def callback_tf(msg):
    x = msg.tranforms[0].tranform.translation.x     # Gets x position
    w = msg.tranforms[0].tranform.rotation.w        # Gets rotation w
    rospy.loginfo("/TF -- translation x: {:.4f} rotation w: {:.4f}".format(x, w))


def callback_imu(msg):
    x = msg.linear_acceleration.x                   # Gets linear acc in x
    rospy.loginfo("/IMU – acceleration x: {:.4f}".format(x))


def listener():
    rospy.init_node('subscriber', anonymous=True)
    rospy.Subscriber('chatter', String, callback)   # Subscriber to chatter topic
    rospy.Subscriber('tf', TFMessage, callback_tf)  # Subscriber to tf topic
    rospy.Subscriber('imu', Imu, callback_imu)      # Subscriber to imu topic

    # Continue listening
    rospy.spin()


if __name__ == "__main__":
    listener()