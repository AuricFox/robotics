# Samuel Kitzerw, kitze012
# Controller Node

'''
This controller node will send a twist message on the /cmd_vel topic at 10Hz with random values in the range
[-5,5] in the .linear.x and .angular.z fields. It should make the Turtlebot “dance” around randomly
'''
import numpy as np
import rospy
from geometry_msgs.msg import Twist

def controller():
    rospy.init_node('controller', anonymous=True)           # Initializing node
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)  # Topic command velocity '/cmd_vel'                         
    rate = rospy.Rate(10)                                   # Frequency of sent commands in Hz
    twist = Twist()                                         # Init twist message

    while not rospy.is_shutdown():

        twist.linear.x = np.random.uniform(-5, 5)       # Enter random input for x
        twist.linear.z = np.random.uniform(-5, 5)       # Enter random input for z
        pub.publish(twist)                              # Publish twist message
        rate.sleep()                                    # Wait then repeat

if __name__ == "__main__":
    controller()