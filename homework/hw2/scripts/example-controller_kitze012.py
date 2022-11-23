# Samuel Kitzerw, kitze012
# Controller Node

'''
This controller node will send a twist message on the /cmd_vel topic at 10Hz with random values in the range
[-5,5] in the .linear.x and .angular.z fields. It should make the Turtlebot “dance” around randomly
'''
import random
import rospy
from std_msgs.msg import String

rospy.init_node('controller')                               # Initializing node
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)    # Topic command velocity '/cmd_vel'
rate = rospy.Rate(10)  # In Hz                              # Frequency of sent commands 

while not rospy.is_shutdown():

    twist = Twist()
    twist.linear.x = random.randint(-5, 5)
    twist.linear.y = random.randint(-5, 5)
    twist.linear.z = 0.0
    rate.sleep()