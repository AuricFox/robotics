# Samuel Kitzerw, kitze012
# Publisher Node

'''
This node will publish a message to a topic called “/chatter”
The message’s data field will be a string with both
A string saying “hello world it is  The current time 
'''

from datetime import datetime
import rospy
from std_msgs.msg import String

rospy.init_node('publisher')
pub = rospy.Publisher('/chatter', String, queue_size=10)
rate = rospy.Rate(10)  # In Hz

while not rospy.is_shutdown():
    date = datetime.now()
    time = date.strftime("%H:%M:%S")

    msg = String()
    msg.data = "hello world it is" + time
    pub.publish(msg)
    rate.sleep()