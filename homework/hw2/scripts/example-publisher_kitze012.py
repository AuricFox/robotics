# Samuel Kitzerw, kitze012
# Publisher Node

'''
This node will publish a message to a topic called “/chatter”
The message’s data field will be a string with both
A string saying “hello world it is  The current time 
'''

import rospy
from std_msgs.msg import String


def talker():
    rospy.init_node('publisher')
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rate = rospy.Rate(10)  # In Hz
    msg = String()

    while not rospy.is_shutdown():

        msg.data = f"hello world it is {rospy.get_time()}"
        pub.publish(msg)
        rate.sleep()


if __name__ == "__main__":
    talker()
