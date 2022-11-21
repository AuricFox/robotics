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
