#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def callback(msg):
    # Print full text as one paragraph
    print(msg.data)
    print("-----")

def listener():
    rospy.init_node('transcription_viewer')
    rospy.loginfo("transcription viewer node started.")
    rospy.Subscriber('/transcribed_text', String, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
