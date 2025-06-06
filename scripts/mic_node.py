#!/usr/bin/env python3
import rospy
import sounddevice as sd
from std_msgs.msg import Int16MultiArray

def mic_stream():
    # Setup
    device_index = rospy.get_param('device_index', 0)
    sample_rate = rospy.get_param('sample_rate', 48000)
    channels = rospy.get_param('channels', 1)
    duration = 5
    chunk_size = int(duration * sample_rate)

    # Init
    rospy.init_node('mic_node')
    rospy.loginfo("mic node started.")
    pub = rospy.Publisher('/mic_audio', Int16MultiArray, queue_size=10)
    rospy.loginfo("Recording from device %d for %d seconds", device_index, duration)

    # Recording loop
    while not rospy.is_shutdown():
        try:
            # Record audio from mic
            audio = sd.rec(chunk_size, samplerate=sample_rate,
                           channels=channels, 
                           dtype='int16', 
                           device=device_index) 
            sd.wait()
            # Publish audio as an array of int16 data
            msg = Int16MultiArray(data=audio.flatten().tolist())
            pub.publish(msg)
        except Exception as e:
            rospy.logerr(f"Error: {e}")

if __name__ == '__main__':
    mic_stream()
