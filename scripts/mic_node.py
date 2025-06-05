#!/usr/bin/env python3
import rospy
import sounddevice as sd
from std_msgs.msg import Int16MultiArray

def mic_stream():
    # Setup
    device_index = 2
    duration = 5
    samplerate = 16000
    chunk_size = int(duration * samplerate)
    channels = 1

    # Init
    rospy.init_node('mic_node')
    rospy.loginfo("mic node started.")
    pub = rospy.Publisher('/mic_audio', Int16MultiArray, queue_size=10)
    rospy.loginfo("Recording from device %d for %d seconds", device_index, duration)

    # Recording loop
    while not rospy.is_shutdown():
        try:
            # need to use device index instead of None, but this is all that works for now
            audio = sd.rec(chunk_size, samplerate=samplerate, channels=channels, dtype='int16', device=None) 
            sd.wait()
            msg = Int16MultiArray(data=audio.flatten().tolist())
            pub.publish(msg)
        except Exception as e:
            rospy.logerr(f"Error: {e}")

if __name__ == '__main__':
    mic_stream()
