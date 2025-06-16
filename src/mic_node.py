#!/usr/bin/env python3
import rospy
import sounddevice as sd
from std_msgs.msg import Int16MultiArray, UInt8

class microphone:
    def __init__(self):
        # Initialize the microphone node
        rospy.init_node('mic_node')
        self.device_index = rospy.get_param('device_index', 0)
        self.sample_rate = rospy.get_param('sample_rate', 48000)
        self.channels = rospy.get_param('channels', 1)
        self.duration = 1 
        self.chunk_size = int(self.duration * self.sample_rate)

        # Setup publisher and subscriber
        self.pub = rospy.Publisher('/mic_audio', Int16MultiArray, queue_size=10)

        # Log startup and wait for trigger
        rospy.loginfo("[MIC NODE]: mic node started. Waiting for trigger...")
        
    def publishAudio(self):
        while not rospy.is_shutdown():
            try:
                # Record audio using sounddevice
                audio = sd.rec(self.chunk_size, samplerate=self.sample_rate,
                            channels=self.channels,
                            dtype='int16',
                            device=self.device_index)
                sd.wait()
                msg = Int16MultiArray(data=audio.flatten().tolist())
                self.pub.publish(msg)
                rospy.loginfo("[MIC NODE]: Successfully recorded and published audio chunk.")
            except Exception as e:
                rospy.logerr(f"Error: {e}")
                rospy.loginfo("[MIC NODE]: Error occured. Waiting for trigger again...")
            

if __name__ == '__main__':
    mic = microphone()
    mic.publishAudio()
    
    
