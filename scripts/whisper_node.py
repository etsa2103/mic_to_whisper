#!/usr/bin/env python3
import rospy
import scipy
from std_msgs.msg import String, Int16MultiArray
import numpy as np
import whisper

model = whisper.load_model("base")
sample_rate = rospy.get_param('sample_rate', 16000)

class WhisperTranscriber:
    def __init__(self):
        rospy.init_node('whisper_node')
        self.pub = rospy.Publisher('/transcribed_text', String, queue_size=10)
        self.transcribed_text = ""  # For accumulating transcriptions
        rospy.Subscriber('/mic_audio', Int16MultiArray, self.callback)
        rospy.loginfo("Whisper transcriber node started.")
        rospy.spin()

    def callback(self, msg):
        rospy.loginfo("Transcribing audio chunk...")
        # Extract audio from message and resample to 16khz
        audio = np.array(msg.data, dtype=np.int16).astype(np.float32) / 32768.0
        audio = scipy.signal.resample_poly(audio, 16000, sample_rate)
        # Run audio through whisper
        result = model.transcribe(audio, fp16=False)
        # Save newly transcribed text and publish on topic
        text = result.get('text', '').strip()
        if text:
            self.transcribed_text += text + " "
            rospy.loginfo(f"Transcribed text: {text}")
            self.pub.publish(self.transcribed_text.strip())

if __name__ == '__main__':
    WhisperTranscriber()
