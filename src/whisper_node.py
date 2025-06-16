#!/usr/bin/env python3
import rospy
import scipy.signal
from collections import deque
from std_msgs.msg import String, Int16MultiArray, UInt8
import numpy as np
import whisper

class WhisperTranscriber:
    def __init__(self):
        # Initialize the node
        rospy.init_node('whisper_node')
        self.model = whisper.load_model("tiny")
        self.sample_rate = rospy.get_param('sample_rate', 48000)
        self.waiting_time = 3
        self.buffer_seconds = 10
        self.buffering = False
        self.buffer_size = self.sample_rate * self.buffer_seconds
        self.audio_buffer = deque(maxlen=self.buffer_size)

        # Set up ROS pub/sub
        self.pub = rospy.Publisher('/whisperer/text', String, queue_size=10)
        rospy.Subscriber('/mic_audio', Int16MultiArray, self.audio_callback)
        rospy.Subscriber('/jackal_teleop/trigger', UInt8, self.trigger_callback)

        rospy.loginfo("[WHISPER NODE]: Started. Buffering audio and waiting for trigger.")
        rospy.spin()
        
    def audio_callback(self, msg):
        if self.buffering:
            audio_chunk = np.array(msg.data, dtype=np.int16)
            self.audio_buffer.extend(audio_chunk)
        
    def trigger_callback(self, msg):    
        if msg.data > 0:
            self.buffering = True
            rospy.loginfo("[WHISPER NODE]: Trigger received. Waiting for speaker to finish speaking.")
            rospy.sleep(self.waiting_time)
            rospy.loginfo("[WHISPER NODE]: Transcribing the next 10 seconds of audio.")
            rospy.sleep(self.buffer_seconds)
            try:
                # Prepare audio data
                audio = np.array(self.audio_buffer, dtype=np.int16).astype(np.float32) / 32768.0
                audio_resampled = scipy.signal.resample_poly(audio, 16000, self.sample_rate)

                # Transcribe
                result = self.model.transcribe(audio_resampled, fp16=False)
                text = result.get('text', '').strip()
                if text:
                    rospy.loginfo(f"[WHISPER NODE]: Transcribed: {text}")
                    self.pub.publish(text)
            except Exception as e:
                rospy.logerr(f"[WHISPER NODE]: Transcription failed: {e}")
            self.buffering = False

if __name__ == '__main__':
    WhisperTranscriber()