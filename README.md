# Whisper
This module does speech-to-text inference with the open-ai [whisper](https://github.com/openai/whisper).

mic_node: Records audio in chunks from a specified input device using sounddevice and publishes it as std_msgs/Int16MultiArray on the /mic_audio topic.

whisper_node: Subscribes to /mic_audio, decodes and transcribes the audio using Whisper, and publishes the resulting text as a single accumulated string on the /transcribed_text topic.

### Configure input device 
To find the correct input device run the following:
 `python3 -c "import sounddevice as sd; print(sd.query_devices())"`
Find the index of the input device you want to use and update 'device_index' in mic_node.py

You can also use `python3 -c "import sounddevice as sd; print(sd.query_devices(<device_index>))"` and replace <device_index> to find details on your input device

### Running Standalone 
To run this standalone run the following:
 - install dependancies: `pip install -r requirements.txt`
 - setup workspace: `source ~/catkin_ws/devel/setup.bash`
 - start the ros nodes: `roslaunch mic_to_whisper mic_to_whisper.launch`
 
### View transcribed text
# add instructions here :)
