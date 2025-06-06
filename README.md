# Whisper
This module does speech-to-text inference with the open-ai [whisper](https://github.com/openai/whisper).

mic_node: Records audio in chunks from a specified input device using sounddevice and publishes it as std_msgs/Int16MultiArray on the /mic_audio topic.

whisper_node: Subscribes to /mic_audio, decodes and transcribes the audio using Whisper, and publishes the resulting text as a single accumulated string on the /transcribed_text topic.

### Running Standalone 
To run this standalone run the following:
 - install dependancies: `pip install -r requirements.txt`
 - run `sudo apt install libportaudio2 libportaudiocpp0 portaudio19-dev`
 - setup workspace: `source ~/catkin_ws/devel/setup.bash`
 - start the ros nodes: `roslaunch mic_to_whisper mic_to_whisper.launch`
 
### Configure input device 
To find the index of your input device, run the following:
 `python3 -c "import sounddevice as sd; print(sd.query_devices())"`
To find the parameters of your input device run the following(Replace <device_index>):
`python3 -c "import sounddevice as sd; print(sd.query_devices(<device_index>))"`
Update the launch file with these parameters before running

### View transcribed text
To view the full transcription as it updates run the following:
`rosrun mic_to_whisper transcription_viewer_node.py`

### Notes
make sure the mic you are using is not set as the computers active input or output device. This gave me problems
