#! python3.7

import io
import os
import speech_recognition as sr
import whisper
import torch
import rospy
from std_msgs.msg import String
from speech.srv import RecordAudio  # Here the package name is 'speech' and the service name is 'RecordAudio'

from datetime import datetime, timedelta
from queue import Queue
from tempfile import NamedTemporaryFile
from time import sleep
from sys import platform

class WhisperRealtime:
    def __init__(self) -> None:
        self.model = "medium"
        self.non_english = True 
        self.energy_threshold = 1000 
        self.record_timeout = 2 
        self.phrase_timeout = 3 
        self.default_microphone = "pulse"

        # Initialize the ROS node
        rospy.init_node('whisper_node', anonymous=True)
        # Create a ROS publisher
        self.pub = rospy.Publisher('whisper_recogs', String, queue_size=10)
        # Create a ROS service
        self.svc = rospy.Service('record_audio', RecordAudio, self.handle_record_audio)

    def handle_record_audio(self, req):
        duration = req.duration
        transcription = self.main(duration)
        return ' '.join(transcription)

    def main(self, duration):
        phrase_time = None
        last_sample = bytes()
        data_queue = Queue()

        recorder = sr.Recognizer()
        recorder.energy_threshold = self.energy_threshold
        recorder.dynamic_energy_threshold = True

        if 'linux' in platform:
            mic_name = self.default_microphone
            if not mic_name or mic_name == 'list':
                print("Available microphone devices are: ")
                for index, name in enumerate(sr.Microphone.list_microphone_names()):
                    print(f"Microphone with name \"{name}\" found")
                return
            else:
                for index, name in enumerate(sr.Microphone.list_microphone_names()):
                    if mic_name in name:
                        source = sr.Microphone(sample_rate=16000, device_index=index)
                        break
        else:
            source = sr.Microphone(sample_rate=16000)

        model = self.model
        if self.model != "large" and not self.non_english:
            model = model + ".en"
        audio_model = whisper.load_model(model)

        record_timeout = self.record_timeout
        phrase_timeout = self.phrase_timeout

        temp_file = NamedTemporaryFile().name
        transcription = ['']

        with source:
            recorder.adjust_for_ambient_noise(source)

        def record_callback(_, audio: sr.AudioData) -> None:
            data = audio.get_raw_data()
            data_queue.put(data)

        recorder.listen_in_background(source, record_callback, phrase_time_limit=record_timeout)

        print("Model loaded.\n")
        start_time = datetime.utcnow()

        while True:
            try:
                now = datetime.utcnow()
                if (now - start_time).total_seconds() > duration:
                    break
                if not data_queue.empty():
                    phrase_complete = False
                    if phrase_time and now - phrase_time > timedelta(seconds=phrase_timeout):
                        last_sample = bytes()
                        phrase_complete = True
                    phrase_time = now

                    while not data_queue.empty():
                        data = data_queue.get()
                        last_sample += data

                    audio_data = sr.AudioData(last_sample, source.SAMPLE_RATE, source.SAMPLE_WIDTH)
                    wav_data = io.BytesIO(audio_data.get_wav_data())

                    with open(temp_file, 'w+b') as f:
                        f.write(wav_data.read())

                    result = audio_model.transcribe(temp_file, fp16=torch.cuda.is_available())
                    text = result['text'].strip()

                    if phrase_complete:
                        transcription.append(text)
                        self.pub.publish(text)
                    else:
                        transcription[-1] = text

                    os.system('cls' if os.name == 'nt' else 'clear')
                    for line in transcription:
                        print(line)
                    print('', end='', flush=True)

                    sleep(0.25)
            except KeyboardInterrupt:
                break

        print("\n\nTranscription:")
        for line in transcription:
            print(line)
        
        return transcription


if __name__ == "__main__":
    ws = WhisperRealtime()
    rospy.spin()






#EX of use :
# import rospy
# from speech.srv import RecordAudio  # Here the package name is 'speech' and the service name is 'RecordAudio'

# def handle_service_response():
#     rospy.wait_for_service('record_audio')  # Corrected to 'record_audio' to match the name of the service when it was declared
#     try:
#         # Create a handler
#         record_audio = rospy.ServiceProxy('record_audio', RecordAudio)
        
#         # Call the service with the duration
#         response = record_audio(5.0) # Here, 5.0 is the duration for which you want to record the audio.
        
#         # Print the response data
#         print(response.data)

#     except rospy.ServiceException as e:
#         print("Service call failed: %s" % e)

# if __name__ == '__main__':
#     handle_service_response()

