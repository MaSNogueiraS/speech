#!/usr/bin/env python3
# -- coding: utf-8 --

import rospy
from speech.srv import RecordRequest  # Here the package name is 'speech' and the service name is 'RecordRequest'
from std_msgs.msg import String
from speech.srv import Speak
import rospy
from speech.srv import Transcribe
from speech.srv import Chat
from speech.srv import KeywordDetect
from speech.srv import QuestionAnswer
from speech.srv import TaskCommand

class whisper_recogs(object):

    def __init__(self) -> None:
        self.text = ''
        self.pub = rospy.Publisher('whisper_recogs', String, queue_size=10)
        self.init_keywords = ['Hera', 'Herra', 'era']  # Initialization keywords
        pass
        
        self.keyword_response_subscriber = rospy.Subscriber(
            'keywords_detected',   
            String,  
            self.keyword_response_callback
        )
        
        self.keywords = None
        
    def keyword_response_callback(self, msg):
        self.keywords = msg.data

    #This function calls the service that uses whisper to recognize a certain amount of time.
    #Note: time must be more than 0.07s
    def handle_service_response(self, time):
        rospy.wait_for_service('record_audio') 
        try:
            # Create a handler
            record_audio = rospy.ServiceProxy('record_audio', RecordRequest)
            # Call the service with the duration
            resp = record_audio(time)
            self.text = resp.response
            self.pub.publish(self.text)
            return self.text

        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
            return None
        
    def speech_call(self, texttospeak):
        rospy.wait_for_service('speech_service')
        try:
            speech_service = rospy.ServiceProxy('speech_service', Speak)
            result = speech_service(texttospeak)
            if result.success:
                print("Successfully spoke the text.")
            else:
                print("Failed to speak the text.")
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


    def chat_call(self, context):
        rospy.wait_for_service('chat_service')
        try:
            chat_service = rospy.ServiceProxy('chat_service', Chat)
            result = chat_service(context)
            print("Received response: " + result.response)
            return result
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return ''

    def whisper_call(self):
        rospy.wait_for_service('whisper_phrase')
        try:
            whisper_phrase = rospy.ServiceProxy('whisper_phrase', Transcribe)
            result = whisper_phrase("start")
            print("Received transcription: " + result.transcription)
            txt = result.transcription
            return txt
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return None
        
    def request_keyword_detection(self, text_to_check):
        rospy.wait_for_service('detect_keyword')
        try:
            detect_keyword = rospy.ServiceProxy('detect_keyword', KeywordDetect)
            resp = detect_keyword(text_to_check)
            key = resp.response
            print(key)
            return key
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return None
        
    def ask_question(self, question):
        rospy.wait_for_service('answer_question')
        try:
            answer_question = rospy.ServiceProxy('answer_question', QuestionAnswer)
            resp = answer_question(question)
            answer = resp.response
            print(answer)
            return answer
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
            return None
        
    def call_task(self, task):
        rospy.wait_for_service('task_command_service')
        try:
            task_command = rospy.ServiceProxy('task_command_service', TaskCommand)
            response = task_command(task)
            return response.success, response.message
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")

    def voice_capture(self):
        while True:
            txts = self.handle_service_response(5.0)
            if any(keyword in txts for keyword in self.init_keywords):
                #say "how can i help you ?"
                phrase = self.chat_call("Hello, my name is Hera, how can i assist you today?")
                self.speech_call(phrase.response)
                text_to_iterate = self.whisper_call()
                keyword = self.request_keyword_detection(text_to_iterate)
                if keyword == "question" :
                    answer_to_talk = self.ask_question(text_to_iterate)
                    self.speech_call(answer_to_talk)
                else:
                    break
            else:
                phrase = self.chat_call("'I do not understand what you have said'")
                self.speech_call(phrase.response)


if __name__ == '__main__':
    rospy.init_node('gpsr_organizer', anonymous=True)
    wr = whisper_recogs()
    wr.voice_capture()
    rospy.spin()