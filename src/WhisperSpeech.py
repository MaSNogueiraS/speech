#!/usr/bin/env python3
# -- coding: utf-8 --

import rospy
from std_msgs.msg import String
from hera_speech.srv import RecordRequest, Speak, Transcribe, Chat, KeywordDetect, QuestionAnswer, TaskCommand


class WhisperSpeech(object):

    def __init__(self) -> None:
        self.host_name = 'Micheal'
        self.host_drink = 'coffee'
        self.context_for_task_receptionist = "You are Hera (Home environment robot assistant), you are now performing an task. in this task you welcome guest in the house of your owner and make some questions to the guest, I will send you an phrase to you use as a model so you can make it more person like, please return just the phrase and in one line please, thank you"
        self.context_for_reask_something_receptionist = "You are Hera (Home environment robot assistant) and you are performing a task that uses speech recognition, but in the case of this context you listem wrongv what the user have said, i will send you an phrase as model so you make it more person like, please just send me the phrase and in one line please, thank you"
        self.context_for_name_receptionist = "I will send you a request with a phrase but you need to return to me just the name of an person on the phrase and nothing more, litteraly just the name, no words additionaly to explain."
        self.context_for_drink_receptionist = "I will send you a request with a phrase but you need to return to me just the name of the drink on the phrase and nothing more, litteraly just the drink, no words additionaly to explain"
        self.context_for_speech_name_receptionist = "You are Hera (Home environment robot assistant), and you received a person in the house of your owner, can you please say something like 'hello its nice to have you here' something like that, but not that exactly. I will send you just an phrase with an name and you say somethin cool as an welcome, and if you must, call the guest by the name i provide you just the phrase and in one line, no need to explain what yopu are saying just say it and thats it. thank you :)"
        self.context_for_speech_drink_receptionist = "You are Hera (Home environment robot assistant), and you received the name of the drink of a guest in the house of your owner i will send you the drink name, than you say something cool about the drink, some sort of data, a fun fact or a frienldly joke, anything like that, just the phrase and in one line, no need to explain what yopu are saying just say it and thats it. thank you :)"
        self.context_for_check_name_receptionist = "I will send you an phrase , it`s your function to detect if there is a name and if is you say just yes, if there is no name say no, if i send you an empty phrase say no, nothing more nothing, and if there is no name on it you just say no, nothing more.JUST YES OR NO. Thank you :)"
        self.context_for_check_drink_receptionist = "I will send you an phrase , it`s your function to detect if there is a drink and if is you say just yes, if there is no drink say no, if i send to you and empty phrase just say no, nothing more, and if there is no drink on it you just say no, nothing more.JUST YES OR NO. Thank you :)"
        pass

    #This function calls the service that uses whisper to recognize a certain amount of time.
    #Note: time must be more than 0.07s
    def whisper_time(self, time):
        rospy.wait_for_service('record_audio') 
        try:
            # Create a handler
            record_audio = rospy.ServiceProxy('record_audio', RecordRequest)
            # Call the service with the duration
            resp = record_audio(time)
            self.text = resp.response
            return self.text

        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
            return None
        
    def talk(self, texttospeak):
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


    def dynamic_responses(self, context, user_input):
        rospy.wait_for_service('chat_service')
        try:
            chat_service = rospy.ServiceProxy('chat_service', Chat)
            result = chat_service(context, user_input)
            print("Received response: " + result.response)
            return result.response
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


    def whisper_phrase(self):
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
        
    def keyword_detection(self, text_to_check):
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
        
    def question_answer(self, question):
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
        
    def init_task(self, task):
        rospy.wait_for_service('task_command_service')
        try:
            task_command = rospy.ServiceProxy('task_command_service', TaskCommand)
            response = task_command(task)
            return response.success, response.message
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")

    def ask_receptionist(self, type):
        if type == "name":
            presentation = self.dynamic_responses(self.context_for_task_receptionist, "Helo my name is Hera, please talk to me after the beep(please do not say beep). What is your name ?")
            self.talk(presentation)
            name = self.whisper_time(5.0) #Always use more then 0.07s
            print(name)
            check_name = self.dynamic_responses(self.context_for_check_name_receptionist, name)
            while True:
                if check_name == "Yes.":
                    break
                re_ask = self.dynamic_responses(self.context_for_reask_something_receptionist, f"Sorry for the inconvinient but i think i do not have heard right what you have said can you please talk your {type} again ?")
                self.talk(re_ask)
                name = self.whisper_time(5.0) #Always use more then 0.07s
                check_name = self.dynamic_responses(self.context_for_check_name_receptionist, name)
            actual_name = self.dynamic_responses(self.context_for_name_receptionist, name)
            print (actual_name)
            welcoming = self.dynamic_responses(self.context_for_speech_name_receptionist, actual_name)
            self.talk(welcoming)
            #self.names.append(actual_name)
            return actual_name

        elif type == "drink":
            presentation = self.dynamic_responses(self.context_for_task_receptionist, f"(Do not say nice to meet you or somethin like that since you already have introduced yourself)Now to introduce yourself to the host, can you please say to me , after the beep(please don`t say beep), what is your favority drink ?")
            self.talk(presentation)
            drink = self.whisper_time(5.0) #Always use more then 0.07s
            print(drink)
            check_drink = self.dynamic_responses(self.context_for_check_drink_receptionist, drink)
            while True :
                if check_drink == "Yes.":
                    break
                re_ask = self.dynamic_responses(self.context_for_reask_something_receptionist, f"Sorry for the inconvinient but i think i do not have heard right what you have said can you please talk your {type} again ?")
                self.talk(re_ask)
                drink = self.whisper_time(5.0) #Always use more then 0.07s
                check_drink = self.dynamic_responses(self.context_for_check_drink_receptionist, drink)

            actual_drink = self.dynamic_responses(self.context_for_drink_receptionist, drink)
            print(actual_drink)
            drink_fact = self.dynamic_responses(self.context_for_speech_drink_receptionist, actual_drink)
            self.talk(drink_fact)
            #self.names.append(actual_drink)
            return actual_drink

        else:
            #Ad error message
            print("Something went wrong with the Speech System, I`m so so sorry (,>_<,)")
            return False


        