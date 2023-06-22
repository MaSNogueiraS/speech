#!/usr/bin/env python3
# -- coding: utf-8 --

import rospy
import gtts
from playsound import playsound
from tempfile import TemporaryFile
from speech.srv import Speak, SpeakResponse

class SpeechService:
    def __init__(self) -> None:
        pass

    def speak(self, req):
        try:
            tts = gtts.gTTS(req.text, lang='en')
            with TemporaryFile() as fp:
                tts.save(fp.name)
                playsound(fp.name)
            return SpeakResponse(success=True)
        except Exception as e:
            rospy.logerr("Failed to speak text: %s", str(e))
            return SpeakResponse(success=False)

if __name__ == "__main__":
    rospy.init_node('speech_service_node')
    ss = SpeechService()
    s = rospy.Service('speech_service', Speak, ss.speak)
    rospy.spin()


#EX of use :

# #!/usr/bin/env python3

# import rospy
# from your_package.srv import Speak

# def speech_call():
#     rospy.init_node('speech_call_node')
#     rospy.wait_for_service('speech_service')
#     try:
#         speech_service = rospy.ServiceProxy('speech_service', Speak)
#         result = speech_service("Hello, how are you?")
#         if result.success:
#             print("Successfully spoke the text.")
#         else:
#             print("Failed to speak the text.")
#     except rospy.ServiceException as e:
#         print("Service call failed: %s"%e)

# if __name__ == "__main__":
#     speech_call()