#!/usr/bin/env python3
# -- coding: utf-8 --

import rospy
from std_msgs.msg import String
from speech.srv import KeywordDetect, KeywordDetectResponse

#Author: MatS
#Contact: mateus.scarpelli03@gmail.com

class KeywordDetector(object):
    def __init__(self):
        rospy.init_node('keyword_detector')
        self.publisher_ = rospy.Publisher('keywords_detected', String, queue_size=10)
        self.text_publisher = rospy.Publisher('last_text', String, queue_size=10)  # New publisher for the last text message
        self.last_msg_data = None

        self.s = rospy.Service('detect_keyword', KeywordDetect, self.detect_keyword)

    def detect_keyword(self, req):
        # Skip this message if it's the same as the last one we processed
        if req.text == self.last_msg_data:
            return KeywordDetectResponse('')

        # Update our record of the last message we processed
        self.last_msg_data = req.text

        # Publish the last text message
        self.text_publisher.publish(String(data=self.last_msg_data))

        question_starts = ['who', 'when', 'where', 'what', 'how', 'why', 'can you', 'could you', 'think']
        direction_keywords = ['go', 'move', 'turn', 'stop']
        request_keywords = ['take', 'bring']
        task_keywords = ['take out the garbage', 'carry my luggage']

        if any(keyword in req.text for keyword in direction_keywords) and not req.text.strip().endswith('?'):
            return KeywordDetectResponse('direction')
        elif any(keyword in req.text for keyword in request_keywords):
            return KeywordDetectResponse('request')
        elif any(keyword in req.text for keyword in task_keywords):
            return KeywordDetectResponse('task')
        elif any(req.text.strip().startswith(keyword) for keyword in question_starts) or req.text.strip().endswith('?'):
            return KeywordDetectResponse('question')
        else:
            return KeywordDetectResponse('None')

def main():
    keyword_detector = KeywordDetector()
    rospy.spin()

if __name__ == '__main__':
    main()

#EX of use :

# import rospy
# from your_package_name.srv import KeywordDetect

# def request_keyword_detection():
#     rospy.wait_for_service('detect_keyword')
#     try:
#         detect_keyword = rospy.ServiceProxy('detect_keyword', KeywordDetect)
#         resp = detect_keyword("What's your name?")
#         print(resp.response)
#     except rospy.ServiceException as e:
#         print("Service call failed: %s"%e)

# if __name__ == "__main__":
#     request_keyword_detection()

