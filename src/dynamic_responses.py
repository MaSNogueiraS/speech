#!/usr/bin/env python3
# -- coding: utf-8 --

import rospy
import openai
from hera_speech.srv import Chat, ChatResponse

class ChatService:
    def __init__(self) -> None:
        self.openai_api_key = "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"  # Replace with your actual key
        openai.api_key = self.openai_api_key

    def chat(self, req):
        try:
            response = openai.ChatCompletion.create(
                model="gpt-3.5-turbo", 
                messages=[
                    {"role": "system", "content": req.context},
                    {"role": "user", "content": req.user_input}
                ]
            )
            return ChatResponse(response=response.choices[0].message['content'])
        except Exception as e:
            rospy.logerr("Failed to generate response: %s", str(e))
            return ChatResponse(response="I am experiencing a minor issue with the AI that generates my speeches. Please attempt once more or reach out to the RoboFEI team for support. I apologize for the inconvenience.")

if __name__ == "__main__":
    rospy.init_node('chat_service_node')
    cs = ChatService()
    s = rospy.Service('chat_service', Chat, cs.chat)
    rospy.spin()



