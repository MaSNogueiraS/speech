import rospy
from std_msgs.msg import String
import openai
from collections import deque
import sys

sys.path.append('/home/robofei/Workspace/catkin_ws/src/hera_robot/hera_tasks/tasks/methods')

import General as General
import Speech as Speech
import Navigation as Navigation

class MovingFromWhisper(object):
    def __init__(self):
        rospy.init_node('Moving_from_whisper')
        self.general = General.General()
        self.navigation = Navigation.Navigation()
        self.speech = Speech.Speech()
        self.model = "gpt-3.5-turbo"
        self.prompt = ""
        self.conversation = deque([{"role": "system", "content": "Você é a HERA (Assistente Robótica de Ambiente Residencial), um robô de serviço gentil e educado projetado para realizar interação e cooperação entre humanos e robôs, desenvolvido pela equipe RoboFEI@Home do Centro Universitário FEI. O nome Hera foi inspirado na deusa grega protetora. A equipe RoboFEI@Home é atualmente campeã mundial na Robocup Thailand."}], maxlen=11)
        self.possible_places = ['kitchen', 'bedroom', 'living room']  # Places saved on the map

        self.text_subscriber = rospy.Subscriber(
            'last_text',  
            String,
            self.text_callback
        )
        
        self.keyword_response_subscriber = rospy.Subscriber(
            'keywords_detected',   
            String,  
            self.keyword_response_callback
        )
        
        self.last_text_data = None
        self.last_processed_text = None
    
    def text_callback(self, msg):
        self.last_text_data = msg.data

    def keyword_response_callback(self, msg):
        if msg.data == 'Following a direction' and self.last_text_data:
            # If the last_text_data is the same as the last processed text, do nothing
            if self.last_text_data == self.last_processed_text:
                return
            
            self.move(self.last_text_data)
            # Store the last_text_data as the last processed text
            self.last_processed_text = self.last_text_data

    def generate_acknowledgement(self):
        try:
            response = openai.ChatCompletion.create(
                model=self.model,
                messages=[{"role": "system", "content": ""},{"role": "user", "content": "What should I say before providing the answer?"}],
                max_tokens=50,
                temperature=0.6
            )
            acknowledgement = response['choices'][0]['message']['content'].strip()
            return acknowledgement
        except openai.api_errors.APIError as e:
            return "Ok, só um segundo."

    def check_location(self, direction):
        for place in self.possible_places:
            if place in direction.lower():
                return place
        return None

    def move(self, direction):
        place_to_go = self.check_location(direction)
        if place_to_go:
            print(f"Moving to {place_to_go}")
            self.navigation.goto(place_to_go)
        else:
            known_places = ', '.join(self.possible_places)
            print(f"I do not know this place, but I know these places: {known_places}")

def main():
    moving_node = MovingFromWhisper()
    rospy.spin()

if __name__ == '__main__':
    main()
