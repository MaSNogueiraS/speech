import rospy
from std_msgs.msg import String
from detector_2d.msg import CoordBoxes
import openai
from collections import deque
import sys
import tf
from sensor_msgs import point_cloud2 as pc2

sys.path.append('/home/robofei/Workspace/catkin_ws/src/hera_robot/hera_tasks/tasks/methods')

import General as General
import Speech as Speech
import Navigation as Navigation

#Author: MatS
#Contact: mateus.scarpelli03@gmail.com

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
        self.possible_objects = ['coke', 'water', 'batatinha'] # Objects trained on the dataset

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

        self.object_detection_subscriber = rospy.Subscriber(
            'boxes_coordinates',  
            CoordBoxes,
            self.object_detection_callback
        )
        
        self.last_text_data = None
        self.last_processed_text = None
        self.detected_objects = []
        # Set your OpenAI API key
        openai.api_key = "your-openai-api-key"
    
    def text_callback(self, msg):
        self.last_text_data = msg.data

    def object_detection_callback(self, msg):
        self.detected_objects = []
        for obj in msg.detected_objects:
            self.detected_objects.append({
                'type': obj.type.data,
                'image_x': obj.image_x.data,
                'image_y': obj.image_y.data
            })

    def keyword_response_callback(self, msg):
        if msg.data == 'Fulfilling a request' and self.last_text_data:
            # If the last_text_data is the same as the last processed text, do nothing
            if self.last_text_data == self.last_processed_text:
                return
            
            self.move(self.last_text_data)
            # Store the last_text_data as the last processed text
            self.last_processed_text = self.last_text_data

    def generate_acknowledgement(self, context):
        try:
            response = openai.ChatCompletion.create(
                model=self.model,
                messages=[{"role": "system", "content": ""},{"role": "user", "content": context}],
                max_tokens=50,
                temperature=0.6
            )
            acknowledgement = response['choices'][0]['message']['content'].strip()
            return acknowledgement
        except openai.api_errors.APIError as e:
            return "Sorry, I have a small problem with the ai that process the text responses and generate dynamic aknolodgment"


    #check for known objects
    def selected_object(self, obj_to_pick): 
        for obj in self.possible_objects:
            if obj in obj_to_pick.lower():
                return obj
        return None

    #check for known locations
    def check_location(self, direction):
        for place in self.possible_places:
            if place in direction.lower():
                return place
        return None 

    #check both the obj and the place
    def search_object(self, obj, direction):
        object_to_pick = self.selected_object(obj)
        place_to_go = self.check_location(direction)
        if obj and place_to_go:
            acknowledgement = self.generate_acknowledgement(f"checking for {object_to_pick}, in the {place_to_go}")
            print(acknowledgement)
            self.navigation.goto(place_to_go)
            return obj
        elif place_to_go:
            acknowledgement = self.generate_acknowledgement("i don`t recognize the object you asked")
            print(acknowledgement)
            return None
        elif obj:
            acknowledgement = self.generate_acknowledgement("i don`t know the room you mentioned")
            print(acknowledgement)
            return None
        else: 
            acknowledgement = self.generate_acknowledgement("i do not know either the place you mentioned neither the object you asked")
            print(acknowledgement)

            return None
        
    #if both, the object and the place to go are recognized the robot will be on the correct room (goto in the def search object)
    def take_obj (self, objct, direction):
        obj = self.search_object(objct, direction)
        #if obj:
        #     detected_obj = next((x for x in self.detected_objects if x['type'] == obj), None)
        #     if detected_obj:
        #         print(f"Found object {obj} at position ({detected_obj['image_x']}, {detected_obj['image_y']})")
        #         return detected_obj['image_x'], detected_obj['image_y']
        #     else:
        #         print(f"Object {obj} not found on the room.")
        #         return None, None
        # else:
        #     print("i don`t recognize the object you have asked me to take.")
        #     return None, None

            #obj_position='service call specific object (obj)'
                #if obj_position:
                    #funtion that aproach the object (ex. return sucessed or something else)
                        #if goto sucessed :
                            # function that take the object based on the tf, like an storing groceries task
                                #if it sucessed 
                                    #goto the initial pose

                

def main():
    moving_node = MovingFromWhisper()
    rospy.spin()

if __name__ == '__main__':
    main()
