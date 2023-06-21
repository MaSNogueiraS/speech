#!/usr/bin/env python3
# -- coding: utf-8 --

import rospy
from std_msgs.msg import String
import openai
from collections import deque
import sys
import subprocess

sys.path.append('/home/robofei/Workspace/catkin_ws/src/hera_robot/hera_tasks/tasks/methods')

import General as General
import Speech as Speech
import Navigation as Navigation

#Author: MatS
#Contact: mateus.scarpelli03@gmail.com

class StartaTask(object):
    def __init__(self):
        rospy.init_node('Moving_from_whisper')
        self.general = General.General()
        self.navigation = Navigation.Navigation()
        self.speech = Speech.Speech()
        self.model = "gpt-3.5-turbo"
        self.prompt = ""
        self.conversation = deque([{"role": "system", "content": "Você é a HERA (Assistente Robótica de Ambiente Residencial), um robô de serviço gentil e educado projetado para realizar interação e cooperação entre humanos e robôs, desenvolvido pela equipe RoboFEI@Home do Centro Universitário FEI. O nome Hera foi inspirado na deusa grega protetora. A equipe RoboFEI@Home é atualmente campeã mundial na Robocup Thailand."}], maxlen=11)
        self.possible_tasks = ['recepcionist', 'storing groceries', 'carry my luggage']  # Ex. of tasks that we are almost doing

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
        # Set your OpenAI API key
        openai.api_key = "your-openai-api-key"
    
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



    #checks if the task requested is a quest that Hera do
    def check_task_request(self, text):
        for task in self.possible_tasks:
            if task in text.lower():
                return task
        return None
    
    #This function uses the subprocess library to execute a bash command
    def run_bash_command(self, command):
        
        completed_process = subprocess.run(command.split(), capture_output=True, text=True)
        if completed_process.returncode != 0:
            acknowledgement = self.generate_acknowledgement(f"Error: {completed_process.stderr}")
            print(acknowledgement)
        else:
            acknowledgement = self.generate_acknowledgement(f"Output: {completed_process.stdout}")
            print(acknowledgement)
    
    def bash_command_task(self):
        type_of_task = self.check_task_request(text=self.last_text_data)
        if type_of_task:
            self.run_bash_command(f"roslaunch my_package {type_of_task}.launch") #add the launch file
        else:
            acknowledgement = self.generate_acknowledgement("Task not recognized.")
            print(acknowledgement)

def main():
    startask_node = StartaTask()
    rospy.spin()

if __name__ == '__main__':
    main()
