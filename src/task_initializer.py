#!/usr/bin/env python3
# -- coding: utf-8 --

import rospy
from std_msgs.msg import String
from speech.srv import TaskCommand, TaskCommandResponse
import subprocess

class StartaTask(object):
    def __init__(self):
        rospy.init_node('start_from_whisper')
        self.service = rospy.Service('task_command_service', TaskCommand, self.handle_start_task_request)
        self.model = "gpt-3.5-turbo"
        self.possible_tasks = ['recepcionist', 'storing groceries', 'carry my luggage']

    def handle_start_task_request(self, request):
        task = self.check_task_request(request.command)
        if task:
            success, message = self.run_bash_command(f"roslaunch my_package {task}.launch")
            return TaskCommandResponse(success, message)
        else:
            return TaskCommandResponse(False, "Task not recognized.")
    
    def check_task_request(self, text):
        for task in self.possible_tasks:
            if task in text.lower():
                return task
        return None

    def run_bash_command(self, command):
        try:
            subprocess.check_call(command.split())
            return True, "Task executed successfully."
        except subprocess.CalledProcessError as e:
            return False, str(e)

def main():
    startask_node = StartaTask()
    rospy.spin()

if __name__ == '__main__':
    main()