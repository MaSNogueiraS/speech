#!/usr/bin/env python3
# -- coding: utf-8 --

import rospy
from speech.srv import MoveToPlace, MoveToPlaceResponse
import sys

sys.path.append('/home/robofei/Workspace/catkin_ws/src/hera_robot/hera_tasks/tasks/methods')

import Navigation as Navigation

#Author: MatS
#Contact: mateus.scarpelli03@gmail.com

class MovingFromWhisper(object):
    def __init__(self):
        rospy.init_node('Moving_from_whisper')
        self.navigation = Navigation.Navigation()
        self.possible_places = ['kitchen', 'bedroom', 'living room']  # Places saved on the map

        self.s = rospy.Service('move_to_place', MoveToPlace, self.move_service)

    def check_location(self, direction):
        for place in self.possible_places:
            if place in direction.lower():
                return place
        return None

    def move_service(self, req):
        place_to_go = self.check_location(req.place)
        if place_to_go:
            print(f"Moving to {place_to_go}")
            self.navigation.goto(place_to_go)
            return MoveToPlaceResponse('Success')
        else:
            known_places = ', '.join(self.possible_places)
            print(f"I do not know this place, but I know these places: {known_places}")
            return MoveToPlaceResponse('Failure')

def main():
    moving_node = MovingFromWhisper()
    rospy.spin()

if __name__ == '__main__':
    main()

#EX of use :

# import rospy
# from your_package_name.srv import MoveToPlace

# def request_move_to_place():
#     rospy.wait_for_service('move_to_place')
#     try:
#         move_to_place = rospy.ServiceProxy('move_to_place', MoveToPlace)
#         resp = move_to_place("kitchen")
#         print(resp.response)
#     except rospy.ServiceException as e:
#         print("Service call failed: %s"%e)

# if __name__ == "__main__":
#     request_move_to_place()
