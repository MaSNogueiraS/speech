#!/usr/bin/env python3
# -- coding: utf-8 --

import rospy
import WhisperSpeech 

class tst_functionalities ():
    def __init__(self) -> None:
        self.speech = WhisperSpeech.WhisperSpeech()
        pass

    def call (self):
        name = self.speech.ask_receptionist("name")
        drink = self.speech.ask_receptionist("drink")
        print (name, drink)

if __name__ == '__main__':
    rospy.init_node('tst', anonymous=True)
    tst = tst_functionalities()
    tst.call()

        