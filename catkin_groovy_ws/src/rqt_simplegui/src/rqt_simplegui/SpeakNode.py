#!/usr/bin/env python

import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('sound_play')

from subprocess import call
import rospy
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient


class SpeakNode():
    
    
    #sound_sig = Signal(SoundRequest)

    def __init__(self):
        self._sound_client = SoundClient()
        rospy.Subscriber('robotsound', SoundRequest, self.sound_cb)
        

    def speak(self, sentence):
        self._sound_client.say(sentence)

    def sound_cb(self, sound_request):
        pass
