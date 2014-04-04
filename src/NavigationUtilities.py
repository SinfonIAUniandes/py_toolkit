#! /usr/bin/env python
# -- encoding: UTF-8 --

import qi 
import argparse
import sys
import rospy
from geometry_msgs.msg import Twist


class FollowYou:
    def _init_(self, session):
        
        self.cmd_velPublisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)


        self.ALMemory = session.service("ALMemory")


        self.ALMotion = session.service("ALMotion")
        self.ALTracker = session.service("ALTracker")
        self.ALTextToSpeech = session.service("ALTextToSpeech")


        self.ALPeoplePerception = session.service("ALPeoplePerception")

        self.ALPeoplePerceptionJustArrivedSubscriber  = 
self.ALMemory.subscriber("PeoplePerception/JustArrived")
        
self.ALPeoplePerceptionJustArrivedSubscriber.signal.connect(self.onPeopleJustArrived)

        self.ALPeoplePerceptionJustLeftSubscriber  = 
self.ALMemory.subscriber("PeoplePerception/JustLeft")
        self.ALPeoplePerceptionJustLeftSubscriber.signal.connect(self.onPeopleJustLeft)
        
        self.id = None
        self.targetName = "People"

        while self.id is None:
            rospy.sleep(1)

        print("I have seen someone")

        self.ALTracker.registerTarget(self.targetName, self.id)
        self.ALTracker.setMode("Move")
        self.ALTracker.track(self.targetName)

        try:
            while True:
                [yaw, pitch] = 
self.ALMemory.getData("PeoplePerception/Person/"+self.id+"/AnglesYawPitch")
                print("yaw: ", yaw, "pitch: ", pitch)
                if yaw < 0:
                    while yaw < 0:
                        self.cmd_velPublisher.publish(Twist((0,0,0), (0,0,0.25)))
                        [yaw, pitch] = 
self.ALMemory.getData("PeoplePerception/Person/"+self.id+"/AnglesYawPitch")
                        rospy.sleep(0.1)
                else:
                    while yaw > 0:
                        self.cmd_velPublisher.publish(Twist((0,0,0), (0,0,-0.25)))
                        [yaw, pitch] = 
self.ALMemory.getData("PeoplePerception/Person/"+self.id+"/AnglesYawPitch")
                        rospy.sleep(0.1)
                rospy.sleep(2)
        except KeyboardInterrupt:
            print()
            print("Interrupted by user")
            print( "Stopping...")

        self.ALTracker.stopTracker()
        self.ALTracker.unregisterAllTargets()
        print("ALTracker stopped.")



        pass

    def onPeopleJustArrived(self, id):
        self.id = id
        print(id)

    def onPeopleJustLeft(self, id):
        self.ALTextToSpeech.say("I have lost you, can you come back please?")
        self.id = None
        while self.id is None:
            rospy.sleep(1)
        self.ALTextToSpeech.say("I have seen you again, I will follow you again")
        self.ALTracker.registerTarget(self.targetName, self.id)
        self.ALTracker.setMode("Move")
        self.ALTracker.track(self.targetName)
        print(id)


if _name_ == '_main_':
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="127.0.0.1",
			help="Robot IP address. On RObot or Local Naoqi: use '127.0.0.1'.")
    parser.add_argument("--port", type=int, default=9559,
			help="Naoqi port number")
    args = parser.parse_args()
    session = qi.Session()
    try:
        session.connect("tcp://" + args.ip + ":" + str(args.port))
    except RuntimeError:
        print("Can't connect to Naoqi at ip \"" + args.ip + "\" on port" + str(args.port) + 
           ".\n""Please check your script arguments. Run with -h option for help.")
        sys.exit(1)
    followYou = FollowYou(session)
    rospy.init_node('pytoolkit')
    rospy.spin()
