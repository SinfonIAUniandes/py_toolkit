#! /usr/bin/env python
# -*- encoding: UTF-8 -*-

import qi
import time
import rospy
import argparse
import sys
from robot_toolkit_msgs.srv import tablet_service_srv, go_to_posture_srv, go_to_posture_srvResponse, tablet_service_srvResponse, go_to_posture_srvRequest, set_output_volume_srv, set_output_volume_srvResponse, set_security_distance_srv, set_security_distance_srvResponse, get_input_srv, set_speechrecognition_srv
from robot_toolkit_msgs.msg import text_to_speech_status_msg, speech_recognition_status_msg 
from std_srvs.srv import SetBool, SetBoolResponse, Empty
import ConsoleFormatter

class PyToolkit:
    # -----------------------------------------------------------------------------------------------------------------------
    # -----------------------------------------------------INIT--------------------------------------------------------------
    # -----------------------------------------------------------------------------------------------------------------------

    def __init__(self, session):

        # Publishers

        self.ALTextToSpeechStatusPublisher = rospy.Publisher('/pytoolkit/ALTextToSpeech/status', text_to_speech_status_msg, queue_size=10)
        print(consoleFormatter.format("ALTextToSpeech/status topic is up!","OKGREEN"))

        self.ALSpeechRecognitionStatusPublisher = rospy.Publisher('/pytoolkit/ALSpeechRecognition/status', speech_recognition_status_msg, queue_size=10)
        print(consoleFormatter.format("ALSpeechRecognition/status topic is up!","OKGREEN"))

        self.ALMemory = session.service("ALMemory")
        
        self.ALTextToSpeechStatusSubscriber = self.ALMemory.subscriber("ALTextToSpeech/Status")
        self.ALTextToSpeechStatusSubscriber.signal.connect(self.on_tts_status)

        self.ALSpeechRecognitionStatusSubscriber = self.ALMemory.subscriber("ALSpeechRecognition/Status")
        self.ALSpeechRecognitionStatusSubscriber.signal.connect(self.on_speech_recognition_status)

        # Service Naoqi Clients
        self.ALAudioDevice = session.service("ALAudioDevice")
        self.ALAutonomousLife = session.service("ALAutonomousLife")
        self.ALBasicAwareness = session.service("ALBasicAwareness")
        self.ALMotion = session.service("ALMotion")
        self.ALRobotPosture = session.service("ALRobotPosture")
        self.ALTabletService = session.service("ALTabletService")
        self.ALSpeechRecognitionService = session.service("ALSpeechRecognition")
        self.ALSpeechRecognitionService.subscribe("potato")
        
        # Service ROS Servers - ALAudioDevice
        self.audioDeviceSetOutputVolumeServer = rospy.Service('pytoolkit/ALAudioDevice/set_output_volume_srv', set_output_volume_srv, self.callback_audio_device_set_output_volume_srv)
        print(consoleFormatter.format('ALAudioDevice/set_output_volume_srv on!', 'OKGREEN'))

        self.audioHearingServer = rospy.Service('pytoolkit/ALSpeechRecognition/set_speechrecognition_srv', set_speechrecognition_srv, self.callback_set_speechrecognition_srv)
        print(consoleFormatter.format('ALAudioDevice/set_output_volume_srv on!', 'OKGREEN'))


        # Service ROS Servers - ALAutonomousLife
        self.autonomousSetStateServer = rospy.Service('pytoolkit/ALAutonomousLife/set_state_srv', SetBool, self.callback_autonomous_set_state_srv)
        print(consoleFormatter.format('ALAutonomousLife/set_state_srv on!', 'OKGREEN'))    


        # Service ROS Servers - ALBasicAwareness
        self.awarenessSetAwarenessServer = rospy.Service('pytoolkit/ALBasicAwareness/set_awareness_srv', SetBool, self.callback_awareness_set_awareness_srv)
        print(consoleFormatter.format('Set_awareness_srv on!', 'OKGREEN'))

        
        # Service ROS Servers - ALMotion
        self.motionSetSecurityDistanceServer = rospy.Service('pytoolkit/ALMotion/set_security_distance_srv', set_security_distance_srv, self.callback_motion_set_security_distance_srv)    
        print(consoleFormatter.format('Set_security_distance_srv on!', 'OKGREEN'))

        
        # Service ROS Servers - ALRobotPosture
        self.postureGoToPostureServer = rospy.Service('pytoolkit/ALRobotPosture/go_to_posture_srv', go_to_posture_srv, self.callback_posture_go_to_posture_srv)
        

        # Service ROS Servers - ALTabletService
        self.tabletShowImageServer = rospy.Service('pytoolkit/ALTabletService/show_image_srv', tablet_service_srv, self.callback_tablet_show_image_srv)
        print(consoleFormatter.format('Show_image_srv on!', 'OKGREEN'))    

        self.tabletShowWebViewServer = rospy.Service('pytoolkit/ALTabletService/show_web_view_srv', tablet_service_srv, self.callback_tablet_show_web_view_srv)
        print(consoleFormatter.format('Show_web_view_srv on!', 'OKGREEN'))    

        self.tabletShowTopicServer = rospy.Service('pytoolkit/ALTabletService/show_topic_srv', tablet_service_srv, self.callback_tablet_topic_srv)
        print(consoleFormatter.format('Show_topic_srv on!', 'OKGREEN')) 

        self.tabletPlayVideoServer = rospy.Service('pytoolkit/ALTabletService/play_video_srv', tablet_service_srv, self.callback_tablet_play_video_srv)
        print(consoleFormatter.format('Play_video_srv on!', 'OKGREEN'))    

        self.tabletGetInputServer = rospy.Service('pytoolkit/ALTabletService/get_input_srv', get_input_srv, self.callback_tablet_get_input_srv)
        print(consoleFormatter.format('Get_input_srv on!', 'OKGREEN'))  

        self.tabletHideServer = rospy.Service('pytoolkit/ALTabletService/hide_srv', Empty, self.callback_tablet_hide_srv)
        print(consoleFormatter.format('Hide_srv on!', 'OKGREEN'))    

        self.tabletOverloadServer = rospy.Service('pytoolkit/ALTabletService/overload_srv', Empty, self.callback_tablet_overload_srv)
        print(consoleFormatter.format('Overload_srv on!', 'OKGREEN'))    

        self.input=""
        self.promise=qi.Promise()

            


    # -----------------------------------------------------------------------------------------------------------------------
    # ----------------------------------------------------SERVICES CALLBACKS-------------------------------------------------
    # -----------------------------------------------------------------------------------------------------------------------

    # ----------------------------------------------------ALSpeechRecognition------------------------------------------------------

    def callback_audio_device_set_output_volume_srv(self, req):
        print(consoleFormatter.format("\nRequested ALAudioDevice/set_output_volume_srv", "WARNING"))
        self.ALAudioDevice.setOutputVolume(req.volume)
        print(consoleFormatter.format('Volume set to ' + str(req.volume), 'OKGREEN'))
        return set_output_volume_srvResponse("OK")

    # ----------------------------------------------------ALAudioDevice------------------------------------------------------

    def callback_set_speechrecognition_srv(self, req):
        if req.subscribe:
            self.ALSpeechRecognitionService.subscribe("isHearing")
        else:
            self.ALSpeechRecognitionService.unsubscribe("isHearing")
        self.ALSpeechRecognitionService.setAudioExpression(req.noise)
        self.ALSpeechRecognitionService.setVisualExpression(req.eyes)
        return "OK"

    
    # ----------------------------------------------------ALAutonomousLife------------------------------------------------

    def callback_autonomous_set_state_srv(self, req):
        print(consoleFormatter.format("\nRequested ALAutonomousLife/set_state_srv", "WARNING"))
        self.ALAutonomousLife.setAutonomousAbilityEnabled("All", req.data)
        if req.data:
            self.ALAutonomousLife.setState("interactive")
            print(consoleFormatter.format('Autonomous life is on!', 'OKGREEN'))
        else:
            self.ALAutonomousLife.setState("disabled")
            self.callback_posture_go_to_posture_srv(go_to_posture_srvRequest("stand"))
            print(consoleFormatter.format('Autonomous life is off!', 'OKGREEN'))
        return SetBoolResponse(True, "OK")

    # ----------------------------------------------------ALBasicAwareness------------------------------------------------
    def callback_awareness_set_awareness_srv(self, req):
        print(consoleFormatter.format("\nRequested ALBasicAwareness/set_awareness_srv", "WARNING"))
        if req.data:
            if not self.ALBasicAwareness.isEnabled():
                self.ALBasicAwareness.setEnabled(True)
            self.ALBasicAwareness.resumeAwareness()
            print(consoleFormatter.format('Awareness is on!', 'OKGREEN'))
        else:
            self.ALBasicAwareness.pauseAwareness()
            print(consoleFormatter.format('Awareness is off!', 'OKGREEN'))
        return SetBoolResponse(True, "OK")

    # ----------------------------------------------------ALMotion------------------------------------------------

    def callback_motion_set_security_distance_srv(self, req):
        print(consoleFormatter.format("\nRequested ALMotion/set_security_distance_srv", "WARNING"))
        self.ALMotion.setOrthogonalSecurityDistance(req.distance)
        self.ALMotion.setTangentialSecurityDistance(req.distance)
        print(consoleFormatter.format('Security distance was set to '+str(req.distance)+' m', 'OKGREEN'))
        return set_security_distance_srvResponse("OK")


    # ----------------------------------------------------ALRobotPosture------------------------------------------------
    
    def callback_posture_go_to_posture_srv(self, req):
        print(consoleFormatter.format("\nRequested ALRobotPosture/go_to_posture_srv", "WARNING"))
        if req.posture == "stand":
            self.ALRobotPosture.goToPosture("Stand", 0.5)
            print(consoleFormatter.format('Robot is in default position!', 'OKGREEN'))
        elif req.posture == "rest":
            self.ALRobotPosture.goToPosture("Crouch", 0.5)
            print(consoleFormatter.format('Robot is in rest position!', 'OKGREEN'))
        return go_to_posture_srvResponse("OK")

    # ----------------------------------------------------ALTabletService------------------------------------------------

    def callback_tablet_show_image_srv(self, req):
        print(consoleFormatter.format("\nRequested ALTabletService/show_image_srv", "WARNING"))
        self.ALTabletService.showImage(req.url)
        print(consoleFormatter.format('Image shown!', 'OKGREEN'))
        return tablet_service_srvResponse("OK")
    

    def callback_tablet_show_web_view_srv(self, req):
        print(consoleFormatter.format("\nRequested ALTabletService/show_web_view_srv", "WARNING"))
        self.ALTabletService.showWebview(req.url)
        print(consoleFormatter.format('Web view shown!', 'OKGREEN'))
        return tablet_service_srvResponse("OK")

    def callback_tablet_topic_srv(self, req):
        print(consoleFormatter.format("\nRequested ALTabletService/show_web_view_srv", "WARNING"))
        self.ALTabletService.showWebview("http://192.168.0.199:8080/stream_viewer?topic="+req.url)
        time.sleep(3)
        script="""
        var img = document.querySelector("img");
        img.style.height = "480px";
        var heading = document.querySelector("h1");
        heading.innerHTML = "";
        """
        self.ALTabletService.executeJS(script)
        print(consoleFormatter.format('Topic view shown!', 'OKGREEN'))
        return tablet_service_srvResponse("OK")

        pytoolkit.ALRobotPosture.goToPosture("Stand", 0.5)

    def getInput(self,event):
        self.input=event
        self.promise.setValue(True)

    def callback_tablet_get_input_srv(self, req):
        self.input=""
        print(consoleFormatter.format("\nRequested ALTabletService/show_web_view_srv", "WARNING"))
        #text es un textbox donde la persona ingresa informacion
        #bool son 2 botones de yes no
        #list es una lista de opciones de la que elige el usuario
        if req.type=="text":
            self.ALTabletService.showWebview("http://198.18.0.1/apps/robot-page/input1.html")
            script="""
            var label = document.getElementById("myLabel");
            label.textContent = "{text}";

            var sendButton = document.getElementById("sendB");
	        sendButton.onclick = function(){codigo};
            """.format(text=req.text,codigo="{var input = document.getElementById('input_id').value;\nALTabletBinding.raiseEvent(input);}")
        elif req.type=="bool":
            self.ALTabletService.showWebview("http://198.18.0.1/apps/robot-page/input2.html")
            script="""
            
            var yesButton = document.getElementById("yesB");
	        yesButton.onclick = function(){codigo};
            var noButton = document.getElementById("noB");
	        noButton.onclick = function(){codigo2};
            """.format(codigo="{ALTabletBinding.raiseEvent('yes');}",codigo2="{ALTabletBinding.raiseEvent('no');}")
        elif req.type=="list":
            self.ALTabletService.showWebview("http://198.18.0.1/apps/robot-page/input3.html")
            #Si el req.text no esta separado por comas tira error
            script="""

            var textbox = document.getElementById('input_id');

            var array = "{text}".split(",");
            for (var i = 0; i<array.length; i++)
            {codigo2}

            var sendButton = document.getElementById("sendB");
	        sendButton.onclick = function(){codigo};
            """.format(text=req.text,codigo="{var input = document.getElementById('input_id').value;\nALTabletBinding.raiseEvent(input);}",codigo2="{var opt = document.createElement('option');\nopt.value = array[i];\nopt.innerHTML=array[i];\ntextbox.appendChild(opt);}")
        time.sleep(1)
        signalID = 0
        signalID = self.ALTabletService.onJSEvent.connect(self.getInput);
        self.ALTabletService.executeJS(script)
        while self.input=="":
            time.sleep(1)
        self.ALTabletService.hide()
        try:
            self.promise.future().hasValue(3000)
        except RuntimeError:
            raise RuntimeError('Timeout: no signal triggered')
        self.ALTabletService.onJSEvent.disconnect(signalID)
        print(consoleFormatter.format('Topic view shown!', 'OKGREEN'))
        self.promise=qi.Promise()
        return self.input
    

    def callback_tablet_play_video_srv(self, req):
        print(consoleFormatter.format("\nRequested ALTabletService/play_video_srv", "WARNING"))
        self.ALTabletService.playVideo(req.url)
        print(consoleFormatter.format('Video played!', 'OKGREEN'))
        return tablet_service_srvResponse("OK")
    

    def callback_tablet_hide_srv(self, req):
        print(consoleFormatter.format("\nRequested ALTabletService/hide_srv", "WARNING"))
        self.ALTabletService.hide()
        print(consoleFormatter.format('Tablet hidden!', 'OKGREEN'))
        return None

    def callback_tablet_overload_srv(self, req):
        for i in range(8):
            pytoolkit.ALTabletService.loadApplication("webdisplay")
            time.sleep(1)
        pytoolkit.ALTabletService.hide()
        return None
    
    # -----------------------------------------------------------------------------------------------------------------------
    # -----------------------------------------------------EVENTS CALLBACKS--------------------------------------------------
    # -----------------------------------------------------------------------------------------------------------------------

    def on_tts_status(self, value):
        idOfConcernedTask, status = value
        self.ALTextToSpeechStatusPublisher.publish(text_to_speech_status_msg(idOfConcernedTask, status))

    def on_speech_recognition_status(self, value):
        status = value
        self.ALSpeechRecognitionStatusPublisher.publish(speech_recognition_status_msg(status))


if __name__ == '__main__':
    consoleFormatter=ConsoleFormatter.ConsoleFormatter()
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
    pytoolkit = PyToolkit(session)
    rospy.init_node('pytoolkit')
    try:
        pytoolkit.ALAutonomousLife.setAutonomousAbilityEnabled("All", False)
        pytoolkit.ALAutonomousLife.setState("disabled")
        pytoolkit.ALRobotPosture.goToPosture("Stand", 0.5)
        if not pytoolkit.ALBasicAwareness.isEnabled():
            pytoolkit.ALBasicAwareness.setEnabled(True)
        pytoolkit.ALBasicAwareness.resumeAwareness()
        pytoolkit.ALBasicAwareness.pauseAwareness()
        pytoolkit.ALBasicAwareness.resumeAwareness()
        print(consoleFormatter.format('Robot is in default position!', 'OKGREEN'))
        print("overloading tablet...")
        for i in range(8):
            pytoolkit.ALTabletService.loadApplication("webdisplay")
            time.sleep(1)
        pytoolkit.ALTabletService.hide()
        time.sleep(1)
        pytoolkit.ALTabletService.showImage("http://198.18.0.1/apps/robot-page/img/logo.png")
        print(consoleFormatter.format(" \n----------------------------------------------------------", "OKGREEN"))  
        print(consoleFormatter.format(" --------- PyToolkit node successfully initialized --------- ", "OKGREEN"))
        print(consoleFormatter.format(" ----------------------------------------------------------\n", "OKGREEN")) 
        rospy.spin()


    except rospy.ROSInterruptException:
        pass
