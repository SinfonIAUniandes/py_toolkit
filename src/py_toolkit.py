#! /usr/bin/env python
# -*- encoding: UTF-8 -*-

import qi
import time
import rospy
import rospkg
import math
import argparse
import sys
from robot_toolkit_msgs.srv import tablet_service_srv, go_to_posture_srv, go_to_posture_srvResponse, tablet_service_srvResponse, go_to_posture_srvRequest, set_output_volume_srv, set_output_volume_srvResponse, set_security_distance_srv, set_security_distance_srvResponse, get_input_srv, set_speechrecognition_srv, point_at_srv, point_at_srvResponse, set_open_close_hand_srv, set_open_close_hand_srvResponse, point_at_srvRequest, move_head_srv, move_head_srvResponse , set_angle_srv , set_angle_srvResponse, get_segmentation3D_srv, get_segmentation3D_srvResponse, set_move_arms_enabled_srv, set_move_arms_enabled_srvResponse
from robot_toolkit_msgs.msg import text_to_speech_status_msg, speech_recognition_status_msg 
from std_srvs.srv import SetBool, SetBoolResponse, Empty
import ConsoleFormatter

class PyToolkit:
    # -----------------------------------------------------------------------------------------------------------------------
    # -----------------------------------------------------INIT--------------------------------------------------------------
    # -----------------------------------------------------------------------------------------------------------------------

    def __init__(self, session):

        #CONSTANTS
        self.PYTOOLKIT_FOLDER=rospkg.RosPack().get_path("py_toolkit")

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
        self.ALCloseObjectDetection = session.service("ALCloseObjectDetection")
        self.ALCloseObjectDetection.subscribe("pytoolkit")
        self.ALMotion = session.service("ALMotion")
        self.ALRobotPosture = session.service("ALRobotPosture")
        self.ALSegmentation3D = session.service("ALSegmentation3D")
        self.ALSegmentation3D.subscribe("pytoolkit")
        self.ALSpeechRecognitionService = session.service("ALSpeechRecognition")
        self.ALTabletService = session.service("ALTabletService")
        self.ALTrackerService = session.service("ALTracker")
        

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

        self.motionSetOpenCloseHandServer = rospy.Service('pytoolkit/ALMotion/set_open_close_hand_srv', set_open_close_hand_srv, self.callback_motion_set_open_close_hand_srv)
        print(consoleFormatter.format('Set_open_close_hand_srv on!', 'OKGREEN'))

        self.motionMoveHeadServer = rospy.Service('pytoolkit/ALMotion/move_head_srv', move_head_srv, self.callback_motion_move_head_srv)
        print(consoleFormatter.format('Move_head_srv on!', 'OKGREEN'))

        self.motionSetAngleServer = rospy.Service('pytoolkit/ALMotion/set_angle_srv', set_angle_srv, self.callback_motion_set_angle_srv)
        print(consoleFormatter.format('set_angle_srv on!', 'OKGREEN'))  

        self.motionSetMoveArmsEnabledServer = rospy.Service('pytoolkit/ALMotion/set_move_arms_enabled_srv', set_move_arms_enabled_srv, self.callback_motion_set_move_arms_enabled_srv)
        print(consoleFormatter.format('Set_move_arms_enabled_srv on!', 'OKGREEN'))

        
        # Service ROS Servers - ALRobotPosture
        self.postureGoToPostureServer = rospy.Service('pytoolkit/ALRobotPosture/go_to_posture_srv', go_to_posture_srv, self.callback_posture_go_to_posture_srv)
        print(consoleFormatter.format('Go_to_posture_srv on!', 'OKGREEN'))

        # Service ROS Servers - ALSegmentation3D
        self.segmentation3DGetSegmentation3DServer = rospy.Service('pytoolkit/ALSegmentation3D/get_segmentation3D_srv', get_segmentation3D_srv, self.callback_segmentation3D_get_segmentation3D_srv)

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


        # Service ROS Servers - ALTracker
        self.trackerPointAtServer = rospy.Service('pytoolkit/ALTracker/point_at_srv', point_at_srv, self.callback_point_at_srv)
        print(consoleFormatter.format('Point_at_srv on!', 'OKGREEN'))    

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
            self.ALBasicAwareness.setEnabled(req.data)
            print(consoleFormatter.format('Awareness is on!', 'OKGREEN'))
        else:
            self.ALBasicAwareness.setEnabled(req.data)
            self.callback_motion
            print(consoleFormatter.format('Awareness is off!', 'OKGREEN'))
        return SetBoolResponse(True, "OK")

    # ----------------------------------------------------ALMotion------------------------------------------------

    def callback_motion_set_security_distance_srv(self, req):
        print(consoleFormatter.format("\nRequested ALMotion/set_security_distance_srv", "WARNING"))
        self.ALMotion.setOrthogonalSecurityDistance(req.distance)
        self.ALMotion.setTangentialSecurityDistance(req.distance)
        print(consoleFormatter.format('Security distance was set to '+str(req.distance)+' m', 'OKGREEN'))
        return set_security_distance_srvResponse("OK")
    
    def callback_motion_set_open_close_hand_srv(self, req):
        print(consoleFormatter.format("\nRequested ALMotion/open_close_hand_srv", "WARNING"))
        if req.hand == "left" or req.hand == "both":
            if req.state == "open":
                self.ALMotion.setAngles("LHand", 1.0, 0.2)
                print(consoleFormatter.format('Left hand is open!', 'OKGREEN'))
            elif req.state == "close":
                self.ALMotion.setAngles("LHand", 0.0, 0.2)
                print(consoleFormatter.format('Left hand is closed!', 'OKGREEN'))
        if req.hand == "right" or req.hand == "both":
            if req.state == "open":
                self.ALMotion.setAngles("RHand", 1.0, 0.2)
                print(consoleFormatter.format('Right hand is open!', 'OKGREEN'))
            elif req.state == "close":
                self.ALMotion.setAngles("RHand", 0.0, 0.2)
                print(consoleFormatter.format('Right hand is closed!', 'OKGREEN'))
        return set_open_close_hand_srvResponse("OK")
    
    def callback_motion_move_head_srv(self, req):
        print(consoleFormatter.format("\nRequested ALMotion/move_head_srv", "WARNING"))
        if req.state == "up":
            self.ALMotion.setAngles(["HeadPitch", "HeadYaw"], [-0.4, 0.0], 0.15)
            print(consoleFormatter.format('Head is up!', 'OKGREEN'))
        elif req.state == "down":
            self.ALMotion.setAngles(["HeadPitch", "HeadYaw"], [0.46, 0.0], 0.2)
            print(consoleFormatter.format('Head is down!', 'OKGREEN'))
        elif req.state == "default":
            self.ALMotion.setAngles(["HeadPitch", "HeadYaw"], [0.0, 0.0], 0.2)
            print(consoleFormatter.format('Head is in default position!', 'OKGREEN'))
        return move_head_srvResponse("OK")
    
    def callback_motion_set_angle_srv(self,req):
        print(consoleFormatter.format("\nRequested ALMotion/set_angle_srv", "WARNING"))
        self.ALMotion.setAngles(tuple(req.name), tuple(req.angle), req.speed)
        print(consoleFormatter.format('Angles set!', 'OKGREEN'))
        return set_angle_srvResponse("OK")
    
    def callback_motion_set_move_arms_enabled_srv(self, req):
        print(consoleFormatter.format("\nRequested ALMotion/set_move_arms_enabled_srv", "WARNING"))
        self.ALMotion.setMoveArmsEnabled(req.LArm, req.RArm)
        while True:
            LArmEnabled = self.ALMotion.getMoveArmsEnabled("LArm")
            RArmEnabled = self.ALMotion.getMoveArmsEnabled("RArm")
            if LArmEnabled != req.LArm or RArmEnabled != req.RArm:
                self.ALMotion.setMoveArmsEnabled(req.LArm, req.RArm)
            else:
                break
        if req.LArm:
            print(consoleFormatter.format("LArm movement has been enabled!", "OKGREEN"))
        else:
            print(consoleFormatter.format("LArm movement has been disabled!", "OKGREEN"))
        if req.RArm:
            print(consoleFormatter.format("RArm movement has been enabled!", "OKGREEN"))
        else:
            print(consoleFormatter.format("RArm movement has been disabled!", "OKGREEN"))
        return set_move_arms_enabled_srvResponse("OK")
    

    
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
    
    # ----------------------------------------------------ALRobotPosture------------------------------------------------

    def callback_segmentation3D_get_segmentation3D_srv(self, req):
        print(consoleFormatter.format("\nRequested ALSegmentation3D/get_segmentation3D_srv", "WARNING"))
        response = get_segmentation3D_srvResponse()
        coordinates = list(self.ALSegmentation3D.getTopOfBlob(-1, 0, False))
        response.coordinates = coordinates
        return response

    # ----------------------------------------------------ALTabletService------------------------------------------------

    def callback_tablet_show_image_srv(self, req):
        print(consoleFormatter.format("\nRequested ALTabletService/show_image_srv", "WARNING"))
        self.ALTabletService.showImage(req.url)
        time.sleep(1)
        print(consoleFormatter.format('Image shown!', 'OKGREEN'))
        return tablet_service_srvResponse("OK")
    

    def callback_tablet_show_web_view_srv(self, req):
        print(consoleFormatter.format("\nRequested ALTabletService/show_web_view_srv", "WARNING"))
        self.ALTabletService.showWebview(req.url)
        print(consoleFormatter.format('Web view shown!', 'OKGREEN'))
        return tablet_service_srvResponse("OK")

    def callback_tablet_topic_srv(self, req):
        ip=open(self.PYTOOLKIT_FOLDER+"/resources/topic_srv.txt","r").read()
        print(consoleFormatter.format("\nRequested ALTabletService/show_web_view_srv", "WARNING"))
        self.ALTabletService.showWebview("http://"+ip+":8080/stream_viewer?topic="+req.url)
        time.sleep(3)
        script = """
        var body = document.querySelector("body");
        body.style.margin = "0";
        body.style.backgroundColor = "#161616";
        body.style.display = "flex";
        body.style.justifyContent = "center";
        var img = document.querySelector("img");
        img.style.height = "614px";
        var heading = document.querySelector("h1");
        heading.innerHTML = "";
        """
        self.ALTabletService.executeJS(script)
        print(consoleFormatter.format('Topic view shown!', 'OKGREEN'))
        return tablet_service_srvResponse("OK")


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
            script=open(self.PYTOOLKIT_FOLDER+"/resources/codigot.txt","r").read().replace("+++++",req.text)
        elif req.type=="bool":
            self.ALTabletService.showWebview("http://198.18.0.1/apps/robot-page/input2.html")
            script=open(self.PYTOOLKIT_FOLDER+"/resources/codigob.txt","r").read().replace("+++++",req.text)
        elif req.type=="list":
            self.ALTabletService.showWebview("http://198.18.0.1/apps/robot-page/input3.html")
            #Si el req.text no esta separado por comas tira error
            script=open(self.PYTOOLKIT_FOLDER+"/resources/codigol.txt","r").read().replace("+++++",req.text)
        time.sleep(1)
        signalID = 0
        signalID = self.ALTabletService.onJSEvent.connect(self.getInput)
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
        for i in range(10):
            pytoolkit.ALTabletService.loadApplication("webdisplay")
            time.sleep(1.5)
        pytoolkit.ALTabletService.hide()
        return None
    
    def callback_point_at_srv(self, req):
        print(consoleFormatter.format("\nRequested ALTracker/point_at_srv", "WARNING"))
        self.ALTrackerService.pointAt(req.effector_name, [req.x, req.y, req.z], req.frame, req.speed)
        print(consoleFormatter.format('Pointing at!', 'OKGREEN'))
        return point_at_srvResponse("OK")
    

    def angular_to_cartesian(self, x_angle, y_angle, distance):
        # Conversión de coordenadas angulares a cartesianas
        x = distance * math.sin(x_angle) * math.cos(y_angle)
        y = distance * math.sin(y_angle)
        z = distance * math.cos(x_angle) * math.cos(y_angle)
        return x, y, z

    
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
        for i in range(10):
            pytoolkit.ALTabletService.loadApplication("webdisplay")
            time.sleep(1.5)
        pytoolkit.ALTabletService.hide()
        time.sleep(1)
        pytoolkit.ALTabletService.showImage("http://198.18.0.1/apps/robot-page/img/logo.png")
        print(consoleFormatter.format(" \n----------------------------------------------------------", "OKGREEN"))  
        print(consoleFormatter.format(" --------- PyToolkit node successfully initialized --------- ", "OKGREEN"))
        print(consoleFormatter.format(" ----------------------------------------------------------\n", "OKGREEN")) 
        rospy.spin()


    except rospy.ROSInterruptException:
        pass
