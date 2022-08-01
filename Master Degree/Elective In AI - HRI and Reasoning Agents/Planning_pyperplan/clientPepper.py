# https://github.com/ilkerkesen/tornado-websocket-client-example
# https://www.georgeho.org/tornado-websockets/
#!/usr/bin/env python
# -*- coding: utf-8 -*-
# coding=utf-8

from tornado.ioloop import IOLoop, PeriodicCallback
from tornado import gen
from tornado.websocket import websocket_connect
import time
import datetime
import os, sys, random
sys.path.append(os.getenv('PEPPER_TOOLS_HOME')+'/cmd_server')
import pepper_cmd
from pepper_cmd import *

client = None

class Client(object):
    def __init__(self, url, timeout):
        self.url = url
        self.timeout = timeout
        self.ioloop = IOLoop.instance()
        self.ws = None
        self.connect()
        PeriodicCallback(self.keep_alive, 20000).start()
        self.ioloop.start()

    @gen.coroutine
    def connect(self):
        print "trying to connect"
        try:
            self.ws = yield websocket_connect(self.url)
        except Exception, e:
            print "connection error"
        else:
            print "connected"
            self.run()

    @gen.coroutine
    def run(self):
        while True:
            data = yield self.ws.read_message()
            received = data.split("_");
            if(received[0]=="doQuiz"):
                print("%s!!Question Level Started!!%s" %(RED,RESET))
                questionLevel()
                self.ws.write_message("okMain")
            if(received[0]=="goDirectlyGame"):
                print("%s!!Exit Questionaire!!%s" %(RED,RESET))
                stopQuestionaire=True
            elif(received[0]=="RuleViolation"):
                print("%s!!RuleViolation!!%s" %(RED,RESET))
                pepper_cmd.robot.asay('You violated the game rules. Try again.'+' '*8)
            elif(received[0]=="EmptyRod"):
                print("%s!!EmptyRod!!%s" %(RED,RESET))
                pepper_cmd.robot.asay('You chose an empty rod. Try again.'+' '*8)        
            elif(received[0]=="ActionDone"):
                print("%s!!ActionDone!!%s" %(GREEN,RESET))
                frasi = ['Now it is your turn.', 'You go', 'Go ahead', 'Make your move']
                r = random.randint(0, 3)
                pepper_cmd.robot.asay(frasi[r]+' '*8)
            elif(received[0]=="Victory"):
                print("%s!!Victory!!%s" %(GREEN,RESET))
                pepper_cmd.robot.green_eyes()
                pepper_cmd.robot.asay('Victory'+' '*8)
                victoryDance();
                pepper_cmd.robot.white_eyes()
                pepper_cmd.robot.asay('Thank you for playing with me. Now, please answer the following questions. You can then choose to play again or leave.'+' '*8)
                pepper_cmd.robot.normalPosture()
                self.ws.write_message("victoryRating")
            elif(received[0]=="moveToLeft"):
                doMoveToLeft()
            elif(received[0]=="moveToRight"):
                doMoveToRight()

    def keep_alive(self):
        if self.ws is None:
            self.connect()
        #else:
            #self.ws.write_message("keep alive")
            
def victoryDance():
	ourSession = pepper_cmd.robot.session_service("ALMotion")
	jointNames = ["RShoulderPitch", "RShoulderRoll", "RElbowRoll", "RWristYaw", "RHand", "HipRoll", "HeadPitch", "LShoulderPitch", "LShoulderRoll", "LElbowRoll", "LWristYaw", "LHand"]
	jointValues = [-0.141, -0.46, 0.892, -0.8, 0.98, -0.07, -0.07, -0.141, 0.46, -0.892, 0.8, -0.98]
	times  = [2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0]
	isAbsolute = True
	ourSession.angleInterpolation(jointNames, jointValues, times, isAbsolute)
	
	for i in range(2):
		jointNames = ["RElbowYaw", "LElbowYaw", "HipRoll", "HeadPitch"]
		jointValues = [2.7, -1.3, -0.07, -0.07]
		times  = [0.8, 0.8, 0.8, 0.8]
		isAbsolute = True
		ourSession.angleInterpolation(jointNames, jointValues, times, isAbsolute)

		jointNames = ["RElbowYaw", "LElbowYaw", "HipRoll", "HeadPitch"]
		jointValues = [1.3, -2.7, -0.07, -0.07]
		times  = [0.8, 0.8, 0.8, 0.8]
		isAbsolute = True
		ourSession.angleInterpolation(jointNames, jointValues, times, isAbsolute)
	
	pepper_cmd.robot.normalPosture()
	return
	
	
def doHello():
	ourSession = pepper_cmd.robot.session_service("ALMotion")

	jointNames = ["RShoulderPitch", "RShoulderRoll", "RElbowRoll", "RWristYaw", "RHand", "HipRoll", "HeadPitch"]
	jointValues = [-0.141, -0.46, 0.892, -0.8, 0.98, -0.07, -0.07]
	times  = [2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0]
	isAbsolute = True
	ourSession.angleInterpolation(jointNames, jointValues, times, isAbsolute)

	for i in range(2):
		jointNames = ["RElbowYaw", "HipRoll", "HeadPitch"]
		jointValues = [1.7, -0.07, -0.07]
		times  = [0.8, 0.8, 0.8]
		isAbsolute = True
		ourSession.angleInterpolation(jointNames, jointValues, times, isAbsolute)

		jointNames = ["RElbowYaw", "HipRoll", "HeadPitch"]
		jointValues = [1.3, -0.07, -0.07]
		times  = [0.8, 0.8, 0.8]
		isAbsolute = True
		ourSession.angleInterpolation(jointNames, jointValues, times, isAbsolute)

	pepper_cmd.robot.normalPosture()
	
	return
	    
def raiseMyArm(whatArm, ourSession):
	if(whatArm=="R"):
	    jointNames = ["RShoulderPitch", "RShoulderRoll", "RElbowRoll", "RWristYaw", "RHand", "HipRoll", "HeadPitch"]
	    jointValues = [-0.141, -0.46, 0.892, -0.8, 0.98, -0.07, -0.07]
	else:
	    jointNames = ["LShoulderPitch", "LShoulderRoll", "LElbowRoll", "LWristYaw", "LHand", "HipRoll", "HeadPitch"]
	    jointValues = [-0.141, 0.46, -0.892, 0.8, -0.98, -0.07, -0.07]
	times  = [2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0]
	isAbsolute = True
	ourSession.angleInterpolation(jointNames, jointValues, times, isAbsolute)

	return
	

def doHello():
	ourSession = pepper_cmd.robot.session_service("ALMotion")

	raiseMyArm("R", ourSession)

	for i in range(2):
		jointNames = ["RElbowYaw", "HipRoll", "HeadPitch"]
		jointValues = [1.7, -0.07, -0.07]
		times  = [0.8, 0.8, 0.8]
		isAbsolute = True
		ourSession.angleInterpolation(jointNames, jointValues, times, isAbsolute)

		jointNames = ["RElbowYaw", "HipRoll", "HeadPitch"]
		jointValues = [1.3, -0.07, -0.07]
		times  = [0.8, 0.8, 0.8]
		isAbsolute = True
		ourSession.angleInterpolation(jointNames, jointValues, times, isAbsolute)

	pepper_cmd.robot.normalPosture()
	
	return
	    

def doMoveToRight():
	ourSession = pepper_cmd.robot.session_service("ALMotion")

	raiseMyArm("R", ourSession)

	jointNames = ["RElbowYaw", "HipRoll", "HeadPitch"]
	jointValues = [0.4, -0.07, -0.07]
	times  = [0.8, 0.8, 0.8]
	isAbsolute = True
	ourSession.angleInterpolation(jointNames, jointValues, times, isAbsolute)
	
	pepper_cmd.robot.normalPosture()
	
	return
		
def doMoveToLeft():
	ourSession = pepper_cmd.robot.session_service("ALMotion")

	raiseMyArm("L", ourSession)

	jointNames = ["LElbowYaw", "HipRoll", "HeadPitch"]
	jointValues = [-0.3, -0.07, -0.07]
	times  = [0.8, 0.8, 0.8]
	isAbsolute = True
	ourSession.angleInterpolation(jointNames, jointValues, times, isAbsolute)
	
	pepper_cmd.robot.normalPosture()

	return
	

stop_flag2 = False
stopQuestionaire = False

def handleWaitingAnswer(question,counting_answers):
	pepper_cmd.robot.say(question+" "*8)
	stop_flag2 = False
	while not stop_flag2 and not stopQuestionaire:
		vocabulary = ["yes", "no"]
		timeout = 10 # seconds after function returns
		answer = pepper_cmd.robot.asr(vocabulary, timeout)
		if answer=="yes":
		    counting_answers+=1
		    stop_flag2=True
                elif answer=="no":
		    stop_flag2=True
                else:
		    print("answer: "+answer)
		    pepper_cmd.robot.say("I didn't quite understand. Answer me only 'yes' or 'no'. "+question+" "*8)
        return counting_answers


def questionLevel():
    counting_answers=0
    counting_answers=handleWaitingAnswer('Are you less than 10 years old?',counting_answers)
    counting_answers=handleWaitingAnswer('Are you already familiar with the towers of Hanoi?',counting_answers)
    counting_answers=handleWaitingAnswer('Have you played before?',counting_answers)
    counting_answers=handleWaitingAnswer('Do you like solving problems on recursion?',counting_answers)
    
    if stopQuestionaire==False:
	if (counting_answers==2 or counting_answers==3):
	    pepper_cmd.robot.asay("I suggest the medium level for you"+" "*8)
	elif (counting_answers==4):
	    pepper_cmd.robot.asay("I suggest the hard level for you"+" "*8)
	else: # valueStopping=="stop" or counting_answers==1 or error
	    pepper_cmd.robot.asay("I suggest the easy level for you"+" "*8)
    pepper_cmd.robot.asay("If you want, you can take a look at the tutorial. Let's start playing!"+" "*8)
    pepper_cmd.robot.normalPosture()
    return



if __name__ == "__main__":
    begin()
    
    # Sonar Activation
    pepper_cmd.robot.startSensorMonitor()
    
    stop_flag = True
    try:
		while stop_flag:
			p = pepper_cmd.robot.sensorvalue()
			if(p[1]!=None and p[1]<3):
				print("I have located the user")
				stop_flag=False
			else:
				if p[1]==None or p[1]=="None":
				    print("I don't locate any users near me")
				else:
					print("I have located the user, but it is not close enough")
				# I stop 3 seconds before checking for another person
				time.sleep(3)
    except KeyboardInterrupt:
        pass 
    
    # Sonar Deactivation
    pepper_cmd.robot.stopSensorMonitor()
    
    if(stop_flag):
		print("\n\nDetected KeyboardInterrupt. Exit\n")
		try:
			sys.exit(0)
		except SystemExit:
			os._exit(0)
	
    pepper_cmd.robot.white_eyes()
	
    Our_tts_service = pepper_cmd.robot.session_service("ALTextToSpeech")
    Our_tts_service.setLanguage("English")
    Our_tts_service.setVolume(1.0)
    Our_tts_service.setParameter("speed", 1.0)

    Our_tts_service.say("Hello! My name is Hanoi. Play with me!"+" "*8, _async=True)
    doHello()
    pepper_cmd.robot.tablet_service.showWebview("http://10.0.1.201/WebApp/starting_page.html")
    Our_tts_service.say("Please, now answer to the following questions so that I can suggest you the most appropriate level for you."+" "*8, _async=False)
    pepper_cmd.robot.normalPosture()


    IPAddr="127.0.0.1:9030"
    client = Client("ws://"+IPAddr+"/websocketserver", 5)
    

# cd pepper tools/memory
# python write.py --key Device/SubDeviceList/Platform/Front/Sonar/Sensor/Value --val 2

# cd pepper tools/asr
# python human say.py--sentence "yes"
