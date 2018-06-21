#CALIFORNIA STATE UNIVERSITY TELEDYNE PATHFINDER DVL DRIVERS - ROBOSUB 2018# 
########### by Diego Santillan in collaboration with Adam Loeffler###########
################### Additional ROS work by Jonathan Song ####################

import rospy
from pathfinder_dvl.msg import DVL
from ez_async_data.msg import Rotation
import serial
import time

class RunDVL:
	def __init__(self):
		self.yaw = []

	#self.yaw will always only have a length of 1 max to keep most updated
	def rCallBack(self, rotation):
		if self.yaw:
			self.pop()
		self.yaw.append(rotation.yaw)

	def main(self):
		# ROS publisher setup
		pub = rospy.Publisher('dvl_status', DVL)
		rospy.init_node('dvl_node', anonymous=True)
		rospy.Subscriber('current_rotation', Rotation, self.rCallBack, queue_size=0)

		msg = DVL()

		#########################INITALIZE DVL SERIAL################################

		#dvl = serial.Serial("COM13", 115200) #Windows Serial
		dvl = serial.Serial("/dev/ttyUSB1", 115200) #Ubuntu Serial


		################PATHFINDER DVL COMMANDS TO STREAM DATA#####################

		dvl.write("===") #DVL Break (PathFinder Guide p. 24 and p.99)
		dvl.write("CR1\r") #set factory defaults.(Pathfinder guide p.67)
		dvl.write("CP1\r") # required command
		dvl.write("PD6\r") #pd6 data format (Pathfinder Guide p.207)
		dvl.write("EX11110\r") #coordinate transformation (Pathfinder guide p.124)
		dvl.write("EA+4500\r") #heading alignment (Pathfinder guide 118)
		dvl.write("EZ10000010\r") #sensor source (Pathfinder guide 125)
		dvl.write("CK\r") #stores present parameters (Pathfinder guide 114)
		dvl.write("CS\r") #start pinning (Pathfinder guide 115)


		#NOTE: the \r character is required for continuous stream i.e (PD6\r")

		################################PROGRAM BEGINS##############################
		print "dvl start"
		heading = 0 
		dvlHeading = 0
		xtrans = 0
		ytrans = 0
		ztrans = 0
		xvel = 0
		yvel = 0
		zvel = 0
		status = 0

		loopTime = time.time()
		pubTimePrev = loopTime
		pubTimeInterval = 0.01
		headingTimePrev = loopTime
		headingTimeInterval = 0.25

		while not rospy.is_shutdown():
			loopTime = time.time()
			if self.yaw:
				heading = self.yaw.pop() #put heading info ** heree from IMU
				heading *= 100 #heading needs to go from 0 to 35999 (see Heading Alignment Pathfinder p.118)
				heading = int(heading)
				if loopTime - headingTimePrev > headingTimeInterval: #needs to wait!***can't update too fast
					headingTimePrev = loopTime
					dvl.write("EH " + str(heading) + "\r") #Update Heading
			if dvl.in_waiting > 0: #If there is a message from the DVL
				line = dvl.readline()
				if line[:3] == ":BD": #If the message is a positional update
					line = line.split(",")
					xtrans = float(line[1])
					ytrans = float(line[2])
					ztrans = float(line[3])
					timeDifference = line[4]
					
				
				if line[:3] == ":BI": #If the message is a velocity update
					line = line.split(",")
					xvel = float(line[1])
					yvel = float(line[2])
					zvel = float(line[3])
					status = line[4]
				if line[:3] == ":SA": #If the message is orientation 
					line = line.split(",")
					dvlHeading = line[3][:-2]
				if line[:3] == ":TS": #If the message is a timestamp
					pass
					#print line
					#print "Heading:", heading, "Xvel:", xvel, "Yvel:", yvel, "Zvel:", zvel, "Status:", status, "\r"
					
				# print "IMU Heading:", heading, "DVL Heading:", dvlHeading, "Xvel:", xvel, "Yvel:", yvel, "Zvel:", zvel, "Status:", status, "Xpos", xtrans, "YPos", ytrans, "ZPos", ztrans
				if (loopTime - pubTimePrev) > pubTimeInterval:
					pubTimePrev = loopTime

					#setup msg to be published to ROS
					msg.xpos = xtrans
					msg.xvel = xvel
					msg.ypos = ytrans
					msg.yvel = yvel
					msg.zpos = ztrans
					msg.zvel = zvel
					pub.publish(msg)

dvl = RunDVL()

if __name__ == '__main__':
    try:
        dvl.main()
    except rospy.ROSInterruptException: 
        pass