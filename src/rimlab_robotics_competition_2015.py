#!/usr/bin/env python
import rospy
import numpy
import math
from random import randint
from wall_following import WallFollowing
import time

# The velocity command message
from geometry_msgs.msg import Twist

# The laser scan message
from sensor_msgs.msg import LaserScan

# The barcode reset message 
from std_msgs.msg import Empty

# The barcode result message
from std_msgs.msg import Char


class RoboticsCompetition(object):
    """RoboticsCompetition class"""

    scan = None

    #States variables
    obstacleAvoidOn = False
    doorDetected = False
    wait = False
    exceed = False

    doorToApproach = {}
    doorAlignmentCounter =0
    exceedText = ""
    exceedDir = 0.0
    turnrate = 0.0
    turn = 0.0
    speed = 0.0
    doorsExceeded = 0  
    competitionEndCounter = 0 
    
    exceedPhaseOne = False
    exceedPhaseTwo = False
    exceedPhaseTree = False
    exceedPhaseFour = False
    exceedPhaseFive = False
    exceedTurnDir = 0
    beamToFreeIndex = None
    wf = None
    corridorStarted = False
    corridorEnded = False
    
    codebarNumber = -1

    #Parameters
    loopRate = 10
    doorsToExceed = 3
    minBeamValue = 0.10
    maxBeamValue = 30.0
    edgeThreshold = 0.15
    deltaXTresholdBaseValue = 0.1
    deltaXTresholdHysteresisValue = 0.20
    deltaXTreshold = deltaXTresholdBaseValue
    minCorridorWidthThreshold = 1.40
    turnThreshod = 3
    omegaMax = 0.2
    speedMax = 0.2
    minDoorWidth = 0.20
    maxDoorWidth = 0.90
    discardDoorThreshold = 0.40
    minDoorDistance = 0.75
    maxAlignmentCyces = 5
    kappa = 0.3

    #Constructor
    def __init__(self):
        
        #Reads params from launch file
        self.scanTopicParam = rospy.get_param('~scan_topic', '/scan')
        self.cmdVelTopicParam = rospy.get_param('~cmd_vel_topic', '/cmd_vel')
        self.reverScanParam = rospy.get_param('~revert_scan', False)
        self.mockCodebar = rospy.get_param('~mock_codebar', False)
        self.doorsToExceed = rospy.get_param('~doors_to_exceed', 3)

        rospy.loginfo("scanTopicParam: {0}".format(self.scanTopicParam))
        rospy.loginfo("cmdVelTopicParam: {0}".format(self.cmdVelTopicParam))
        rospy.loginfo("reverScanParam: {0}".format(self.reverScanParam))
        rospy.loginfo("mockCodebar: {0}".format(self.mockCodebar))
        rospy.loginfo("doorsToExceed: {0}".format(self.doorsToExceed))

        # Subscriber for the laser data
        self.sub = rospy.Subscriber(self.scanTopicParam, LaserScan, self.laserCallback)

        # Publisher for movement commands
        self.pub = rospy.Publisher(self.cmdVelTopicParam, Twist, queue_size=10)
        
        # Subscriber for the barcode data
        self.subBarcode = rospy.Subscriber('/codebar', Char, self.codebarCallback)

        # Publisher for barcode commands
        self.pubBarcode = rospy.Publisher('/light', Empty, queue_size=10)

        # Subscriber for the barcode command
        self.subBarcodeCommand = rospy.Subscriber('/light', Empty, self.codebarCommandCallback)

        time.sleep(2)

        #Reset codebar reader
        self.startReadCodebar()
        

    def getLoopRate(self):
        return self.loopRate


    def laserCallback(self,scan):

        self.scan = scan
        
        if self.reverScanParam:         
            reversedRanges = self.scan.ranges[::-1]
            reversedIntensities = self.scan.intensities[::-1]
            self.scan.ranges = reversedRanges
            self.scan.intensities = reversedIntensities

        #Check for 0 beam
        cleanedRanges = [self.minBeamValue]
        #From 0 to 539
        for i in range(len(self.scan.ranges) -1):
            if self.scan.ranges[i+1] < self.minBeamValue:
                cleanedRanges.append(cleanedRanges[i])
            else:
                 cleanedRanges.append(self.scan.ranges[i+1])

        self.scan.ranges = tuple(cleanedRanges)

        #Update wall follwer data
        if self.wf != None:
            self.wf.updateScanData(self.scan)


    def mockCodebarCallback(self, number):
        print("Read {0}".format(number))
        
        if number % 2 == 0:
            #Go to right
            self.exceedText = "We must go to right"
            self.exceedSearchDir = "right"
            self.exceedTurnDir = -1
        else:
            #Go to left
            self.exceedText = "We must go to left"
            self.exceedSearchDir = "left"
            self.exceedTurnDir = 1

        self.exceed = True
        self.wait = False


    def codebarCallback(self, charMsg):

        #From char to number
        number = charMsg.data

        print("Read {0}".format(number))
        if number < 0 or number > 9 or number == self.codebarNumber:
            print("We must reset Codebar node")
            self.startReadCodebar()
            return

        if number % 2 == 0:
            #Go to right
            self.exceedText = "We must go to right"
            self.exceedSearchDir = "right"
            self.exceedTurnDir = 1
        else:
            #Go to left
            self.exceedText = "We must go to left"
            self.exceedSearchDir = "left"
            self.exceedTurnDir = -1

        self.exceed = True
        self.wait = False
        self.codebarNumber = number
        print(self.exceedText)


    def codebarCommandCallback(self, msg):
        print("Detected Empty msg on /light topic")


    def startReadCodebar(self):

        while self.pubBarcode.get_num_connections() == 0:
            print("Wait for /light Subscriber")
        print("Founded /light Subscriber")

        self.pubBarcode.publish()


    def printLaserScan(self):

        if self.scan != None:
            print("\n\n")
            for i in range(90,450):
                rospy.loginfo('Range {0}: {1}m'.format(i, self.scan.ranges[i]))


    def publishTwistMsg(self, speedX, turnZ):

        self.speed = speedX
        self.turn = turnZ

        command = Twist()

        command.linear.x = self.speed
        command.linear.y = 0.0
        command.linear.z = 0.0
        command.angular.x = 0.0
        command.angular.y = 0.0
        command.angular.z = self.turn

        self.pub.publish(command)


    def doorDetector(self):

        allEdges = []

        #Set to turnThreshold beams greater that turnThreshold
        zeroedRanges = []
        for i in range(len(self.scan.ranges)):
            if self.scan.ranges[i] < self.turnThreshod:
                zeroedRanges.append(self.scan.ranges[i])
            else:
                zeroedRanges.append(self.turnThreshod)

        imin = 90
        imax = 450

        #Finds abrupt change in the data and puts it in the first tier of edges
        for i in range(imin, imax-1):

            tempPoi = {}
            #    __
            #   |
            # __|
            if zeroedRanges[i+1] - zeroedRanges[i] < -self.edgeThreshold:
                #tempPoi['distance'] = zeroedRanges[i]
                #tempPoi['angle'] = i * self.scan.angle_increment
                #tempPoi['index'] = i
                tempPoi['distance'] = zeroedRanges[i+1]                
                tempPoi['angle'] = (i+1) * self.scan.angle_increment  
                tempPoi['index'] = i+1
                tempPoi['type'] = 0
                allEdges.append(tempPoi)

            elif zeroedRanges[i+1] - zeroedRanges[i] > self.edgeThreshold:
                # __  
                #   |
                #   |__
                #tempPoi['distance'] = zeroedRanges[i+1]
                #tempPoi['angle'] = (i+1) * self.scan.angle_increment
                #tempPoi['index'] = i+1
                tempPoi['distance'] = zeroedRanges[i]                
                tempPoi['angle'] = (i) * self.scan.angle_increment                
                tempPoi['index'] = i
                tempPoi['type'] = 1
                allEdges.append(tempPoi)

        edgeList = []        
        edgeList = allEdges

        #Removes redundant edges from edgeList
        cleanedEdgeList = []
        if len(edgeList) > 0:
            #print("\nCleaned Edges")            
            cleanedEdgeList.append(edgeList[0])
            for edge in edgeList[1:]:
                index = edgeList.index(edge)
                if edgeList[index-1]['type'] == edgeList[index]['type']:
                    if edgeList[index]['type'] == 1:                        
                        cleanedEdgeList.pop()
                        cleanedEdgeList.append(edge)
                    #else:
                    #    print("2 zeros do nothing")
                else:
                    cleanedEdgeList.append(edge)
         
        edgeList = []
        edgeList = cleanedEdgeList

        doors = []
        if len(edgeList) > 0:
            doors = self.doorWidth(edgeList)

        self.doorToApproach = {}
        if len(doors) > 0:
            print(doors)
            closestDoor = {}
            closestDoor = min(doors, key=lambda x:x['distance'])
            self.doorToApproach = closestDoor
            return True
        return False


    def getDirBetweenEdges(self, edgeA, edgeB):

        x = (edgeA['distance'] * numpy.cos(edgeA['angle']) + edgeB['distance'] * numpy.cos(edgeB['angle']))/2 
        y = (edgeA['distance'] * numpy.sin(edgeA['angle']) + edgeB['distance'] * numpy.sin(edgeB['angle']))/2

        dirAngle = math.atan2(y, x)

        return dirAngle


    def doorWidth(self, edgeList):

        doors = []

        i = 0
        while i < len(edgeList) -1:
            if edgeList[i]['type'] == 1:
                i = i + 1
                continue

            dw = self.cosLaw(
                edgeList[i]['distance'],
                edgeList[i+1]['distance'],
                math.fabs(edgeList[i]['angle'] - edgeList[i+1]['angle'])
            )

            print("Door width {0}").format(dw)

            if dw >= self.minDoorWidth and dw <= self.maxDoorWidth:
                #print("Door founded")
                d = {}
                d['width'] = dw
                d['indexLeft'] = edgeList[i]['index']
                d['indexRight'] = edgeList[i+1]['index']
                d['index'] = int(math.floor((edgeList[i+1]['index'] + edgeList[i]['index']) / 2))
                d['distance'] = self.scan.ranges[d['index']]
                d['dir'] = ((d['index'] * self.scan.angle_increment) + self.scan.angle_min)
                d['degreeDir'] = 180*d['dir']/numpy.pi

                #Discard door to cloose
                if d['distance'] >= self.discardDoorThreshold:
                    doors.append(d)
                else:
                    print("Door too close!! DISCARDED!!!")

            i = i + 1

        return doors


    def cosLaw(self, a, b, theta):
        return math.sqrt(math.pow(a,2) + math.pow(b,2) - 2*a*b*math.cos(theta))


    def competitionEndBehavior(self):

        print("<< Competition ended!!!")
        
        if True or self.competitionEndCounter < (10 * self.loopRate):

            self.competitionEndCounter += 1

            self.computeMinRanges()
            if self.minRanges['center']['dist'] > 0.70 and not self.obstacleAvoidOn:
                self.stayInTheMiddleBehavior()
            else:
                self.obstacleAvoidanceBehavior()
        else:
            self.turn = 0.0
            self.speed = 0.0
    

    def checkCorridorWitdh(self):

        self.computeMinRanges()
        
        corridorWidth = self.scan.ranges[450] + self.scan.ranges[90]

        threshold = self.minCorridorWidthThreshold
        if self.corridorStarted:
            corridorWidth = self.minRanges['left']['dist'] + self.minRanges['right']['dist']
            threshold = threshold + 0.50#0.30

        print("Corridor width {0} - threshold {1}".format(corridorWidth, threshold))

        return corridorWidth < threshold


    def getDistFront(self):
        """
        Returns the min distance into the front 15 degrees cone
        """

        return min(self.scan.ranges[255:285])


    def exceedBehavior(self):

        print("<< Exceed - %s" %self.exceedText)        

        treshold = 1.1#1.0
        frontThreshold = 0.75
        backThreshold = 0.8#0.9#1.1

        #Exceed phase one
        if self.exceedPhaseOne:
            if self.scan.ranges[270] < backThreshold:
                print("Exceed phase 1 - Go Back")
                self.turn = 0.0
                self.speed = -0.2 
                return
            
            self.beamToFreeIndex = None                
            if self.exceedSearchDir == "left":
                self.beamToFreeIndex = 290
            elif self.exceedSearchDir == "right":
                self.beamToFreeIndex = 250

            self.exceedPhaseOne = False
            self.exceedPhaseTwo = True
        
        print("Corridor started: {0}".format(self.corridorStarted))
        print("Corridor ended: {0}".format(self.corridorEnded))
        
        if not self.corridorStarted:
            self.corridorStarted = self.checkCorridorWitdh()
        elif not self.corridorEnded:
            self.corridorEnded = not self.checkCorridorWitdh()
        else:
            #Corridor ended end of exceed behavior
            """
            self.exceed = False
            self.doorsExceeded += 1
            self.turn = -self.exceedTurnDir * 0.1
            self.speed = 0.0
            print("Exceed end!")
            """
            self.exceedPhaseFour = False
            self.exceedPhaseFive = True

        #Exceed phase two
        if self.exceedPhaseTwo:

            baseIncrement = 10
            if self.exceedSearchDir == "left":
                #Beam sequence 290 270 250 230                
                lastBeamToFree = 230#240#230
            else:
                #Beam sequence 250 270 290 310            
                lastBeamToFree = 310#300#310

            print("Turn dir {0}".format(self.exceedTurnDir))
            
            if self.scan.ranges[self.beamToFreeIndex] < treshold:# and not self.stopExceedTurn:
                print("Exceed phase 2 - Free Beams")
                print("Beam to free {1}: {0}".format(self.beamToFreeIndex, self.scan.ranges[self.beamToFreeIndex]))
                self.turn = self.exceedTurnDir * 0.2
                self.speed = 0.0
            elif self.beamToFreeIndex == lastBeamToFree:
                    print("Stop exceed turn")
                    self.exceedPhaseTwo = False
                    self.exceedPhaseTree = True
                
            else:
                #Update beam to free
                print("Beam to from {0} to {1}".format(self.beamToFreeIndex, self.beamToFreeIndex - self.exceedTurnDir * baseIncrement))
                self.beamToFreeIndex = self.beamToFreeIndex - self.exceedTurnDir * baseIncrement
                self.turn = self.exceedTurnDir * 0.2
                self.speed = 0.0
        
        #Exceed phase tree
        if self.exceedPhaseTree:

            if self.getDistFront() > frontThreshold:
                print("Exceed phase 3 - Go Straight")
                self.turn = 0.0
                self.speed = 0.2
            else:
                self.exceedPhaseTree = False
                self.exceedPhaseFour = True   
                self.wf = WallFollowing(0.45, 0.15, self.exceedTurnDir, 5, 0, 0.1, 0.2)             

        #Exceed phase four
        if self.exceedPhaseFour:

            print("Exceed phase 4 - Wall Following")      
            self.wf.computeVelocities()
            self.speed = self.wf.getLinearVelocity()
            self.turn = self.wf.getAngularVelocity()

        if self.exceedPhaseFive:

            if self.exceedSearchDir == "left":                         
                beamToFreeIndex = 300
            else:                          
                beamToFreeIndex = 240
            
            print("Exceed phase 5 - Free beam {0}".format(beamToFreeIndex))

            if self.scan.ranges[beamToFreeIndex] < treshold:
                
                
                self.turn = - self.exceedTurnDir * 0.2
                self.speed = 0.0
            else:
                self.exceedPhaseFive = False
                self.exceed = False
                self.doorsExceeded += 1
                self.turn = -self.exceedTurnDir * 0.1
                self.speed = 0.0
                print("Exceed end!")

        
        print("speed {0}".format(self.speed))
        print("turn {0}".format(self.turn))


    def waitBehavior(self):

        print("<< Wait")
        print("Alignment durin wait {0} degree".format(180*self.doorToApproach['dir']/numpy.pi))
        
        self.speed = 0.0
        self.turn = 0.0
        
        #Reset for exeed behaviors        
        self.exceedPhaseOne = True
        self.exceedPhaseTwo = False
        self.exceedPhaseTree = False
        self.exceedPhaseFour = False
        self.exceedPhaseFive = False
        self.corridorStarted = False
        self.corridorEnded = False

        #Mock codebarCallback
        if self.mockCodebar:
            self.mockCodebarCallback(randint(0,9))
            #self.mockCodebarCallback(1)


    def approachBehavior(self):

        print("DoorDetected {0}, DoorDistance {1}, DoorDirDegree {2}".format(
            self.doorDetected, 
            self.doorToApproach['distance'], 
            180*self.doorToApproach['dir']/numpy.pi)
        )
        if self.doorToApproach['distance'] <= self.minDoorDistance:
            
            if abs(self.doorToApproach['dir']) > 0.005 and self.doorAlignmentCounter < self.maxAlignmentCyces:
                self.speed = 0.0
                #Old version
                #self.turn = self.omegaMax * math.sin(10*self.doorToApproach['dir'])
                
                turnDir = numpy.sign(self.doorToApproach['dir'])
                turnModule = self.omegaMax / 2
                if abs(self.doorToApproach['dir']) < 0.05:
                    turnModule = self.omegaMax / 8
                self.turn = turnDir * turnModule
                self.doorAlignmentCounter += 1
                print("\t<< Alignment Cycle {0}".format(self.doorAlignmentCounter))
                print("speedX {0} turnZ {1}".format(self.speed,self.turn))
            else:

                #Door reached and robot aligned
                self.speed = 0.0
                self.turn = 0.0                
                self.wait = True
                self.doorAlignmentCounter = 01
                #Treshold value reset
                self.deltaXTreshold = self.deltaXTresholdBaseValue

                #Force codebar read
                self.startReadCodebar()
            
        else:
            #Goto door
            print("<< Approach")                
            
            print("Door distance {0}m".format(self.doorToApproach['distance']))
            print("Door dir {0} degrees".format(180*self.doorToApproach['dir']/numpy.pi))

            corridorWidth = self.scan.ranges[90] + self.scan.ranges[450]
            #if corridorWidth < 2.2:
            #    corridorWidth = 2.2
            #print("Corridor width: {0}m".format(corridorWidth))

            deltaX = self.scan.ranges[450] - (corridorWidth/2)
            #print("DeltaX NOT SCALED: {0}".format(deltaX))
            deltaX = deltaX / (corridorWidth/2)
            
            #print("DeltaX scaled: {0}".format(deltaX))

            if abs(deltaX) >= self.deltaXTreshold:
                #print("Min deltaX")
                self.deltaXTreshold = self.deltaXTresholdBaseValue
                self.turn = self.omegaMax * math.sin(deltaX)            
                self.speed = self.speedMax * math.cos(deltaX)                
            else:
                #print("Min door dir")

                #Set treshold with Hysteresis
                self.deltaXTreshold = self.deltaXTresholdHysteresisValue
                dirError = 5 * self.doorToApproach['dir']                
                #Error saturation
                #1.5 rad equal to 85 degree
                if dirError > 1.2:
                    dirError = 1.2

                #print("Error Dir: {0}".format(5*self.doorToApproach['dir']))
                                
                self.turn = self.omegaMax * math.sin(dirError)
                self.speed = self.speedMax * math.cos(dirError)
                if self.speed < 0.1:
                    self.speed = 0.1
           # print("deltaXTreshold {0}m".format(self.deltaXTreshold))
            print("\nspeed {0}".format(self.speed))
            print("turn {0}".format(self.turn))
            

    def computeMinRanges(self):

        turn = 0.0

        angle = 30.00
        angleLeft = numpy.pi/180.0 * -90.0    
        angleCenterLeft = numpy.pi/180.0 * -angle
        angleCenterRight = numpy.pi/180.0 * angle    
        angleRight = numpy.pi/180.0 * 90.0

        #New model
        irightBeg = 90
        irightEnd = 210
        icenterBeg = irightEnd + 1
        icenterEnd= 330
        ileftBeg = icenterEnd + 1 
        ileftEnd = 450
        
        #Clean zero beam setting them to big value
        for k in range(ileftBeg,irightEnd):
            if self.scan.ranges[k] < self.minBeamValue:
                self.scan.ranges[k] = self.minBeamValue

        distLeft = min(self.scan.ranges[ileftBeg:ileftEnd])
        ileftMin = self.scan.ranges.index(distLeft)
        distCenter = min(self.scan.ranges[icenterBeg:icenterEnd])
        icenterMin = self.scan.ranges.index(distCenter)
        distRight = min(self.scan.ranges[irightBeg:irightEnd])
        irightMin = self.scan.ranges.index(distRight)

        #Angle associate with the min distance beam
        thetaRight = self.scan.angle_min + self.scan.angle_increment * irightMin
        thetaCenter = self.scan.angle_min + self.scan.angle_increment * icenterMin
        thetaLeft = self.scan.angle_min + self.scan.angle_increment * ileftMin

        self.minRanges = {}
        #Left values
        self.minRanges['left'] = {}
        self.minRanges['left']['dist'] = distLeft
        self.minRanges['left']['index'] = ileftMin
        self.minRanges['left']['theta'] = thetaLeft

        #Center values
        self.minRanges['center'] = {}
        self.minRanges['center']['dist'] = distCenter
        self.minRanges['center']['index'] = icenterMin
        self.minRanges['center']['theta'] = thetaCenter

        #Center values
        self.minRanges['right'] = {}
        self.minRanges['right']['dist'] = distRight
        self.minRanges['right']['index'] = irightMin
        self.minRanges['right']['theta'] = thetaRight


    def stayInTheMiddleBehavior(self):

        rospy.loginfo("<< Stay in the middle")
      
        self.turn = -0.2 * (numpy.sin(self.minRanges['right']['theta']) 
                    * math.exp(-self.kappa * self.minRanges['right']['dist']) + 
                    numpy.sin(self.minRanges['left']['theta']) * 
                    math.exp(-self.kappa * self.minRanges['left']['dist']))
        self.speed= 0.15  

        
    def obstacleAvoidanceBehavior(self):

        rospy.loginfo("<< Obstacle avoid")

        if not self.obstacleAvoidOn:
            if self.minRanges['center']['theta'] > 0.0:
                self.turnrate = -1.0
            else:
                self.turnrate =  1.0

            self.obstacleAvoidOn = True
    
        elif self.minRanges['center']['dist'] >= 0.70:
            self.obstacleAvoidOn = False
  
        self.speed=0.0
        self.turn = self.turnrate * 0.2


    def mainLoop(self):

        print("\n")

        if self.scan == None or len(self.scan.ranges) == 0:
            #No data no action
            return
        
        if self.doorsExceeded == self.doorsToExceed:
            self.competitionEndBehavior()
            self.publishTwistMsg(self.speed, self.turn)
            return
                
        if self.exceed:
            self.exceedBehavior()
            self.publishTwistMsg(self.speed, self.turn)
            return
        
        if self.wait:
            self.waitBehavior()            
            self.publishTwistMsg(self.speed, self.turn)
            return

        self.doorDetected = self.doorDetector()
        if self.doorDetected:                
            self.approachBehavior()
            self.publishTwistMsg(self.speed, self.turn)
            return
        else:
            print("Door NOT detected")

        self.computeMinRanges()
        if self.minRanges['center']['dist'] > 0.70 and not self.obstacleAvoidOn:
            self.stayInTheMiddleBehavior()
        else:
            self.obstacleAvoidanceBehavior()
        
        self.publishTwistMsg(self.speed, self.turn)        


if __name__ == '__main__':
    
    try:
        #Init ros node
        rospy.init_node('RoboticsCompetition', anonymous=False)
        pc = RoboticsCompetition()
        rate = rospy.Rate(pc.getLoopRate())
    
        while not rospy.is_shutdown():
            pc.mainLoop()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass    