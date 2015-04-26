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

    # Parameters
    LOOP_RATE = 10    
    MIN_BEAM_VALUE = 0.10
    MAX_BEAM_VALUE = 30.0
    MIN_BEAM_INDEX = 90
    MAX_BEAM_INDEX = 450
    FRONT_COLLISION_THRESHOLD = 0.70
    EDGE_THRESHOLD = 0.15
    DELTA_X_THRESHOLD_BASE_VALUE = 0.1
    DELTA_X_THRESHOLD_HYSTERESIS_VALUE = 0.2    
    MIN_CORRIDOR_WIDTH_THRESHOLD = 1.40
    TURN_THRESHOLD = 3
    OMEGA_MAX = 0.2
    SPEED_MAX = 0.2
    MIN_DOOR_WIDTH = 0.20
    MAX_DOOR_WIDTH = 0.90
    DISCARD_DOOR_THRESHOLD = 0.40
    MIN_DOOR_DISTANCE = 0.75
    MAX_ALIGNMENT_CYCLES = 5
    KAPPA = 0.3

    _scan = None

    # States variables
    _obstacle_avoid_on = False
    _door_detected = False
    _wait = False
    _exceed = False

    _door_to_approach = {}
    _door_alignment_counter =0
    _exceed_text = ""
    _exceed_dir = 0.0
    _turn_rate = 0.0
    _turn = 0.0
    _speed = 0.0
    _doors_exceeded = 0  
    
    _exceed_phase_one = False
    _exceed_phase_two = False
    _exceed_phase_tree = False
    _exceed_phase_four = False
    _exceed_phase_five = False
    _exceed_turn_dir = 0
    _beam_to_free_index = None
    _wf = None
    _corridor_started = False
    _corridor_ended = False
    
    _codebar_number = -1

    _delta_x_treshold = DELTA_X_THRESHOLD_BASE_VALUE

    # Constructor
    def __init__(self):
        
        # Reads params from launch file
        self._scan_topic_param = rospy.get_param('~scan_topic', '/scan')
        self._cmd_vel_topic_param = rospy.get_param('~cmd_vel_topic', '/cmd_vel')
        self._revert_scan_param = rospy.get_param('~revert_scan', False)
        self._mock_codebar = rospy.get_param('~mock_codebar', False)
        self._doors_to_exceed = rospy.get_param('~doors_to_exceed', 3)

        rospy.loginfo("_scan_topic_param: {0}".format(self._scan_topic_param))
        rospy.loginfo("_cmd_vel_topic_param: {0}".format(self._cmd_vel_topic_param))
        rospy.loginfo("_revert_scan_param: {0}".format(self._revert_scan_param))
        rospy.loginfo("_mock_codebar: {0}".format(self._mock_codebar))
        rospy.loginfo("_doors_to_exceed: {0}".format(self._doors_to_exceed))

        # Subscriber for the laser data
        self._sub = rospy.Subscriber(self._scan_topic_param, LaserScan, self.laser_callback)

        # Publisher for Twist commands
        self._pub = rospy.Publisher(self._cmd_vel_topic_param, Twist, queue_size=10)
        
        # Subscriber for the barcode data
        self._sub_barcode = rospy.Subscriber('/codebar', Char, self.codebar_callback)

        # Publisher for barcode commands
        self._pub_barcode = rospy.Publisher('/light', Empty, queue_size=10)

        # Subscriber for the barcode command
        self._sub_barcode_command = rospy.Subscriber('/light', Empty, self.codebar_command_callback)

        time.sleep(2)
 
        # Reset codebar reader
        self._start_read_codebar()
        

    def get_loop_rate(self):

        return self.LOOP_RATE


    def laser_callback(self,scan):

        self._scan = scan
        
        if self._revert_scan_param:         
            reversed_ranges = self._scan.ranges[::-1]
            reversed_intensities = self._scan.intensities[::-1]
            self._scan.ranges = reversed_ranges
            self._scan.intensities = reversed_intensities

        #Check for 0 beam
        cleaned_ranges = [self.MIN_BEAM_VALUE]

        #From 0 to 539
        for i in range(len(self._scan.ranges) -1):
            if self._scan.ranges[i+1] < self.MIN_BEAM_VALUE:
                cleaned_ranges.append(cleaned_ranges[i])
            else:
                cleaned_ranges.append(self._scan.ranges[i+1])

        self._scan.ranges = tuple(cleaned_ranges)

        #Update wall follwer data
        if self._wf != None:
            self._wf.update_scan_data(self._scan)


    def _mock_codebar_callback(self, number):
        print("Read {0}".format(number))
        
        if number % 2 == 0:
            # Go to right
            self._exceed_text = "We must go to right"
            self._exceed_search_dir = "right"
            self._exceed_turn_dir = -1
        else:
            # Go to left
            self._exceed_text = "We must go to left"
            self._exceed_search_dir = "left"
            self._exceed_turn_dir = 1

        self._exceed = True
        self._wait = False


    def codebar_callback(self, char_msg):

        # From char to number
        number = char_msg.data

        print("Read {0}".format(number))
        if number < 0 or number > 9 or number == self._codebar_number:
            print("We must reset Codebar node")
            self._start_read_codebar()
            return

        if number % 2 == 0:
            #Go to right
            self._exceed_text = "We must go to right"
            self._exceed_search_dir = "right"
            self._exceed_turn_dir = 1
        else:
            #Go to left
            self._exceed_text = "We must go to left"
            self._exceed_search_dir = "left"
            self._exceed_turn_dir = -1

        self._exceed = True
        self._wait = False
        self._codebar_number = number
        print(self._exceed_text)


    def codebar_command_callback(self, msg):

        print("Detected Empty msg on /light topic")


    def _start_read_codebar(self):

        while self._pub_barcode.get_num_connections() == 0:
            print("Wait for /light Subscriber")
        print("Founded /light Subscriber")

        self._pub_barcode.publish()


    def _print_laser_scan(self):

        if self._scan != None:
            print("\n\n")
            for i in range(self.MIN_BEAM_INDEX, self.MAX_BEAM_INDEX):
                rospy.loginfo('Range {0}: {1}m'.format(i, self._scan.ranges[i]))


    def _publish_twist_msg(self, speed_x, turn_z):

        self._speed = speed_x
        self._turn = turn_z

        command = Twist()

        command.linear.x = self._speed
        command.linear.y = 0.0
        command.linear.z = 0.0
        command.angular.x = 0.0
        command.angular.y = 0.0
        command.angular.z = self._turn

        self._pub.publish(command)


    def _door_detector(self):

        all_edges = []

        # Set to TURN_THRESHOLD beams greater that turnThreshold
        zeroed_ranges = []
        for i in range(len(self._scan.ranges)):
            if self._scan.ranges[i] < self.TURN_THRESHOLD:
                zeroed_ranges.append(self._scan.ranges[i])
            else:
                zeroed_ranges.append(self.TURN_THRESHOLD)

        
        # Finds abrupt change in the data and puts it in the first tier of edges
        for i in range(self.MIN_BEAM_INDEX, self.MAX_BEAM_INDEX - 1):

            temp_poi = {}
            #    __
            #   |
            # __|
            if zeroed_ranges[i+1] - zeroed_ranges[i] < -self.EDGE_THRESHOLD:
                #temp_poi['distance'] = zeroed_ranges[i]
                #temp_poi['angle'] = i*self._scan.angle_increment
                #temp_poi['index'] = i
                temp_poi['distance'] = zeroed_ranges[i+1]                
                temp_poi['angle'] = (i + 1) * self._scan.angle_increment  
                temp_poi['index'] = i + 1
                temp_poi['type'] = 0
                all_edges.append(temp_poi)

            elif zeroed_ranges[i+1] - zeroed_ranges[i] > self.EDGE_THRESHOLD:
                # __  
                #   |
                #   |__
                #temp_poi['distance'] = zeroed_ranges[i+1]
                #temp_poi['angle'] = (i + 1)*self._scan.angle_increment
                #temp_poi['index'] = i + 1
                temp_poi['distance'] = zeroed_ranges[i]                
                temp_poi['angle'] = i*self._scan.angle_increment                
                temp_poi['index'] = i
                temp_poi['type'] = 1
                all_edges.append(temp_poi)

        edge_list = []        
        edge_list = all_edges

        # Removes redundant edges from edge_list
        cleaned_edge_list = []
        if len(edge_list) > 0:
                    
            cleaned_edge_list.append(edge_list[0])
            for edge in edge_list[1:]:
                index = edge_list.index(edge)
                if edge_list[index - 1]['type'] == edge_list[index]['type']:
                    if edge_list[index]['type'] == 1:                        
                        cleaned_edge_list.pop()
                        cleaned_edge_list.append(edge)
                else:
                    cleaned_edge_list.append(edge)
         
        edge_list = []
        edge_list = cleaned_edge_list

        doors = []
        if len(edge_list) > 0:
            doors = self._door_width(edge_list)

        self._door_to_approach = {}
        if len(doors) > 0:
            print(doors)
            closest_door = {}
            closest_door = min(doors, key=lambda x:x['distance'])
            self._door_to_approach = closest_door
            return True
        return False


    def _get_dir_between_dges(self, edge_a, edge_b):

        x = (edge_a['distance'] * numpy.cos(edge_a['angle']) + edge_b['distance'] * numpy.cos(edge_b['angle']))/2 
        y = (edge_a['distance'] * numpy.sin(edge_a['angle']) + edge_b['distance'] * numpy.sin(edge_b['angle']))/2

        angle_dir = math.atan2(y, x)

        return angle_dir


    def _door_width(self, edge_list):

        doors = []

        i = 0
        while i < len(edge_list) - 1:
            if edge_list[i]['type'] == 1:
                i = i + 1
                continue

            dw = self._cos_law(
                edge_list[i]['distance'],
                edge_list[i + 1]['distance'],
                math.fabs(edge_list[i]['angle'] - edge_list[i + 1]['angle'])
            )

            print("Door width {0}").format(dw)

            if dw >= self.MIN_DOOR_WIDTH and dw <= self.MAX_DOOR_WIDTH:
                #print("Door founded")
                d = {}
                d['width'] = dw
                d['index_left'] = edge_list[i]['index']
                d['index_right'] = edge_list[i + 1]['index']
                d['index'] = int(math.floor((edge_list[i + 1]['index'] + edge_list[i]['index']) / 2))
                d['distance'] = self._scan.ranges[d['index']]
                d['dir'] = ((d['index'] * self._scan.angle_increment) + self._scan.angle_min)
                d['degree_dir'] = 180*d['dir']/numpy.pi

                #Discard door to cloose
                if d['distance'] >= self.DISCARD_DOOR_THRESHOLD:
                    doors.append(d)
                else:
                    print("Door too close!! DISCARDED!!!")

            i = i + 1

        return doors


    def _cos_law(self, a, b, theta):

        return math.sqrt(math.pow(a,2) + math.pow(b,2) - 2*a*b*math.cos(theta))


    def _competition_end_behavior(self):

        print("<< Competition ended!!!")
        
        self._compute_min_ranges()
        if self._min_ranges['center']['dist'] > 0.70 and not self._obstacle_avoid_on:
            self._stay_in_the_middle_behavior()
        else:
            self._obstacle_avoidance_behavior()


    def _check_corridor_witdh(self):

        self._compute_min_ranges()
        
        corridor_width = self._scan.ranges[self.MIN_BEAM_INDEX] + self._scan.ranges[self.MAX_BEAM_INDEX]

        threshold = self.MIN_CORRIDOR_WIDTH_THRESHOLD
        if self._corridor_started:
            corridor_width = self._min_ranges['left']['dist'] + self._min_ranges['right']['dist']
            threshold = threshold + 0.50#0.30

        print("Corridor width {0} - threshold {1}".format(corridor_width, threshold))

        return corridor_width < threshold


    def _get_dist_front(self):
        """
        Returns the min distance into the front 15 degrees cone
        """

        return min(self._scan.ranges[255:285])


    def _exceed_behavior(self):

        print("<< Exceed - %s" %self._exceed_text)        

        treshold = 1.1#1.0
        front_threshold = 0.75
        back_threshold = 0.8#0.9#1.1

        # Exceed phase one
        if self._exceed_phase_one:
            if self._scan.ranges[270] < back_threshold:
                print("Exceed phase 1 - Go Back")
                self._turn = 0.0
                self._speed = -0.2 
                return
            
            self._beam_to_free_index = None                
            if self._exceed_search_dir == "left":
                self._beam_to_free_index = 290
            elif self._exceed_search_dir == "right":
                self._beam_to_free_index = 250

            self._exceed_phase_one = False
            self._exceed_phase_two = True
        
        print("Corridor started: {0}".format(self._corridor_started))
        print("Corridor ended: {0}".format(self._corridor_ended))
        
        if not self._corridor_started:
            self._corridor_started = self._check_corridor_witdh()
        elif not self._corridor_ended:
            self._corridor_ended = not self._check_corridor_witdh()
        else:
            # Corridor ended end of exceed phase four
            
            self._exceed_phase_four = False
            self._exceed_phase_five = True

        # Exceed phase two
        if self._exceed_phase_two:

            base_increment = 10
            if self._exceed_search_dir == "left":
                # Beam sequence 290 270 250 230                
                last_beam_to_free = 230#240#230
            else:
                # Beam sequence 250 270 290 310            
                last_beam_to_free = 310#300#310

            print("Turn dir {0}".format(self._exceed_turn_dir))
            
            if self._scan.ranges[self._beam_to_free_index] < treshold:
                print("Exceed phase 2 - Free Beams")
                print("Beam to free {1}: {0}".format(self._beam_to_free_index, self._scan.ranges[self._beam_to_free_index]))
                self._turn = self._exceed_turn_dir * 0.2
                self._speed = 0.0
            elif self._beam_to_free_index == last_beam_to_free:
                    print("Stop exceed turn")
                    self._exceed_phase_two = False
                    self._exceed_phase_tree = True
                
            else:
                #Update beam to free
                print("Beam to from {0} to {1}".format(self._beam_to_free_index, self._beam_to_free_index - self._exceed_turn_dir * base_increment))
                self._beam_to_free_index = self._beam_to_free_index - self._exceed_turn_dir * base_increment
                self._turn = self._exceed_turn_dir * 0.2
                self._speed = 0.0
        
        # Exceed phase tree
        if self._exceed_phase_tree:

            if self._get_dist_front() > front_threshold:
                print("Exceed phase 3 - Go Straight")
                self._turn = 0.0
                self._speed = 0.2
            else:
                self._exceed_phase_tree = False
                self._exceed_phase_four = True   
                self._wf = WallFollowing(0.45, 0.15, self._exceed_turn_dir, 5, 0, 0.1, 0.2)             

        # Exceed phase four
        if self._exceed_phase_four:

            print("Exceed phase 4 - Wall Following")      
            self._wf.compute_velocities()
            self._speed = self._wf.get_linear_velocity()
            self._turn = self._wf.get_angular_velocity()

        if self._exceed_phase_five:

            if self._exceed_search_dir == "left":                         
                self._beam_to_free_index = 300
            else:                          
                self._beam_to_free_index = 240
            
            print("Exceed phase 5 - Free beam {0}".format(self._beam_to_free_index))

            if self._scan.ranges[self._beam_to_free_index] < treshold:                
                
                self._turn = - self._exceed_turn_dir * 0.2
                self._speed = 0.0
            else:
                self._exceed_phase_five = False
                self._exceed = False
                self._doors_exceeded += 1
                self._turn = -self._exceed_turn_dir * 0.1
                self._speed = 0.0
                print("Exceed end!")

        
        print("speed {0}".format(self._speed))
        print("turn {0}".format(self._turn))


    def _wait_behavior(self):

        print("<< Wait")
        print("Alignment durin wait {0} degree".format(180*self._door_to_approach['dir']/numpy.pi))
        
        self._speed = 0.0
        self._turn = 0.0
        
        # Reset for exeed behaviors        
        self._exceed_phase_one = True
        self._exceed_phase_two = False
        self._exceed_phase_tree = False
        self._exceed_phase_four = False
        self._exceed_phase_five = False
        self._corridor_started = False
        self._corridor_ended = False

        # Mock codebarCallback
        if self._mock_codebar:
            self._mock_codebar_callback(randint(0,9))


    def _approach_behavior(self):

        print("DoorDetected {0}, DoorDistance {1}, DoorDirDegree {2}".format(
            self._door_detected, 
            self._door_to_approach['distance'], 
            180*self._door_to_approach['dir']/numpy.pi)
        )
        if self._door_to_approach['distance'] <= self.MIN_DOOR_DISTANCE:
            
            if abs(self._door_to_approach['dir']) > 0.005 and self._door_alignment_counter < self.MAX_ALIGNMENT_CYCLES:
                self._speed = 0.0
                # Old version
                # self._turn = self.OMEGA_MAX * math.sin(10*self._door_to_approach['dir'])
                
                turn_dir = numpy.sign(self._door_to_approach['dir'])
                turn_module = self.OMEGA_MAX / 2
                if abs(self._door_to_approach['dir']) < 0.05:
                    turn_module = self._door_to_approach / 8
                self._turn = turn_dir * turn_module
                self._door_alignment_counter += 1
                print("\t<< Alignment Cycle {0}".format(self._door_alignment_counter))
                print("speedX {0} turnZ {1}".format(self._speed, self._turn))
            else:

                # Door reached and robot aligned
                self._speed = 0.0
                self._turn = 0.0                
                self._wait = True
                self._door_alignment_counter = 1
                # Treshold value reset
                self._delta_x_treshold = self.DELTA_X_THRESHOLD_BASE_VALUE

                # Force codebar read
                self._start_read_codebar()
            
        else:
            #Goto door
            print("<< Approach")                
            
            print("Door distance {0}m".format(self._door_to_approach['distance']))
            print("Door dir {0} degrees".format(180*self._door_to_approach['dir']/numpy.pi))

            corridor_width = self._scan.ranges[self.MIN_BEAM_INDEX] + self._scan.ranges[self.MAX_BEAM_INDEX]
           
            delta_x = self._scan.ranges[self.MAX_BEAM_INDEX] - (corridor_width/2)
            
            # Scaled corridor width
            delta_x = delta_x / (corridor_width/2)
            
            if abs(delta_x) >= self._delta_x_treshold:
                #print("Min delta_x")

                self._delta_x_treshold = self.DELTA_X_THRESHOLD_BASE_VALUE
                self._turn = self.OMEGA_MAX * math.sin(delta_x)            
                self._speed = self.SPEED_MAX * math.cos(delta_x)                
            else:
                #print("Min door dir")

                # Set treshold with Hysteresis
                self._delta_x_treshold = self.DELTA_X_THRESHOLD_BASE_VALUE
                dir_error = 5 * self._door_to_approach['dir']                
                
                # Error saturation
                # 1.5 rad equal to 85 degree
                if dir_error > 1.2:
                    dir_error = 1.2

                #print("Error Dir: {0}".format(5*self._door_to_approach['dir']))
                                
                self._turn = self.OMEGA_MAX * math.sin(dir_error)
                self._speed = self.SPEED_MAX * math.cos(dir_error)
                if self._speed < 0.1:
                    self._speed = 0.1
            #print("deltaXTreshold {0}m".format(self._delta_x_treshold))
            print("\n_speed {0}".format(self._speed))
            print("_turn {0}".format(self._turn))
            

    def _compute_min_ranges(self):

        turn = 0.0

        angle = 30.00
        angle_left = numpy.pi/180.0 * -90.0    
        angle_center_left = numpy.pi/180.0 * -angle
        angle_center_right = numpy.pi/180.0 * angle    
        angle_right = numpy.pi/180.0 * 90.0

        #New model
        i_right_beg = self.MIN_BEAM_INDEX
        i_right_end = 210
        i_center_beg = i_right_end + 1
        i_center_end= 330
        i_left_beg = i_center_end + 1 
        i_left_end = self.MAX_BEAM_INDEX
        
        #Clean zero beam setting them to big value
        for k in range(i_left_beg, i_right_end):
            if self._scan.ranges[k] < self.MIN_BEAM_VALUE:
                self._scan.ranges[k] = self.MIN_BEAM_VALUE

        dist_left = min(self._scan.ranges[i_left_beg:i_left_end])
        i_left_min = self._scan.ranges.index(dist_left)
        dist_center = min(self._scan.ranges[i_center_beg:i_center_end])
        i_center_min = self._scan.ranges.index(dist_center)
        dist_right = min(self._scan.ranges[i_right_beg:i_right_end])
        i_right_min = self._scan.ranges.index(dist_right)

        #Angle associate with the min distance beam
        theta_right = self._scan.angle_min + self._scan.angle_increment * i_right_min
        theta_center = self._scan.angle_min + self._scan.angle_increment * i_center_min
        theta_left = self._scan.angle_min + self._scan.angle_increment * i_left_min

        self._min_ranges = {}
        #Left values
        self._min_ranges['left'] = {}
        self._min_ranges['left']['dist'] = dist_left
        self._min_ranges['left']['index'] = i_left_min
        self._min_ranges['left']['theta'] = theta_left

        #Center values
        self._min_ranges['center'] = {}
        self._min_ranges['center']['dist'] = dist_center
        self._min_ranges['center']['index'] = i_center_min
        self._min_ranges['center']['theta'] = theta_center

        #Center values
        self._min_ranges['right'] = {}
        self._min_ranges['right']['dist'] = dist_right
        self._min_ranges['right']['index'] = i_right_min
        self._min_ranges['right']['theta'] = theta_right


    def _stay_in_the_middle_behavior(self):

        rospy.loginfo("<< Stay in the middle")
      
        self._turn = -0.2 * (numpy.sin(self._min_ranges['right']['theta']) 
                    * math.exp(-self.KAPPA * self._min_ranges['right']['dist']) + 
                    numpy.sin(self._min_ranges['left']['theta']) * 
                    math.exp(-self.KAPPA * self._min_ranges['left']['dist']))
        self._speed= 0.15  

        
    def _obstacle_avoidance_behavior(self):

        rospy.loginfo("<< Obstacle avoid")

        if not self._obstacle_avoid_on:
            if self._min_ranges['center']['theta'] > 0.0:
                self._turn_rate = -1.0
            else:
                self._turn_rate =  1.0

            self._obstacle_avoid_on = True
    
        elif self._min_ranges['center']['dist'] >= self.FRONT_COLLISION_THRESHOLD:
            self._obstacle_avoid_on = False
  
        self._speed = 0.0
        self._turn = self._turn_rate * 0.2


    def main_loop(self):

        print("\n")

        if self._scan == None or len(self._scan.ranges) == 0:
            #No data no action
            return
        
        if self._doors_exceeded == self._doors_to_exceed:
            self._competition_end_behavior()
            self._publish_twist_msg(self._speed, self._turn)
            return
                
        if self._exceed:
            self._exceed_behavior()
            self._publish_twist_msg(self._speed, self._turn)
            return
        
        if self._wait:
            self._wait_behavior()            
            self._publish_twist_msg(self._speed, self._turn)
            return

        self._door_detected = self._door_detector()
        if self._door_detected:                
            self._approach_behavior()
            self._publish_twist_msg(self._speed, self._turn)
            return
        else:
            print("Door NOT detected")

        self._compute_min_ranges()
        if self._min_ranges['center']['dist'] > self.FRONT_COLLISION_THRESHOLD and not self._obstacle_avoid_on:
            self._stay_in_the_middle_behavior()
        else:
            self._obstacle_avoidance_behavior()
        
        self._publish_twist_msg(self._speed, self._turn)        


if __name__ == '__main__':
    
    try:
        #Init ros node
        rospy.init_node('RoboticsCompetition', anonymous=False)
        rc = RoboticsCompetition()
        rate = rospy.Rate(rc.get_loop_rate())
    
        while not rospy.is_shutdown():
            rc.main_loop()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass    