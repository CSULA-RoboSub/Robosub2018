import rospy
import numpy as np
import math
from pathfinder_dvl.msg import DVL
from ez_async_data.msg import Rotation
from std_msgs.msg import Float32

class Waypoint():
            
    def __init__(self):
        self.dvl_msg = DVL()
        self.heading = None
        self.depth = None
        self.waypoint_list = []

        # rospy.init_node('waypoint_node', anonymous=True)
        rospy.Subscriber('dvl_status', DVL, self.dvl_callback, queue_size=1)
        rospy.Subscriber('current_depth', Float32, self.depth_callback, queue_size=1)
        rospy.Subscriber('current_rotation', Rotation, self.rotation_callback, queue_size=1)
        # rospy.Subscriber('dvl_heading', Float32, self.rotation_callback, queue_size=1)

        self.directions = {
            0 : 'left',
            1 : 'staying',
            2 : 'right'
        }

    #store information DVL is sending for later use
    def dvl_callback(self, dvl_msg):
        #x is east axis y is north axis, dvl uses compass north east as axis
        #currently in meters
        self.dvl_msg.xpos = dvl_msg.xpos
        self.dvl_msg.xvel = dvl_msg.xvel
        self.dvl_msg.ypos = dvl_msg.ypos
        self.dvl_msg.yvel = dvl_msg.yvel
        self.dvl_msg.zpos = dvl_msg.zpos
        self.dvl_msg.zvel = dvl_msg.zvel

    #rename functions to be descriptive, make a new function to convert degrees
    def rotation_callback(self, rotation_msg):
        #yaw value from imu will be +- 180 deg, so convert to match dvl 0-360
        if 90 <= rotation_msg.yaw and rotation_msg.yaw <= 180:
            heading = rotation_msg.yaw - 90
        else:
            heading = rotation_msg.yaw + 270
        self.heading = heading

    def depth_callback(self, depth):
        #currently in feet
        self.depth = depth.data

    def is_empty(self):
        if self.waypoint_list:
            return False
        
        return True

    def clear_all(self):
        self.waypoint_list = []

    #operate waypoint_queue like a queue using slot 0 as front
    def enqueue(self, xpos, ypos, depth):
        if xpos and ypos:
            self.waypoint_list.append([xpos, ypos, depth])

    def dequeue(self):
        if self.waypoint_list:
            ret = self.waypoint_list.pop(0)
            #returns tuple of x, y
            return ret[0], ret[1], ret[2]

        return None, None, None

    def peek_front(self):
        if self.waypoint_list:
            ret = self.waypoint_list[0]
            #returns tuple of x, y
            return ret[0], ret[1], ret[2]

        return None, None, None

    def enqueue_current_position(self):
        cur_x, cur_y, cur_depth = self.get_position()
        if cur_x and cur_y and cur_depth:
            print('queued (x,y,depth): %.2f, %.2f, %.2f' %(cur_x,cur_y,cur_depth))
            self.enqueue(cur_x, cur_y, cur_depth)

    #operate waypoint_queue like a stack using the end as the top
    def push(self, xpos, ypos, depth):
        if xpos and ypos:
            self.waypoint_list.append([xpos, ypos, depth])

    def pop(self):
        if self.waypoint_list:
            ret = self.waypoint_list.pop()
            #returns tuple of x, y
            return ret[0], ret[1], ret[2]

        return None, None, None

    def peek_top(self):
        if self.waypoint_list:
            ret = self.waypoint_list[-1]
            #returns tuple of x, y
            return ret[0], ret[1], ret[2]

        return None, None, None

    def push_current_position(self):
        cur_x, cur_y, cur_depth = self.get_position()
        if cur_x and cur_y and cur_depth:
            print('pushed (x,y,depth): %.2f, %.2f, %.2f' %(cur_x,cur_y,cur_depth))
            self.push(cur_x, cur_y, cur_depth)

######################## waypoint getters ########################
    def get_position(self):
        if self.dvl_msg.xpos and self.dvl_msg.ypos:
            return self.dvl_msg.xpos, self.dvl_msg.ypos, self.depth

        return None, None, None
    
    def get_depth(self):
        return self.depth
        
    def get_dvl_yaw(self):
        return self.heading

    def get_depth_directions(self, new_depth):
        if new_depth is None or self.depth is None:
            return 'staying', 0
        direction_val = new_depth - self.depth
        if direction_val > 0:
            direction = 'down'
            distance = direction_val
        elif direction_val < 0:
            direction = 'up'
            distance = -direction_val
        else:
            direction = 'staying'
            distance = 0
        return direction, distance

    def get_directions(self, x2, y2):
        #sub needs left or right, degree amount, and distance
        if not x2 or not y2:
            return None, None, None
        x1 = self.dvl_msg.xpos
        y1 = self.dvl_msg.ypos
        #theta = 
        direction_degree = math.atan2(y2-y1, x2-x1) * 180 / np.pi
        # dvl_yaw = 0

        #convert to degrees dvl uses
        if direction_degree >= 0:
            direction_degree = 180 - direction_degree
        else:
            direction_degree = -180 - direction_degree
            
        if 90 <= direction_degree and direction_degree <= 180:
            dvl_yaw = direction_degree - 90
        else:
            dvl_yaw = direction_degree + 270
        # print('dvl_yaw: %.2f' %(dvl_yaw))
        # print('current_yaw %.2f' %self.heading)
        yaw_diff = dvl_yaw - self.heading

        l1 = max(x1,x2) - min(x1,x2)
        l2 = max(y1,y2) - min(y1,y2)
        distance = math.sqrt(l1*l1 + l2*l2)

        if yaw_diff > 0:
            if yaw_diff > 180:
                #if yaw_diff is greater than 180 then rotation is left and 360-yaw_diff degrees
                degree = 360 - yaw_diff
                direction = self.directions[0]
            else:
                degree = yaw_diff
                direction = self.directions[2]
        elif yaw_diff < 0:
            if yaw_diff < -180:
                degree = 360 + yaw_diff
                direction = self.directions[2]
            else:
                degree = -yaw_diff
                direction = self.directions[0]
        else:
            degree = 0
            direction = self.directions[1]

        return direction, degree, distance

    def get_distance(self, x2, y2):
        x1 = self.dvl_msg.xpos
        y1 = self.dvl_msg.ypos
        l1 = max(x1,x2) - min(x1,x2)
        l2 = max(y1,y2) - min(y1,y2)
        distance = math.sqrt(l1*l1 + l2*l2)

        return distance

    def get_directions_with_heading(self, to_heading):
        if to_heading is None or self.heading is None:
            # print('error to_heading: {} self.heading: {}'.format(to_heading, self.heading))
            return self.directions[1], 0

        heading_diff = to_heading - self.heading
        # print('heading_diff: {}'.format(heading_diff))
        direction = self.directions[1]

        if heading_diff > 180:
            direction = self.directions[0]
            heading_diff = 360 - heading_diff
        elif heading_diff <= -180:
            direction = self.directions[2]
            heading_diff = 360 + heading_diff

        #default no direction
        elif heading_diff < 0:
            #direction left
            direction = self.directions[0]
        elif heading_diff > 0:
            #direction right
            direction = self.directions[2]

        return direction, abs(heading_diff)

        
