import utils
import time
import PathClassifier as pc
import PathPreprocessor as pp
import cv2
import math

class PathFollowDetector():

    def __init__(self):
        self.classifier = pc.PathClassifier()
        self.preprocess = pp.PathPreprocessor()
        self.found = False
        self.directions = [0,0,0]
        self.isTaskComplete = False
        self.shapes = {1: "vertical", 2: "horizontal", 3: "square"} # so we can change names quicker
        self.shape_buffer = 15
        self.shape_list = []
        self.rotation_buffer = 5

    # takes a single-roi coordinate as (x, y, w, h) and a buffer as an int
    # returns the shape as a string
    def get_shape(self, roi, buff):
        if roi == None:
            return None
        else:
            x, y, w, h = roi

        #if ( (h >= (w + buff) ) or (h >= (w - buff) )):
        if h - w > buff:
            return self.shapes[1] # vertical
        #elif ( (h <= (w + buff) ) or (h <= (w - buff) )):
        elif w - h > buff:    
            return self.shapes[2] # horizontal
        else:
            return self.shapes[3] # square



    def detect(self, frame):
        height, width, ch = frame.shape
        center = (width / 2, height / 2)
        regions_of_interest = self.preprocess.get_interest_regions(frame)
        
        if regions_of_interest:
            path = utils.get_max_area(regions_of_interest)
            x, y, w, h = path
            split1 = frame[y : y + h/2, x : x+w]
            split2 = frame[y+h/2 : y+h, x : x+w]

            split1_path = utils.get_max_area(self.preprocess.get_interest_regions(split1))
            split2_path = utils.get_max_area(self.preprocess.get_interest_regions(split2))

            rotation_direction = self.get_rotation_direction(split1_path, split2_path)
        else:
            path = None
            rotation_direction = 0

        for roi in regions_of_interest:
            utils.draw_red_box(frame, roi)        
            
        path_shape = self.get_shape(path, self.shape_buffer)
        
        if (path == None):
            self.directions = [0, 0, 0]
            self.found = False
            w, h = 0, 0
        else:
            x, y, w, h = path
            split1_path = (split1_path[0] + x, split1_path[1] + y, split1_path[2], split1_path[3])
            split2_path = (split2_path[0] + x, split2_path[1] + y+h/2, split2_path[2], split2_path[3])
            
            utils.draw_blue_box(frame, path)
            utils.draw_green_box(frame, split1_path)
            utils.draw_green_box(frame, split2_path)
            self.directions = utils.get_directions_bottom(center, x, y, w, h)
            self.directions.append(rotation_direction)
            self.found = True
        return (self.found, self.directions, path_shape, (w, h))

    def get_rotation_direction(self, split1_path, split2_path):
        s1_xl, s1_y, s1_w, s1_h = split1_path
        s2_xl, s2_y, s2_w, s2_h = split2_path
        s1_xr = s1_xl + s1_w
        s2_xr = s2_xl + s2_w

        # print('s1_xl: {}, s1_xr: {} -- s2_xl: {}, s2_xr: {}'.format(s1_xl, s1_xr, s2_xl, s2_xr))
        
        if (s1_xl-self.rotation_buffer < s2_xl < s1_xl+self.rotation_buffer) and (s1_xr-self.rotation_buffer < s2_xr < s1_xr+self.rotation_buffer):
            #both within buffer go straight
            # print 'go straight'
            return 0
        #ruled out straight case 
        elif s2_xl <= s1_xl and s2_xr <= s1_xr:
            #path slanted right blow sub, rotate right
            # print 'rotate right'
            return 1
        elif s2_xl >= s1_xl and s2_xr >= s1_xr:
            #path slanted left rotate left
            # print 'rotate left'
            return -1
        else:
            # print('Error in PathFollowDetector get_rotation_direction()')
            # print 'go straight'
            return 0