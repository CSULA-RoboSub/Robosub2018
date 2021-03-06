import utils
import time
import GateClassifier as gc
import GatePreprocessor as gp
import cv2

class GateDetector:

    def __init__(self):
        self.classifier = gc.GateClassifier()
        self.found =  False
        self.preprocess = gp.GatePreprocessor()
        self.directions = [0,0]
        self.isTaskComplete = False
        self.shapes = {1: "vertical", 2: "horizontal", 3: "square"} # so we can change names quicker
        self.shape_buffer = 15
        self.shape_list = []
        self.is_direction_center = True
        self.is_red_left = False
        self.frame_size = (744, 480)

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

    ## MARK's DICE METHOD
    def find_pips(self, img):
    # Set up the detector with default parameters.
        detector = cv2.SimpleBlobDetector_create()
        # Detect blobs.
        keypoints = detector.detect(img)
        
        return keypoints

    ## MARK's helper
    def get_crop_from_bounding_box(self, img, box):
        x, y, w, h = box
        return img[y:y+h, x:x+w]

    # now returns (found, directions, shape-of-roi, size)
    def detect(self, frame):
        if frame is not None:
            height, width, ch = frame.shape
            center = (width / 2, height / 2)
            regions_of_interest = self.preprocess.get_interest_regions(frame)

            for x, y, w, h in regions_of_interest:
                cv2.rectangle(frame, (x, y), (x + w, y + h), utils.colors["red"], 2)

            gate = self.classifier.classify(frame, regions_of_interest)


            gate_shape = self.get_shape(gate, self.shape_buffer)

            if gate_shape == self.shapes[3] or gate_shape == self.shapes[1]:
                gate = None

            if (gate == None):
                self.directions = [0, 0]
                self.found = False
                gate_shape = None
                w, h = 0, 0
            else:
                x, y, w, h = gate
                cv2.rectangle(frame, (x, y), (x + w, y + h), utils.colors["blue"], 6)

                w_pad = w / 7
                h_pad = h / 7
                if self.is_direction_center:
                    self.directions = utils.get_directions(center, x, y, w, h)
                    cv2.rectangle(frame, (x + (3*w_pad), y + (3*h_pad)), (x + (4 * w_pad), y + (4 * h_pad)), utils.colors["green"], 2)
                else:
                    if self.is_red_left:
                        self.directions = utils.get_directions_left(center, x, y, w, h)
                        cv2.rectangle(frame, (x + (2*w_pad), y + (3*h_pad)), (x + (3 * w_pad), y + (4 * h_pad)), utils.colors["green"], 2)
                    else:
                        self.directions = utils.get_directions_right(center, x, y, w, h)
                        cv2.rectangle(frame, (x + (4*w_pad), y + (3*h_pad)), (x + (5 * w_pad), y + (4 * h_pad)), utils.colors["green"], 2)
                self.found = True
            return (self.found, self.directions, gate_shape, (w, h))
        else:
            print('error no frame')
            return False, None, None, None
