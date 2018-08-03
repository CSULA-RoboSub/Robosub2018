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

    # now returns (found, directions, shape-of-roi, size)
    def detect(self, frame):
        if frame is not None:
            height, width, ch = frame.shape
            center = (width / 2, height / 2)
            regions_of_interest = self.preprocess.get_interest_regions(frame)

            for x, y, w, h in regions_of_interest:
                cv2.rectangle(frame, (x, y), (x + w, y + h), utils.colors["red"], 2)

            classified_gate = self.classifier.classify(frame, regions_of_interest)

            X_df, y_df = self.preprocess.create_dataset(classified_gate) # not using y_df

            contour_pairs = self.preprocess.nearest_neighbors(X_df)

            converted_pairs = self.preprocess.create_pairs(contour_pairs)

            roi_pairs = self.preprocess.return_box_pairs(classified_gate, converted_pairs)

            gate = self.detect_whole_gate(roi_pairs, self.shapes[1])

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

    def detect_whole_gate(self, interest_regions, shape):
        if interest_regions:

            area_max = 0
            area_max2 = 0
            len_rois = len(interest_regions)
            if len_rois > 0:
                counted_rois = None
                for i in range(0, len_rois):
                    area_mult = 0.4

                    cx, cy, cw, ch = interest_regions[i][0]
                    cx2, cy2, cw2, ch2 = interest_regions[i][1]

                    carea = float(cw) * float(ch)
                    neighbor_area_buffer = carea * area_mult
                    area_check_upper = carea + neighbor_area_buffer
                    area_check_lower = carea - neighbor_area_buffer
                    carea2 = float(cw2) * float(ch2)
                    neighbor_area_buffer2 = carea2 * area_mult
                    area_check_upper2 = carea2 + neighbor_area_buffer2
                    area_check_lower2 = carea2 - neighbor_area_buffer2

                    max_carea = max(carea, carea2)

                    if (max_carea >= area_max or max_carea >= area_max2) and (area_check_lower2 <= (cw*ch) <= area_check_upper2) and self.get_shape(interest_regions[i][0]) == shape and self.get_shape(interest_regions[i][1]) == shape:
                        # neighbor_count += 1
                        counted_rois = interest_regions[0]
                        area_max = carea
                        area_max2 = carea2

                min_x = self.frame_size[0]
                min_y = self.frame_size[1]
                max_x = 0
                max_y = 0
                max_w = 0
                max_h = 0

                for cr in counted_rois:
                    if min_x > cr[0]:
                        min_x = cr[0]
                    if min_y > cr[1]:
                        min_y = cr[1]
                    if max_x + max_w < cr[0] + cr[2]:
                        max_x = cr[0]
                        max_w = cr[2]
                    if max_y + max_h < cr[1] + cr[3]:
                        max_y = cr[1]
                        max_h = cr[3]

                if counted_rois is not None:
                    w_ret = max_x - min_x + max_w
                    h_ret = max_y - min_y + max_h

                    return (min_x, min_y, w_ret, h_ret)

        return None
