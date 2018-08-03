import rospy
import math
import cv2
import sys
import os
import time
import gi
import threading
import copy
import numpy as np
import datetime

from threading import Thread

from modules.sensors.computer_vision import GateDetector
# from modules.sensors.computer_vision import BuoyDetector
from modules.sensors.computer_vision import DiceDetector
from modules.sensors.computer_vision import PathDetector
from modules.sensors.computer_vision import PathFollowDetector
from modules.sensors.computer_vision import RouletteDetector
from modules.sensors.computer_vision import SlotsDetector

try:
    gi.require_version("Tcam", "0.1")
    gi.require_version("Gst", "1.0")

    from gi.repository import Tcam, Gst, GLib, GObject
except:
    print '*******unable to import sub camera drivers*******'


class CVController():
    def __init__(self):
        ###########################################
        # DEBUG MODE SHOW OUTPUT
        self.is_debug = True
        ###########################################
        # CAMERA MODE
        self.is_camera = True
        self.is_camera_640x480 = False
        ################ INSTANCES ################
        # self.buoydetector = BuoyDetector.BuoyDetector()
        self.gatedetector = GateDetector.GateDetector()
        self.pathdetector = PathDetector.PathDetector()
        self.pathfollowdetector = PathFollowDetector.PathFollowDetector()
        self.dicedetector = DiceDetector.DiceDetector()
        # self.chipdetector = ChipDetector.ChipDetector()
        self.roulettedetector = RouletteDetector.RouletteDetector()
        self.slotsdetector = SlotsDetector.SlotsDetector()
        # self.pingerdetector = PingerDetector.PingerDetector()
        # self.cashindetector = CashInDetector.CashInDetector()

        ################ FPS COUNTER ################
        self.fps_output = 15

        ################ CAMERA FRAME ################
        self.current_raw_frame = None
        self.current_processed_frame = None

        ################ DICTIONARIES ################
        self.tasks = {
            'gate': self.gatedetector,
            'path': self.pathdetector,
            'path_follow': self.pathfollowdetector,
            'dice': self.dicedetector,
            'slots': self.slotsdetector,
            'roulette': self.roulettedetector
        }
            # 'chip': self.chipdetector,

            # 'pinger_b': self.pingerdetector,
            # 'pinger_a': self.pingerdetector,
            # 'cash_in': self.cashindetector

        # set.models = {

        # }

        self.camera_start_dictionary = {
            0: self.opencv_camera_start,
            1: self.sub_driver_camera_start
        }
        
        self.camera_detect = {
            0: self.opencv_camera_detect,
            1: self.sub_driver_camera_detect
        }
        
        ################ VIDEOCAMERA INSTANCES ################
        ################ SUB CAMERA DRIVER AND OPENCV ################
        ##### DO NOT CHANGE ######### 
        self.camera_serials = {
            'forward' : '07714031',
            'down' : '35710219'
        }
        self.camera_ids = {
            'forward' : '/dev/v4l/by-id/usb-The_Imaging_Source_Europe_GmbH_DFK_22AUC03_07714031-video-index0',
            'down' : '/dev/v4l/by-id/usb-The_Imaging_Source_Europe_GmbH_DFK_22AUC03_35710219-video-index0'
        }

        self.exposure = 10
        #############################
        self.sample = {
            'forward' : None,
            'down' : None
        }
        self.pipeline = {
            'forward' : None,
            'down' : None
        }
        self.display_pipeline = {
            'forward' : None,
            'down' : None
        }
        self.display_input = {
            'forward' : None,
            'down' : None
        }
        self.display_buffers = {
            'forward' : None,
            'down' : None
        }
        self.camera_callbacks = {
            'forward' : self.camera_forward_callback,
            'down' : self.camera_down_callback
        }
        
        self.time_delay = 1.0
        self.camera_direction = 'forward'
        self.cap = None
        self.frame = None
        self.thread = None
        self.is_stop = False


        try:
            self.loop = GLib.MainLoop()
            # self.loop = None
            self.sub_camera_found = 1
            print '*******initialize Glib.MainLoop() successful*******'
        except:
            self.sub_camera_found = 0
            print '*******unable to initialize Glib.MainLoop()*******'

    # start ##################################################################################
    def start(self, task_name):
        self.is_stop = False
        if self.sub_camera_found == 1:
            self.camera_start_dictionary[self.sub_camera_found](task_name)
        #do not elif we want to check the 2nd one if first fails aswell
        if self.sub_camera_found == 0:
            self.camera_start_dictionary[self.sub_camera_found](task_name)
        # time.sleep(self.time_delay)
        print 'start cvcontroller'

    # stop ##################################################################################
    def stop(self):
        self.is_stop = True
        self.outraw.release()
        self.outprocessed.release()
        self.close_pipeline()
        if self.cap:
            self.cap.release()
            self.cap = None
            print 'laptop/default camera released'
        cv2.destroyAllWindows()
        print 'stop cvcontroller'

    # set_lower_color ##################################################################################
    def set_lower_color(self, task_name, lower):
        self.tasks[task_name].preprocess.set_lower_color(task_name, lower)

    # set_upper_color ##################################################################################
    def set_upper_color(self, task_name, upper):
        self.tasks[task_name].preprocess.set_upper_color(task_name, upper)

    # set_model ##################################################################################
    def set_model(self, task_name=None):
        pass
        # TODO set model for detectors

    # camera selection functions ######################################################################
    def change_camera_to(self, camera_direction, task_name):
        self.stop()
        self.camera_direction = camera_direction
        self.start(task_name)
        
    # raw_frame ##################################################################################
    def raw_frame(self):
        return self.current_raw_frame
    
    # processed_frame ##################################################################################
    def processed_frame(self):
        return self.current_processed_frame
    
    # setup_video_output ##################################################################################
    def setup_video_output(self, task_name):
        self.fourcc = cv2.VideoWriter_fourcc(*'XVID')
        now = datetime.datetime.now()
        timestamp = '%d-%d-%d_%dh%dm%ds' % (now.year, now.month, now.day, now.hour, now.minute, now.second)
        if self.sub_camera_found == 1:
            if not self.is_camera_640x480:
                res = (744, 480)
            else:
                res = (640, 480)
        else:
            res = (744, 480)
            # res = (640, 480)

        self.outraw = cv2.VideoWriter('video_output/raw_' + task_name + '_' + timestamp + '_output.avi', self.fourcc, self.fps_output, res)
        self.outprocessed = cv2.VideoWriter('video_output/processed_' + task_name + '_' + timestamp + '_output.avi', self.fourcc, self.fps_output, res)

    # sub_driver_camera_start ##################################################################################
    def sub_driver_camera_start(self, task_name):
        print 'setup pipeline'
        # for key in self.pipeline:
        #     self.setup_pipeline(key)
        self.setup_pipeline(self.camera_direction)
        if self.sub_camera_found == 0:
            return
        # self.setup_pipeline('down')
        self.thread=Thread(target=self.start_loop)
        self.thread.start()
        self.setup_video_output(task_name)
        print 'sub camera found'

    # opencv_camera_start ##################################################################################
    def opencv_camera_start(self, task_name):

        self.cap = None
        # self.cap = cv2.VideoCapture(self.camera_ids[self.camera_direction])
        # self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 744)
        # self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 640)
        # self.cap.set(cv2.CAP_PROP_FPS, 60)

        #--------------------------------------------
        # self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)
        # self.cap.set(cv2.CAP_PROP_EXPOSURE, 1000.0)
        
        if self.is_camera:
            self.cap = cv2.VideoCapture(self.camera_ids[self.camera_direction])
            # self.cap = cv2.VideoCapture(0)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 744)
            # self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            # self.cap.set(cv2.CAP_PROP_FPS, 60)
            self.start_camera_async()
        else:
            #path
            # self.cap = cv2.VideoCapture('video_output/7-25-18/raw_path_follow_2018-7-25_18h0m36s_output.avi')
            # self.cap = cv2.VideoCapture('video_output/7-25-18/raw_path_follow_2018-7-25_18h11m40s_output.avi')
            # self.cap = cv2.VideoCapture('video_output/7-31-18/d1/raw_path_follow_2018-7-31_path1.avi')
            # self.cap = cv2.VideoCapture('video_output/7-31-18/d1/raw_path_follow_2018-7-31_path2.avi')
            # self.cap = cv2.VideoCapture('video_output/8-2-18/a2/raw_path_follow_2018-8-2_9h49m1s_output.avi')
            self.cap = cv2.VideoCapture('video_output/8-1-18/b1/raw_path_follow_2018-8-1_9h39m50s_output.avi')
            

            #dark gate
            # self.cap = cv2.VideoCapture('video_output/7-25-18/raw_gate_2018-7-25_19h40m32s_output.avi')

            #bright gate
            # self.cap = cv2.VideoCapture('video_output/7-25-18/raw_gate_2018-7-25_16h48m45s_output.avi')
            # self.cap = cv2.VideoCapture('video_output/7-20-18/raw_gate_2018-7-20_16h38m23s_output.avi')

            #dice  
            # self.cap = cv2.VideoCapture('video_output/7-27-18/raw_dice_2018-7-27_17h47m39s_output.avi')
            # self.cap = cv2.VideoCapture('video_output/7-27-18/raw_dice_2018-7-27_19h45m40s_output.avi')
            # self.cap = cv2.VideoCapture('video_output/7-27-18/raw_dice_2018-7-27_19h42m13s_output.avi')
            # self.cap = cv2.VideoCapture('video_output/7-28-18/raw_dice_2018-7-28_15h38m38s_output.avi')
            # self.cap = cv2.VideoCapture('video_output/8-1-18/b1/raw_dice_2018-8-1_9h42m44s_output.avi')

            #green
            # self.cap = cv2.VideoCapture('video_output/8-2-18/b2/raw_gate_2018-8-2_15h37m36s_output.avi')
            # self.cap = cv2.VideoCapture('video_output/8-2-18/b2/raw_gate_2018-8-2_15h43m15s_output.avi')
            # self.cap = cv2.VideoCapture('video_output/8-2-18/b2/raw_path_follow_2018-8-2_15h38m59s_output.avi')
            
            # self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 744)
            # self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 640)
            # self.cap.set(cv2.CAP_PROP_FPS, 1)

        self.setup_video_output(task_name)
        self.setup_output_pipline(self.camera_direction)
        print 'laptop/default camera found'

    def display_output(self, camera_direction):
        if self.sample[camera_direction]:
            buf = self.sample[camera_direction].get_buffer()

            caps = self.sample[camera_direction].get_caps()
            width = caps[0].get_value("width")
            height = caps[0].get_value("height")
            # try:
            res, mapinfo = buf.map(Gst.MapFlags.READ)
            # actual image buffer and size
            # data = mapinfo.data
            # size = mapinfo.size

            # Create a numpy array from the data
            img_array = np.asarray(bytearray(mapinfo.data), dtype=np.uint8)
            frame = img_array.reshape((height, width, 3))
            self.show_img(camera_direction, frame)

            # except KeyboardInterrupt:
            #     self.state.is_detect_done = True
            #     # raise
            # finally:
            buf.unmap(mapinfo)
                
    # sub_driver_camera_detect ##################################################################################
    def sub_driver_camera_detect(self, task, camera_direction = None):
        if not task:
            return None, None, None, None
            
        if not camera_direction:
            # camera_direction = 'down'
            camera_direction = self.camera_direction

        self.cv_task = self.tasks[task]
        # self.display_output('down')
        # self.display_output(camera_direction)
        if self.sample[camera_direction]:
            # print("have sample")
            buf = self.sample[camera_direction].get_buffer()

            caps = self.sample[camera_direction].get_caps()
            width = caps[0].get_value("width")
            height = caps[0].get_value("height")
            # try:
            res, mapinfo = buf.map(Gst.MapFlags.READ)
            # actual image buffer and size
            # data = mapinfo.data
            # size = mapinfo.size

            # Create a numpy array from the data
            img_array = np.asarray(bytearray(mapinfo.data), dtype=np.uint8)

            # Give the array the correct dimensions of the video image
            frame = img_array.reshape((height, width, 3))
            # self.auto_exposure(camera_direction, frame)
            # print(type(frame))

            # self.outraw.write(frame)
            # self.msg.found, coordinates = self.state.detect(frame)
            # self.outprocessed.write(frame)

            # self.last_reading.append(coordinates)
            if self.is_debug:
                self.outraw.write(frame)
            # self.current_raw_frame = copy.copy(frame)
            found, coordinates, shape, width_height = self.cv_task.detect(frame)
            
            # self.current_processed_frame = copy.copy(frame)
            if self.is_debug:
                self.outprocessed.write(frame)
                self.show_img(camera_direction, frame)

            # except KeyboardInterrupt:
            #     self.state.is_detect_done = True
            #     # raise
            # finally:
            buf.unmap(mapinfo)

            return found, coordinates, shape, width_height
        return None, None, None, None

    # opencv_camera_detect ##################################################################################
    def opencv_camera_detect(self, task):

        self.cv_task = self.tasks[task]

        if not self.is_camera:
            check, frame = self.cap.read()
            if check:
                self.frame = frame

        frame = copy.copy(self.frame)
        if frame is not None:
            self.outraw.write(frame)
            # self.current_raw_frame = copy.copy(frame)
            found, directions, shape, width_height = self.cv_task.detect(frame)
            #found, directions, gate_shape, width_height = self.gatedetector.detect(frame)
            # print frame.shape

            if self.is_debug:
                self.show_img(self.camera_direction, frame)

            self.outprocessed.write(frame)
            # self.current_processed_frame = copy.copy(frame)
            return found, directions, shape, width_height
        return False, None, None, None
    
    def start_camera_async(self):            
        self.thread = Thread(target=self.run_camera)
        self.thread.start()

    def run_camera(self):
        while not self.is_stop:
            check, frame = self.cap.read()
            if check:
                # self.frame = self.white_balance(frame)
                # cv2.balanceWhite(frame, self,frame, cv2.WHITE_BALANCE_SIMPLE)
                self.frame = frame
                if self.is_camera:
                    self.auto_exposure(self.camera_direction, balanced_frame)

    # detect ##################################################################################
    def detect(self, task):
        # try:
        return self.camera_detect[self.sub_camera_found](task)
        # except:
        #     print 'detect for that task is not available'
        #     return False, [0,0], None, (0,0)

    # change_die_num ##################################################################################
    def change_dice(self, dice = None):
        self.dicedetector.change_dice(dice)

    # show_img ##################################################################################
    def show_img(self, camera_direction, img):
        bytebuffer = img.tobytes()
        self.display_buffers[camera_direction].append(bytebuffer)
        new_buf = Gst.Buffer.new_wrapped_full(Gst.MemoryFlags.READONLY, bytebuffer, len(bytebuffer), 0, None, lambda x: self.display_buffers[camera_direction].pop(0))
        self.display_input[camera_direction].emit("push-buffer", new_buf)

    # setup_pipeline ##################################################################################
    def setup_output_pipline(self, camera_direction):
        if not self.is_debug:
            return

        Gst.init(sys.argv)  # init gstreamer

        TARGET_FORMAT = "video/x-raw,width=744,height=480,format=BGR"

        src_name = "cam_" + camera_direction
        self.display_pipeline[camera_direction] = Gst.parse_launch("appsrc name=" + src_name + " ! videoconvert ! ximagesink")
        self.display_input[camera_direction] = self.display_pipeline[camera_direction].get_by_name(src_name)
        self.display_input[camera_direction].set_property("caps", Gst.Caps.from_string(TARGET_FORMAT))
        
        self.display_buffers[camera_direction] = []
        self.display_pipeline[camera_direction].set_state(Gst.State.PLAYING) 

    def setup_pipeline(self, camera_direction = None):
        if camera_direction == None:
            print 'need camera_direction to setup pipeline'
            return
        elif camera_direction not in self.pipeline:
            print 'invalid camera'
            return

        Gst.init(sys.argv)  # init gstreamer
        # print Gst

        # We create a source element to retrieve a device list through it
        source = Gst.ElementFactory.make("tcambin")

        # for name in source.get_tcam_property_names():
        #     temp = source.get_tcam_property(name)
        #     print( name + " " + str(temp))

        serial = self.camera_serials[camera_direction]

        # source = Gst.ElementFactory.make("v4l2src")
        # source.set_property('device', self.camera_ids[camera_direction])

        # retrieve all available serial numbers
        serials = source.get_device_serials()
        if not serials:
            self.sub_camera_found = 0
            print 'error: no cameras found'
            return
        # create a list to have an easy index <-> serial association
        # device_list = []

        # for s in serials:
        #     device_list.append(s)
        
        # serial = serials[0]
        # print serial
        if serial is not None:
            source.set_property("serial", serial)
            # source_tcambin.set_property('name', 'DFK 22UC03')
            # source_tcambin.set_property('name', 'DFK 22UC03')

        # print source.get_by_name('tcamautoexposure')
        # print source.get_by_name('tcamwhitebalance')
        # print source.get_by_name('tcamautofocus')
        # Define the format of the video buffers that will get passed to opencv
        if self.is_camera_640x480:
            TARGET_FORMAT = "video/x-raw,width=640,height=480,format=BGR"
        else:
            TARGET_FORMAT = "video/x-raw,width=744,height=480,format=BGR"

        # Ask the user for the format that should be used for capturing
        fmt =  self.select_format(source.get_by_name("tcambin-source")) #----------------------- replace
        # fmt =  self.select_format(source) #----------------------- replace
        # If the user selected a bayer format, we change it to BGRx so that the
        # tcambin will decode the bayer pattern to a color image
        if fmt.get_name() == "video/x-bayer":
            fmt.set_name("video/x-raw")
            fmt.set_value("format", "BGRx")
        # Use a capsfilter to determine the video format of the camera source
        capsfilter = Gst.ElementFactory.make("capsfilter")
        capsfilter.set_property("caps", Gst.Caps.from_string(fmt.to_string()))
        # Add a small queue. Everything behind this queue will run in a separate
        # thread.
        queue = Gst.ElementFactory.make("queue")
        queue.set_property("leaky", True)
        queue.set_property("max-size-buffers", 2)
        # Add a videoconvert and a videoscale element to convert the format of the
        # camera to the target format for opencv
        convert = Gst.ElementFactory.make("videoconvert")
        scale = Gst.ElementFactory.make("videoscale")
        # Add an appsink. This element will receive the converted video buffers and
        # pass them to opencv
        output = Gst.ElementFactory.make("appsink")
        output.set_property("caps", Gst.Caps.from_string(TARGET_FORMAT))
        output.set_property("emit-signals", True)

        # tcamautoexposure auto-exposure=true exposure-max=300000
        if self.is_camera_640x480:
            auto_exposure = Gst.ElementFactory.make("tcamautoexposure")
            auto_exposure.set_property('auto-exposure', 1)
        # # auto_exposure.set_property('Gain Max', 1.1)
        # # auto_exposure.set_property('Exposure Max', 1)
        # # auto_exposure.set_property('Brightness Reference', 64)
        # auto_exposure.set_property('exposure-max', 500)
        # auto_exposure.set_property('gain-max', 1.1)
        # auto_exposure.set_property('brightness-reference', 32)

        # auto_focus = Gst.ElementFactory.make("tcamautofocus")
        # auto_focus.set_property('auto-focus', False)

        # white_balance = Gst.ElementFactory.make("tcamwhitebalance")
        # # white_balance.set_property('value', False)
        # auto_focus = Gst.ElementFactory.make("tcamautofocus")
        # # auto_focus.set_property('value', False)

        self.pipeline[camera_direction] = Gst.Pipeline.new()
        #video/x-raw,format=BGR,width=744,height=480,framerate=60/1
        # parse_str = 'tcambin serial=' + ser/ial + ' ! capsfilter caps="video/x-raw,format=BGRx" ! videoconvert ! videoscale ! queue max-size-buffers=2 ! appsink caps="'+ TARGET_FORMAT +'" emit-signals=true'
        # parse_str = 'v4l2src device={} ! video/x-raw,format=BGRx,framerate=60/1,width=640,height=480 ! videoconvert ! videoscale ! queue max-size-buffers=2 ! appsink name=sink caps="{}" emit-signals=true'.format(self.camera_ids[camera_direction], TARGET_FORMAT)
        # print parse_str
        # self.pipeline[camera_direction] = Gst.parse_launch(parse_str)


        # Add all elements
        self.pipeline[camera_direction].add(source)
        if self.is_camera_640x480:
            self.pipeline[camera_direction].add(auto_exposure)
        # self.pipeline[camera_direction].add(auto_focus)
        # self.pipeline[camera_direction].add(white_balance)

        self.pipeline[camera_direction].add(capsfilter)
        self.pipeline[camera_direction].add(queue)
        self.pipeline[camera_direction].add(convert)
        self.pipeline[camera_direction].add(scale)
        self.pipeline[camera_direction].add(output)

        # output = self.pipeline[camera_direction].get_by_name("sink")
        # output.set_property("caps", Gst.Caps.from_string(TARGET_FORMAT))
        # output.set_property("emit-signals", True)

        # Link the elements
        source.link(capsfilter)
        capsfilter.link(queue)
        queue.link(convert)
        convert.link(scale)
        scale.link(output)
        if self.is_camera_640x480:
            output.link(auto_exposure)
        # auto_exposure.link(auto_focus)
        # output.link(auto_exposure)
        
        # Usually one would use cv2.imgshow(...) to display an image but this is
        # tends to hang in threaded environments. So we create a small display
        # pipeline which we could use to display the opencv buffers.
        src_name = "cam_" + camera_direction

        if self.is_debug:
            self.display_pipeline[camera_direction] = Gst.parse_launch("appsrc name=" + src_name + " ! videoconvert ! ximagesink")
            self.display_input[camera_direction] = self.display_pipeline[camera_direction].get_by_name(src_name)
            self.display_input[camera_direction].set_property("caps", Gst.Caps.from_string(TARGET_FORMAT))
            self.display_pipeline[camera_direction].set_state(Gst.State.PLAYING) 

        self.display_buffers[camera_direction] = []

        #source.get_by_name("tcambin-source")
        # pip_source = self.pipeline[camera_direction].get_by_name("source")
        # pip_source = self.pipeline[camera_direction].get_by_name("tcambin-source")
        # pip_source = source
        # PropertyName = 'Exposure Auto'
        # value = False
        # pip_source.set_tcam_property(PropertyName,GObject.Value(type(value),value)) 

        output.connect("new-sample", self.camera_callbacks[camera_direction])
        self.pipeline[camera_direction].set_state(Gst.State.PLAYING)
        
        # self.cap = cv2.VideoCapture(self.camera_ids[self.camera_direction])
        # self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 744)
        # self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        # self.cap.set(cv2.CAP_PROP_FPS, 60)
        # self.pipeline[camera_direction].set_state(Gst.State.NULL)

        # pip_source = self.pipeline[camera_direction].get_by_name("source")
        # pip_source = Gst.Bin.get_by_name(Gst.Bin(self.pipeline[camera_direction]), "source")
        # PropertyName = 'Exposure Auto'
        # value = False
        # pip_source.set_tcam_property(PropertyName,GObject.Value(type(value),value))
        
        # for name in pip_source.get_tcam_property_names():
        #     temp = pip_source.get_tcam_property(name)
        #     print( name + " " + str(temp))

        # print self.pipeline[camera_direction].get_by_name('tcamautoexposure')
        # print self.pipeline[camera_direction].get_by_name('tcamwhitebalance')
        # print self.pipeline[camera_direction].get_by_name('tcamautofocus')
        # Gst.Bin.remove(source.get_tcam_property('tcamautoexposure'))
        # Gst.Bin.remove(source.get_tcam_property('tcamwhitebalance'))
        # Gst.Bin.remove(source.get_tcam_property('tcamautofocus'))
        # for name in source.get_tcam_property_names():
        #     temp = source.get_tcam_property(name)
        #     print( name + " " + str(temp))
        # self.set_camera_auto_exposure_manual(camera_direction)
        print 'done setting up pipeline'

    # close_pipeline ##################################################################################
    def close_pipeline(self):
        # try:
        for key in self.pipeline:
            if self.pipeline[key]:
                self.pipeline[key].set_state(Gst.State.NULL)
                self.pipeline[key] = None
            
            if self.sample[key]:
                self.sample[key] = None

            if self.display_buffers[key]:
                self.display_buffers[key] = None

            if self.display_input[key]:
                self.display_input[key] = None

            if self.display_pipeline[key]:
                self.display_pipeline[key].set_state(Gst.State.NULL)
                self.display_pipeline[key] = None
        # except:
        #     print 'error in cvcontroller close_pipeline'
        # if self.loop:
        try:
            if self.loop is not None:
                self.loop.quit()
        except:
            print 'unable to use self.loop.quit()'
        self.thread = None

        print 'closed pipeline'

    # callback ##################################################################################
    def camera_forward_callback(self, sink):
        # print("in forward callback")
        if not self.is_camera_640x480:
            self.sample['forward'] = sink.emit("pull-sample")
        else:
            sample = sink.emit("pull-sample")

            buf = sample.get_buffer()

            caps = sample.get_caps()
            width = caps[0].get_value("width")
            height = caps[0].get_value("height")
            # try:
            res, mapinfo = buf.map(Gst.MapFlags.READ)

            # Create a numpy array from the data
            img_array = np.asarray(bytearray(mapinfo.data), dtype=np.uint8)

            # Give the array the correct dimensions of the video image
            frame = img_array.reshape((height, width, 3))
            self.auto_exposure('forward', frame)
            self.sample['forward'] = sample
        return Gst.FlowReturn.OK

    def camera_down_callback(self, sink):
        # print("in down callback")
        if not self.is_camera_640x480:
            self.sample['down'] = sink.emit("pull-sample")
        else:
            sample = sink.emit("pull-sample")
            
            buf = sample.get_buffer()

            caps = sample.get_caps()
            width = caps[0].get_value("width")
            height = caps[0].get_value("height")
            # try:
            res, mapinfo = buf.map(Gst.MapFlags.READ)

            # Create a numpy array from the data
            img_array = np.asarray(bytearray(mapinfo.data), dtype=np.uint8)

            # Give the array the correct dimensions of the video image
            frame = img_array.reshape((height, width, 3))
            self.auto_exposure('down', frame)
            self.sample['down'] = sample
        return Gst.FlowReturn.OK

    # list_formats ##################################################################################
    def list_formats(self, source):
        """Returns a list of all video formats supported by a video source."""

        # source needs to be at least in READY state to be able to read formats
        old_state = source.get_state(10 * Gst.SECOND)
        if old_state.state == Gst.State.NULL:
            source.set_state(Gst.State.READY)

        caps = source.pads[0].query_caps()

        # create a list of all structures contained in the caps
        ret = [caps.get_structure(i) for i in range(caps.get_size())]

        # cleanup
        source.set_state(old_state.state)

        return ret

    # select_format ##################################################################################
    def select_format(self, source):
        """Helper function that prompts the user to select a video format.

        Returns: Gst.Structure of format
        """
        formats = self.list_formats(source)
        if not self.is_camera_640x480:
            fmt = formats[3]
        else:
            fmt = formats[4]
        print(fmt)
        frame_rates = self.get_frame_rate_list(fmt)
        rate = frame_rates[1]
        print(rate)

        # work around older GI implementations that lack proper Gst.Fraction/Gst.ValueList support
        if type(rate) == Gst.Fraction:
            fmt.set_value("framerate", rate)
        else:
            numerator, denominator = rate.split("/")
            fmt.set_value("framerate", Gst.Fraction(int(numerator), int(denominator)))

        # fmt is a Gst.Structure but Caps can only be generated from a string,
        # so a to_string conversion is needed
        return fmt

    # get_frame_rate_list ##################################################################################
    def get_frame_rate_list(self, fmt):
        """Get the list of supported frame rates for a video format.
        This function works arround an issue with older versions of GI that does not
        support the GstValueList type"""
        try:
            rates = fmt.get_value("framerate")
        except TypeError:
            import re
            # Workaround for missing GstValueList support in GI
            substr = fmt.to_string()[fmt.to_string().find("framerate="):]
            # try for frame rate lists
            _unused_field, values, _unsued_remain = re.split("{|}", substr, maxsplit=3)
            rates = [x.strip() for x in values.split(",")]
        return rates

    # start_loop ##################################################################################
    def start_loop(self):
        self.loop.run()


    def auto_exposure(self, camera_direction, frame):
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) # to HSV colorspace
        brightness = np.mean(hsv_frame[:,:,2]) # average brightness
        # exposure is a variable that is initialized to a value (usually around 40) and changes depending on brightness
        if brightness < 85:
            self.exposure = self.exposure + 1
        elif brightness > 170.0:
            self.exposure = self.exposure - 1
        # self.exposure = 6

        os.system('v4l2-ctl -d %s -c exposure_absolute=%s' %(self.camera_ids[camera_direction], str(self.exposure)))
        # self.cap.set(cv2.CAP_PROP_EXPOSURE, self.exposure)
    # def set_camera_auto_exposure_manual(self, camera_direction):
    #     #1 is manual exposure 3 is auto
    #     os.system('v4l2-ctl -d %s -c exposure_auto=1' %(self.camera_ids[camera_direction]))


    def change_gate_target(self, is_center = None):
        if is_center is None:
            is_center = True
        self.gatedetector.is_direction_center = is_center

    def white_balance(self, img):
        # result = cv.cvtColor(img, cv.COLOR_BGR2LAB)
        img_int16 = img.astype(np.int16)
        avg_g = np.average(img_int16[:, :, 1])
        avg_b = np.average(img_int16[:, :, 2])
        img_int16[:, :, 1] = img_int16[:, :, 1] - avg_g
        # img_int16[:, :, 1] = img_int16[:, :, 1] - img_int16[:, :, 1].min()
        # img[:, :, 1] = img[:, :, 1]/2
        img_int16[:, :, 2] = img_int16[:, :, 2] - avg_b
        # img[:, :, 2] = img[:, :, 2] - img[:, :, 2].min()
        # img[:, :, 2] = img[:, :, 2]/1.5
        # print img_int16[img_int16 < 0]  
        img_int16[img_int16 < 0] = 0
        # result = cv.cvtColor(result, cv.COLOR_LAB2BGR)

        # avg_b = np.average(img[:, :, 0])
        # img[:, :, 0] = img[:, :, 0] - avg_b
        return img_int16.astype(np.uint8)
