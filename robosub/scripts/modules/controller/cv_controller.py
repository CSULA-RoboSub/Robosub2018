import rospy
import math
import cv2
import sys
import time
import gi
import threading
import numpy as np

from threading import Thread

from modules.sensors.computer_vision import GateDetector
# from modules.sensors.computer_vision import BuoyDetector
from modules.sensors.computer_vision import DiceDetector

try:
    gi.require_version("Tcam", "0.1")
    gi.require_version("Gst", "1.0")

    from gi.repository import Tcam, Gst, GLib
except:
    print('*******unable to import sub camera drivers*******')


class CVController():
    
    def __init__(self):
        ################ INSTANCES ################
        self.gatedetector = GateDetector.GateDetector()
        #self.buoydetector = BuoyDetector.BuoyDetector()
        self.dicedetector = DiceDetector.DiceDetector()

        ################ FPS COUNTER ################
        self.fps_output = 20

        ################ CAMERA FRAME ################
        self.current_raw_frame = None
        self.current_processed_frame = None

        ################ DICTIONARIES ################
        self.tasks = {
            'gate': self.gatedetector,
            'dice': self.dicedetector
        }

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
        try:
            self.loop = GLib.MainLoop()
            self.sample = None
            self.pipeline = None
            self.thread = None
            self.sub_camera_found = 1
            print('*******initialize Glib.MainLoop() successful*******')
        except:
            self.cap = cv2.VideoCapture(0)
            self.sub_camera_found = 0
            print('*******unable to initialize Glib.MainLoop()*******')

    # start ##################################################################################
    def start(self, task_name):
        self.camera_start_dictionary[self.sub_camera_found](task_name)
        print 'start cvcontroller'

    # stop ##################################################################################
    def stop(self):
        try:
            self.closePipline()
            print 'pipline closed'
        except:
            self.cap.release()
            print 'laptop/default camera released'
        #cv2.destroyAllWindows()
        print 'stop cvcontroller'

    # raw_frame ##################################################################################
    def raw_frame(self):
        return self.current_raw_frame
    
    # processed_frame ##################################################################################
    def processed_frame(self):
        return self.current_processed_frame
    
    # sub_driver_camera_start ##################################################################################
    def sub_driver_camera_start(self, task_name):
        print("setup pipeline")
        self.setupPipline()
        self.thread=Thread(target=self.start_loop)
        self.thread.start()
        self.fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.outraw = cv2.VideoWriter('video_output/raw' + task_name + '-' + str(time.time()) + '_output.avi', self.fourcc, self.fps_output, (744, 480))
        self.outprocessed = cv2.VideoWriter('video_output/processed' + task_name     + '-' + str(time.time()) + '_output.avi', self.fourcc, self.fps_output, (744, 480))
        print 'sub camera found'

    # opencv_camera_start ##################################################################################
    def opencv_camera_start(self, task_name):
        self.cap = None
        self.cap = cv2.VideoCapture(0)
        self.fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.outraw = cv2.VideoWriter('video_output/raw' + task_name + '-' + str(time.time()) + '_output.avi', self.fourcc, self.fps_output, (640, 480))
        self.outprocessed = cv2.VideoWriter('video_output/processed' + task_name + '-' + str(time.time()) + '_output.avi', self.fourcc, self.fps_output, (640, 480))
        print 'laptop/default camera found'

    # sub_driver_camera_detect ##################################################################################
    def sub_driver_camera_detect(self, task):
        if not task:
            return None, None, None, None
            
        self.cv_task = self.tasks[task]
        if self.sample:
            # print("have sample")
            buf = self.sample.get_buffer()

            caps = self.sample.get_caps()
            width = caps[0].get_value("width")
            height = caps[0].get_value("height")
            try:
                res, mapinfo = buf.map(Gst.MapFlags.READ)
                # actual image buffer and size
                # data = mapinfo.data
                # size = mapinfo.size

                # Create a numpy array from the data
                img_array = np.asarray(bytearray(mapinfo.data), dtype=np.uint8)

                # Give the array the correct dimensions of the video image
                frame = img_array.reshape((height, width, 3))
                # print(type(frame))

                # self.outraw.write(frame)
                # self.msg.found, coordinates = self.state.detect(frame)
                # self.outprocessed.write(frame)

                # self.last_reading.append(coordinates)
                self.outraw.write(frame)
                self.current_raw_frame = frame.copy()
                found, coordinates, shape, width_height = self.cv_task.detect(frame)
                self.outprocessed.write(frame)
                self.current_processed_frame = frame.copy()

                self.show_img(frame)

            except KeyboardInterrupt:
                self.state.is_detect_done = True
                # raise
            finally:
                buf.unmap(mapinfo)

            return found, coordinates, shape, width_height
        return None, None, None, None

    # opencv_camera_detect ##################################################################################
    def opencv_camera_detect(self, task):
        self.cv_task = self.tasks[task]
        _, frame = self.cap.read()
        self.outraw.write(frame)
        self.current_raw_frame = frame.copy()
        found, directions, shape, width_height = self.cv_task.detect(frame)
        #found, directions, gate_shape, width_height = self.gatedetector.detect(frame)
        self.outprocessed.write(frame)
        self.current_processed_frame.copy()
        return found, directions, shape, width_height
    
    # detect ##################################################################################
    def detect(self, task):
        return self.camera_detect[self.sub_camera_found](task)

    # show_img ##################################################################################
    def show_img(self, img):
        bytebuffer = img.tobytes()
        self.display_buffers.append(bytebuffer)
        new_buf = Gst.Buffer.new_wrapped_full(Gst.MemoryFlags.READONLY, bytebuffer, len(bytebuffer), 0, None, lambda x: self.display_buffers.pop(0))
        self.display_input.emit("push-buffer", new_buf)

    # setupPipline ##################################################################################
    def setupPipline(self):
        Gst.init(sys.argv)  # init gstreamer

        # We create a source element to retrieve a device list through it
        source = Gst.ElementFactory.make("tcambin")

        # retrieve all available serial numbers
        serials = source.get_device_serials()

        # create a list to have an easy index <-> serial association
        # device_list = []

        # for s in serials:
        #     device_list.append(s)
        
        serial = serials[0]
        print(serial)
        if serial is not None:
            source.set_property("serial", serial)

        # Define the format of the video buffers that will get passed to opencv
        # TARGET_FORMAT = "video/x-raw,width=640,height=480,format=GRAY8"
        TARGET_FORMAT = "video/x-raw,width=744,height=480,format=BGR"

        # Ask the user for the format that should be used for capturing
        fmt =  self.select_format(source.get_by_name("tcambin-source")) #----------------------- replace
        
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
        self.pipeline = Gst.Pipeline.new()

        # Add all elements
        self.pipeline.add(source)
        self.pipeline.add(capsfilter)
        self.pipeline.add(queue)
        self.pipeline.add(convert)
        self.pipeline.add(scale)
        self.pipeline.add(output)

        # Link the elements
        source.link(capsfilter)
        capsfilter.link(queue)
        queue.link(convert)
        convert.link(scale)
        scale.link(output)
        
        # Usually one would use cv2.imgshow(...) to display an image but this is
        # tends to hang in threaded environments. So we create a small display
        # pipeline which we could use to display the opencv buffers.
        display_pipeline = Gst.parse_launch("appsrc name=src ! videoconvert ! ximagesink")
        self.display_input = display_pipeline.get_by_name("src")
        self.display_input.set_property("caps", Gst.Caps.from_string(TARGET_FORMAT))
        output.connect("new-sample", self.callback)

        self.display_buffers = []
        display_pipeline.set_state(Gst.State.PLAYING)  

        self.pipeline.set_state(Gst.State.PLAYING)
        print("done setting up pipeline")

    # closePipline ##################################################################################
    def closePipline(self):
        self.outraw.release()
        self.outprocessed.release()
        self.pipeline.set_state(Gst.State.NULL)
        self.display_buffers = []
        self.last_reading = []
        self.display_input = None
        self.pipeline = None
        self.sample = None
        cv2.destroyAllWindows()
        # 
        self.loop.quit()

        print("closed pipeline")

    # callback ##################################################################################
    def callback(self, sink):
        # print("in callback")
        self.sample = sink.emit("pull-sample")
        
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
        fmt = formats[3]
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
