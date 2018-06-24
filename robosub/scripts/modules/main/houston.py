import rospy
import cv2
import sys
import time

import gi
import threading

from threading import Thread
import numpy as np

gi.require_version("Tcam", "0.1")
gi.require_version("Gst", "1.0")

from gi.repository import Tcam, Gst, GLib

from robosub.msg import CVIn
from robosub.msg import CVOut
from config.config import Config
# TODO create new msg for Task and CVData
#from robosub.msg import Task
#from robosub.msg import CVData

from modules.tasks.gate import Gate
from modules.tasks.path import Path
from modules.tasks.dice import Dice
from modules.tasks.chip import Chip
from modules.tasks.roulette import Roulette
from modules.tasks.slots import Slots
from modules.tasks.pinger_a import PingerA
from modules.tasks.pinger_b import PingerB
from modules.tasks.cash_in import CashIn
from modules.tasks.buoy import Buoy
from modules.tasks.task import Task

from modules.control.navigation import Navigation

#will need to move CVControll
from modules.controller.cv_controller import CVController

from collections import Counter
from itertools import combinations

# TODO houston will communicate with CV controller through ROS
# houston will interpet the coordinates and then send it to navigation
class Houston():
    # implements(Task)
    
    def __init__(self):
        """ To initilize Houston """
        self.is_killswitch_on = False
        self.navigation = Navigation()
        self.config = Config()
        self.coordinates = []
        self.counts = Counter()

        # will eventually move variables to task modules
        self.task_timer = 300
        self.last_time = time.time()

        self.rotation = 15
        self.power = 120

        # setting class instances of the tasks to none
        # to be used to prevent mutiple instances of same class
        self.gate = Gate(self)
        self.path_1 = Path(self)
        self.dice = Dice(self)
        self.path_2 = Path(self)
        self.chip_1 = Chip(self)        
        self.chip_2 = Chip(self)
        self.roulette = Roulette(self)
        self.slots = Slots(self)
        self.pinger_a = PingerA(self)
        self.pinger_b = PingerB(self)
        self.cash_in = CashIn(self)
        #self.buoy = Buoy(Houston)

        """
        self.tasks values listed below
        'gate', 'path', 'dice', 'chip', 'path', 'chip', 'slots', 'pinger_b', 
        'roulette', 'pinger_a', 'cash_in'
        """
        self.state_num = 0
        self.states = [self.gate, self.path_1, self.dice, self.chip_1, self.path_2, 
                        self.slots, self.chip_2, self.pinger_a, self.roulette,
                        self.pinger_b, self.cash_in]
    
        self.queue_direction = []

        #self.rotational_movement = {-1: }
        self.height = 1
        self.break_timer = 300

        # TODO move to CVcontroller
        # self.cap = cv2.VideoCapture(0)

        # init_node only needs to be ran once, which is already called in auv.py
        #rospy.init_node('cv_talker', anonymous=True)
        self.r = rospy.Rate(30) #30hz
        self.msg = CVIn()
        self.sample = None
        self.pipeline = None
        self.loop = GLib.MainLoop()
        self.thread = None

    def do_task(self):
        
        # self.thread=Thread(target=self.do_gate)
        # self.thread.start()
        try:
            self.do_gate()
        except KeyboardInterrupt:
            print('keyboard interrupt on cv')
            self.state.is_detect_done = True
        
        self.state.reset()
        # self.start_loop()

    def do_gate(self):
        # when state_num is > 10, there will be no more tasks to complete
        if self.state_num > 10:
            print 'no more tasks to complete'
            
        break_loop = 0
        self.state = self.states[self.state_num]
        print("setup pipeline")
        self.setupPipline()
        self.thread=Thread(target=self.start_loop)
        self.thread.start()
        # TODO must eventually move to CVController
        self.fourcc = cv2.VideoWriter_fourcc(*'XVID')
        self.outraw = cv2.VideoWriter('video_output/raw' + self.tasks[self.state_num] + '-' + str(time.time()) + '_output.avi', self.fourcc, 20.0, (744, 480))
        self.outprocessed = cv2.VideoWriter('video_output/processed' + self.tasks[self.state_num] + '-' + str(time.time()) + '_output.avi', self.fourcc, 20.0, (744, 480))

        while not self.state.is_detect_done and not break_loop > self.break_timer:

            # _, frame = self.cap.read()
            #if a sample exists run cv
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

                    self.outraw.write(frame)
                    self.msg.found, coordinates = self.state.detect(frame)
                    self.outprocessed.write(frame)

                    #self.show_img(frame)
                    # self.last_reading.append(coordinates)
                except KeyboardInterrupt:
                    self.state.is_detect_done = True
                    # raise
                finally:
                    buf.unmap(mapinfo)
                    
                self.outraw.write(frame)
                self.msg.found, coordinates, gate_shape, width_height = self.state.detect(frame)
                self.outprocessed.write(frame)

                self.queue_direction.append(coordinates)

                # TODO must eventually move to CVController
                # try:
                #     cv2.imshow(self.tasks[self.state_num],frame)
                # except Exception as e:
                #     print(e)
                # cv2.imshow('kill window', self.img)
                # key = cv2.waitKey(1) & 0xFF

                # if the `q` key is pressed, break from the loop
                # if key == ord("q"):
                #     self.navigation.cancel_h_nav()
                #     self.navigation.cancel_r_nav()
                #     self.navigation.cancel_m_nav()
                #     break

                # will run through whenever at least 1 second has passed
                if (time.time()-self.last_time > 0.1):# and not self.msg.found):
                    most_occur_coords = self.get_most_occur_coordinates(self.queue_direction, self.counts)
                    self.state.navigate(self.navigation, self.msg.found, most_occur_coords, self.power, self.rotation, gate_shape, width_height)
                    
                    """break_loop used for temp breaking of loop"""
                    #print 'press q to quit task or wait 30 secs'

                    self.counts = Counter()
                    self.queue_direction = []
                    self.last_time = time.time()

                    if self.msg.found:
                        self.foundcoord = coordinates
                    break_loop += 1
                #else:
                #    self.state.navigate(self.navigation, self.msg.found, coordinates, self.power, self.rotation)
                
                print 'task will stop in 300'
                print 'gate shape: {}, widthxheight: {}'.format(gate_shape, width_height)
                print 'current count: {}'.format(break_loop)
                print 'coordinates: {}'.format(coordinates)
                print '--------------------------------------------'

        print("exit loop")
        #if self.state.is_detect_done:
        #    self.state_num += 1

        self.foundcoord = None
        self.closePipline()
        self.navigation.cancel_h_nav()
        self.navigation.cancel_r_nav()
        self.navigation.cancel_m_nav()
        self.state.reset()
    
    # created to get most frequent coordinates from detect methods
    # once most frequent coordinates are found, sub will navigate to it
    # rather than just going to last coordinates
    def get_most_occur_coordinates(self, last, counts):
        for sublist in last:
            counts.update(combinations(sublist, 2))
        for key, count in counts.most_common(1):
            most_occur = key
        return most_occur

    def get_task(self):
        self.tasks = self.config.get_config('auv', 'tasks')
        # ['gate', 'path', 'dice', 'chip', 'path', 'chip', 'slots', 'pinger_b', 
        # 'roulette', 'pinger_a', 'cash_in']

    def start(self):
        self.get_task()
        # similar start to other classes, such as auv, and keyboard
        #self.is_killswitch_on = True
        self.navigation.start()
    
    def stop(self):
        # similar start to other classes, such as auv, and keyboard
        #self.is_killswitch_on = False
        self.navigation.stop()


    def show_img(self, img):
        bytebuffer = img.tobytes()
        self.display_buffers.append(bytebuffer)
        new_buf = Gst.Buffer.new_wrapped_full(Gst.MemoryFlags.READONLY, bytebuffer, len(bytebuffer), 0, None, lambda x: self.display_buffers.pop(0))
        self.display_input.emit("push-buffer", new_buf)

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
    def callback(self, sink):
        # print("in callback")
        self.sample = sink.emit("pull-sample")
        
        return Gst.FlowReturn.OK

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
    def start_loop(self):
        print("start loop")

        # try:
        self.loop.run()
        # except KeyboardInterrupt:
        #     print("Ctrl-C pressed, terminating")

