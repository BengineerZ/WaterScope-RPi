from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import cv2
import os
import datetime
# Richard's fix gain
from set_picamera_gain import set_analog_gain, set_digital_gain
from serial_communication import serial_controller_class, Arduinos
import yaml
import threading
import numpy as np


class OpencvClass():
    def __init__(self):
        # initialize the camera and grab a reference to the raw camera capture
        self.camera = PiCamera()
        self.stream_resolution = (824, 616)
        self.image_resolution = (3280,2464)
        self.image_seq = 0
        self.starting_time = datetime.datetime.now().strftime('%Y%m%d-%H:%M:%S')
        self.ROI = []
        self.sample_ID = 0
        self.auto_focus_status = ""
        # allow the camera to warmup
        time.sleep(0.1)

    def update_camera_setting(self):
        with open('config_picamera.yaml') as config_file:
            config = yaml.load(config_file)
            # consistent imaging condition
            self.camera.awb_mode = config['awb_mode']
            self.camera.awb_gains = (config['red_gain'], config['blue_gain'])
            # Richard's library to set analog and digital gains
            set_analog_gain(self.camera, config['analog_gain'])
            set_digital_gain(self.camera, config['digital_gain'])
            self.camera.shutter_speed = config['shutter_speed']
            self.camera.saturation = config['saturation']
            self.camera.led = False
            

    def start_streaming(self):
        # for arduino
        self.send_serial("success")

        self.camera.resolution = self.stream_resolution
        self.camera.framerate = 10
        self.rawCapture = PiRGBArray(self.camera, self.stream_resolution)
        # cv2.namedWindow("stream")
        # capture frames from the camera
        for frame in self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=True):
            # grab the raw NumPy array representing the image, then initialize the timestamp
            # and occupied/unoccupied text
            self.image = frame.array

            # find the ROI and measure the focus
            self.define_ROI(0.2)
            self.variance_of_laplacian()

            # show the frame
            # cv2.imshow("stream", self.image)

            
            key = cv2.waitKey(1) & 0xFF
            # clear the stream in preparation for the next frame
            self.rawCapture.truncate(0)


            # read serial command:
            self.read_serial()

            # NOTE: after autofocus
            if self.auto_focus_status =="ready":
                    time.sleep(1)
                    self.capture_image()
                    time.sleep(1)
                    self.move_to(0)
                    self.send_serial('results=123,456')
                    self.auto_focus_status = ''

            # if the `q` key was pressed, break from the loop
            if key == ord("q"):
                cv2.destroyAllWindows()
                break
        
            if key == ord("c"):
                self.capture_image()

            if key == ord("a"):
                self.start_auto_focus_thread()

            if key == ord("w"):
                self.send_serial("move_opt({})".format(10))

            if key == ord("s"):
                self.send_serial("move_opt({})".format(-10))

            if key == ord("s"):
                self.move_to(0)

    def initialise_data_folder(self):
        if not os.path.exists('timelapse_data'):
            os.mkdir('timelapse_data')
        self.folder_path = 'timelapse_data/{}'.format(self.starting_time)
        if not os.path.exists(self.folder_path):
            os.mkdir(self.folder_path)

    def capture_image(self, filename = '', resolution='high_res'):
        self.initialise_data_folder()
        # NOTE: when file name is not specified, use a counter
        if filename == '':
            filename = self.folder_path+'/{}_{:04d}_{}.jpg'.format(self.sample_ID, self.image_seq, datetime.datetime.now().strftime('%Y%m%d-%H:%M:%S'))
        else:
            filename = self.folder_path+'/{}_{:04d}-{}.jpg'.format(self.sample_ID, self.image_seq, filename)
        if resolution == 'normal':
            print('taking image')
            self.camera.capture(filename, format = 'jpeg', quality=100, bayer = False, use_video_port=True)
        elif resolution == 'high_res':
            print('taking high_res image')
            # when taking photos at high res, need to stop the video channel first
            # self.camera.stop_recording(splitter_port=1)
            # self.camera.stop_recording()
            # self.stop_streaming()
            time.sleep(0.1)
            self.camera.resolution = self.image_resolution
            # Remove bayer = Ture if dont care about RAW
            self.camera.capture(filename, format = 'jpeg', quality=100, bayer = False)
            time.sleep(0.1)
            # reduce the resolution for video streaming
            self.camera.resolution = self.stream_resolution
            # resume the video channel
            # Warning: be careful about the self.camera.start_recording. 'bgr' for opencv and 'mjpeg' for picamera
            # self.camera.start_recording(self.stream, format='mjpeg', quality = self.stream_quality, splitter_port=1)

        self.image_seq += 1

    # NOTE: opencv specific modification code
    def define_ROI(self,  box_ratio = 0.2):
        # do some modification
        # the opencv size is (y,x)
        image_y, image_x = self.image.shape[:2]

        # a square from the centre of image
        box_size = int(image_x*box_ratio)
        roi_box = {
            'x1': int(image_x/2-box_size/2), 'y1':int(image_y/2-box_size/2), 
            'x2': int(image_x/2+box_size/2), 'y2':int(image_y/2+box_size/2)}
        
        # the rectangle affects the laplacian, draw it outside the ROI
        # draw the rectangle
        cv2.rectangle(
            self.image, 
            pt1=(roi_box['x1']-5, roi_box['y1']-5),
            pt2=(roi_box['x2']+5, roi_box['y2']+5), 
            color=(0,0,255),
            thickness=2)
        
        # crop the image
        self.ROI = self.image[roi_box['y1']: roi_box['y2'], roi_box['x1']:roi_box['x2']]

    def variance_of_laplacian(self):
            ''' focus calculation ''' 
            if self.ROI == []:
                self.ROI = self.image
            # compute the Laplacian of the image and then return the focus
            # measure, which is simply the variance of the Laplacian
            self.focus_value = cv2.Laplacian(self.ROI, cv2.CV_64F).var()
            print(self.focus_value)
            focus_text = 'f: {:.2f}'.format(self.focus_value)
            # CV font
            font = cv2.FONT_HERSHEY_DUPLEX
            cv2.putText(
                self.image, focus_text,
                (int(self.image.shape[0]*0.1), int(self.image.shape[1]*0.1)), 
                font, 2, (0, 0, 255))

    def annotating(self, annotation_text):
        # CV font
        font = cv2.FONT_HERSHEY_DUPLEX
        cv2.putText(
            self.image, annotation_text,
            (int(self.image.shape[0]*0.9), int(self.image.shape[1]*0.1)), 
            font, 2, (0, 0, 255))


    # NOTE: serial communication 
    def initialise_serial_connection(self):
        ''' all the arduino connection is done via this function''' 
        try:
            # print(' serial connections already exist')
            Arduinos.serial_controllers
        except AttributeError:
            with open('config_serial.yaml') as config_serial_file:
                serial_controllers_config = yaml.load(config_serial_file)
            Arduinos.available_arduino_boards = []

            for board_name in serial_controllers_config:
                if serial_controllers_config[board_name]['connect'] is True:
                    Arduinos.available_arduino_boards.append(board_name)

            print(Arduinos.available_arduino_boards)
            # initialise the serial port if it does not exist yet.
            #print('initialising the serial connections')
            Arduinos.serial_controllers = {}
            for name in Arduinos.available_arduino_boards:
                Arduinos.serial_controllers[name] = serial_controller_class()
                Arduinos.serial_controllers[name].serial_connect(
                    port_address=serial_controllers_config[name]['port_address'],
                    port_names=serial_controllers_config[name]['port_names'], 
                    baudrate=serial_controllers_config[name]['baudrate'])
                Arduinos.serial_controllers[name].serial_read_threading(
                    options=serial_controllers_config[name]['serial_read_options'], 
                    parsers=serial_controllers_config[name]['serial_read_parsers'])

    def send_serial(self, serial_command = "LED_ON"):
        self.initialise_serial_connection()
        Arduinos.serial_controllers['waterscope'].serial_write(serial_command, parser='waterscope')

    def move_to(self, new_z):
           self. send_serial("abs_opt({})".format(new_z))

    def read_serial(self):
        self.initialise_serial_connection()
        try:
            income_serial_command = Arduinos.serial_controllers['waterscope'].income_serial_command
        except AttributeError:
            income_serial_command = ''
        
        if income_serial_command == "auto_focus":
            self.start_auto_focus_thread()

        elif income_serial_command == "capture":
            self.capture_image()
        try:
            self.sample_ID = Arduinos.serial_controllers['waterscope'].sample_ID
            self.sample_ID = self.sample_ID.replace("ID=", "")
        except AttributeError:
            self.sample_ID = 0

        Arduinos.serial_controllers['waterscope'].income_serial_command = ""

    def start_auto_focus_thread(self):
        # threading for auto focusing    
        threading_af = threading.Thread(target=self.auto_focus)
        threading_af.daemon = True
        threading_af.start()


    def auto_focus(self):
        def focus_measure_at_z(new_z):
            self.move_to(new_z)
            # DEBUG: wait for 1 second to stablise the mechanical stage
            time.sleep(0.3)
            focus_value =  self.focus_value
            print("Focus value at {0:.0f} is: {1:.2f}".format(new_z, focus_value))
            self.focus_table.update({new_z: focus_value})
            # print(self.focus_table)
            print(focus_value)
            return focus_value

        def scan_z_range(central_point = 50, range = 100, nubmer_of_points = 10):
            " using a central point and scan up and down with half of the range"
            z_scan_map = np.linspace(central_point-range/2, central_point+range/2, nubmer_of_points, endpoint=True)
            print(z_scan_map)
            for new_z in z_scan_map:
                focus_value = focus_measure_at_z(new_z)

        def iterate_z_scan_map(starting_z=50):
            ' automatically create several scan map from coarse to fine, using first guess and next best focus point' 
            start_time = time.time()
            self.move_to(0)
            time.sleep(2)
            # first scan is based on the best guess
            scan_z_range(50, 100, 10)
            # this will refine the best focus within range/points*2 = 400

            # Then finer scan  use the best focus value as the index for the z value
            for z_scan_range in [20, 5]:
                optimal_focus_z = max(self.focus_table, key=self.focus_table.get)
                scan_z_range(optimal_focus_z, z_scan_range, 5)
            
            print(self.focus_table)
            global_optimal_z = max(self.focus_table, key=self.focus_table.get)
            print('optimal: {}'.format(global_optimal_z))
            self.move_to(global_optimal_z)
            print('find the focus in {0:.2f} seconds at Z: {1}'.format(time.time() - start_time, global_optimal_z))

        # NOTE: Autofocus code runs from here
        print('starting to auto focus')
        self.annotating('Autofocusing')
        # home the stage for absolute_z
        self.move_to('home')

        #  a dictionary to record different z with its corresponding focus value
        self.focus_table = {}

        iterate_z_scan_map()
        print('you are at the best focus now')

        #  annotate when auto focus finish to notify the user
        self.annotating('Focus ready')
        # send requests to change the status to done
        self.send_serial('af_complete')

        self.auto_focus_status = "ready"


while __name__ == "__main__":
    opencv_class = OpencvClass()
    opencv_class.start_streaming()

