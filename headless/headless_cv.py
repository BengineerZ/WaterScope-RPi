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
import signal
import subprocess
import serial as bluetooth
import base64


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
        self.sample_location = ""
        self.sample_time = ""
        self.sample_comment = ""
        self.auto_focus_status = ""
        # NOTE: turn this on when in deployment
        self.headless = True
        self.stop_streaming = False
        self.bt_open=0
        self.update_camera_setting()

        # wipe the file
        with open('image_to_analyse.txt', 'w+') as file:
            pass
        
        # allow the camera to warmup
        time.sleep(0.1)
        #self.open_bluetooth()
    def open_bluetooth(self):
            
        try:
            self.bt = bluetooth.Serial('/dev/rfcomm0', 1000000)
            self.bt.close()
            self.bt = bluetooth.Serial('/dev/rfcomm0', 1000000)
            self.bt_open=True
            print('BT port open')
        except:
      #      pass
            print ('port not open')
 
        
        
    def bt_readline(self):
        eol = b'$\n'
        leneol = len(eol)
        line = bytearray()
        while True:
            c = self.bt.read(1)
            if c:
                line += c
                if line[-leneol:] == eol:
                    break
            else:
                break
        return bytes(line[:-leneol])

    def read_bluetooth(self):
        if self.bt.in_waiting>1:
            bt_data = (self.bt_readline()).decode("utf-8")
            print(bt_data)
            if ((bt_data[0:2]) == 'id'):
                self.sample_ID = int(bt_data.split(",")[0][3:])
                print(self.sample_ID)
                self.sample_location = bt_data.split(",")[1][9:]
                self.sample_time=bt_data.split(",")[2][5:]
                self.sample_comment=bt_data.split(",")[3][8:]
            if ((bt_data[0:3]) == 'ard'):
                self.send_serial(bt_data[4:])
           
            if(bt_data=="full_analysis"):
               # self.write_bluetooth('coliform=15,ecoli=23')
             #   with open("preview.jpg", "rb") as imageFile:
               #     text = base64.b64encode(imageFile.read())
                
                self.start_auto_focus_thread()
            if(bt_data=="sample_preview"):
                image_path = self.filename.replace('.jpg', '_result.jpg')
                
                with open(image_path, "rb") as imageFile:
                   text = base64.b64encode(imageFile.read())
                if(self.bt_open==True):
                   self.bt.write('preview=data:image/png;base64,'.encode("utf-8"))
                   self.bt.write(text)
                   self.bt.write('$\n'.encode("utf-8"))             
                print('preview_requested')
            if(bt_data=="raw_preview"):
                image_path = self.filename
                with open(image_path, "rb") as imageFile:
                   text = base64.b64encode(imageFile.read())
                if(self.bt_open==True):
                   self.bt.write('raw=data:image/png;base64,'.encode("utf-8"))
                   self.bt.write(text)
                   self.bt.write('$\n'.encode("utf-8"))             
                print('raw_preview_requested')

    def write_bluetooth(self,bt_out):
        eol = '$\n'
        if(self.bt_open==True):
            self.bt.write((bt_out+eol).encode("utf-8"))
            
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
            self.image_resolution = config['image_resolution']
            self.default_LED_RGB = config['default_LED_RGB']
            

    def start_streaming(self):
        # for arduino
        self.send_serial("success")
        # self.send_serial("LED_RGB=7,10,7")
        self.send_serial("LED_RGB={},{},{}".format(self.default_LED_RGB[0], self.default_LED_RGB[1], self.default_LED_RGB[2]))

        

        self.camera.resolution = self.stream_resolution
        self.camera.framerate = 10
        self.rawCapture = PiRGBArray(self.camera, self.stream_resolution)
      # bluetooth start
       # self.establish_connection()
        self.open_bluetooth()

       # cv2.namedWindow("stream")
        # capture frames from the camera
        for frame in self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=True):
            # grab the raw NumPy array representing the image, then initialize the timestamp
            # and occupied/unoccupied text
            self.image = frame.array

            # find the ROI and measure the focus
            self.define_ROI(0.2)
            self.variance_of_laplacian()

            #  NOTE: disable this for autostart
            # show the frame
            if self.headless == False:
                cv2.imshow("stream", self.image)
            else:
                cv2.destroyAllWindows()

            
            key = cv2.waitKey(1) & 0xFF
            # clear the stream in preparation for the next frame
            self.rawCapture.truncate(0)
            
            #read bluetooth
            if self.bt_open == False:
                self.open_bluetooth()
            else:
                try:
                    self.read_bluetooth()
                except Exception as e:
                    print('disconnected bt')
                    print(e)
                    self.bt_open = False
                 #   os.killpg(os.getpgid(pro.pid), signal.SIGTERM)
                  #  self.establish_connection()
                  
            
            # read serial command:
            self.read_serial()
            # capture one image
            # self.analysis_result()

            # NOTE: after autofocus 
            if self.auto_focus_status =="ready":
                    time.sleep(1)
                    # self.capture_image()
                    self.analysis_result()
                    self.move_to(0)
                    self.send_serial('results={},{}'.format(self.result['coliforms'], self.result['E.coli']))
                    self.auto_focus_status = ''
                    self.annotating("E.coli: {}, coliforms: {}".format(self.result['E.coli'], self.result['coliforms']))
                    print('coliform={},ecoli={}'.format(self.result['coliforms'], self.result['E.coli']))
                    self.write_bluetooth('coliform={},ecoli={}'.format(self.result['coliforms'], self.result['E.coli']))
                    self.send_serial("led_off")  
          # if the `q` key was pressed, break from the loop
            # if key == ord("q"):
            #     cv2.destroyAllWindows()
            #     break
        
            if key == ord("c"):
                self.capture_image()

            if key == ord("h"):
                self. send_serial("home")

            if key == ord("m"):
                self.analysis_result()

            if key == ord("a"):
                self.start_auto_focus_thread()

            if key == ord("w"):
                self.send_serial("move_opt({})".format(10))

            if key == ord("s"):
                self.send_serial("move_opt({})".format(-10))

            if key == ord("q"):
                self.headless = not self.headless
            
            if self.stop_streaming == True:
                cv2.destroyAllWindows()
                break


    def initialise_data_folder(self):
        if not os.path.exists('timelapse_data'):
            os.mkdir('timelapse_data')
        self.folder_path = 'timelapse_data/{}'.format(self.sample_ID)
        print(self.sample_ID)
        if not os.path.exists(self.folder_path):
            os.mkdir(self.folder_path)

    def capture_image(self, filename = '', resolution='high_res'):
        self.initialise_data_folder()
        # NOTE: when file name is not specified, use a counter
        if filename == '':
            self.filename = self.folder_path+'/{:04d}_{}.jpg'.format(self.image_seq, datetime.datetime.now().strftime('%Y%m%d-%H:%M:%S'))
        else:
            self.filename = self.folder_path+'/{:04d}-{}.jpg'.format(self.image_seq, self.filename)
        if resolution == 'normal':
            print('taking image')
            self.camera.capture(self.filename, format = 'jpeg', quality=100, bayer = False, use_video_port=True)
        elif resolution == 'high_res':
            print('taking high_res image')
            # when taking photos at high res, need to stop the video channel first
            # self.camera.stop_recording(splitter_port=1)
            # self.camera.stop_recording()

            time.sleep(0.1)
            self.camera.resolution = self.image_resolution
            # Remove bayer = Ture if dont care about RAW
            self.camera.capture(self.filename, format = 'jpeg', quality=100, bayer = False)
            time.sleep(0.1)
            # reduce the resolution for video streaming
            self.camera.resolution = self.stream_resolution
 
            # resume the video channel
            # Warning: be careful about the self.camera.start_recording. 'bgr' for opencv and 'mjpeg' for picamera
            # self.camera.start_recording(self.stream, format='mjpeg', quality = self.stream_quality, splitter_port=1)

        self.image_seq += 1

    def analysis_result(self):
        print('Capture an image')
        
        self.capture_image()
        print(self.filename)
        time.sleep(1)
        with open('image_to_analyse.txt', 'w+') as file:
            file.write(self.filename)

        # self.headless = True
        print('Wait for ML to work out stuff')

        while True:
            # keep checking whether result it out
            if os.path.exists(self.filename.replace('.jpg', '_result.txt')):
                with open(self.filename.replace('.jpg', '_result.txt')) as file:
                    lines = file.readlines()
                    self.result = {}
                    for line in lines:
                        bacteria_name = line.split(':')[0].replace(' ', '').replace('\t','').replace('\n', '')
                        count = line.split(':')[1].replace(' ', '').replace('\t','').replace('\n', '')
                        self.result[bacteria_name] =  count
                        
                break

            else:
                time.sleep(2)

        print(self.result)

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
            # print(self.focus_value)
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
            self.sample_ID = self.sample_ID.replace("ID=", "").strip()
        except AttributeError:
            #self.sample_ID = 0
            pass

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
            time.sleep(0.5)
            # averaging the focus value
            focus_value_collection = []
            for i in range(5):
                focus_value_collection.append(self.focus_value)
                time.sleep(0.1)
            focus_value = np.average(focus_value_collection)

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
            for z_scan_range in  [25, 10]:
                optimal_focus_z = max(self.focus_table, key=self.focus_table.get)
                scan_z_range(optimal_focus_z, z_scan_range, 5)
            
            print(self.focus_table)
            global_optimal_z = max(self.focus_table, key=self.focus_table.get)
            print('optimal: {}'.format(global_optimal_z))
            self.move_to(global_optimal_z)
            print('find the focus in {0:.2f} seconds at Z: {1}'.format(time.time() - start_time, global_optimal_z))

            # disable twiching
            self.send_serial('motor_off')


        # NOTE: Autofocus code runs from here
        print('starting to auto focus')
        self.annotating('Autofocusing')
        # home the stage for absolute_z
        self.send_serial("home")
        self.send_serial("led_on")
        self.send_serial("abs_opt=120")
        self.send_serial("temp=70")
        time.sleep(150)
        self.send_serial("home")
        time.sleep(2)
        #  a dictionary to record different z with its corresponding focus value
        self.focus_table = {}

        iterate_z_scan_map()
        print('you are at the best focus now')

        #  annotate when auto focus finish to notify the user
        self.annotating('Focus ready')
        # send requests to change the status to done
        self.send_serial('af_complete')

        # For the arduino to know
        # DEBUG: disable this temporarily
        self.auto_focus_status = "ready"
        


while __name__ == "__main__":
    opencv_class = OpencvClass()
    opencv_class.start_streaming()


