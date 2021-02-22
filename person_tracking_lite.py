import numpy as np
from collections import deque
from scipy.optimize import linear_sum_assignment as linear_assignment
from threading import Thread
import helpers
import tracker
import cv2
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import RPi.GPIO as GPIO
import sys
from tflite_runtime.interpreter import Interpreter
import os
import logging
logging.basicConfig(level=logging.ERROR)
import pigpio
from servo import *

""" 
servo
equation used: y = -147*x + 122667
stored x is used to ignore small adjustments for the servo
SERVO_THRESHOLD is the minimum amount of pixel difference between the current and the previous x coord necessary to make the servo move
"""
stored_x = 0
SERVO_THRESHOLD = 30
MAX_DUTY = 122000
MIN_DUTY = 28000

# Global variables to be used by functions of VideoFileClop
frame_count = 0 # frame counter

max_age = 5  # no.of consecutive unmatched detection before 
             # a track is deleted

min_hits = 1  # no. of consecutive matches needed to establish a track

tracker_list =[] # list for trackers

# maximum amount of people to track at the same time
max_tracking = 10

# list for track ID
track_id_list = deque([str(x) for x in range(1, max_tracking + 1)])

debug = False

id_to_track = None

# ultrasonic sensor
TRIG = 22
ECHO = 11
SPEED = 34300
dist = 0
stop_measure = False

CENTER_X_CAM = 320

# tflite model interpreter
min_det_threshold = 0.55
imW, imH = 640, 480
MODEL_DIR = f"{os.path.abspath(os.path.dirname(__file__))}/obj_det_model"
PATH_TO_TFLITE = f"{MODEL_DIR}/ssd_mobilenet.tflite"
PATH_TO_LABELS = f"{MODEL_DIR}/ssd_mobilenet.txt"

with open(PATH_TO_LABELS, 'r') as f:
    labels = [line.strip() for line in f.readlines()]


class Model:
    def __init__(self, path_to_model):
        self.interpreter = Interpreter(model_path=path_to_model)
        self.interpreter.allocate_tensors()
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()
        self.height = self.input_details[0]['shape'][1]
        self.width = self.input_details[0]['shape'][2]
    
    def resize(self, frame):
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame_resized = cv2.resize(frame_rgb, (self.width, self.height))
        input_data = np.expand_dims(frame_resized, axis=0)
        return input_data
    
    def detect(self, input_data):
        self.interpreter.set_tensor(self.input_details[0]['index'],input_data)
        self.interpreter.invoke()
        self.boxes = self.interpreter.get_tensor(self.output_details[0]['index'])[0] # Bounding box coordinates of detected objects
        self.classes = self.interpreter.get_tensor(self.output_details[1]['index'])[0] # Class index of detected objects
        self.scores = self.interpreter.get_tensor(self.output_details[2]['index'])[0] # Confidence of detected objects
    
    def get_persons(self):
        persons = []
        for i in range(len(self.scores)):
            if ((self.scores[i] > min_det_threshold) and (self.scores[i] <= 1.0)):
                object_name = labels[int(self.classes[i])] # Look up object name from "labels" array using class index
                if object_name == "person":
                    # Get bounding box coordinates
                    # Interpreter can return coordinates that are outside of image dimensions, need to force them to be within image using max() and min()
                    ymin = int(max(1,(self.boxes[i][0] * imH)))         # highest y
                    xmin = int(max(1,(self.boxes[i][1] * imW)))         # left x
                    ymax = int(min(imH,(self.boxes[i][2] * imH)))       # lowest y
                    xmax = int(min(imW,(self.boxes[i][3] * imW)))       # right x
                    
                    persons.append(np.array([ymin, xmin, ymax, xmax]))
        return persons
    
    def get_output(self, frame):
        self.detect(self.resize(frame))
        return self.get_persons()


def get_dist():
    global dist
    global stop_measure
    while not stop_measure:
        GPIO.output(TRIG, True)
        time.sleep(0.00001)
        GPIO.output(TRIG, False)

        while GPIO.input(ECHO)==0:
          start_time = time.time()

        while GPIO.input(ECHO)==1:
          end_time = time.time()      
          
        dist = round((end_time - start_time) / 2 * SPEED, 2)
        time.sleep(1)

def find_id_to_track(boxes, center_x_cam):
    ids = [int(box.id) for box in boxes]
    boxes_center_x = [int((box.box[1] + box.box[3]) / 2) for box in boxes]
    
    # calculate offset (camera and center of the box) and take the minimum to find the id to track
    offsets = [abs(center_x_cam - center_x) for center_x in boxes_center_x]
    min_offset = min(offsets)
    x_to_track = boxes_center_x[offsets.index(min_offset)]
    return ids[offsets.index(min_offset)], x_to_track

def motor(x):
    global stored_x
    if x != None:
        if abs(x - stored_x) >= SERVO_THRESHOLD:
            print(f"Difference: {abs(x - stored_x)}")
            # pixels to duty
            duty = (lambda x: -147*x + 122667)(x)
            if duty > MAX_DUTY:
                duty = MAX_DUTY
            elif duty < MIN_DUTY:
                duty = MIN_DUTY
            change_duty(duty)
            stored_x = x

def assign_detections_to_trackers(trackers, detections, iou_thrd = 0.3):
    '''
    From current list of trackers and new detections, output matched detections,
    unmatched trackers, unmatched detections.
    '''    
    
    IOU_mat= np.zeros((len(trackers),len(detections)),dtype=np.float32)
    for t,trk in enumerate(trackers):
        #trk = convert_to_cv2bbox(trk) 
        for d,det in enumerate(detections):
         #   det = convert_to_cv2bbox(det)
            IOU_mat[t,d] = helpers.box_iou2(trk,det) 
    
    # Produces matches       
    # Solve the maximizing the sum of IOU assignment problem using the
    # Hungarian algorithm (also known as Munkres algorithm)
    
    matched_idx = linear_assignment(-IOU_mat)
    matched_idx = np.transpose(np.asarray(matched_idx))

    unmatched_trackers, unmatched_detections = [], []
    for t,trk in enumerate(trackers):
        if(t not in matched_idx[:,0]):
            unmatched_trackers.append(t)

    for d, det in enumerate(detections):
        if(d not in matched_idx[:,1]):
            unmatched_detections.append(d)

    matches = []
   
    # For creating trackers we consider any detection with an 
    # overlap less than iou_thrd to signifiy the existence of 
    # an untracked object
    
    for m in matched_idx:
        if(IOU_mat[m[0],m[1]]<iou_thrd):
            unmatched_trackers.append(m[0])
            unmatched_detections.append(m[1])
        else:
            matches.append(m.reshape(1,2))
    
    if(len(matches)==0):
        matches = np.empty((0,2),dtype=int)
    else:
        matches = np.concatenate(matches,axis=0)
    
    return matches, np.array(unmatched_detections), np.array(unmatched_trackers)       
    


def pipeline(img, boxes):
    '''
    Pipeline function for detection and tracking
    '''
    global frame_count
    global tracker_list
    global max_age
    global min_hits
    global track_id_list
    global debug
    global id_to_track
    
    frame_count+=1
    
    #img_dim = (img.shape[1], img.shape[0])
    z_box = boxes # measurement

    # no person detected
    if len(z_box) == 0:
        id_to_track = None

    if debug:
       print('Frame:', frame_count)
       
    x_box =[]
    if debug: 
        for i in range(len(z_box)):
           img1= helpers.draw_box_label(img, z_box[i], box_color=(255, 0, 0))
           plt.imshow(img1)
        plt.show()
    
    if len(tracker_list) > 0:
        x_to_track = None
        if id_to_track == None or id_to_track not in tracker_list:
            id_to_track, x_to_track = find_id_to_track(tracker_list, CENTER_X_CAM)
            print(id_to_track)
        
        for trk in tracker_list:
            x_box.append(trk.box)
            if int(trk.id) == id_to_track:
                # trk.box layout [y_up, x_left, y_down, x_right]
                x_to_track = int((trk.box[1] + trk.box[3])/2)
                #center_y = int((trk.box[0] + trk.box[2])/2)
                #cv2.circle(img, (x_to_track, 200), 10, (255,0,0), 5)
        #print(f"ID to track : {id_to_track}\nX to track : {x_to_track}")
        motor(x_to_track)
    else:
        id_to_track = None
    
    
    matched, unmatched_dets, unmatched_trks \
    = assign_detections_to_trackers(x_box, z_box, iou_thrd = 0.3)  
    if debug:
         print('Detection: ', z_box)
         print('x_box: ', x_box)
         print('matched:', matched)
         print('unmatched_det:', unmatched_dets)
         print('unmatched_trks:', unmatched_trks)
    
         
    # Deal with matched detections     
    if matched.size >0:
        for trk_idx, det_idx in matched:
            z = z_box[det_idx]
            z = np.expand_dims(z, axis=0).T
            tmp_trk= tracker_list[trk_idx]
            tmp_trk.kalman_filter(z)
            xx = tmp_trk.x_state.T[0].tolist()
            xx =[xx[0], xx[2], xx[4], xx[6]]
            x_box[trk_idx] = xx
            tmp_trk.box =xx
            tmp_trk.hits += 1
    
    # Deal with unmatched detections      
    if len(unmatched_dets)>0:
        for idx in unmatched_dets:
            z = z_box[idx]
            z = np.expand_dims(z, axis=0).T
            tmp_trk = tracker.Tracker() # Create a new tracker
            x = np.array([[z[0], 0, z[1], 0, z[2], 0, z[3], 0]]).T
            tmp_trk.x_state = x
            tmp_trk.predict_only()
            xx = tmp_trk.x_state
            xx = xx.T[0].tolist()
            xx =[xx[0], xx[2], xx[4], xx[6]]
            tmp_trk.box = xx
            tmp_trk.id = track_id_list.popleft() # assign an ID for the tracker
            #print(tmp_trk.id)
            tracker_list.append(tmp_trk)
            x_box.append(xx)
    
    # Deal with unmatched tracks       
    if len(unmatched_trks)>0:
        for trk_idx in unmatched_trks:
            tmp_trk = tracker_list[trk_idx]
            tmp_trk.no_losses += 1
            tmp_trk.predict_only()
            xx = tmp_trk.x_state
            xx = xx.T[0].tolist()
            xx =[xx[0], xx[2], xx[4], xx[6]]
            tmp_trk.box =xx
            x_box[trk_idx] = xx
            
       
    # The list of tracks to be annotated  
    good_tracker_list =[]
    for trk in tracker_list:
        if ((trk.hits >= min_hits) and (trk.no_losses <=max_age)):
             good_tracker_list.append(trk)
             x_cv2 = trk.box
             if debug:
                 print('updated box: ', x_cv2)
                 print()
             img= helpers.draw_box_label(trk.id,img, x_cv2) # Draw the bounding boxes on the 
                                             # images
    # Book keeping
    deleted_tracks = filter(lambda x: x.no_losses > max_age, tracker_list)  
    
    for trk in deleted_tracks:
            track_id_list.append(trk.id)
    
    tracker_list = [x for x in tracker_list if x.no_losses<=max_age]
    
    if debug:
       print('Ending tracker_list: ',len(tracker_list))
       print('Ending good tracker_list: ',len(good_tracker_list))
    
    cv2.imshow("frame",img)
    return img
    
if __name__ == "__main__":    
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(LED1, GPIO.OUT)
    GPIO.setup(LED2, GPIO.OUT)
    GPIO.setup(TRIG, GPIO.OUT)
    GPIO.setup(ECHO, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
    
    cam = PiCamera()
    cam.resolution = (imW, imH)
    cam.framerate = 30

    # flip vertical and horizontal
    cam.vflip = True
    cam.hflip = True

    rawCapture = PiRGBArray(cam, size=(imW, imH))

    #warm up camera
    time.sleep(0.1)
    
    model = Model(path_to_model=PATH_TO_TFLITE)
        
    # start=time.time()
    # output = 'test_v7.mp4'
    # clip1 = VideoFileClip("project_video.mp4")#.subclip(4,49) # The first 8 seconds doesn't have any cars...
    # clip = clip1.fl_image(pipeline)
    # clip.write_videofile(output, audio=False)
    # end  = time.time()
    
    #fourcc = cv2.VideoWriter_fourcc(*'XVID')
    #out = cv2.VideoWriter('output.avi',fourcc, 8.0, (640,480))
    
    dist_thread = Thread(target=get_dist)
    dist_thread.daemon = True
    dist_thread.start()
    
        
    for frame in cam.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        try:
            img = frame.array
            cv2.imshow("frame", img)
            
            np.asarray(img)
            boxes = model.get_output(img)
            pipeline(img, boxes)

            #out.write(new_img)
            #clear stream
            rawCapture.truncate(0)

            print(f"Distance: {dist}cm")

            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
        except Exception as e:
            print(str(repr(e)))
            break
    
    stop_measure = True
    change_duty(0)
    pwm.stop()
    cam.close()
    cv2.destroyAllWindows()
    GPIO.cleanup()
