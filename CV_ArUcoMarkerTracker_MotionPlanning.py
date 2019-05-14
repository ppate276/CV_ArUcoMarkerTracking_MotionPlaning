'''
    Created by:
    Parth Patel
    Oluwadamilola Saka
    Likhith Vennam
    Spring 2019
'''
import os
import cv2
from cv2 import aruco
import numpy as np
import yaml
from picamera.array import PiRGBArray
from picamera import PiCamera
import time
import RPi.GPIO as GPIO
from AlphaBot import AlphaBot

# load calibration file
with open('calibration.yaml') as f:
    loadeddict = yaml.load(f)

camera_matrix = loadeddict.get('camera_matrix')
dist_coeffs = loadeddict.get('dist_coeff')

camera_matrix = np.matrix(camera_matrix)
dist_coeffs = np.matrix(dist_coeffs)

image_size = (640, 480)
map1, map2 = cv2.initUndistortRectifyMap(camera_matrix, dist_coeffs, None, None, image_size, cv2.CV_16SC2)

#import 6x6 arUco tags lib
aruco_dict = aruco.Dictionary_get( aruco.DICT_6X6_1000 )

# marker sizes that we used were ~ 10cmx10cm
markerLength = 9.60 # Here, our measurement unit is centimetre.

arucoParams = aruco.DetectorParameters_create()

camera = PiCamera()
camera.resolution = (640, 480)
camera.framerate = 32
rawCapture = PiRGBArray(camera, size=(640, 480))

#*****
#track wheel rotation via encoders
Ab = AlphaBot();
Ab.stop();

cntl = 8;
cntr = 7;

EncR = 0.0;
EncL = 0.0;

nt = 0.0;

n = 1;

def updateEncoderL(channel):
    global EncL;
    EncL += 1;
    #print 'valEncL = %d' %EncL

def updateEncoderR(channel):
    global EncR;
    EncR += 1;
    #print 'valEncR = %d' %EncR

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False);
GPIO.setup(cntr, GPIO.IN);
GPIO.setup(cntl, GPIO.IN);
GPIO.add_event_detect(cntr, GPIO.BOTH, updateEncoderR)
GPIO.add_event_detect(cntl, GPIO.BOTH, updateEncoderL)

#*****
# allow the camera to warmup
time.sleep(0.1)

Ab.stop();
x = 2000;

while(True): 
    # ret, img = cap.read() # Capture frame-by-frame
    # grab an image from the camera
    #search for a marker
    
    while x > 0:
        Ab.setMotor(30,30);
        x = x - 1;
        
    Ab.stop();
    x = 2000;
    
    #grab an image
    camera.capture(rawCapture, format="bgr")
    img = rawCapture.array
    rawCapture.truncate(0)
    imgRemapped = cv2.remap(img, map1, map2, cv2.INTER_LINEAR, cv2.BORDER_CONSTANT) # for fisheye remapping
    imgRemapped_gray = cv2.cvtColor(imgRemapped, cv2.COLOR_BGR2GRAY)    # aruco.etectMarkers() requires gray image
    corners, ids, rejectedImgPoints = aruco.detectMarkers(imgRemapped_gray, aruco_dict, parameters=arucoParams) # Detect aruco
    
    # when marker is found, calculate distance and angle to it. move towards it.
    if np.all(ids == n): # if aruco marker detected
        EncR = 0;
        EncL = 0
        Ab.stop();
        rvec, tvec, _objPoints = aruco.estimatePoseSingleMarkers(corners, markerLength, camera_matrix, dist_coeffs) # For a single marker
        #print "R:", rvec
        print "T:", tvec
        #print "Tvec1:", tvec[:,:,1]
        tx = float(tvec[:,:,0])
        ty = float(tvec[:,:,1])
        tz = float(tvec[:,:,2])
        print tx
        print ty
        print tz
        
        #calculate how many revs to marker
        nt = (tz / 20.4885);
        # stop 1 rev short
        nt = nt - 1;

        # Make nt turns
        while ((EncR + EncL)/2)/40.0 < nt:
            Ab.setMotor(-40,40);
            print 'EncR = %d' %EncR;
            print 'EncL = %d' %EncL;
        Ab.stop();
        #x = x- 1
        print "ID:", ids
        #imgWithAruco = aruco.drawDetectedMarkers(imgRemapped, corners, ids, (0,255,0))
        #imgWithAruco = aruco.drawAxis(imgWithAruco, camera_matrix, dist_coeffs, rvec, tvec, 100) # axis length 100 can be changed according to your requirement
        n = n + 1;
        time.sleep(3);
        
    else:   # if aruco marker is NOT detected
        imgWithAruco = imgRemapped  # assign imRemapped_color to imgWithAruco directly

    cv2.imshow("aruco", imgWithAruco)   # display
    if n > 7:
        break
    if cv2.waitKey(2) & 0xFF == ord('q'):   # if 'q' is pressed, quit.
        break
