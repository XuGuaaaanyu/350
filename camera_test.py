# import libraries
import cv2
from picamera2 import Picamera2
import time
import numpy as np

# setting up the Pi camera
picam2 = Picamera2()
dispW=1280
dispH=720
picam2.preview_configuration.main.size = (dispW,dispH)
picam2.preview_configuration.main.format = "RGB888"
picam2.preview_configuration.controls.FrameRate=30
picam2.preview_configuration.align()
picam2.configure("preview")
picam2.start()

# define variables
fps=0
pos=(30,60)
font=cv2.FONT_HERSHEY_SIMPLEX
height=1.5
weight=3
myColor=(0,0,255)
GhueLow = 10
GhueHigh = 179
GsatLow = 66
GsatHigh = 255
GvalLow = 65
GvalHigh = 255
RhueLow = 0
RhueHigh = 10
RsatLow = 100
RsatHigh = 255
RvalLow = 100
RvalHigh = 255


# main program
while True:
    # start timing
    tStart=time.time()
    
    # grab a frame from Pi camera
    frame= picam2.capture_array()
    
    # if you need to invert the picture, uncomment the following line
    #frame=cv2.flip(frame,-1)
    
    # transformation of color encoding from BGR (not RGB) to HSV (easier to adjust by hand)
    frameHSV=cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)
    
    # add FPS text
    cv2.putText(frame,str(int(fps))+' FPS',pos,font,height,myColor,weight)

    
    # define my masks
    GlowerBound=np.array([GhueLow,GsatLow,GvalLow])
    GupperBound=np.array([GhueHigh,GsatHigh,GvalHigh])
    GmyMask=cv2.inRange(frameHSV,GlowerBound,GupperBound)
    
    RlowerBound=np.array([RhueLow,RsatLow,RvalLow])
    RupperBound=np.array([RhueHigh,RsatHigh,RvalHigh])
    RmyMask=cv2.inRange(frameHSV,RlowerBound,RupperBound)
    
    # define my objects
    GmyObject=cv2.bitwise_and(frame,frame, mask=GmyMask)
    RmyObject=cv2.bitwise_and(frame,frame, mask=RmyMask)
    
    # get the contour
    Gcontours,junk=cv2.findContours(GmyMask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    Rcontours,junk=cv2.findContours(RmyMask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    if len(Gcontours)>0:
        # sort all contours that satisfy our HSV requirement
        Gcontours=sorted(Gcontours,key=lambda x:cv2.contourArea(x),reverse=True)
        
        # get the largest contour
        Gcontour=Gcontours[0]
        
        # define a rectangle that surrounds the largest contour
        x1,y1,w1,h1=cv2.boundingRect(Gcontour)
        cv2.rectangle(frame,(x1,y1),(x1+w1,y1+h1),(0,255,0),3)
    
        if len(Rcontours)>0:
            # sort all contours that satisfy our HSV requirement
            Rcontours=sorted(Rcontours,key=lambda x:cv2.contourArea(x),reverse=True)
            
            # get the largest contour
            Rcontour=Rcontours[0]
            
            # define a rectangle that surrounds the largest contour
            x2,y2,w2,h2=cv2.boundingRect(Rcontour)
            cv2.rectangle(frame,(x2,y2),(x2+w2,y2+h2),(0,0,255),3)
            if x2+w2/2>x1+w1/2:
                print('Green is on the left')
            if x2+w2/2<x1+w1/2:
                print('Green is on the right')
    print()
    cv2.imshow("Camera", frame)
    
    # exit the program when key 'q' is pressed
    if cv2.waitKey(1)==ord('q'):
        break
    
    # stop timing
    tEnd=time.time()
    
    # calculate frames per second
    loopTime=tEnd-tStart
    fps=.9*fps + .1*(1/loopTime)
cv2.destroyAllWindows()