import cv2
import numpy as np
import time
from djitellopy import tello
# https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_objdetect/py_face_detection/py_face_detection.html
#https://raw.githubusercontent.com/opencv/opencv/master/data/haarcascades/haarcascade_frontalface_default.xml
#^haarcascade download link
pid=[0.4,0.4,0] # pid - p -proportional, i- integral, d- derivative
#pid is used to calculate the error in process variables like speed, position, temperature etc and apple principle of proportional, derivation and integration to calculate the action needed to make these variables as close to their target values
#https://www.omega.co.uk/prodinfo/pid-controllers.html
# may have to tune pid now
pError=0
vspeed=0 # vertical speed
verror=0
facedetected=0
drone = tello.Tello()
drone.connect()
drone.streamon()
drone.takeoff()
drone.send_rc_control(0,0,25,0)
time.sleep(2.5)
#cap= cv2.VideoCapture(0)
print (drone.get_battery())



w, h = 360, 240
fbRange=[6200,6800]
def findFace(img):
    #use a method proposed by viola-jones of haarcascades, all paramterrs and info of a model to detect different objects. This file is available on official open cv
    faceCascade = cv2.CascadeClassifier("Resources/haarcascade_frontalface_default.xml") # cascade classifier is a function that has been trained with machine learning to detect different objects
    # Cascading refers to the process of performing multiple operations/tasks in a single line of programming code
    #we use functions from the cascade file to see if it is a face or not
    imgGray= cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)# converting image to grayscale
    faces =faceCascade.detectMultiScale(imgGray,1.2,8)# deteccting the face on grayscale
    #1.2 is scale factor and 8 is  minimum neighbours. This function returns 4 values which we unpack later to get face position and area
    myFaceListC=[]  # ccoordinates of fce detected
    myFaceListArea= [] # area of face detected
    for(x,y,w,h) in faces: # faces will have 4 values and we are unopacking the values
        cv2.rectangle(img,(x,y),(x+w,y+h),(0,0,255),2) # w- width, y - height
        cx= x+ w//2
        cy = y+ h//2 # means it will  return integer for division
        area= w*h
        cv2.circle(img,(cx,cy),5,(0,255,0), cv2.FILLED)
        myFaceListC.append([cx,cy])
        myFaceListArea.append(area)
    if len(myFaceListArea)!=0:
        i= myFaceListArea.index(max(myFaceListArea))
        return img, [myFaceListC[i], myFaceListArea[i]]
    else:
        return img, [[0,0],0]

def trackFace(info,w,h,pid, pError):
    area= info[1]
    x,y= info[0]
    fb=0
    vspeed=0
    verror= h//2-y
    error= x- w//2 # w/2 is centre of image. x is pos of face and we are finding how farobject is from centre
    speed= pid[0]*error + pid[1]*(error - pError) # pid[1] is derivative
    speed = int(np.clip(speed,-100,100)) #calculate yaw speed of the drone
    #fb is forward and backward speed
    if y!=0 and abs(verror)>3:
        print(verror)
        vspeed= int(np.clip(verror,-30,30))
        print(vspeed)
    if area>fbRange[0] and area< fbRange[1]: # too big move back
        fb=0
    elif area>fbRange[1]:
        fb=-20
    elif area< fbRange[0] and area !=0: # too small move forward
        fb=20
    if x==0:
        speed=0
        error=0

    drone.send_rc_control(0,fb,vspeed,speed)
    return error

while True:
    #_, img= cap.read() #"_" will get the next frame in the camera (via "cap"). "img" will obtain return value from getting the camera frame, either true of false
    img = drone.get_frame_read().frame
    img = cv2.resize(img,(w,h)) #now we need to get face from drone camera image
    findFace(img)
    img, info= findFace(img)
    pError= trackFace( info, w, h,pid,pError)
    print("centre", info[0],"Area", info[1])
    cv2.imshow("Image", img)
    if cv2.waitKey(1) & 0xFF== ord('l'):
        drone.land() # safety feature to land drone
        break
#-->ord('l') returns the Unicode code point of l

#-->cv2.waitkey(1) returns a 32-bit integer corresponding to the pressed key

#-->& 0xFF is a bit mask which sets the left 24 bits to zero, because ord() returns a value betwen 0 and 255, since your keyboard only has a limited character set

#-->Therefore, once the mask is applied, it is then possible to check if it is the corresponding key.
