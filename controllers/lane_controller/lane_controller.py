#CSCI 5302 Autonomous Driving Controller
#Srikanth Venkataraman 

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Camera, Motor, DistanceSensor
import numpy as np
import cv2 as cv
import math

from vehicle import Driver
 
# create the Robot instance.
#robot = Robot()
robot = Driver()
front_camera = robot.getCamera("front_camera")
rear_camera = robot.getCamera("rear_camera")
lidar = robot.getLidar("Sick LMS 291")

#for att in dir(robot):
#    print(att,getattr(robot,att))
# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())
print(timestep)
#print(dir(robot))

# You should insert a getDevice-like function in order to get the
#instance of a device of the robot. Something like:
 # motor = robot.getMotor('motorname')
 # ds = robot.getDistanceSensor('dsname')
 # ds.enable(timestep)


front_camera.enable(15)
rear_camera.enable(30)
# lidar.enable(100)
# lidar.enablePointCloud()
    

def process_front(img1,angle):
    #Return slope and midpoint 


    img = np.zeros([img1.shape[0], img1.shape[1], img1.shape[2]-1],np.uint8)
    img[:,:,0] = img1[:,:,0]
    #img[:,:,1] = img1[:,:,1]
    img[:,:,2] = img1[:,:,2]
   # img[:,:,3] = img1[:,:,3]


    img = img[35*8:46*8,199:824 ]
    dim = img.shape
    #print(dim)
    # cv.line(img,(561,79),(384 ,10),(255,0,0),1)
    # cv.line(img,(42,79),(229,10),(255,0,0),1)
    imgray = cv.cvtColor(img,cv.COLOR_BGR2GRAY)
    dstx = np.zeros([dim[0],dim[1]], np.uint8)
    kernel = np.ones((5,5),np.float32)/25
    img = cv.filter2D(img,-1,kernel)
    #dstx = cv.Sobel(imgray, cv.CV_8U, 0, 1, ksize=5);
    if(abs(angle>0.5)):
        ret,thresh = cv.threshold(imgray,79,255,0)
    else:
        ret,thresh = cv.threshold(imgray,73,255,0) 
    img = img[:,75:dim[1]-75]
    # cv.namedWindow("main", cv.WINDOW_NORMAL)
    # cv.imshow('main', thresh)          
    # cv.waitKey()


    rect = np.zeros((4, 2), dtype = "float32")
    rect[0] = [229,10]
    rect[2] =(561,79)
    rect[1] = (384 ,10)
    rect[3] = (42,79)
    width = 518
    height = 198
    dst = np.array([[0, 0],[width, 0],[width , height ],[0, height]], dtype = "float32")
    M = cv.getPerspectiveTransform(rect, dst)
    thresh = cv.warpPerspective(thresh, M, (width, height))
    
    # cv.namedWindow("main", cv.WINDOW_NORMAL)
    # cv.imshow('main', thresh)
    # cv.waitKey()
    # image, contours = cv.findContours(thresh,cv.RETR_TREE,cv.CHAIN_APPROX_SIMPLE)
    # img = cv.drawContours(im, contours, -1, (0,255,0), 3)
    contours, hierarchy = cv.findContours(thresh,cv.RETR_TREE,cv.CHAIN_APPROX_SIMPLE)
    cxs = np.zeros(250)
    cys = np.zeros(250)
    valid = [0,0]
    count = 0
#https://www.learnopencv.com/find-center-of-blob-centroid-using-opencv-cpp-python/
    if(len(contours)>1):
        for c in range(0,len(contours)):
    		
            M = cv.moments(contours[c])
    
            if M["m00"] != 0:
                cxs[c] = M["m10"] / M["m00"]
                cys[c] = M["m01"] / M["m00"]
                if(count<2): 
                    print(M["m00"])
                    if(M["m00"]<80):
                        return -1
                    valid[count] = c
                    count+=1
    
    else:
    	return -1      #no match                                
    
    slope = math.atan2((cys[valid[1]]-cys[valid[0]]),(cxs[valid[1]]-cxs[valid[0]]))+(math.pi/2)
    xm,ym = thresh.shape
    # print(cxs[valid[1]])
    # print(cxs[valid[0]])
    #midx = (cxs[valid[1]]+cxs[valid[0]])/2 -ym/2
    if(cys[valid[0]]>cys[valid[1]]):
        midx = cxs[valid[0]]-ym/2
    else:
        midx = cxs[valid[1]]-ym/2
    #uncomment this to see line drawn on image for each step
    # cv.line(thresh,(int(cxs[valid[0]]),int(cys[valid[0]])),(int(cxs[valid[1]]),int(cys[valid[1]])),(255,0,0),1)
    # cv.namedWindow("main", cv.WINDOW_NORMAL)
    # cv.imshow('main', thresh)
    # cv.waitKey()

    #print(slope)               
    print(midx)
    return((slope,midx))
# Main loop:
# - perform simulation steps until Webots is stopping the controller
steer = 0
b_max = 10
counter = 0
steer_last = 0
angle_last = 0
kp = .248
#kp = .21
#kp2 = 0.025
kp2 = .012
kd=-1.4
dsteer_max = .025
dsteer = 0
n_aq = 0
aq_l = 0
angle_buffer =np.zeros(b_max)
while robot.step() != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    img_f = front_camera.getImage()
    img_cv = np.frombuffer(img_f, np.uint8).reshape((front_camera.getHeight(), front_camera.getWidth(), 4))
    

    nav_con = process_front(img_cv,angle_last)

    if(nav_con!=-1):
        if(abs(nav_con[0]-angle_buffer[counter])>2):
            nav_con =-1
        else:
            angle_buffer[counter] = nav_con[0]
            xmid = nav_con[1]
            robot.setCruisingSpeed(30)
            #offset = nav_con[1]
            counter+=1
            #n_aq+=1
            #aq_l = 1
            if(counter==b_max):
                counter = 0
    else:
        xmid=0
        robot.setCruisingSpeed(30)
        #aq_l = 0
        
    angle = np.average(angle_buffer)
    da = angle-angle_last
    if(abs(angle)<.05):
        robot.setCruisingSpeed(55)
     #pid
    #steer = kp*math.sin(angle) +kd*da 
    steer = kp*angle +kd*da 
    if(angle<1 and angle>0 and xmid<-5 and steer>-.05):
        #print("correction left")
        steer = steer+ kp2*xmid/2
    if(angle>-1 and angle<0 and xmid>5 and steer<.05):
        #print("correction right")
        steer = steer+ kp2*xmid/2
    if(angle<1.5 and angle>1 and xmid<-5 and steer>-.05):
       # print("correction left")
        steer = steer+ kp2*xmid/4
    if(angle>-1.5 and angle<-1 and xmid>5 and steer<.05):
        #print("correction right")
        steer = steer+ kp2*xmid/4

    print("kda= " , kd*da)
    print("kpa= " , kp*math.sin(angle))
    

    if(nav_con==-1):

        steer = kp*1.27*math.sin(angle) 

    else:
        #steer = kp*math.sin(angle) +kd*da 
        dsteer = steer-steer_last
        steer_last = steer
        angle_last = angle
        if(steer-steer_last>dsteer_max):
            steer = steer_last +dsteer_max
        if(steer-steer_last<-dsteer_max):
            steer = steer_last -dsteer_max


    print(angle)
    print(steer)
    #print(n_aq)
    robot.setSteeringAngle(steer)


    #front_camera.saveImage("testimg.png",100)
    #robot.setCruisingSpeed(25)
    #robot.setSteeringAngle(0)
    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass


    
    