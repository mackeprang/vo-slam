#####################
### Project in VO ###
## Mads Mackeprang ##
##  Copyright 2018 ##
##   DTU Denmark   ##
#####################

import cv2
import socket
import time
from pathlib import Path
from sys import argv
from imutils.video import FPS
import numpy as np
from numpy.linalg import norm
import multiprocessing as mul
import RPi.GPIO as GPIO
from picamera import PiCamera
from picamera.array import PiRGBArray
import threading, thread
import math
import netifaces as ni
import smbus

host = 'localhost'
port = 5002

# Possible commandline arguments
verbose = False
save_3d_points = False
display_flow = False
print_fps = False
data_ready = False
start_server = False
with_imu = False
threadLock = threading.Lock()
backward_flow_threshold = 3
for i, argument in enumerate(argv):
    if argument in '--verbose':
        verbose = True
    if argument in '--save_3d':
        save_3d_points = True
    if argument in '--display_flow':
        print("Display flow is on.")
        display_flow = True
    if argument in '--fps':
        print("Printing FPS in console.")
        print_fps = True
    if argument in '--server':
        start_server = True
    if argument in '--port':
        if int(argv[i+1]) > 4000 and int(argv[i+1])<6000:
            print "Using new port: " + argv[i+1]
            port = int(argv[i+1])
        else:
            print "Port number has to be between 4000 - 6000. \nUsing default portnumber: " + str(port)
    if argument in '--imu':
        with_imu = True
        
# Configuration parameters:
camera_resolution = (320,240)

good_feature_params = dict(maxCorners = 2000,
                           qualityLevel = 0.01,
                           minDistance = 5,
                           blockSize = 3,
                           useHarrisDetector = False)

optical_flow_params = dict(winSize = (10,10),
                           maxLevel = 2,
                           criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.01))

class TCP_server(threading.Thread):
    def __init__(self,host,port,name):
        threading.Thread.__init__(self)
        self.stop = False
        self.data = []
        self.data_ready = False
        self.name = name
        self.sock = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        self.host = ni.ifaddresses('wlan0')[ni.AF_INET][0]['addr']
        self.port = port
        self.sock.bind((self.host,self.port))
        
    def run(self):
        conn = None
        self.sock.listen(1)
        self.sock.settimeout(1)
        print("TCP Online. Waiting for a connection. ip address: {}".format(self.host))
        while True:
            if conn is None:
                try:
                    conn, addr = self.sock.accept()
                    print("Connected to: {}".format(str(addr)))
                except:
                    pass
            else:
                try:
                    if self.data_ready:
                        self.data_ready = False
##                        t,x,y,z = self.data
##                        data = '{0:.5f} {1:.3f} {2:.3f} {3:.3f}\n'.format(float(t),float(x),float(y),float(z))
                        x,y,z = self.data
                        data = '{0:.3f} {1:.3f} {2:.3f}\n'.format(float(x),float(y),float(z))
                        conn.send(data)
                except Exception as e:
                    conn = None
                    print(e)
                    pass
                
            if self.stop:
                try:
                    conn.send('q')
                finally:
                    time.sleep(1)
                    break
            time.sleep(0.1)
        print "Exiting " + self.name
        if conn is not None:
            conn.close()
        self.sock.close()
        
    def send_data(self,data):
        self.data = data
        self.data_ready = True
            
class IMU(threading.Thread):
    def __init__(self,loop_time=0.001,name='imu_thread'):
        threading.Thread.__init__(self)
        self.loop_time = loop_time
        self.time_const = 0.001
        self.stop = False
        self.gravity = np.array([[0],[0],[0]])
        self.data = np.array([[0],[0],[0]])
        self.speed = np.array([[0],[0],[0]])
        self.data_ready = False
        self.name = name
        self.vel = 0.0
        self.i2c = smbus.SMBus(1)
        self.IMU_ADDRESS             = 0x68
        self.MPU9150_PWR_MGMT_1      = 0x6B
        self.MPU9150_GYRO_CONFIG     = 0x1B
        self.MPU9150_ACCEL_CONFIG    = 0x1C
        self.ACCEL_OUT               = 0x3B
        self.init = self.init()
        self.bias = np.array([[0],[0],[0]])
        self.bias = self.calibrate()

    def init(self):
        try:
            self.i2c.write_byte_data(self.IMU_ADDRESS,self.MPU9150_PWR_MGMT_1,0x00)
            time.sleep(2)
            self.i2c.write_byte_data(self.IMU_ADDRESS,self.MPU9150_GYRO_CONFIG,0x00)
            time.sleep(0.1)
            self.i2c.write_byte_data(self.IMU_ADDRESS,self.MPU9150_ACCEL_CONFIG,0x00)
            time.sleep(0.1)
            return True
        except:
            return False

    def run(self):
        if not self.init:
            return False
        print("Starting IMU")
        t_start = time.clock()
        t_diff = 0.0
        t_diff_start = 0.0
        while not self.stop:
            t_end = time.clock()
            try:
                if t_end-t_start > self.loop_time:
                    t_start = time.clock()
                    self.data = self.read_data()
                    t_diff = time.clock()-t_diff_start
                    t_diff_start = time.clock()
                    alpha = self.time_const/(self.time_const+t_diff)
                    self.gravity = np.add(alpha*self.gravity, (1-alpha)*self.data)
                    acc = np.subtract(self.data,self.gravity)
                    acc = np.add(acc,self.bias)
                    self.speed = np.add(self.speed, acc*t_diff)
                    vel = norm(self.speed)
                    self.vel = 0.5*vel + (1-0.5)*self.vel
            except Exception as e:
                print(e)
                break
            time.sleep(0.0001)
            
    def read_data(self,n=3):
        acc = np.array([[0],[0],[0]])
        for i in range(n):
            data = self.i2c.read_i2c_block_data(self.IMU_ADDRESS,self.ACCEL_OUT,6)
            acc_x = (self.data_conv(data[1],data[0])/16384.0)*9.82
            acc_y = (self.data_conv(data[3],data[2])/16384.0)*9.82
            acc_z = (self.data_conv(data[5],data[4])/16384.0)*9.82
            acc = np.add(acc,np.array([[acc_x],[acc_y],[acc_z]]))
        return np.divide(acc,n)

    def calibrate(self,n=2000):
        if not self.init:
            return False
        print("Calibrating the IMU, {} iterations".format(n))
        bias = np.array([[0],[0],[0]])
        gravity = np.array([[0],[0],[0]])
        t_start = time.clock()
        t_diff_start = 0.0
        count = 0
        while count <= n:
            t_end = time.clock()
            if t_end-t_start > self.loop_time:
                count = count+1
                t_start = time.clock()
                acc = self.read_data(n=3)
                t_diff = time.clock()-t_diff_start
                t_diff_start = time.clock()
                alpha = self.time_const/(self.time_const+t_diff)
                gravity = np.add(np.dot(alpha,gravity), np.dot((1-alpha),acc))
                bias = np.add(bias,np.subtract(acc,gravity))
        print("Calibration done")
        print(np.divide(bias,n))
        #print(np.divide(gravity,n))
        #print(t_diff)
        return np.divide(bias,n)

    def data_conv(self,data1,data2):
        value = data1 | (data2 << 8)
        if (value & (1 << 16 -1)):
            value -= (1<<16)
        return value
    
    def get_speed_imu(self):
        return self.vel
        
# Methods
def find_3d_points(Q1,P1,Q2,P2):
    Q1 = np.reshape(Q1,(-1 , 2))
    Q2 = np.reshape(Q2,(-1 , 2))
    Q = []
    for (q1, q2) in zip(Q1, Q2):
        B = np.array([P1[2] * q1[0] - P1[0],
                      P1[2] * q1[1] - P1[1],
                      P2[2] * q2[0] - P2[0],
                      P2[2] * q2[1] - P2[1]])

        u, s, v = np.linalg.svd(B, full_matrices=0)
        v = np.transpose(v)
        v = v[:, -1]
        Q = np.hstack((Q, v[:3] / np.repeat(v[3], 3)))
    return Q.reshape(-1, 3)

def draw_flow(img,p1,p2,mask=None):
    if mask is None:
        p1 = np.reshape(p1,(-1 , 2))
        p2 = np.reshape(p2,(-1 , 2))
        if len(p1) > 0:
            for i, (new,old) in enumerate(zip(p1,p2)):
                a,b = new.ravel()
                c,d = old.ravel()
                img = cv2.line(img,(a,b),(c,d),(0,255,0),1)
    else:
        p1_inliers = p1[mask==1]
        p2_inliers = p2[mask == 1]
        p1_outliers = p1[mask == 0]
        p2_outliers = p2[mask == 0]
        if len(p1_inliers) > 0:
            p1_inliers = np.reshape(p1_inliers,(-1,2))
            p2_inliers = np.reshape(p2_inliers, (-1, 2))
            for i, (new, old) in enumerate(zip(p1_inliers, p2_inliers)):
                a, b = new.ravel()
                c, d = old.ravel()
                img = cv2.line(img, (a, b), (c, d), (0, 255, 0), 1)
        if len(p1_outliers) > 0:
            p1_outliers = np.reshape(p1_outliers, (-1, 2))
            p2_outliers = np.reshape(p2_outliers, (-1, 2))
            for i, (new, old) in enumerate(zip(p1_outliers, p2_outliers)):
                a, b = new.ravel()
                c, d = old.ravel()
                img = cv2.line(img, (a, b), (c, d), (0, 0, 255), 1)
    return img

def sparse_optical_flow(img1,img2,points,fb_threshold,optical_flow_params):
    old_points = points.copy()
    new_points, status , err = cv2.calcOpticalFlowPyrLK(img1,img2,points,None,**optical_flow_params)
    if fb_threshold>0:
        new_points_r, status_r, err = cv2.calcOpticalFlowPyrLK(img2,img1,new_points,None,**optical_flow_params)
        new_points_r[status_r==0] = False#np.nan
        fb_good = (np.fabs(new_points_r-points) < fb_threshold).all(axis=2)
        new_points[~fb_good] = np.nan
        old_points = np.reshape(points[~np.isnan(new_points)],(-1,1,2))
        new_points = np.reshape(new_points[~np.isnan(new_points)],(-1,1,2))
    return new_points,old_points

def update_motion(points1,points2,Rpos,tpos,cam_mat=None,scale = 1.0):
    E, mask = cv2.findEssentialMat(points1,points2,cameraMatrix=cam_mat,method=cv2.RANSAC,prob=0.999, mask=None)
    newmask = np.copy(mask)
    _,R,t,newmask = cv2.recoverPose(E,points1,points2,cameraMatrix=cam_mat,mask=newmask)
    tp = tpos+np.dot(Rpos,t)*scale
    Rp = np.dot(R,Rpos)
    return Rp,tp,mask

def get_scale(p1,p2):
    scale = 0
    p1 = np.reshape(p1,(-1 , 2))
    p2 = np.reshape(p2,(-1 , 2))
    for i, (new, old) in enumerate(zip(p1, p2)):
        x1, y1 = new.ravel()
        x2, y2 = old.ravel()
        xd = x2-x1
        yd = y2-y1
        scale += math.sqrt(xd*xd+yd*yd)
    scale = (scale/len(p1))*0.1
    return scale
        

###### VISUEL ODOMETRY ######
## Initialization of parameters
prev_frame = None
Q = None
mask = None
threads = []
Rpos = np.eye(3,3,dtype=np.float32)
tpos = np.zeros((3,1),dtype=np.float32)
cam_mat = np.loadtxt("cam_mat.csv", dtype=np.float32, delimiter=',')
dist_coeff = np.loadtxt("dist_coeff.csv", dtype=np.float32, delimiter=',')
camera = PiCamera(resolution=camera_resolution)
stream  = PiRGBArray(camera,size=camera_resolution)
time.sleep(2)
key = ''
scale = 0.1
fps = FPS().start()
t1 = None
t2 = None
if start_server:
    print("Starting server thread.")
    thr1 = TCP_server(host,port,"server")
    thr1.start()
    threads.append(thr1)

if with_imu:
    print("Starting IMU thread.")
    thr2 = IMU()
    thr2.start()
    threads.append(thr2)
    
## Starting Visual odometry
print("Starting VO")
try:
    for frame in camera.capture_continuous(stream,format='bgr',use_video_port=True):
        stream.truncate()
        stream.seek(0)
        frame = stream.array
        frame = cv2.flip(frame,-1)
        gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
        key = cv2.waitKey(2)
        if key & 0xFF == ord('q'):
            break
        if key & 0xFF == ord('r'):
            print("Reset position")
            Rpos = np.eye(3,3,dtype=np.float32)
            tpos = np.zeros((3,1),dtype=np.float32)

        if prev_frame is None:
            prev_frame = gray.copy()
            prev_points = cv2.goodFeaturesToTrack(prev_frame, mask=None, **good_feature_params)
            try:
                min_features = len(prev_points)
            except:
                prev_points = []
                prev_frame = gray.copy()
                continue
            print("Number of found feature: {}".format(min_features))
            continue

        if len(prev_points)<min_features*0.5 or len(prev_points)<50:
            prev_points = cv2.goodFeaturesToTrack(prev_frame, mask=None, **good_feature_params)
            try:
                min_features = len(prev_points)
            except:
                prev_points = []
                prev_frame = gray.copy()
                continue
            
            print("Updating number of feature: {}".format(min_features))

        new_points,prev_points = sparse_optical_flow(prev_frame,gray,prev_points,backward_flow_threshold,optical_flow_params)
        if len(new_points) < 16:
            prev_frame = gray.copy()
            stream.truncate()
            stream.seek(0)
            continue
        old_tpos = np.copy(tpos)
        P1 = np.dot(cam_mat,np.hstack((Rpos,old_tpos)))
        if with_imu:
            scale = thr2.get_speed_imu()
            scale = abs(scale-0.1)
            print("Speed is: {}".format(scale))
        else:
            scale = get_scale(prev_points,new_points)
        Rpos,tpos,mask = update_motion(prev_points,new_points,Rpos,old_tpos,cam_mat,scale)
        if start_server:
            try:
                thr1.send_data(tpos)
            except Exception as e:
                print(e)
                pass
        P2 = np.dot(cam_mat,np.hstack((Rpos,tpos)))

        if save_3d_points:
            if Q is None:
                Q = find_3d_points(prev_points, P1, new_points, P2)
            else:
                Q = np.vstack((Q,find_3d_points(prev_points, P1, new_points, P2)))
        
        fps.update()
        fps.stop()
        flow_img = draw_flow(frame,prev_points,new_points,mask=mask)
        if display_flow:
            flow_img = cv2.putText(flow_img,
                               "FPS: {:.2f}".format(fps.fps()),
                               #np.array_str(tpos,precision=2,suppress_small=True),
                               (int(flow_img.shape[1]//100),
                                int(flow_img.shape[0]//10)),
                               cv2.FONT_HERSHEY_SIMPLEX,
                               flow_img.shape[1]/(640*1.1),
                               (0,255,0),2)

            cv2.imshow('Flow_img', flow_img)

        prev_frame = gray.copy()
        prev_points = new_points.copy()
        if print_fps:
            print("FPS: {:.2f}".format(fps.fps()))
        if verbose:
            t1 = time.time()
        if t2 is not None:
            t3 = time.time()
            print "Rest: " + str(t3-t2)
            
except KeyboardInterrupt:
    print "\n\nKeyboardInterrupt"
    pass
except Exception as e:
    print(e)
    pass

# Clean-up
fps.stop()
camera.close()
cv2.destroyAllWindows()
if threads: #No threads started
    for t in threads:
        t.stop = True
        t.join()
        print("Terminating: {}".format(t.name))
    
print("Program ended.")
if save_3d_points:
    points3d_file = open('points3d.txt','w')
    for point in Q:
        x,y,z = point
        points3d_file.write('{0:f},{1:f},{2:f}\n'.format(x,y,z))
