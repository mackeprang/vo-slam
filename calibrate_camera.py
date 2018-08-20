#####################
### Project in VO ###
## Mads Mackeprang ##
##  Copyright 2018 ##
##   DTU Denmark   ##
##-----------------##
## To calibrate    ##
## pi camera       ##
#####################

#  Part of this code is copied from https://docs.opencv.org/3.1.0/dc/dbb/tutorial_py_calibration.html #

import cv2
import numpy as np
import glob
from sys import argv
import os
import shutil
from picamera import PiCamera
from picamera.array import PiRGBArray
import time

images_path = "calib_data/"
create_images = False
for i,argument in enumerate(argv):
    if argument in '--images_path':
        images_path = str(argv[i+1])

    if argument in ('-ci','--create_images'):
        # Config parameters
        images_path = "calib_data/"
        create_images = True
        img_count = -1
        if not os.path.exists(images_path):
            os.makedirs(images_path)
        else:
            answer = raw_input("Delete all calibrate images? (y/n): ")
            if answer is "y":
                shutil.rmtree(images_path)
                os.makedirs(images_path)

if create_images:
    camera_resolution=(640/2,480/2)
    with PiCamera(resolution=camera_resolution,framerate = 24) as camera:
        with PiRGBArray(camera,size=camera_resolution) as stream:
            time.sleep(2)
            for frame in camera.capture_continuous(stream,format='bgr',use_video_port=True):
                stream.truncate()
                stream.seek(0)
                if img_count is -1:
                    img_count += 1
                    continue
                img = frame.array
                img = cv2.flip(img,-1)
                cv2.imshow("Calibrate Image",img)
                #if cv2.waitKey(20) == 32: # Space
                time.sleep(2)
                img_count += 1
                print("Image number: {}".format(img_count))
                img_name = images_path+"img%.2d" % img_count + ".png"
                cv2.imwrite(img_name, img)
                stream.truncate(0)
                if img_count == 60 or cv2.waitKey(20) == ord('q'):
                    break


cv2.destroyAllWindows()

if images_path is not None:
    ### CONFIG ###
    # image dimensions

    scale = 25.4  # [mm] real world size of checker board squares (1 inch)
    # number of chessboard squares:
    chessboard_w = 6
    chessboard_h = 9
    ##############

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    objp = np.zeros((chessboard_w*chessboard_h, 3), np.float32)
    objp[:, :2] = np.mgrid[0:chessboard_h, 0:chessboard_w].T.reshape(-1, 2)
    objp = objp*scale

    objpoints = []  # 3d point in real world space
    imgpoints = []  # 2d points in image plane.

    images = glob.glob(''.join((images_path,'*.png')))
    imshape = None
    for i,fname in enumerate(images):
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        imshape = gray.shape

        ret, corners = cv2.findChessboardCorners(gray, (chessboard_h, chessboard_w), None)

        if ret is True:
            objpoints.append(objp)
            cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners)
            print(i)
            #cv2.drawChessboardCorners(img, (chessboard_h, chessboard_w), corners, ret)
            #fname_new = fname[0:-4]
            #cv2.imshow("img", img)
            #cv2.imwrite(fname_new, img)
            #print "Img number: %d" % i+1
            #key = cv2.waitKey(100) & 0xFF
             

    cv2.destroyAllWindows()
    ret, cam_mat, distcoeff, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, imshape[::-1], None, None)
    save_mat = raw_input("Save camera matrix and distortions parameters: ")
    if save_mat is "y":
        csv_cam_mat = open("cam_mat.csv",'w')
        csv_dist_coeff = open("dist_coeff.csv",'w')
    print(ret)
    np.savetxt("cam_mat.csv",cam_mat,delimiter=',')
    np.savetxt("dist_coeff.csv", distcoeff, delimiter=',')
    print(cam_mat)
##    print(distcoeff)
