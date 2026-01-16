#!/usr/bin/env python3
from __future__ import print_function
import cv2 as cv
import argparse
import numpy as np


#
# parser = argparse.ArgumentParser(description='This program shows how to use background subtraction methods provided by \
#                                               OpenCV. You can process both videos and images.')
# parser.add_argument('--input', type=str, help='Path to a video or a sequence of image.', default='../data/vtest.avi')
# parser.add_argument('--back', type=str, help='Path to a image.', default='../data/fruits.jpg')
# parser.add_argument('--algo', type=str, help='Background subtraction method (KNN, MOG2).', default='MOG2')
# args = parser.parse_args()

## [create]
#create Background Subtractor objects
# default method is gaussian mixture-based
backSub = cv.createBackgroundSubtractorMOG2()
# if you want to another Background Subtractor objects, please uncomment the below line.
#backSub = cv.createBackgroundSubtractorKNN()
## [create]

## [capture]
capture = cv.VideoCapture('../data/vtest.avi')
if not capture.isOpened:
    print('Unable to open: ' + args.input)
    exit(0)

## [create]
## [background picture]
img_bg = cv.imread('../data/test1.jpeg')
img_bg_size = img_bg.shape
## [capture]

while True:
    ret, frame = capture.read()
    if frame is None:
        break

    source_size= frame.shape

    ## [resize background]
    fx = source_size[1] / img_bg_size[1]
    fy = source_size[0] / img_bg_size[0]
    img_back_resize = cv.resize(img_bg, (0, 0), fx=fx, fy=fy, interpolation=cv.INTER_CUBIC)

    cv.imshow('Background', img_back_resize)
    ## [apply]
    #update the background model
    fgMask = backSub.apply(frame)
    ## [apply]

    ## [display_frame_number]
    #get the frame number and write it on the current frame
    cv.rectangle(frame, (10, 2), (100,20), (255,255,255), -1)
    cv.putText(frame, str(capture.get(cv.CAP_PROP_POS_FRAMES)), (15, 15),
               cv.FONT_HERSHEY_SIMPLEX, 0.5 , (0,0,0))
    ## [display_frame_number]

    ## erode and dilate
    erode=cv.erode(fgMask,None,iterations=1)
    dilate=cv.dilate(erode,None,iterations=4)
    cv.imshow('dilate', dilate)


    # [fusion]
    try:
        for i in range(img_bg_size[0]):
            for j in range(img_bg_size[1]):
                if dilate[i,j] == 255: # white point in Mask image
                    img_back_resize[i,j] = frame[i,j]
    except:
        pass


    ## [show]
    #show the current frame and the fg masks
    cv.imshow('Frame', frame)
    cv.imshow('FG Mask', fgMask)
    ## [show]
    #show the fused frame
    cv.imshow('Fused', img_back_resize)
    keyboard = cv.waitKey(30)
    if keyboard == 'q' or keyboard == 27:
        break
