#!/usr/bin/env python3

# libraries Include
import numpy as np
from ultralytics import YOLO
import torch
import cv2
import time

test_img_path = "/home/jetsonx/catkin_ws/src/pub_sub/scripts/test_image.png"
test_img = cv2.imread(test_img_path)
test_img = cv2.resize(test_img,(640,640))
print(test_img.shape)
cv2.imshow('test',test_img)
print(cv2.__version__)

cap = cv2.VideoCapture(0)

if cap.isOpened():
	print('cam success')
else:
	print('cam failed')

torch.cuda.set_device(0)
device= torch.device('cuda')

#############################################
# Model initialization
#############################################

model_weights_path = "/home/jetsonx/catkin_ws/src/pub_sub/scripts/best.pt"    
# This line assigns the file path to the variable.

model =YOLO(model_weights_path)                                                           
model.to(device=device)
# passing the weights file to YOLO Model.

# Define the Labels by taking the instance Number and return the Instance itself:
def getInstanceName(instance_number):
    
    labels = ['gate','path marker','badge','gun','box','hand','barrel','note','phone','bottle','gman','bootlegger','axe','dollar','beer']
    return labels[instance_number] 


while(1):
#This line captures a frame from a video capture 
	result = model.predict(source=test_img , show = True)
    
