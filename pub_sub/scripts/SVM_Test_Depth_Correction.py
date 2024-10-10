# -*- coding: utf-8 -*-
"""
Created on Tue Mar  5 12:30:47 2024

@author: Samer
"""

import joblib
import numpy as np

# Raw camera readings (in meters)
x_cam = 0.222269
y_cam = 0.122342
z_cam = 3.36979

# Load the model from the file
correction_svm_model = joblib.load('depth_correction_model.pkl')
corrected_z = correction_svm_model.predict(np.array([x_cam,y_cam,z_cam]).reshape(1,-1))

# Publish x_cam, y_cam and corrected_z in meters ya 3m Mohab 3la ROS :D