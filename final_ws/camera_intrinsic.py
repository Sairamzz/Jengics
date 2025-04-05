import numpy as np

'''Zhi Tan's Camera Logitech c270'''

# TODO: "DONE" assign actual camera intrinsic values through calibration
camera_matrix = np.array([[800.34154022,   0. ,        330.068031],
 [  0.        , 801.58117864 ,270.23508093],
 [  0.         ,  0.       ,    1.        ]], dtype=np.float32)  # 0, 0, 1


dist_coeffs =  np.array([[-3.27782913e-03,  1.50646740e+00,  2.47993711e-03, -7.50227307e-03, -5.82926301e+00]])


'''My Laptop HP Envy'''
# camera_matrix = np.array( [[526.72911527, 0.0, 320.98251076],
#  [0.0, 528.41016751, 251.78354461],
#  [0.0, 0.0, 1.0]]
# , dtype=np.float32)  # 0, 0, 1

# dist_coeffs =  np.array( [[ 0.0325677 , -0.10725443 , 0.0055427  , 0.01178159 , 0.9617012 ]])



# TODO: "DONE" use actual tags size here
tag_size = 0.0096

# Define the 3D points of the AprilTag corners in its local coordinate frame
obj_points = np.array([
    [-tag_size / 2, -tag_size / 2, 0],  # Bottom-left
    [ tag_size / 2, -tag_size / 2, 0],  # Bottom-right
    [ tag_size / 2,  tag_size / 2, 0],  # Top-right
    [-tag_size / 2,  tag_size / 2, 0]   # Top-left
], dtype=np.float32)

CAMERA_MODE = 0 # 0 for hp envy, 2 or 1 for Zhi Tan's camera