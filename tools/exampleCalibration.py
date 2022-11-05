import sys
import cv2
import math
import numpy as np
import pyKinectAzure.pykinect_azure as pykinect
from pyKinectAzure.pykinect_azure.k4a import _k4a

import ctypes

if __name__ == "__main__":

    # Initialize the library, if the library is not found, add the library path as argument
    pykinect.initialize_libraries()


    
    # Modify camera configuration
    device_config = pykinect.default_configuration

    ## 카메라 화소 및 depth 마다 Camera Matrix가 다르므로 여건에 맞춰서 설정
    device_config.color_resolution = pykinect.K4A_COLOR_RESOLUTION_720P
    device_config.depth_mode = pykinect.K4A_DEPTH_MODE_WFOV_2X2BINNED

    # Start device & Calibration 
    device = pykinect.start_device(config=device_config)
    calibration = device.get_calibration(device_config.depth_mode, device_config.color_resolution)



    ### Print Camera Matrix  

    """
    color_matrix = calibration.get_matrix("color")
    depth_matrix = calibration.get_matrix("depth")
    intrinsics = calibration.__str__()
    
    print(color_matrix)
    print(depth_matrix)
    print(intrinsics)

    #k4a_float3_t camera_coordinate
    """

    # 우선은 이미지로 실험
    image = cv2.imread("./depth_image/depth_8bit_height330.png",cv2.IMREAD_UNCHANGED)
    #image = cv2.imread("./depth_image/depth_height330.png",cv2.IMREAD_UNCHANGED)
    np.asarray(image)


    #k4a_float3_t Image Plane Coordinate => 현재는 임의로 머리와 발 픽셀 설정
    head_pixel = _k4a.k4a_float2_t()
    head_pixel.xy.x = 268
    head_pixel.xy.y = 123


    feet_pixel = _k4a.k4a_float2_t()
    feet_pixel.xy.x = 268
    feet_pixel.xy.y = 408

    Head_Depth = image[268,123].astype('float')
    Feet_Depth = image[268,408].astype('float')

    # Image Plane Coordinate을 Camera Coordinate으로 변환
    head = calibration.convert_2d_to_3d( head_pixel,  Head_Depth, 0, 0)
    feet = calibration.convert_2d_to_3d( feet_pixel,  Feet_Depth, 0, 0)
    #k4a_calibration_2d_to_3d(depth_camera_calibration, (256,256), fDepth, K4A_CALIBRATION_TYPE_DEPTH, K4A_CALIBRATION_TYPE_DEPTH, xyz)


    #Camera Coordinate에서 키 계산
    print(head.v[0])
    print(head.v[1])
    print(head.v[2])

    print(feet.v[0])
    print(feet.v[1])
    print(feet.v[2])

    height = math.sqrt(pow(head.v[0] - feet.v[0],2) + pow(head.v[1] - feet.v[1],2) + pow(head.v[2] - feet.v[2],2))

    print(height)