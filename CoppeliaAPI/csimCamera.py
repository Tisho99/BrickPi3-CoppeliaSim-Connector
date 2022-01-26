# Author: Alberto Martínez Rodríguez
# Date: January 2022

import numpy as np
import time
import cv2
import CoppeliaAPI.lib.b0RemoteApi as b0RemoteApi
import CoppeliaAPI.cfg as cfg
from CoppeliaAPI.cfg import csimLock

cfg = cfg.Cfg()

class PiCamera():
    """Provides a pure Python interface to the CoppeliaSim camera."""
    def __init__(self):
        """Constructor"""
        self.resolution = (512, 512) #Default resolution
        self.client = b0RemoteApi.RemoteApiClient('b0RemoteApiNodeCamera','b0RemoteApiChannel')
        self.client.cameraHandle = self.client.simxGetObjectHandle('visionCamera' + str(cfg.ROBOT_ID),self.client.simxServiceCall())[1]
        self.client.simxStartSimulation(self.client.simxDefaultPublisher())
        time.sleep(0.25)


    def __getstate__(self):
        state = self.__dict__.copy()
        del state["client"]
        return state

    def __setstate__(self, state):
        self.__dict__.update(state)
        # Add baz back since it doesn't exist in the pickle
        self.client = b0RemoteApi.RemoteApiClient('b0RemoteApiNodeBrickpi','b0RemoteApiChannel')
        self.client.cameraHandle = self.client.simxGetObjectHandle('visionCamera' + str(cfg.ROBOT_ID),self.client.simxServiceCall())[1]

    def close(self):
        """Finalizes the state of the camera."""
        del self.client


    def capture(self, output, format="bgr", **options):
        """
        Capture an image from the camera, storing it in *output*.

        :param output: rerefence parameter to return the BGR image
        :param format: color formar of the image, "rbg" and "bgr" supported
        optional parameters are for not causing errors, this only returns a BGR image
        """
        reverse = cfg.CAM_REVERSE

        with csimLock:
            ok, self.true_resolution, imageBytes = self.client.simxGetVisionSensorImage(self.client.cameraHandle, False, self.client.simxServiceCall())
        if(not ok): raise SystemError("CoppeliaSim isn't able to return image, check configurations")
        self.true_resolution = tuple(self.true_resolution)

        # Construct the RGB numpy image
        img = np.zeros([self.true_resolution[1], self.true_resolution[0], 3], dtype = "uint8")
        for i in range(self.true_resolution[1]):
            for j in range(self.true_resolution[0]):
                pixel = list(imageBytes[3*(j+i*self.true_resolution[0]) : 3*(j+i*self.true_resolution[0]) + 3])
                img[self.true_resolution[1] - 1 - i][j] = pixel

        # Check format
        if format == "bgr": img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        elif format == "rgb": pass
        else: raise ValueError("Only rgb and bgr formats supported")

        if reverse: img = cv2.flip(cv2.flip(img, 0), 1)

        # Create PiRGBArray class for transparence
        if type(output) != PiRGBArray:
            output = PiRGBArray(self, size = self.true_resolution)
        elif(hasattr(output, 'size') and output.size != self.resolution):
             raise ValueError("Camera resolution and PiRGBArray resolution aren't equal")
        output.array = cv2.resize(img, self.resolution, interpolation = cv2.INTER_AREA)

        

class PiRGBArray():
    """Class that contains the image in the self.array field stored as a numpy array (same interface as PiCamera Class)"""

    def __init__(self, camera, size=None):
        self.camera = camera
        self.size = size
        self.array = None
