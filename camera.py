#!/usr/bin/env python

import socket
import numpy as np
import cv2
import os
import time
import struct


class Camera(object):

    def __init__(self):

        # Data options (change me)
        self.im_height = 720
        self.im_width = 1280
        self.tcp_host_ip = '127.0.0.1'
        self.tcp_port = 50010
        self.buffer_size = 10*4 + self.im_height*self.im_width*5 # in bytes


        # Connect to server
        self.tcp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_socket.connect((self.tcp_host_ip, self.tcp_port))

        self.intrinsics = None
        self.get_data()


    def get_data(self):

        # Ping the server with anything
        self.tcp_socket.send(b'asdf')

        # Fetch TCP data:
        #     color camera intrinsics, 9 floats, number of bytes: 9 x 4
        #     depth scale for converting depth from uint16 to float, 1 float, number of bytes: 4
        #     depth image, self.im_width x self.im_height uint16, number of bytes: self.im_width x self.im_height x 2
        #     color image, self.im_width x self.im_height x 3 uint8, number of bytes: self.im_width x self.im_height x 3
        data = b''
        while len(data) < (9*4+9*4+16*4+4+8+self.im_height*self.im_width*5):
                data += self.tcp_socket.recv(self.buffer_size)

        # while len(data) < (10*4 + self.im_height*self.im_width*5):
        #     data += self.tcp_socket.recv(self.buffer_size)

        # Reorganize TCP data into color and depth frame
        self.color_intr = np.fromstring(data[0:(9*4)],np.float32).reshape(3,3)
        self.depth_intr = np.fromstring(data[(9*4):(9*4+9*4)],np.float32).reshape(3,3)
        self.depth2color_extr = np.fromstring(data[(9*4+9*4):(9*4+9*4+16*4)],np.float32).reshape(4,4)
        depth_scale = np.fromstring(data[(9*4+9*4+16*4):(9*4+9*4+16*4+4)],np.float32)[0]
        self.timestamp = np.fromstring(data[(9*4+9*4+16*4+4):(9*4+9*4+16*4+4+8)],np.long)[0]
        depth_im = np.fromstring(data[(9*4+9*4+16*4+4+8):((9*4+9*4+16*4+4+8)+self.im_width*self.im_height*2)],np.uint16).reshape(self.im_height,self.im_width)
        self.color_im = np.fromstring(data[((9*4+9*4+16*4+4+8)+self.im_width*self.im_height*2):],np.uint8).reshape(self.im_height,self.im_width,3)
        depth_im = depth_im.astype(float) * 0.0001 # * depth_scale

        # Set invalid depth pixels to zero
        depth_im = depth_im.copy()
        #depth_im[depth_im > self.max_depth] = 0.0
        self.depth_im = depth_im
        return self.color_im,self.depth_im
