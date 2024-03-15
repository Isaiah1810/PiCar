import serial
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from rclpy.clock import Clock
import time
import math
import numpy as np

####################################
#Library for working with LD20 Lidar over USB port(or other device)
####################################


#Class that stores lidar data when more than points and intensities are needed
class LidarData:
    def __init__(self,start_angle, end_angle, points, intensities):
        self.start_angle = start_angle
        self.end_angle = end_angle
        self.points = points
        self.intensities = intensities


#Lidar class. Creates connection upon initialization. port defaults to usb0, 
#frame_id is the frame of the laser scan message, defaults to 'world'.
class Lidar:
    def __init__(self, port='/dev/ttyUSB0', baud_rate=230400, frame_id='world'):
        self.port = port
        self.baud_rate = baud_rate
        self.POINTS_PACK = 12
        self.PACK_SIZE = (1 + 2 + 2 + (self.POINTS_PACK * 3) + 2 + 2 + 1)
        self.ser = serial.Serial(port=self.port,
                                 baudrate=self.baud_rate,
                                 timeout=5.0,
                                 bytesize=8,
                                 parity='N',
                                 stopbits=1)
        self.full_scan_num = 667
        self.frame_id = frame_id

    #Creates a ROS LaserScan message from lidar data
    def create_msg(self, start_angle, end_angle, angle_increment, time_diff, points, intensities):
        msg = LaserScan()
        msg.header = Header()
        msg.header.stamp = Clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.angle_min = float(end_angle/100*math.pi/180)
        msg.angle_max = float(start_angle/100*math.pi/180)
        msg.angle_increment = 2*math.pi/self.full_scan_num
        msg.time_increment = float(time_diff/self.full_scan_num)
        msg.scan_time = float(time_diff)
        msg.range_max = 8.0
        msg.range_min = 0.3
        msg.ranges = points
        msg.intensities = intensities
        return msg


    #Read a packet and output full lidar data
    def read_pack(self):
        start = self.ser.read()
        while(start != bytes('T', "ascii")):
            start = self.ser.read()
        msg = self.ser.read(self.PACK_SIZE)
        return self.process_pack(msg)

    #Takes raw bytes and returns a LidarData object
    def process_pack(self, msg):
        start_angle = msg[3] | (msg[4] << 8)
        end_angle = msg[self.PACK_SIZE-5] | (msg[self.PACK_SIZE-4] << 8)
        #################
        #Unused components
        ##################
        #ver_len = msg[0]
        #radar_speed = msg[1] | (msg[2] << 8)
        #time_stamp = msg[self.PACK_SIZE-3] | (msg[self.PACK_SIZE-2] << 8)
        #crc = msg[self.PACK_SIZE-1]

        data_points = []
        intensities = []
        #The LD20 Lidar does 12 measurments for each scan
        for i in range(12):
            point = msg[5+3*i] | (msg[6+3*i] << 8)
            strength = msg[7+3*i]
            data_points.append(point)
            intensities.append(strength)
        return LidarData(start_angle,end_angle,data_points, intensities)
    
    #Read one packet and return only distances and intensities
    def partial_read(self):
        start = self.ser.read()
        while(start != bytes('T', "ascii")):
            start = self.ser.read()
        msg = self.ser.read(self.PACK_SIZE)
        return self.partially_process_pack(msg)

    #Take raw lidar bytes and return distances and intensities
    def partially_process_pack(self, msg):
        crc = msg[self.PACK_SIZE-1]
        data_points = []
        intensities = []
        for i in range(12):
            point = msg[5+3*i] | (msg[6+3*i] << 8)
            strength = msg[7+3*i]
            data_points.append(point/1000)
            intensities.append(float(strength))
        return data_points, intensities

    #Do take one full 360 scan and return ROS LaserScan message
    def full_scan(self):
        data_points = []
        intensities = []
        for i in range(self.full_scan_num):
            if (i == 0):
                time_init = time.time()
                scan = self.read_pack()
                start_angle = scan.start_angle
                angle_increment = None
                data_points.extend(scan.points)
                intensities.extend(scan.intensities)
            elif (i == (self.full_scan_num-1)):
                time_diff = (time.time()-time_init)
                scan = self.read_pack()
                end_angle = scan.end_angle
                data_points.extend(scan.points)
                intensities.extend(scan.intensities)
            else:
                points, strengths = self.partial_read()
                data_points.extend(points)
                intensities.extend(strengths)
        return self.create_msg(start_angle, end_angle, angle_increment, time_diff, data_points, intensities)

    def close(self):
        self.ser.close()

