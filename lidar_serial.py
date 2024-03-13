import serial
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from rclpy.clock import Clock
#from std_msgs.msg import Time
import time
import math

####################################
#Library for working with LD20 Lidar over USB port(or other device)
####################################


#Class for storing lidar data
#Contins radar_speed, start_angle, end_angle, time_stamp, and points
#points is an array of tuples containing distance and confidence
class LidarData:
    def __init__(self, radar_speed, start_angle, 
                 end_angle, time_stamp, points, intensities, laser_scan=True):
        self.speed = radar_speed
        self.start_angle = start_angle
        self.end_angle = end_angle
        self.time_stamp = time_stamp
        self.points = points
        self.intensities = intensities
        # if laser_scan:
        #     self.angle_distance = (self.end_angle-self.start_angle)/12
        #     self.scan_time = 1/6
        #     self.range_min = 0.8
        #     self.range_max = 5
        #     self.time_increments = 1/4000
        #     self.header = Header()
        #     self.header.stamp = Clock().now().to_msg()
        #     self.header.frame_id = 'lidar'
        #     self.scan = LaserScan()
        #     self.scan.header = self.header
        #     self.scan.angle_min = float(self.start_angle)
        #     self.scan.angle_max = float(self.end_angle)
        #     self.scan.angle_increment = float(self.angle_distance)
        #     self.scan.time_increment = float(self.time_increments)
        #     self.scan.scan_time = float(self.scan_time)
        #     self.scan.range_max = float(self.range_max)
        #     self.scan.range_min = float(self.range_min)
        #     self.scan.ranges = []
        #     self.scan.intensities = []
        #     for point in self.points:
        #         self.scan.ranges.append(float(point[0]))
        #         self.scan.intensities.append(float(point[1]))
            
                




#Lidar class. Creates connection upon initialization. port defaults to usb
#Use read_pack to read one package
class Lidar:
    def __init__(self, port='/dev/ttyUSB0', baud_rate=230400, frame_id='world', laser_scan=False):
        self.port = port
        self.baud_rate = baud_rate
        self.POINTS_PACK = 12
        self_laser_scan = laser_scan
        self.PACK_SIZE = (1 + 2 + 2 + (self.POINTS_PACK * 3) + 2 + 2 + 1)
        self.ser = serial.Serial(port=self.port,
                                 baudrate=self.baud_rate,
                                 timeout=5.0,
                                 bytesize=8,
                                 parity='N',
                                 stopbits=1)
        self.full_scan_num = 667
        self.frame_id = frame_id

    def create_msg(self, start_angle, end_angle, angle_increment, time_diff, points, intensities):
        msg = LaserScan()
        msg.header = Header()
        msg.header.stamp = Clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.angle_min = (start_angle*100*math.pi/180)
        msg.angle_max = (end_angle*100*math.pi/180)
        msg.angle_increment = angle_increment
        msg.time_increment = time_diff/self.full_scan_num
        msg.scan_time = time_diff
        msg.ranges = map((lambda x: x*1000), points)
        msg.intensities = intensities
        return msg

    def read_pack(self):
        start = self.ser.read()
        while(start != bytes('T', "ascii")):
            start = self.ser.read()
        msg = self.ser.read(self.PACK_SIZE)
        return self.process_pack(msg)

    def process_pack(self, msg):
        ver_len = msg[0]
        radar_speed = msg[1] | (msg[2] << 8)
        start_angle = msg[3] | (msg[4] << 8)
        end_angle = msg[self.PACK_SIZE-5] | (msg[self.PACK_SIZE-4] << 8)
        time_stamp = msg[self.PACK_SIZE-3] | (msg[self.PACK_SIZE-2] << 8)
        crc = msg[self.PACK_SIZE-1]
        data_points = []
        intensities = []
        for i in range(12):
            point = msg[5+3*i] | (msg[6+3*i] << 8)
            strength = msg[7+3*i]
            data_points.append(point)
            intensities.append(strength)
        return LidarData(radar_speed,start_angle,end_angle,time_stamp,data_points, intensities, laser_scan=True)
    
    def partial_read(self):
        start = self.ser.read()
        while(start != bytes('T', "ascii")):
            start = self.ser.read()
        msg = self.ser.read(self.PACK_SIZE)
        return self.partially_process_packprocess_pack(msg)

    def partially_process_pack(self, msg):
        crc = msg[self.PACK_SIZE-1]
        data_points = []
        intensities = []
        for i in range(12):
            point = msg[5+3*i] | (msg[6+3*i] << 8)
            strength = msg[7+3*i]
            data_points.append(point)
            intensities.append(strength)
        return data_points, strength

    def full_scan(self):
        data_points = []
        intensities = []
        for i in range(self.full_scan_num):
            if (i == 0):
                time_init = time.time()
                scan = self.read_pack()
                start_angle = scan.start_angle
                angle_increment = (scan.end_angle-scan.start_angle)/12
                data_points.append(scan.points)
                intensities.append(scan.intensities)
            elif (i == (self.full_scan_num-1)):
                time_diff = (time_init - time.time())
                scan = self.read_pack()
                end_angle = scan.end_angle
                data_points.append(scan.points)
                intensities.append(scan.intensities)
            else:
                points, strengths = self.partial_read()
                data_points.append(points)
                intensities.append(strengths)
        return self.create_msg(self, start_angle, end_angle, angle_increment, time_diff, points, intensities)





    def close(self):
        self.ser.close()

