import sys
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from autoware_msgs.msg import VehicleCmd  
import math
import time

counter = 0

now = []
class lidar_feature:
    
    
    def __init__(self):
        self.msg = VehicleCmd()
        self.subcriber = rospy.Subscriber("/velodine_point", PointCloud2, self.callbackLidar, queue_size=10)
        #self.pub = rospy.Publisher('/vehicle_cmd',VehicleCmd, queue_size = 10)
        
        rospy.init_node('lidar',anonymous=True)
        rate = rospy.Rate(10) # 10hz
        
    def callbackLidar(self, ros_data):
    
        print(len(ros_data.data))
        
        for p in point_cloud2.read_points(ros_data, skip_nans=True):
            
            px = p[0]
            py = p[1]
            pz = p[2]
            now.append([p[0],p[1]])

        right =abs(now[13051][1])
        left =abs(now[13951][1])
        karsı = abs(now[12601][0])
        print(karsı)
        print(right,left)
        

def main(args):

    lf = lidar_feature()
    try:
        rospy.spin()
        
    except KeyboardInterrupt:
        print("Shutdown")


main(sys.argv)