import rospy
import numpy as np
from sensor_msgs.msg import PointCloud2 
def callback(data):
    
if __name__ == "__main__":  
    rospy.init_node("merge_cloud")
    rospy.Subscriber("/livox/lidar",PointCloud2,callback,queue_size=10)
    rospy.spin()