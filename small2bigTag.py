import sys
ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
if ros_path in sys.path:
    sys.path.remove(ros_path)
import cv2 
import numpy as np
import os
import glob
if __name__ == "__main__":
    images=[]
    max_img=np.zeros((800,800),dtype=np.uint8) 
    images = glob.glob("/home/ramlab/Documents/apriltag-master/apriltag-imgs/tag36h11/*.png")
    images.sort()
    file_len=len(images)
    zoomout_ratio=100
    for i,fname in enumerate(images):
        img = cv2.imread(fname)
        file_name=fname.split('/')[-1]
        # print(file_name)
        sys.stdout.write('  [Process]:%.2f%%' % float(100*i/file_len) + '\r')
        for i in range(10):
            for j in range(10):
                if i==0 or i==9 or j==0 or j==9:
                    continue
                max_img[(i-1)*zoomout_ratio:(i)*zoomout_ratio,(j-1)*zoomout_ratio:j*zoomout_ratio]=img[i,j,0]
                save_path="/home/ramlab/Documents/apriltag-master/big_tags/tag36h11/"+file_name
                cv2.imwrite(save_path,max_img)
    print("Done!")
