import sys
ros_path = '/opt/ros/kinetic/lib/python2.7/dist-packages'
if ros_path in sys.path:
    sys.path.remove(ros_path)
import cv2 
import numpy as np
import os
import glob
if __name__ == "__main__":
    zoomout_ratio=200
    cut_border = True
    images=[]
    images = glob.glob("/home/ramlab/Documents/apriltag-master/apriltag-imgs/tag36h11/*.png")
    images.sort()
    file_len=len(images)
    for i,fname in enumerate(images):
        img = cv2.imread(fname)
        if cut_border:
            max_img=np.zeros(((img.shape[0]-2)*zoomout_ratio,(img.shape[1]-2)*zoomout_ratio),dtype=np.uint8) 
        else:
            max_img=np.zeros((img.shape[0]*zoomout_ratio,img.shape[1]*zoomout_ratio),dtype=np.uint8) 
        file_name=fname.split('/')[-1]
        # print(file_name)
        sys.stdout.write('  [Process]:%.2f%%' % float(100*i/file_len) + '\r')
        for i in range(img.shape[0]):
            for j in range(img.shape[1]):
                if cut_border:
                    if i == 0 or i == 9 or j==0 or j==9:
                        continue
                if cut_border:
                    max_img[(i-1)*zoomout_ratio:i*zoomout_ratio,(j-1)*zoomout_ratio:j*zoomout_ratio]=img[i,j,0]
                else: 
                    max_img[i*zoomout_ratio:((i+1)*zoomout_ratio),j*zoomout_ratio:((j+1)*zoomout_ratio)]=img[i,j,0]
                save_path="/home/ramlab/Documents/apriltag-master/big_tags/tag36h11/"+file_name
                cv2.imwrite(save_path,max_img)
    print("Done!")
