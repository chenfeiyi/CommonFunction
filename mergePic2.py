import sys
sys.path.remove("/opt/ros/kinetic/lib/python2.7/dist-packages")
import cv2
import numpy as np
import os 
import glob

def func(wall_img, img, row, col):
    wall_img[size * row+offset: size * row+offset + img.shape[0], size * col+offset: size * col+offset + img.shape[0], :] = img


if __name__ == "__main__":

    wall_size=2#10mX10m
    tag_size=0.5#50cmX50cm
    grid_size = np.floor(wall_size/tag_size)
    print("grid_size ",grid_size)

    file_list=glob.glob("/home/ramlab/Documents/CornerSeg/catkin_ws/tag36h11/*.png")
    file_list.sort()
    img1=cv2.imread(file_list[0])
    img_shape=img1.shape
    print(img_shape)

    offset = 50
    size = img_shape[0] + offset
    left_wall_img=np.ones((int(grid_size*img_shape[0]+offset*5),int(grid_size*img_shape[0]+offset*5),int(img_shape[2])),np.uint8)*255
    
    for i in range(4):
        for j in range(4):
            img1=cv2.imread(file_list[i*4+j])
            func(left_wall_img, img1, i, j)


    cv2.imwrite("/home/ramlab/Documents/CornerSeg/catkin_ws/left_wall.png",left_wall_img)

    right_wall_img=np.ones((int(grid_size*img_shape[0]+offset*5),int(grid_size*img_shape[0]+offset*5),int(img_shape[2])),np.uint8)*255

    for i in range(4):
        for j in range(4):
            img1=cv2.imread(file_list[16+i*4+j])
            func(right_wall_img, img1, i, j)
    cv2.imwrite("/home/ramlab/Documents/CornerSeg/catkin_ws/right_wall.png",right_wall_img)

    ground_wall_img=np.ones((int(grid_size*img_shape[0]+offset*5),int(grid_size*img_shape[0]+offset*5),int(img_shape[2])),np.uint8)*255
    for i in range(4):
        for j in range(4):
            img1=cv2.imread(file_list[32+i*4+j])
            func(ground_wall_img, img1, i, j)
    cv2.imwrite("/home/ramlab/Documents/CornerSeg/catkin_ws/ground_wall.png",ground_wall_img)


    # wall_img[wall_img.shape[0]-img_shape[0]-1:-1,wall_img.shape[1]-img_shape[1]-1:-1,:]=img1
    # cv2.imshow("color",wall_img)
    # cv2.waitKey(0)


