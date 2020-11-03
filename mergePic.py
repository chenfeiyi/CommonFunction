import sys
sys.path.remove("/opt/ros/kinetic/lib/python2.7/dist-packages")
import cv2
import numpy as np
import os 
import glob

def func(wall_img, img, row, col):
    print(wall_img.shape[0])
    wall_img[size * row: size * row + img.shape[0], size * col: size * col + img.shape[0], :] = img


if __name__ == "__main__":

    wall_size=2.5#10mX10m
    tag_size=0.5#50cmX50cm
    grid_size = np.floor(wall_size/tag_size)
    print("grid_size ",grid_size)

    file_list=glob.glob("/home/ramlab/Documents/CornerSeg/catkin_ws/tagStandard52h13/*.png")
    file_list.sort()
    img1=cv2.imread(file_list[0])
    img_shape=img1.shape
    print(img_shape)

    offset = 10
    size = img_shape[0] + offset
    left_wall_img=np.ones((int(grid_size*img_shape[0]),int(grid_size*img_shape[0]),int(img_shape[2])),np.uint8)*255
    

    img1=cv2.imread(file_list[0])
    func(left_wall_img, img1, 3, 3)
    img1=cv2.imread(file_list[1])
    func(left_wall_img, img1, 2, 2)
    img1=cv2.imread(file_list[2])
    func(left_wall_img, img1, 3, 1)
    img1=cv2.imread(file_list[3])
    func(left_wall_img, img1, 2, 0)


    cv2.imwrite("/home/ramlab/Documents/CornerSeg/catkin_ws/left_wall.png",left_wall_img)

    right_wall_img=np.ones((int(grid_size*img_shape[0]),int(grid_size*img_shape[0]),int(img_shape[2])),np.uint8)*255

    img1=cv2.imread(file_list[4])
    func(right_wall_img, img1, 3, 0)
    img1=cv2.imread(file_list[5])
    func(right_wall_img, img1, 2, 1)
    img1=cv2.imread(file_list[6])
    func(right_wall_img, img1, 3, 2)
    img1=cv2.imread(file_list[7])
    func(right_wall_img, img1, 2, 3)
    cv2.imwrite("/home/ramlab/Documents/CornerSeg/catkin_ws/right_wall.png",right_wall_img)

    ground_wall_img=np.ones((int(grid_size*img_shape[0]),int(grid_size*img_shape[0]),int(img_shape[2])),np.uint8)*255
    img1=cv2.imread(file_list[8])
    func(ground_wall_img, img1, 1, 0)
    img1=cv2.imread(file_list[9])
    func(ground_wall_img, img1, 2, 1)
    img1=cv2.imread(file_list[10])
    func(ground_wall_img, img1, 1, 2)
    img1=cv2.imread(file_list[11])
    func(ground_wall_img, img1, 2, 3)
    cv2.imwrite("/home/ramlab/Documents/CornerSeg/catkin_ws/ground_wall.png",ground_wall_img)


    # wall_img[wall_img.shape[0]-img_shape[0]-1:-1,wall_img.shape[1]-img_shape[1]-1:-1,:]=img1
    # cv2.imshow("color",wall_img)
    # cv2.waitKey(0)


