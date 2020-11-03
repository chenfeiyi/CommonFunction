#include "ros/ros.h"    
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>

#include <iostream>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/common/common.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Char.h>
#include <std_msgs/String.h>

#include <vector>
#include <string>

#include <fstream>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;

ros::Publisher PointCloudInfo_pub;
ros::Publisher ImageInfo_pub;

PointCloud2 syn_pointcloud;
Image syn_iamge;

int counter;

void Syncallback(const PointCloud2ConstPtr& ori_pointcloud,const ImageConstPtr& ori_image)
{
    cout << "\033[1;32m Syn! \033[0m" << endl;
    syn_pointcloud = *ori_pointcloud;
    syn_iamge = *ori_image;
    cout << "syn pointcloud' timestamp : " << syn_pointcloud.header.stamp << endl;
    cout << "syn image's timestamp : " << syn_iamge.header.stamp << endl;


    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    cv_bridge::CvImagePtr cv_image_ptr;

    cv::Mat cv_image;
    

    cv_image_ptr = cv_bridge::toCvCopy(ori_image, sensor_msgs::image_encodings::BGR8);
    
    cv_image = cv_image_ptr->image;
    
    pcl::fromROSMsg(*ori_pointcloud, *pcl_cloud);

    vector<int> compression_params;
	compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);  //选择jpeg
	compression_params.push_back(100); //在这个填入你要的图片质量
    cv::imwrite("/home/ramlab/Documents/CornerSeg/dataset/corner_aprilTag_point/carla/img1080/" + std::to_string(counter) + ".jpg", cv_image,compression_params);
    pcl::io::savePCDFileASCII("/home/ramlab/Documents/CornerSeg/dataset/corner_aprilTag_point/carla/pcd1080/" + std::to_string(counter) + ".pcd", *pcl_cloud);
    
    counter ++;
    PointCloudInfo_pub.publish(syn_pointcloud);
    ImageInfo_pub.publish(syn_iamge);
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "hw1");
    ros::NodeHandle node;

    counter = 0;

    // cout << "\033[1;31m hw1! \033[0m" << endl;
    ROS_WARN("synchronizer");

    // 建立需要订阅的消息对应的订阅器
    message_filters::Subscriber<PointCloud2> PointCloudInfo_sub(node, "/carla/ego_vehicle/lidar/lidar1/point_cloud", 1);
    message_filters::Subscriber<Image> ImageInfo_sub(node, "/carla/ego_vehicle/camera/rgb/front/image_color", 1);
    
    typedef sync_policies::ApproximateTime<PointCloud2, Image> MySyncPolicy; 
    
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), PointCloudInfo_sub, ImageInfo_sub); //queue size=10
    sync.registerCallback(boost::bind(&Syncallback, _1, _2));


    PointCloudInfo_pub = node.advertise<PointCloud2>("/djq_pc", 10);
    ImageInfo_pub = node.advertise<Image>("/djq_image", 10);

    ros::spin();
    return 0;
}
