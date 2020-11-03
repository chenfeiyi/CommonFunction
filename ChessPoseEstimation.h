#pragma once

// #include <opencv/highgui.h>
// #include <opencv/cv.h>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
// #include <opencv2/core/eigen.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <vector>
#include <ros/ros.h>
#include "dirent.h"
#include "iterator"
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include "Kabsch.hpp"

// #include "pcl/for_each_type.h"

#include "tagStandard41h12.h"
#include "tagStandard52h13.h"
#include "tag36h11.h"
#include <apriltag_pose.h>
#include <apriltag.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <pcl/common/transforms.h>	
// user defined header
#include "CloudNormalEstimation.h"

using namespace cv;
using namespace std;
#define TAG_HEIGHT 0.346 //the lowest height of apriltag on the vertical wall plane
struct TagInfo{
    int32_t id;
    uint32_t seq_num;
    double centralPointDistance;
    double tagCornerPixelCoor[4][2];
    double cPixelCoor[2];
    Eigen::Vector3f tagCornerCameraCoor[4];
    Eigen::Vector3f tagCentralCameraCoor;
    Eigen::Vector3f normal;
};
bool resetTagCoor(TagInfo &tag1,TagInfo &tag2,TagInfo &tag3);
bool resetCoor(Eigen::Vector3f &Cx,Eigen::Vector3f &Cy,Eigen::Vector3f &Cz);
void getFileList(std::vector<std::string> &file_list,std::string folder_path);
//for chessboard
void CalcBoardCornerPositions(vector<Point3f>& corner_object_points, Size board_size=Size(3, 4), float square_size=13.67/*(in millimeter)*/);
bool GetPose(Mat& undist_in_img, Mat& transfvec12, Mat& rvec, Mat& tvec, vector<Point3f> corner_object_points/*(3D object points)*/, Mat camera_matrix, Mat dist_coeffs, bool print=true, Size board_size=Size(3, 4));
void DrawObjCoordFrame(Mat& undist_in_img, Mat rvec, Mat tvec, Mat camera_matrix, Mat dist_coeffs);
//for apriltag
void DrawDetections(cv::Mat &image,apriltag_detection_t *det,apriltag_detection_info_t info,apriltag_pose_t pose);
Eigen::Vector3d CameraCoor2PixelCoor(Eigen::Vector3d &pt,Eigen::Matrix3d intrinsic,apriltag_pose_t pose);
//for both
void UndistortImage(Mat& in_img, Mat camera_matrix, Mat dist_coeffs);
void UndistortImage(Mat& in_img, Eigen::Matrix3f camera_matrix, Eigen::Matrix<float,5,1> dist_coeffs);
bool eval(string base_file_path);
/**
 * @brief This class is going recognize apriltag and give the position information back
 *        The precidure will be 1) new a object of this class and init with a constructer function or default and init() function
 *                              2) input your image and get information
 *                              3) if you want to visualize it you can call DrawCoor() function
 *         CornerTag cornerTag(info,family_name) or  CornerTag cornerTag() and  cornerTag.init()
 *         DrawCoor.Detect(img)
 *         DrawCoor.DrawCoor()
*/         
class CornerTag
{
public:
    CornerTag(apriltag_detection_info_t info,std::string tag_family="tag36h11");
    CornerTag();
    ~CornerTag();
    void Destory();
    void init(apriltag_detection_info_t info,std::string tag_family="tag36h11");
    void RefPlane2CameraCoor(Eigen::Vector3d &pt,apriltag_pose_t pose);
    void Detect(cv::Mat img,std::vector<TagInfo> &tagInfo);
    bool GetTagInfo(std::vector<TagInfo> &tagInfo);

    // int Detect(cv_bridge::CvImagePtr img,std::vector<TagInfo> &tagInfo);
    cv::Mat DrawCoor();

private:
    Eigen::Matrix3d intrinsics_;
    apriltag_detection_info_t info_;
    std::vector<apriltag_pose_t> poses_;
    apriltag_detector_t *td_ ;
    apriltag_family_t *tf_;
    std::vector<TagInfo> tagInfos_;
    zarray_t *detections_;
    cv::Mat raw_img_;
    std::string tag_family_;
    bool is_destory_;
};
class ExtrinsicCalculator
{
public:
    ExtrinsicCalculator();

    void AddLidarPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_points);
    bool AddCameraPointCloud(pcl::PointCloud<pcl::PointXYZ>,bool is_reset=false);
    bool AddCameraPointCloud(std::vector<TagInfo>);
    bool AddCameraPointCloud(cv::Mat color_img,apriltag_detection_info_t info,std::string family="tagStandard52h13");
    bool CalExtrinsic(bool verbose=false);
    bool CalExtrinsic2(std::vector<TagInfo>,bool verbose=false);
    void LidarPorj2Camera(cv::Mat &rawImg,pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud,cv::Mat intrinsic);
    Eigen::Vector3f getTranslation(void);
    Eigen::Matrix3f getRotation(void);
private:
    void findLowestPoint(pcl::PointCloud<pcl::PointXYZ>::Ptr CameraLowestPointExcludeGround,Eigen::Vector3f ground_normal);
    std::vector<Eigen::Vector4f> camera_normal_;
    std::vector<pcl::PointCloud<pcl::PointXYZ> > camera_clouds_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_cloud_;
    std::vector<Eigen::Vector4f> lidar_normal_;
    Eigen::Matrix3f R_;
    Eigen::Vector3f T_;
    bool CalResult_;
    std::vector<int> TagIdDistribution_;
	Eigen::Matrix3f cameraCoor2WorldCoor_;
    Eigen::Vector3f mean_normal[3];
    CornerTag corn_tag_;

};

// int FindTarget(Mat& transfvec12, Mat& img, bool print, bool show_img);

