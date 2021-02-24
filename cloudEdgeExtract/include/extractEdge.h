#pragma once
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/flann.h>
#include <pcl/point_cloud.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <unistd.h>
#include <vector>
#include <algorithm>

#include <pcl/visualization/cloud_viewer.h>
#include <iostream>
namespace edge{
class ExtractEdge
{
private:
    /* data */
    float radius_;
    float lambda_;
    float lambdaI_;
    pcl::PointCloud<pcl::PointXYZI> edgePoints_;
public:
    typedef  pcl::PointXYZI PointT;
    ExtractEdge(float radius,float lambda);
    ~ExtractEdge();
    bool extractI(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in,bool verbose=true);
    bool findCluster(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in,int k,std::vector<float> &centroid_pos,std::vector<std::vector<int> > &cloud_indices);
    bool Point3DRANSAC(pcl::PointCloud<PointT>::Ptr cloud_in,Eigen::Vector4d &equation);
    bool Point3DLineRANSAC(pcl::PointCloud<PointT>::Ptr cloud_in,pcl::ModelCoefficients::Ptr coefficients,pcl::PointIndices::Ptr inliers);
    bool extractXYZ(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in,pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out);
    bool extractXYZ(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out);
    bool extract(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in,std::vector<pcl::PointCloud<pcl::PointXYZI> > &cloud_out
                    ,std::vector<Eigen::Matrix<double,6,1> > &line_quation,Eigen::Vector4d &plane_normal);
    bool findAxis(std::vector<pcl::PointCloud<pcl::PointXYZI> > &cloud_out,std::vector<Eigen::Matrix<double,6,1> > &line_quation,
                    Eigen::Vector4d &plane_normals,Eigen::Vector3d &cross_point);
    bool setRespectiveExpectedLineEquation(std::vector<Eigen::Vector2d> line_equation,Eigen::Matrix3d intrinsics);//设置像素坐标系下的每个边界点云对应的直线方程,也就是对应的逼近方程
    bool optimize(Eigen::Matrix3d &initial_R_lidar2cam,Eigen::Vector3d &initial_T_lidar2cam);
};

}