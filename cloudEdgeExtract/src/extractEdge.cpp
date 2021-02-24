#include "extractEdge.h"
namespace edge{

/**
 * @param radius: mean shift 聚类的半径;
 * @param lambda: mean shift中心点移动的倍率,如果中心点移动距离>radius/lambda,就认为是边界点
*/
ExtractEdge::ExtractEdge(float radius,float lambda):
radius_(radius),
lambda_(lambda)
{
    if(lambda_<1)
    {
        std::cerr<<"lambda should greater than 1"<<std::endl;
        lambda_=1;
    }
}

ExtractEdge::~ExtractEdge()
{

}
bool ExtractEdge::extractXYZ(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in,pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out)
{
    //step 1: build kdtree to find neighbours
    std::vector<float> centroid_pos;
    std::vector<std::vector<int> > cloud_indices;
    pcl::KdTreeFLANN<pcl::PointXYZI> pt_tree;
    pt_tree.setInputCloud(cloud_in);
    for(int i=0;i<cloud_in->size();i++)
    {
        std::vector<int> indices;
        std::vector<float> distance;
        int num=pt_tree.radiusSearch(cloud_in->points[i],radius_,indices,distance);
        if(num<5)
            continue;
    //step 2:  mean shift to find candidate edge points
        Eigen::Vector3d centroid;
        centroid<<0,0,0;
        for(auto index: indices)
        {
            centroid[0] = centroid[0]+cloud_in->points[index].x;
            centroid[1] = centroid[1]+cloud_in->points[index].y;
            centroid[2] = centroid[2]+cloud_in->points[index].z;
        }
        centroid<<centroid[0]/indices.size(),centroid[1]/indices.size(),centroid[2]/indices.size();
        Eigen::Vector3d current_pt(cloud_in->points[i].x,cloud_in->points[i].y,cloud_in->points[i].z);
        Eigen::Vector3d c2p_dis = centroid-current_pt;
        int dis_min = std::min_element(distance.begin(),distance.end())-distance.begin();
        Eigen::Vector3d nearest_pt(cloud_in->points[indices[1]].x,cloud_in->points[indices[1]].y,cloud_in->points[indices[1]].z);
        Eigen::Vector3d nearst_dis = current_pt-nearest_pt;

        if(c2p_dis.norm()>(radius_/lambda_))
        {
            edgePoints_.push_back(cloud_in->points[i]);
        }
    }

    pcl::copyPointCloud(edgePoints_,*cloud_out);
    return true;
}
bool ExtractEdge::extractXYZ(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in,pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out)
{
    //step 1: build kdtree to find neighbours
    std::vector<float> centroid_pos;
    std::vector<std::vector<int> > cloud_indices;
    // findCluster(cloud_in,3,centroid_pos,cloud_indices);
    // int max_index= std::max_element(centroid_pos.begin(),centroid_pos.end())-centroid_pos.begin();
    // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_partial(new pcl::PointCloud<pcl::PointXYZI>);

    // pcl::copyPointCloud(*cloud_in,cloud_indices[max_index],*cloud_partial);

    pcl::KdTreeFLANN<pcl::PointXYZ> pt_tree;
    pt_tree.setInputCloud(cloud_in);

    for(int i=0;i<cloud_in->size();i++)
    {
        std::vector<int> indices;
        std::vector<float> distance;
        int num=pt_tree.radiusSearch(cloud_in->points[i],radius_,indices,distance);
        if(num<5)
            continue;
    //step 2:  mean shift to find candidate edge points
        Eigen::Vector3d centroid;
        centroid<<0,0,0;
        for(auto index: indices)
        {
            centroid[0] = centroid[0]+cloud_in->points[index].x;
            centroid[1] = centroid[1]+cloud_in->points[index].y;
            centroid[2] = centroid[2]+cloud_in->points[index].z;
        }
        centroid<<centroid[0]/indices.size(),centroid[1]/indices.size(),centroid[2]/indices.size();
        Eigen::Vector3d current_pt(cloud_in->points[i].x,cloud_in->points[i].y,cloud_in->points[i].z);
        Eigen::Vector3d c2p_dis = centroid-current_pt;
        int dis_min = std::min_element(distance.begin(),distance.end())-distance.begin();
        Eigen::Vector3d nearest_pt(cloud_in->points[indices[1]].x,cloud_in->points[indices[1]].y,cloud_in->points[indices[1]].z);
        Eigen::Vector3d nearst_dis = current_pt-nearest_pt;

        if(c2p_dis.norm()>(radius_/lambda_))
        {
            pcl::PointXYZI pt;
            pt.x=cloud_in->points[i].x;
            pt.y=cloud_in->points[i].y;
            pt.z=cloud_in->points[i].z;
            pt.intensity=0;
            edgePoints_.push_back(pt);
        }
    }
    pcl::copyPointCloud(edgePoints_,*cloud_out);
    return true;
}

bool ExtractEdge::extractI(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in,bool verbose)
{
    //step 1: build kdtree to find neighbours
   
    pcl::KdTreeFLANN<pcl::PointXYZI> pt_tree;
    pt_tree.setInputCloud(cloud_in);

    for(int i;i<cloud_in->size();i++)
    {
        std::vector<int> indices;
        std::vector<float> distance;
        int num=pt_tree.radiusSearch(cloud_in->points[i],radius_,indices,distance);
        if(num<5)
            continue;
    //step 2:  mean shift to find candidate edge points
        float centroid=0;
        for(auto index: indices)
        {
            centroid = centroid+cloud_in->points[index].intensity;
        }
        centroid=centroid/indices.size();
        float c2p_dis = centroid-cloud_in->points[i].intensity;
        if(fabs(c2p_dis)>radius_/lambda_)
        {
            // cloud_in->points[i].intensity=200;   
            edgePoints_.push_back(cloud_in->points[i]);
        }
    }
    pcl::io::savePCDFile("/home/ramlab/Documents/catkin_ws/src/merge_cloud/whole_cloud_partial_edge.pcd",edgePoints_);
    if(verbose)
    {
        pcl::visualization::CloudViewer viewer("edge cloud");
        viewer.showCloud(edgePoints_.makeShared());
        pause();
    }
    return true;
}
bool ExtractEdge::extract(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in,std::vector<pcl::PointCloud<pcl::PointXYZI> > &cloud_out
                            ,std::vector<Eigen::Matrix<double,6,1> > &line_equation,Eigen::Vector4d &plane_normal)
{
    //提取出这个tag的点云
    std::vector<float> centroid;
    std::vector<std::vector<int> > indices; 
    findCluster(cloud_in,3,centroid,indices);
    int max_index= std::max_element(centroid.begin(),centroid.end())-centroid.begin();
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_partial(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::copyPointCloud(*cloud_in,indices[max_index],*cloud_partial);
    //计算提取出来的平面方程
    Eigen::Vector4d equation;
    Point3DRANSAC(cloud_partial,equation);
    if(equation[3]<0)
        plane_normal = -equation;
    
    //将这上面的点全部投影到这个平面上
    pcl::PointCloud<pcl::PointXYZI> plane_points;
    for(auto pt:*cloud_partial)
    {
        Eigen::Vector4d current_pt(pt.x,pt.y,pt.z,1);
        float t = (current_pt.dot(equation))/(equation.block<3,1>(0,0).norm()*equation.block<3,1>(0,0).norm());
        pt.x = pt.x-equation[0]*t;
        pt.y = pt.y-equation[1]*t;
        pt.z = pt.z-equation[2]*t;
        plane_points.push_back(pt);
    }
    
    //将压缩到一个平面的点云提取边界点
    pcl::PointCloud<pcl::PointXYZI>::Ptr edge_points(new pcl::PointCloud<pcl::PointXYZI>);
    extractXYZ(plane_points.makeShared(),edge_points);

    //提取的边界点还是会有很多的噪声,在进行一次ransac去除噪声点
    //边界ransac
    pcl::PointCloud<pcl::PointXYZI> lines_cloud;
    std::vector<pcl::ModelCoefficients> coefficients_vec;
    pcl::ExtractIndices<pcl::PointXYZI> extractIndices;
    float raw_edge_size = edge_points->size();
    while((float)edge_points->size()/raw_edge_size>0.1) //提取直线,直到点较少到原来的0.1倍
    {
        pcl::PointCloud<pcl::PointXYZI> cloudF;
        bool continue_flag=false;
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        Point3DLineRANSAC(edge_points,coefficients,inliers);
        extractIndices.setIndices(inliers);
        extractIndices.setInputCloud(edge_points);
        //如果是平行线就只选取之前的直线内点
        for(int k=0;k<coefficients_vec.size();k++) 
        {
            Eigen::Vector3d dir1(coefficients_vec[k].values[3],coefficients_vec[k].values[4],coefficients_vec[k].values[5]);
            Eigen::Vector3d dir2(coefficients->values[3],coefficients->values[4],coefficients->values[5]);
            float cos_angle = dir1.dot(dir2);
            if(fabs(cos_angle)>0.712)
            {
                continue_flag=true;
                break;
            } 
        }
        if(!continue_flag)
        {
            extractIndices.setNegative(false);
            extractIndices.filter(cloudF);
            coefficients_vec.push_back(*coefficients);
            lines_cloud+=cloudF;
            cloud_out.push_back(cloudF);
            Eigen::Matrix<double,6,1> coeff;
            coeff<<coefficients->values[0],coefficients->values[1],coefficients->values[2],
                    coefficients->values[3],coefficients->values[4],coefficients->values[5];
            line_equation.push_back(coeff);
        }
        //去除已经提取的内点
        extractIndices.setNegative(true);
        extractIndices.filter(*edge_points);
    }

    //显示最后的处理结果
    // boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    // viewer->setBackgroundColor(255,255,255);
    // viewer->addCoordinateSystem(2);
    // viewer->addPointCloud<pcl::PointXYZI>(lines_cloud.makeShared());
    // viewer->setBackgroundColor(0,0,0);
    // while (!viewer->wasStopped())
    // {
    //     viewer->spinOnce (100);
    //     boost::this_thread::sleep (boost::posix_time::microseconds (1000));
    // }
    return true;
}

bool ExtractEdge::findCluster(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in,int k,std::vector<float> &centroid_pos,std::vector<std::vector<int> > &cloud_indices)
{
    //step 1 find intensity value using k-means
    int max_iteration=10;//最大迭代次数
    float stop_rate=0.1; //当迭代一次中心点移动的幅度小于上一次的0.1时,认为收敛并停止循环.


    std::vector<float> centroid(k);
    for(int i=0;i<k;i++)
    {
        centroid[i]=i*255/(k-1);
    }
    std::vector<std::vector<int> > indices(k);
    //开始遍历点云,将intensity作为分类标准,将其放在
    float last_drop=1;
    std::vector<float> his_distance_sum(k);

    for(int iteration=0;iteration<max_iteration;iteration++)
    {
        std::vector<float> distance_sum(k);
        std::vector<float> intensity_sum(k);
        for(int j=0;j<k;j++)
        {
            indices[j].clear();
        }

        for(int i=0;i<cloud_in->size();i++)//迭代,根据中心点查找最近的临近点集合,并计算下一个中心点
        {
            
            std::vector<float> distance(k);
            for(int j=0;j<k;j++)
            {
                distance[j]=(fabs(cloud_in->points[i].intensity-centroid[j]));
            }

            int min_index=std::min_element(distance.begin(),distance.end())-distance.begin();

            if(min_index>=k)
            {
                std::cerr<<"Fail to cluster intensity!"<<std::endl;
                return false;
            }
            indices[min_index].push_back(i);
            distance_sum[min_index]+=distance[min_index];
            intensity_sum[min_index]+=cloud_in->points[i].intensity;
            distance.clear();
        }
        float drop = std::accumulate(distance_sum.begin(),distance_sum.end(),0)- 
                        std::accumulate(his_distance_sum.begin(),his_distance_sum.end(),0);

        if(iteration!=0&&fabs(drop/last_drop)<stop_rate) //满足收敛条件,就停止循环
        {
            centroid_pos= centroid;
            cloud_indices = indices;
            return true;
        }else{
            //计算下一个中心点
            for(int i=0;i<k;i++)
            {
                centroid[i] = intensity_sum[i]/indices[i].size();
            }
        }
        his_distance_sum = distance_sum;
        last_drop=drop;
    }
    centroid_pos= centroid;
    cloud_indices = indices;
    return true;
}


bool ExtractEdge::Point3DRANSAC(pcl::PointCloud<PointT>::Ptr in_cloud,Eigen::Vector4d &equation)
{
	std::vector<int> *indices;
    //inliers表示误差能容忍的点 记录的是点云的序号
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    // 创建一个分割器
    pcl::SACSegmentation<PointT> seg;
    // Optional，这个设置可以选定结果平面展示的点是分割掉的点还是分割剩下的点。
    seg.setOptimizeCoefficients(true);
    // Mandatory-设置目标几何形状
    seg.setModelType(pcl::SACMODEL_PLANE);
    //分割方法：随机采样法
    seg.setMethodType(pcl::SAC_RANSAC);
    //设置误差容忍范围，也就是我说过的阈值
    seg.setDistanceThreshold(0.01);
    //输入点云
    try{
        seg.setInputCloud(in_cloud);
        //分割点云
        seg.segment(*inliers, *coefficients);
        equation<<coefficients->values[0],coefficients->values[1],coefficients->values[2],coefficients->values[3];
		return true;
    }catch(...){
		std::cout<<"Point3DRansac: error to calculate coefficient!"<<std::endl;
		return false;
    }
 
}
bool ExtractEdge::Point3DLineRANSAC(pcl::PointCloud<PointT>::Ptr cloud_in, pcl::ModelCoefficients::Ptr coefficients,pcl::PointIndices::Ptr inliers)
{
    // std::vector<int> *indices;
    //inliers表示误差能容忍的点 记录的是点云的序号
    // pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

    // 创建一个分割器
    pcl::SACSegmentation<PointT> seg;
    // Optional，这个设置可以选定结果平面展示的点是分割掉的点还是分割剩下的点。
    seg.setOptimizeCoefficients(true);
    // Mandatory-设置目标几何形状
    seg.setModelType(pcl::SACMODEL_LINE);
    //分割方法：随机采样法
    seg.setMethodType(pcl::SAC_RANSAC);
    //设置误差容忍范围，也就是我说过的阈值
    seg.setDistanceThreshold(0.005);
    //输入点云
    try{
        seg.setInputCloud(cloud_in);
        //分割点云
        seg.segment(*inliers, *coefficients);
        // indices=seg.getIndices();
		return true;
    }catch(...){
		std::cout<<"Point3DLineRANSAC: error to calculate coefficient!"<<std::endl;
		return false;
    }
}
//根据提取出来的点云边界,确定哪个属于x方向,y方向,默认是只存在两个边界的,同时默认两个边界是垂直的,平面法向量作为Z轴,且指向原点
/**
 * @brief 输入边界点, cross_point输出边界点两个边界的大概交点位置,重新调整line_quation直线方程中的直线方程的方向,使得以交点为原点,指向有点的一边
*/
bool ExtractEdge::findAxis(std::vector<pcl::PointCloud<pcl::PointXYZI> > &cloud_in
            ,std::vector<Eigen::Matrix<double,6,1> > &line_quation,Eigen::Vector4d &plane_normals,Eigen::Vector3d &cross_point)
{
    if(cloud_in.size()!=2||(line_quation.size()!=2))
    {
        std::cout<<"Error: plane edge don't equal to 2"<<std::endl;
        return false;
    }
    Eigen::Vector3d dir1(line_quation[0][3],line_quation[0][4],line_quation[0][5]);
    Eigen::Vector3d dir2(line_quation[1][3],line_quation[1][4],line_quation[1][5]);
    dir1.normalize();
    dir2.normalize();
    plane_normals=plane_normals[3]>0?-plane_normals:plane_normals;
    Eigen::Vector4d cross_plane_normal;
    cross_plane_normal.block<3,1>(0,0)= plane_normals.block<3,1>(0,0).cross(dir1);
    Eigen::Vector4d current_pt(line_quation[0][0],line_quation[0][1],line_quation[0][2],1);
    cross_plane_normal[3] = -cross_plane_normal.block<3,1>(0,0).dot(current_pt.block<3,1>(0,0));
    current_pt<<line_quation[1][0],line_quation[1][1],line_quation[1][2],1;
    float t = (current_pt.dot(cross_plane_normal))/(cross_plane_normal.block<3,1>(0,0).norm()*cross_plane_normal.block<3,1>(0,0).norm());
    current_pt[0] = current_pt[0]-cross_plane_normal[0]*t;
    current_pt[1] = current_pt[1]-cross_plane_normal[1]*t;
    current_pt[2] = current_pt[2]-cross_plane_normal[2]*t;
    cross_point = current_pt.block<3,1>(0,0);
    float max_distance[2];
    int max_index[2];
    for(int i=0;i<cloud_in.size();i++)
    {
        std::vector<float> distances;
        for(int j=0;j<cloud_in[i].size();j++)
        {
            Eigen::Vector3d pt(cloud_in[i].points[j].x,cloud_in[i].points[j].y,cloud_in[i].points[j].z);
            pt = pt-current_pt.block<3,1>(0,0);
            distances.push_back(fabs(pt.norm()));
        }
        max_index[i] = std::max_element(distances.begin(),distances.end()) - distances.begin();
        max_distance[i]=distances[max_index[i]];
        
        std::cout<<"max dis: "<<max_distance[i]<<std::endl;
    }
    

    Eigen::Vector3d line_dir(cloud_in[0].points[max_index[0]].x,cloud_in[0].points[max_index[0]].y,cloud_in[0].points[max_index[0]].z);
    line_dir = line_dir - current_pt.block<3,1>(0,0);
    dir1 = dir1*max_distance[0];
    dir2 = dir2*max_distance[1];
    
    if(line_dir.dot(dir1)<0)
    {
        dir1=-dir1;
        line_quation[0][3]=dir1[0];
        line_quation[0][4]=dir1[1];
        line_quation[0][5]=dir1[2];
    }
    line_dir<<cloud_in[1].points[max_index[1]].x,cloud_in[1].points[max_index[1]].y,cloud_in[1].points[max_index[1]].z;
    line_dir = line_dir - current_pt.block<3,1>(0,0);
    if(line_dir.dot(dir2)<0)
    {
        dir2=-dir2;
        line_quation[1][3]=dir2[0];
        line_quation[1][4]=dir2[1];
        line_quation[1][5]=dir2[2];
    }
    return true;
}

}