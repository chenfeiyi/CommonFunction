#include <Eigen/Dense>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <cmath>
#include "cmdline.h"
#include "Kabsch.hpp"
#include "extractEdge.h"
#include <fstream>
#include <ceres/ceres.h>
using namespace std;
/**
 * this is project is goning to extract cloud edge and corner
 * 
*/
class LineCostFunctor{
    public:
    LineCostFunctor(const Eigen::Vector3d &point,const std::vector<Eigen::Vector3d> &verteies):
    point_(point),
    verteies_(verteies){}
    template<class T>
    bool operator()(const T* angle,const T*  Trans,T* residual) const{
        Eigen::AngleAxis<T> Axis_X(angle[0],Eigen::Matrix<T,3,1>::UnitX());
        Eigen::AngleAxis<T> Axis_Y(angle[1],Eigen::Matrix<T,3,1>::UnitY());
        Eigen::AngleAxis<T> Axis_Z(angle[2],Eigen::Matrix<T,3,1>::UnitZ());
        Eigen::Matrix<T,3,3> R;
        R = Axis_Z*Axis_Y*Axis_X; 
        Eigen::Matrix<T,3,1> Translation(Trans);//(Trans);
        Eigen::Matrix<T,3,1> pointT(T(point_(0)),T(point_(1)),T(point_(2)));//(Trans);
        Eigen::Matrix<T,3,1> Porject_vec= R*pointT+Translation;
        Eigen::Matrix<T,3,1> vertex[2];
        for(int i=0;i<verteies_.size();i++)
        {
            vertex[i]<<T(verteies_[i](0)),T(verteies_[i](1)),T(verteies_[i](2));
        }
        Eigen::Matrix<T,3,1> length = (vertex[0]-vertex[1]);
        Eigen::Matrix<T,3,1> distance1 = (Porject_vec-vertex[0]);
        Eigen::Matrix<T,3,1> distance2 = (Porject_vec-vertex[1]);
        if(distance1.dot(length)>T(0))
        {
            residual[0] = distance1.dot(length)-distance2.dot(length);

        }else{
            residual[0] = -distance1.dot(length)+distance2.dot(length);
        }
        return true;
    }
    const Eigen::Vector3d point_;
    const std::vector<Eigen::Vector3d> verteies_;
};
int main(int argc, char *argv[])
{
    cmdline::parser cmdParser;
    cmdParser.add<string>("path",'p',"pcd file path",true,"");
    cmdParser.parse_check(argc,argv);

    string file_path = cmdParser.get<string>("path");
    //加载文件,并检查文件是否存在
    fstream f(file_path);
    if(!f.is_open())
    {
        cout<<"file do not exist!"<<endl;
        return 1;
    }else
    {
        cout<<"Loading......."<<endl;
    }
    f.close();

    //******已知参数*********
    float  width=0.1;//unit m
    float length=1.0;//unit m

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZI>);
    std::vector<pcl::PointCloud<pcl::PointXYZI> > cloud_out;
    std::vector<Eigen::Matrix<double,6,1> > line_equation;
    Eigen::Vector4d plane_equation;
    pcl::io::loadPCDFile(file_path,*cloud_in);
    edge::ExtractEdge extacter(0.02,3);
    extacter.extract(cloud_in,cloud_out,line_equation,plane_equation); //企图点云中的两个垂直的边界
    cout<<"----------extract information---------\n";
    cout<<"plane equation: "<<plane_equation[0]<<","<<plane_equation[1]<<","<<plane_equation[2]<<","<<plane_equation[3]<<endl;
    for(int i=0;i<cloud_out.size();i++)
    {
        cout<<"cloud "<<i<<" size: "<<cloud_out[i].size()<<endl;
        cout<<"line equation: "<<line_equation[i][0]<<","<<line_equation[i][1]<<","<<line_equation[i][2]<<","
                            <<line_equation[i][3]<<","<<line_equation[i][4]<<","<<line_equation[i][5]<<endl;
    }
    std::cout<<"----------------------"<<std::endl;
    //开始根据优化算法计算损失函数,求得最优解
    ceres::Problem problem;
    Eigen::Vector3d Translation(0,0,0);
    Eigen::Matrix3d Rotation;
    std::cout<<"start construct problem, and initial parameters is:\n";
    //添加损失函数,需要根据不同边长的点对应的边,这个需要事先对应好,这部分算法比较琐碎
    //根据长短边确定板子的朝向,不然损失函数计算出来的就有四个局部最优解.
    Eigen::Vector3d cross_point;
    extacter.findAxis(cloud_out,line_equation,plane_equation,cross_point);
    std::vector<Eigen::Vector3d> coeff[2];
    Eigen::Vector3d cloud_axis_X,cloud_axis_Y,cloud_axis_Z;
    if(line_equation[0].block<3,1>(3,0).norm()>line_equation[1].block<3,1>(3,0).norm())
    {
        coeff[0].push_back(Eigen::Vector3d(1,0,0));
        coeff[0].push_back(Eigen::Vector3d(0,0,0));
        coeff[1].push_back(Eigen::Vector3d(0,0.1,0));
        coeff[1].push_back(Eigen::Vector3d(0,0,0));
        
    }else{
        coeff[0].push_back(Eigen::Vector3d(0,0.1,0));
        coeff[0].push_back(Eigen::Vector3d(0,0,0));
        coeff[1].push_back(Eigen::Vector3d(1,0,0));
        coeff[1].push_back(Eigen::Vector3d(0,0,0));
    }
    if(plane_equation.block<3,1>(0,0).dot(line_equation[0].block<3,1>(3,0).cross(line_equation[1].block<3,1>(3,0)))>0)
    {

        cloud_axis_X<<line_equation[0][3],line_equation[0][4],line_equation[0][5];
        cloud_axis_Y<<line_equation[1][3],line_equation[1][4],line_equation[1][5];
        cloud_axis_Z=plane_equation.block<3,1>(0,0);
    }else
    {
        cloud_axis_Y<<line_equation[0][3],line_equation[0][4],line_equation[0][5];
        cloud_axis_X<<line_equation[1][3],line_equation[1][4],line_equation[1][5];
        cloud_axis_Z=plane_equation.block<3,1>(0,0);
    }
    //初始化R,T
    cloud_axis_X.normalize();
    cloud_axis_Y.normalize();
    cloud_axis_Z.normalize();
    Eigen::Matrix3d P; //点云的坐标轴
	P.col(0) <<1,0,0;
	P.col(1)<< 0,1,0;
	P.col(2) <<0,0,1;
	
	Eigen::Matrix3d Q;//点云的坐标轴
	
    Q.col(0) = cloud_axis_X.cast<double>() + cross_point.cast<double>();
	Q.col(1) = cloud_axis_Y.cast<double>() + cross_point.cast<double>();
	Q.col(2) = cloud_axis_Z.cast<double>() + cross_point.cast<double>();
    Eigen::Affine3d affine = Kabsch::Find3DAffineTransform(Q,P, false);
	Rotation = affine.linear();
	Translation = affine.translation();
    Eigen::Vector3d angle(0,0,0);//roll,pitch,yaw的顺序
    angle = Rotation.eulerAngles(2,1,0);
    std::cout<<"Rotation\n";
    cout<<Rotation<<std::endl;
    cout<<"angles: \n";
    cout<<angle<<endl;
    std::cout<<"Translation\n";
    cout<<Translation<<std::endl;
    for(int i=0;i<2;i++)
    {
        
        for(auto pt: cloud_out[i])
        {
            Eigen::Vector3d points(pt.x,pt.y,pt.z);
            ceres::CostFunction* costFunction =  new ceres::AutoDiffCostFunction<LineCostFunctor,1,3,3>(new LineCostFunctor(points,coeff[i]));
            problem.AddResidualBlock(costFunction,nullptr,angle.data(),Translation.data());
        }

    }
    ceres::Solver::Options options;
    options.linear_solver_type=ceres::DENSE_QR;
    options.max_num_iterations=100;
    options.minimizer_progress_to_stdout=true;
    ceres::Solver::Summary summary;
    Solve(options,&problem,&summary);
    std::cout<<summary.BriefReport()<<std::endl;
    Eigen::AngleAxisd Axis_X(angle[0],Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd Axis_Y(angle[1],Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd Axis_Z(angle[2],Eigen::Vector3d::UnitZ());
    Rotation = Axis_Z*Axis_Y*Axis_X;
    std::cout<<"Finished: \n";
    std::cout<<"Rotation\n";
    cout<<Rotation<<std::endl;
    cout<<"angle: \n";
    cout<<angle<<endl;
    std::cout<<"Translation\n";
    cout<<Translation<<std::endl;
    pcl::PointCloud<pcl::PointXYZI> final_out;
    pcl::PointCloud<pcl::PointXYZ> square;
    for(int i=0;i<2;i++)
    {

        for(auto pt: cloud_out[i])
        {
            Eigen::Vector3d ptvec(pt.x,pt.y,pt.z);
            Eigen::Vector3d out_pt=Rotation*ptvec+Translation;
            pcl::PointXYZI pttt;
            pttt.x=out_pt(0),pttt.y=out_pt(1),pttt.z=out_pt(2),pttt.intensity=pt.intensity;
            final_out.push_back(pttt);
        }
    }
    square.push_back(pcl::PointXYZ(0,0,0));
    square.push_back(pcl::PointXYZ(1,0,0));
    square.push_back(pcl::PointXYZ(1,0.1,0));
    square.push_back(pcl::PointXYZ(0,0.1,0));
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer2"));
    viewer->setBackgroundColor(255,255,255);
    viewer->addCoordinateSystem(0.1);
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> fildColor(final_out.makeShared(), "intensity");
    viewer->addPointCloud<pcl::PointXYZI>(final_out.makeShared(),fildColor,"final");
    viewer->addPolygon<pcl::PointXYZ>(square.makeShared(),255,0,0);
    viewer->setBackgroundColor(0,0,0);
    while (!viewer->wasStopped())
    {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (1000));
    }
    return 0;
}
