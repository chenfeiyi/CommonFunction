#include "ChessPoseEstimation.h"
#define	PI	3.14159265358979323846
int color[21][3] = 
    {
        {255, 0, 0}, {255, 69, 0}, {255, 99, 71}, 
        {255, 140, 0}, {255, 165, 0}, {238, 173, 14},
        {255, 193, 37}, {255, 255, 0}, {255, 236, 139},
        {202, 255, 112}, {0, 255, 0}, {84, 255, 159},
        {127, 255, 212}, {0, 229, 238}, {152, 245, 255},
        {178, 223, 238}, {126, 192, 238}, {28, 134, 238},
        {0, 0, 255}, {72, 118, 255}, {122, 103, 238} 
    };
// TCP/IP Port to use: 50058
void getFileList(std::vector<std::string> &file_list,std::string folder_path)
{
    dirent *ptr;
    DIR *dir;
    file_list.clear();
    dir=opendir(folder_path.c_str());
    std::size_t pos;
    folder_path.find_last_of("/",pos);
    if(pos!=(folder_path.size()-1))
        folder_path.push_back('/');
    // cout<<folder_path<<endl;

    while((ptr=readdir(dir))!=NULL)
    {
        if(strcmp(ptr->d_name,".")==0||strcmp(ptr->d_name,"..")==0)
            continue;
        std::stringstream ss;
        ss<<folder_path<<ptr->d_name;
        file_list.emplace_back(ss.str());
    }
    closedir(dir);
}
bool resetTagCoor(TagInfo &tag1,TagInfo &tag2,TagInfo &tag3)
{
	Eigen::Vector3f normal1,normal2,normal3;
	TagInfo tag1_buf,tag2_buf,tag3_buf;
	float thres=0.9;
	normal1=tag1.normal,normal2=tag2.normal,normal3=tag3.normal;
	bool flag=resetCoor(normal1,normal2,normal3);
	if(flag==false)
	   return false;
	if(normal1.dot(tag1.normal)>thres)
	{
		tag1_buf=tag1;
		if(normal2.dot(tag2.normal)>thres)
		{
			tag2_buf=tag2;
			tag3_buf=tag3;
		}else
		{
			tag2_buf=tag3;
			tag3_buf=tag2;
		}		
	}else if(normal1.dot(tag2.normal)>thres)
	{
		tag1_buf=tag2;
		if(normal2.dot(tag1.normal)>thres)
		{
			tag2_buf=tag1;
			tag3_buf=tag3;
		}else
		{
			tag2_buf=tag3;
			tag3_buf=tag1;
		}		
	}else if(normal1.dot(tag3.normal)>thres)
	{
		tag1_buf=tag3;
		if(normal2.dot(tag1.normal)>thres)
		{
			tag2_buf=tag1;
			tag3_buf=tag2;
		}else
		{
			tag2_buf=tag2;
			tag3_buf=tag1;
		}		
	}else
	{
		return false;
	}
	tag1=tag1_buf,tag2=tag2_buf,tag3=tag3_buf;
	return true;
}
bool resetCoor(Eigen::Vector3f &Cx,Eigen::Vector3f &Cy,Eigen::Vector3f &Cz)
{
	Cx=Cx/Cx.norm();
	Cy=Cy/Cy.norm();
	Cz=Cz/Cz.norm();

	Eigen::Vector3f cx,cy,cz;
	Eigen::Vector3f cross_vec;
	if(Cz(2)>0.90)
	{
		cz=Cz;
		cross_vec=Cx.cross(Cy);
		if(cross_vec.dot(cz)>0.5)
		{
			cx=Cx,cy=Cy;
		}else if(cross_vec.dot(cz)<-0.5)
		{
			cx=Cy,cy=Cx;
		}else{
			return false;
		}

	}else if(Cy(2)>0.9)
	{
		cz=Cy;
		cross_vec=Cx.cross(Cz);
		if(cross_vec.dot(cz)>0.5)
		{
			cx=Cx,cy=Cz;
		}else if(cross_vec.dot(cz)<-0.5)
		{
			cx=Cz,cy=Cx;
		}else{
			return false;
		}
	}else if(Cx(2)>0.9)
	{
		cz=Cx;
		cross_vec=Cy.cross(Cz);
		if(cross_vec.dot(cz)>0.5)
		{
			cx=Cy,cy=Cz;
		}else if(cross_vec.dot(cz)<-0.5)
		{
			cx=Cz,cy=Cy;
		}else{
			return false;
		}
	}else{
		return false;
	}
	Cx=cx,Cy=cy,Cz=cz;
	return true;
}
void ConvertIntoHomogeneous(Mat& matrix)
{
	// Get Homogeneous (4X4) Matrix from Non-Homogeneous (3X3) Matrix:
	hconcat(matrix, (Mat)(Mat_<double>(3,1) << 0.0, 0.0, 0.0), matrix);
	vconcat(matrix, (Mat)(Mat_<double>(1,4) << 0.0, 0.0, 0.0, 1.0), matrix);
}

void ConvertIntoNonHomogeneous(Mat& matrix)
{
	// Get Non-Homogeneous (3X3) Matrix from Homogeneous (4X4) Matrix:
	matrix	= matrix(Rect(0, 0, 3, 3));
}
//
Mat RotationMatrix(unsigned char axis, double angle)
{
	// Get Homogeneous (4X4) Rotation Matrix along specified axis, with specified angle:
	if (axis == 'x')
	{
		return	(Mat_<double>(4,4) <<	1.0,			0.0,			0.0,			0.0,
										0.0,			cos(angle),		-sin(angle),	0.0,
										0.0,			sin(angle),		cos(angle),		0.0,
										0.0,			0.0,			0.0,			1.0);
	}
	else if (axis == 'y')
	{
		return	(Mat_<double>(4,4) <<	cos(angle),		0.0,			sin(angle),		0.0,
										0.0,			1.0,			0.0,			0.0,
										-sin(angle),	0.0,			cos(angle),		0.0,
										0.0,			0.0,			0.0,			1.0);
	}
	else if (axis == 'z')
	{
		return	(Mat_<double>(4,4) <<	cos(angle),		-sin(angle),	0.0,			0.0,
										sin(angle),		cos(angle),		0.0,			0.0,
										0.0,			0.0,			1.0,			0.0,
										0.0,			0.0,			0.0,			1.0);
	}
}
//for chessboard
Mat TranslationMatrix(double tx, double ty, double tz)
{
	// Get Homogeneous (4X4) Translation Matrix, with specified tx (translation along x-axis),
	// ty (translation along y-axis), and tz (translation along z-axis):
	return	(Mat_<double>(4,4) <<	1.0,			0.0,			0.0,			tx,
									0.0,			1.0,			0.0,			ty,
									0.0,			0.0,			1.0,			tz,
									0.0,			0.0,			0.0,			1.0);
}
//for chessboard
void CalcBoardCornerPositions(vector<Point3f>& corner_object_points, Size board_size/*=Size(3, 4)*/, float square_size/*=13.67(in millimeter)*/)
{
    corner_object_points.clear();

    for(int y = 0; y < board_size.height; ++y)
	{
		for(int x = 0; x < board_size.width; ++x)
		{
			corner_object_points.push_back(Point3f(float(x*square_size), float(-y*square_size), 0));
		}
	}
}

void UndistortImage(Mat& in_img, Mat camera_matrix, Mat dist_coeffs)
{
	// Undistort input image:
	Mat temp		= in_img.clone();
	undistort(temp, in_img, camera_matrix, dist_coeffs);
}
void UndistortImage(Mat& in_img, Eigen::Matrix3f camera_matrix, Eigen::Matrix<float,5,1> dist_coeffs)
{
	// Undistort input image:
	cv::Mat intrinsic=(cv::Mat_<float>(3,3)<<camera_matrix(0.0),camera_matrix(0.1),camera_matrix(0.2),
											camera_matrix(1.0),camera_matrix(1.1),camera_matrix(1.2),
											camera_matrix(2.0),camera_matrix(2.1),camera_matrix(2.2));
	Mat temp		= in_img.clone();
	cv::Mat distor(5,1,CV_32FC1);
	distor=(Mat_<float>(5,1)<<dist_coeffs(0),dist_coeffs(1),dist_coeffs(2),dist_coeffs(3),dist_coeffs(4));
	undistort(temp, in_img, intrinsic, distor);
}
//for chessboard
bool GetPose(Mat& undist_in_img, Mat& transfvec12, Mat& rvec, Mat& tvec, 
            vector<Point3f> corner_object_points/*(3D object points)*/, 
            Mat camera_matrix, Mat dist_coeffs, bool print/*=true*/, Size board_size/*=Size(3, 4)*/)
{
	// Let's say we have 3D object point P, which is seen as 2D image point p on the image plane;
	// the point p is related to point P by applying a rotation matrix R and a translation vector t to P, or mathematically:
	// p = [R|t] * P
	// See the theoretical explanation here: http://docs.opencv.org/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
	// Points:
	vector<Point2f>	corner_image_points;			// 2D image points (on image plane)

	bool			found;
	
	// Rotation Matrix:
	Mat				rmat;
	
	// Overall Transformation Matrices:
	Mat				transfmatCam;					// Overall Transformation Matrix (3X4) in reference to Camera Coordinate System
	Mat				transfmatCamHomogeneous;		// Overall Homogeneous Transformation Matrix (4X4) in reference to Camera Coordinate System
	Mat				transfmatSBHomogeneous;			// Overall Homogeneous Transformation Matrix (4X4) in reference to SuperBot's End-Effector Coordinate System
	Mat				transfmatSB;					// Overall Transformation Matrix (3X4) in reference to SuperBot's End-Effector Coordinate System

	// Images:
	Mat				undist_in_img_gray;
	
	found = findChessboardCorners(undist_in_img, board_size, corner_image_points, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);
	if (found)		// If done with success,
	{
		// improve the found corners' coordinate accuracy for chessboard:
		cvtColor(undist_in_img, undist_in_img_gray, CV_BGR2GRAY);
		cornerSubPix(undist_in_img_gray, corner_image_points, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1));

		// Draw the corners:
		// drawChessboardCorners(undist_in_img, board_size, Mat(corner_image_points), found);

		// Compute Rotation Vector (rvec) and Translation Vector (tvec) using solvePnPRansac algorithm:
		solvePnPRansac(corner_object_points, corner_image_points, camera_matrix, dist_coeffs, rvec, tvec);
		// Compute Rotation Matrix (rmat, size 3-by-3 matrix) from Rotation Vector (rvec, size 3-by-1 matrix) using Rodrigues transform
		// (http://docs.opencv.org/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#rodrigues):
		Rodrigues(rvec, rmat);
		// Rotate Target Chessboard Coordinate System 180 degrees along x-axis (Roll-direction)
		// (to align Target Chessboard Coordinate System with Camera Coordinate System):
		ConvertIntoHomogeneous(rmat);
		rmat					= rmat * RotationMatrix('x', PI);
		ConvertIntoNonHomogeneous(rmat);
		// Update the Rotation Vector (rvec) after the change above:
		Rodrigues(rmat, rvec);
		if (print)
		{
			cout<<"rvec = "<<rvec<<endl;
			cout<<"rmat = "<<rmat<<endl;
			cout<<"tvec = "<<tvec<<endl;
		}
		// Combine Rotation Matrix (rmat) and Translation Vector (tvec) into the overall transformation matrix
		// in reference to Camera coordinate system (transfmatCam, size 3-by-4 matrix), that is [rmat|tvec]:
		hconcat(rmat, tvec, transfmatCam);
		// Convert transfmatCam into transfmatSB (overall transformation matrix in reference 
		// to SuperBot's End-Effector coordinate system):
		Mat temp				= (Mat_<double>(1,4) <<	0.0, 0.0, 0.0, 1.0);
		vconcat(transfmatCam, temp, transfmatCamHomogeneous);
		Mat Rx					= RotationMatrix('x', -(PI/2));
		Mat Ry					= RotationMatrix('y', (PI/2));
		// Camera Offset from SuperBot's End-Effector coordinate system's point of origin:
		Mat T1					= TranslationMatrix(40.0, 70.0, 0.0);
		transfmatSBHomogeneous	= T1 * Ry * Rx * transfmatCamHomogeneous;
		transfmatSB				= transfmatSBHomogeneous(Rect(0, 0, 4, 3));
		//cout<<"transfmatSB = "<<transfmatSB<<endl;
		// Final transfmat re-shaping to satisfy requirements of manipulator program:
		transpose(transfmatSB, transfvec12);
		transfvec12				= transfvec12.reshape(12, 1);
	}
	else		// If NOT found,
	{
		// return extremely huge transfvec12 (which means error/failure):
		transfvec12		= (Mat_<double>(12,1) <<	10.0e+20,
													10.0e+20,
													10.0e+20,
													10.0e+20,
													10.0e+20,
													10.0e+20,
													10.0e+20,
													10.0e+20,
													10.0e+20,
													10.0e+20,
													10.0e+20,
													10.0e+20);
	}
	
	if ((found) && (print))
	{
		cout<<"transfvec12 = "<<transfvec12<<endl;
		cout<<endl;
	}
	
	return found;
}
//for chessboard
void DrawObjCoordFrame(Mat& undist_in_img, Mat rvec, Mat tvec, Mat camera_matrix, Mat dist_coeffs)
{
	vector<cv::Point3f>	coord_frame_object_points;		// 3D coordinate frame (origin, x-axis pointer, y-axis pointer, z-axis pointer) points
	vector<cv::Point2f>	coord_frame_image_points;		// 2D image points (on image plane)

	// 3D object point coordinates of the axes' pointer of the target object plane (imprinted with the chessboard pattern)'s coordinate frame:
	Point3f			OBJ_COORD_ORIGIN(0.0f, 0.0f, 0.0f),
					OBJ_COORD_X(200.0f, 0.0f, 0.0f),
					OBJ_COORD_Y(0.0f, 200.0f, 0.0f),
					OBJ_COORD_Z(0.0f, 0.0f, -200.0f);
	
	// Push in 3D descriptor points of target plane (object)'s coordinate frame:
	coord_frame_object_points.push_back(OBJ_COORD_ORIGIN);
	coord_frame_object_points.push_back(OBJ_COORD_X);
	coord_frame_object_points.push_back(OBJ_COORD_Y);
	coord_frame_object_points.push_back(OBJ_COORD_Z);

	//cout<<"rmat = "<<rmat<<endl;
	//cout<<"rvec = "<<rvec<<endl;
	//cout<<"tvec = "<<tvec<<endl;
	
	// Project the 3D descriptor points of target plane's coordinate frame into image plane using computed rvec and tvec:
	projectPoints(coord_frame_object_points, rvec, tvec, camera_matrix, dist_coeffs, coord_frame_image_points);
	// Draw the projected X-axis of the target plane (object)'s coordinate frame on image plane, on the output image:
	line(undist_in_img, Point(coord_frame_image_points[0].x, coord_frame_image_points[0].y), Point(coord_frame_image_points[1].x, coord_frame_image_points[1].y), Scalar(0,0,255), 5, CV_AA);
	putText(undist_in_img, "X", Point(coord_frame_image_points[1].x, coord_frame_image_points[1].y), 1, 1, Scalar(0,0,255));
	// Draw the projected Y-axis of the target plane (object)'s coordinate frame on image plane, on the output image:
	line(undist_in_img, Point(coord_frame_image_points[0].x, coord_frame_image_points[0].y), Point(coord_frame_image_points[2].x, coord_frame_image_points[2].y), Scalar(0,255,0), 5, CV_AA);
	putText(undist_in_img, "Y", Point(coord_frame_image_points[2].x, coord_frame_image_points[2].y), 1, 1, Scalar(0,255,0));
	// Draw the projected Z-axis of the target plane (object)'s coordinate frame on image plane, on the output image:
	line(undist_in_img, Point(coord_frame_image_points[0].x, coord_frame_image_points[0].y), Point(coord_frame_image_points[3].x, coord_frame_image_points[3].y), Scalar(255,0,0), 5, CV_AA);
	putText(undist_in_img, "Z", Point(coord_frame_image_points[3].x, coord_frame_image_points[3].y), 1, 1, Scalar(255,0,0));
}
//for apriltag
void DrawDetections (cv::Mat &image,apriltag_detection_t *det,apriltag_detection_info_t info,apriltag_pose_t pose)
{
    // Check if this ID is present in config/tags.yaml
    // Check if is part of a tag bundle
    int tagID = det->id;
	double err = estimate_tag_pose(&info, &pose);
    // Draw tag outline with edge colors green, blue, blue, red
    // (going counter-clockwise, starting from lower-left corner in
    // tag coords). cv::Scalar(Blue, Green, Red) format for the edge
    // colors!
	Eigen::Vector3d norm_origin,norm_lp,norm_ld,norm_rp,norm_rd;
	Eigen::Matrix3d intrinsic;
	norm_origin<<0.0,0.0,-info.tagsize;
	norm_lp<<-info.tagsize/2.0,-info.tagsize/2.0,-info.tagsize;
	norm_ld<<-info.tagsize/2.0,info.tagsize/2.0,-info.tagsize;
	norm_rp<<info.tagsize/2.0,-info.tagsize/2.0,-info.tagsize;
	norm_rd<<info.tagsize/2.0,info.tagsize/2.0,-info.tagsize;
	intrinsic<<info.fx,0.0,info.cx,0.0,info.fy,info.cy,0.0,0.0,1.0;
	CameraCoor2PixelCoor(norm_origin,intrinsic,pose);
	CameraCoor2PixelCoor(norm_lp,intrinsic,pose);
	CameraCoor2PixelCoor(norm_ld,intrinsic,pose);
	CameraCoor2PixelCoor(norm_rp,intrinsic,pose);
	CameraCoor2PixelCoor(norm_rd,intrinsic,pose);

    line(image, cv::Point((int)det->p[0][0], (int)det->p[0][1]),
         cv::Point((int)det->p[1][0], (int)det->p[1][1]),
         cv::Scalar(0, 0xff, 0),2); // green
    line(image, cv::Point((int)det->p[0][0], (int)det->p[0][1]),
         cv::Point((int)det->p[3][0], (int)det->p[3][1]),
         cv::Scalar(0, 0, 0xff),2); // red
    line(image, cv::Point((int)det->p[1][0], (int)det->p[1][1]),
         cv::Point((int)det->p[2][0], (int)det->p[2][1]),
         cv::Scalar(0xff, 0, 0),2); // blue
    line(image, cv::Point((int)det->p[2][0], (int)det->p[2][1]),
         cv::Point((int)det->p[3][0], (int)det->p[3][1]),
         cv::Scalar(0xff, 0, 0),2); // blue

	line(image, cv::Point((int)det->p[0][0], (int)det->p[0][1]),
         cv::Point((int)norm_ld(0)/norm_ld(2), (int)norm_ld(1)/norm_ld(2)),
         cv::Scalar(0, 0, 0xff)); // red
	line(image, cv::Point((int)det->p[1][0], (int)det->p[1][1]),
         cv::Point((int)norm_rd(0)/norm_rd(2), (int)norm_rd(1)/norm_rd(2)),
         cv::Scalar(0, 0, 0xff)); // red
	line(image, cv::Point((int)det->p[2][0], (int)det->p[2][1]),
         cv::Point((int)norm_rp(0)/norm_rp(2), (int)norm_rp(1)/norm_rp(2)),
         cv::Scalar(0, 0, 0xff)); // red
	line(image, cv::Point((int)det->p[3][0], (int)det->p[3][1]),
         cv::Point((int)norm_lp(0)/norm_lp(2), (int)norm_lp(1)/norm_lp(2)),
         cv::Scalar(0, 0, 0xff)); // red
	
	line(image,cv::Point((int)norm_ld(0)/norm_ld(2), (int)norm_ld(1)/norm_ld(2)),
         cv::Point((int)norm_lp(0)/norm_lp(2), (int)norm_lp(1)/norm_lp(2)),
         cv::Scalar(0, 0, 0xff)); // red 3-0
	line(image,cv::Point((int)norm_rd(0)/norm_rd(2), (int)norm_rd(1)/norm_rd(2)),
         cv::Point((int)norm_ld(0)/norm_ld(2), (int)norm_ld(1)/norm_ld(2)),
         cv::Scalar(0, 0, 0xff)); // red 0-1
	line(image,cv::Point((int)norm_rd(0)/norm_rd(2), (int)norm_rd(1)/norm_rd(2)),
         cv::Point((int)norm_rp(0)/norm_rp(2), (int)norm_rp(1)/norm_rp(2)),
         cv::Scalar(0, 0, 0xff)); // red 1-2
	line(image,cv::Point((int)norm_lp(0)/norm_lp(2), (int)norm_lp(1)/norm_lp(2)),
         cv::Point((int)norm_rp(0)/norm_rp(2), (int)norm_rp(1)/norm_rp(2)),
         cv::Scalar(0, 0, 0xff)); // red 2-3

	line(image,cv::Point((int)norm_rd(0)/norm_rd(2), (int)norm_rd(1)/norm_rd(2)),
         cv::Point((int)norm_lp(0)/norm_lp(2), (int)norm_lp(1)/norm_lp(2)),
         cv::Scalar(0, 0, 0xff)); // red 1-3
	line(image,cv::Point((int)norm_ld(0)/norm_ld(2), (int)norm_ld(1)/norm_ld(2)),
         cv::Point((int)norm_rp(0)/norm_rp(2), (int)norm_rp(1)/norm_rp(2)),
         cv::Scalar(0, 0, 0xff)); // red 0-2


    // Print tag ID in the middle of the tag
    std::stringstream ss;
    ss << det->id<<": "<<pose.t->data[2];
    cv::String text = ss.str();
    int fontface = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
    double fontscale = 2.0;
    int baseline;
    cv::Size textsize = cv::getTextSize(text, fontface,
                                        fontscale, 2, &baseline);
    cv::putText(image, text,
                cv::Point((int)(det->c[0]-textsize.width/2),
                          (int)(det->c[1]+textsize.height/2)),
                fontface, fontscale, cv::Scalar(0xff, 0x99, 0), 2);
}

Eigen::Vector3d CameraCoor2PixelCoor(Eigen::Vector3d &pt,Eigen::Matrix3d intrinsic,apriltag_pose_t pose)
{
	Eigen::Matrix3d rotation;
	Eigen::Vector3d translation;
	rotation<<pose.R->data[0],pose.R->data[1],pose.R->data[2],
			pose.R->data[3],pose.R->data[4],pose.R->data[5],
			pose.R->data[6],pose.R->data[7],pose.R->data[8];
	translation<<pose.t->data[0],pose.t->data[1],pose.t->data[2];
	Eigen::Vector3d normal=rotation*pt+translation;
	pt=intrinsic*normal;
	return pt;
}




//class 
CornerTag::CornerTag(apriltag_detection_info_t info,std::string tag_family):
	info_(info)
{
	is_destory_=false;
	td_ = apriltag_detector_create();
	if(tag_family=="tag36h11")
	{
		tf_ = tag36h11_create();
	}else if(tag_family=="tagStandard41h12")
	{
		tf_=tagStandard41h12_create();
	}else if(tag_family=="tagStandard52h13")
	{
		tf_=tagStandard52h13_create();
	}else
	{
		ROS_ERROR("This tag is not supported!");
	}
    td_->quad_decimate = (float)1.0;
    td_->quad_sigma = (float)0.0;
    td_->nthreads = 2;
    td_->debug = 0;
    td_->refine_edges = 1;
	apriltag_detector_add_family(td_, tf_);
	// apriltag_detector_add_family_bits(td_, tf_,1);
	intrinsics_<<info.fx,0.0,info.cx,0.0,info.fy,info.cy,0.0,0.0,1.0;

}


CornerTag::CornerTag()
{
	is_destory_=true;
}

CornerTag::~CornerTag()
{
	if(is_destory_==true)
		return;
  	apriltag_detector_destroy(td_);
	if(tag_family_=="tag36h11")
	{
    	tag36h11_destroy(tf_);
	}else if(tag_family_=="tagStandard41h12")
	{
    	tagStandard41h12_destroy(tf_);
	}else if(tag_family_=="tagStandard52h13")
	{
    	tagStandard52h13_destroy(tf_);
	}else
	{
		ROS_ERROR("This tag is not supported!");
	}
	
}
void CornerTag::Destory()
{
	is_destory_=true;
	apriltag_detector_destroy(td_);
	if(tag_family_=="tag36h11")
	{
    	tag36h11_destroy(tf_);
	}else if(tag_family_=="tagStandard41h12")
	{
    	tagStandard41h12_destroy(tf_);
	}else if(tag_family_=="tagStandard52h13")
	{
    	tagStandard52h13_destroy(tf_);
	}else
	{
		ROS_ERROR("This tag is not supported!");
	}
}
void CornerTag::init(apriltag_detection_info_t info,std::string tag_family)
{
	is_destory_=false;
	info_=info;
	td_ = apriltag_detector_create();
	tag_family_=tag_family;
	if(tag_family=="tag36h11")
	{
		tf_ = tag36h11_create();
	}else if(tag_family=="tagStandard41h12")
	{
		tf_=tagStandard41h12_create();
	}else if(tag_family=="tagStandard52h13")
	{
		tf_=tagStandard52h13_create();
	}else
	{
		ROS_ERROR("This tag is not supported!");
	}
	ros::Time time2=ros::Time::now();

    td_->quad_decimate = (float)1.0;
    td_->quad_sigma = (float)0.0;
    td_->nthreads = 2;
    td_->debug = 0;
    td_->refine_edges = 1;
	// apriltag_detector_add_family(td_, tf_);
	apriltag_detector_add_family_bits(td_, tf_,1);
	intrinsics_<<info.fx,0.0,info.cx,0.0,info.fy,info.cy,0.0,0.0,1.0;
}
/**
 * @param[in] img: undistorted image, gary or color
 * @param[in] tagInfo: vector of struct TagInfo to forward detected apriltag information
 * @return  void
*/
void CornerTag::Detect(cv::Mat img,std::vector<TagInfo> &tagInfo)
{
	cv::Mat gray_img;
	tagInfos_.clear();
	tagInfo.clear();
	poses_.clear();
	raw_img_=img.clone();
	if(img.channels()!=1)
	{
		cv::cvtColor(img,gray_img,CV_BGR2GRAY);
	}else{
		gray_img=img.clone();
	}
	image_u8_t apriltag_image = { .width = gray_img.cols,
								.height = gray_img.rows,
								.stride = gray_img.cols,
								.buf = gray_img.data };

    detections_ = apriltag_detector_detect(td_, &apriltag_image);
    for (int i = 0; i < zarray_size(detections_); i++) {
        apriltag_detection_t *det;
		apriltag_pose_t pose;
		TagInfo tag_info;
        zarray_get(detections_, i, &det);
        int tagID = det->id;
		// det.
        // Do stuff with detections here.
        info_.det=det;

        double err = estimate_tag_pose(&info_, &pose);


		//calculate tag inforamtion
		tag_info.centralPointDistance=pose.t->data[2];
		tag_info.cPixelCoor[0]=det->c[0],tag_info.cPixelCoor[1]=det->c[1];
		tag_info.tagCornerPixelCoor[0][0]=det->p[0][0],tag_info.tagCornerPixelCoor[0][1]=det->p[0][1];
		tag_info.tagCornerPixelCoor[1][0]=det->p[1][0],tag_info.tagCornerPixelCoor[1][1]=det->p[1][1];
		tag_info.tagCornerPixelCoor[2][0]=det->p[2][0],tag_info.tagCornerPixelCoor[2][1]=det->p[2][1];
		tag_info.tagCornerPixelCoor[3][0]=det->p[3][0],tag_info.tagCornerPixelCoor[3][1]=det->p[3][1];

		tag_info.id=tagID;

		Eigen::Vector3d norm_origin,lp_corner,rp_corner,ld_corner,rd_corner;
		norm_origin<<0.0,0.0,-1.0;
		lp_corner<<-info_.tagsize/2.0,-info_.tagsize/2.0,0;
		ld_corner<<-info_.tagsize/2.0,info_.tagsize/2.0,0;
		rp_corner<<info_.tagsize/2.0,-info_.tagsize/2.0,0;
		rd_corner<<info_.tagsize/2.0,info_.tagsize/2.0,0;
		RefPlane2CameraCoor(norm_origin,pose);
		RefPlane2CameraCoor(lp_corner,pose);
		RefPlane2CameraCoor(ld_corner,pose);
		RefPlane2CameraCoor(rp_corner,pose);
		RefPlane2CameraCoor(rd_corner,pose);
		tag_info.normal[0]=norm_origin(0)-pose.t->data[0],tag_info.normal[1]=norm_origin(1)-pose.t->data[1],tag_info.normal[2]=norm_origin(2)-pose.t->data[2];
		double normal_norm=sqrt(pow(tag_info.normal[0],2)+pow(tag_info.normal[1],2)+pow(tag_info.normal[2],2));
		tag_info.normal[0]/=normal_norm,tag_info.normal[1]/=normal_norm,tag_info.normal[2]/=normal_norm;
		tag_info.tagCornerCameraCoor[0][0]=ld_corner(0),tag_info.tagCornerCameraCoor[0][1]=ld_corner(1),tag_info.tagCornerCameraCoor[0][2]=ld_corner(2);
		tag_info.tagCornerCameraCoor[1][0]=rd_corner(0),tag_info.tagCornerCameraCoor[1][1]=rd_corner(1),tag_info.tagCornerCameraCoor[1][2]=rd_corner(2);
		tag_info.tagCornerCameraCoor[2][0]=rp_corner(0),tag_info.tagCornerCameraCoor[2][1]=rp_corner(1),tag_info.tagCornerCameraCoor[2][2]=rp_corner(2);
		tag_info.tagCornerCameraCoor[3][0]=lp_corner(0),tag_info.tagCornerCameraCoor[3][1]=lp_corner(1),tag_info.tagCornerCameraCoor[3][2]=lp_corner(2);
		tag_info.tagCentralCameraCoor[0]=pose.t->data[0],tag_info.tagCentralCameraCoor[1]=pose.t->data[1],tag_info.tagCentralCameraCoor[2]=pose.t->data[2];
		tagInfos_.emplace_back(tag_info);
		tagInfo.emplace_back(tag_info);
		poses_.emplace_back(pose);
        // std::cout<<"ID: "<<tag_info.id<<" normal: "<<tag_info.normal[0]<<","<<tag_info.normal[1]<<","<<tag_info.normal[2]<<std::endl;
    }
}
/**
 * @brief 相机的参考平面转换到相机实际的坐标系中,参考平面为z=0
 * 
*/
void CornerTag::RefPlane2CameraCoor(Eigen::Vector3d &pt,apriltag_pose_t pose)
{
	Eigen::Matrix3d rotation;
	Eigen::Vector3d translation;
	rotation<<pose.R->data[0],pose.R->data[1],pose.R->data[2],
			pose.R->data[3],pose.R->data[4],pose.R->data[5],
			pose.R->data[6],pose.R->data[7],pose.R->data[8];
	translation<<pose.t->data[0],pose.t->data[1],pose.t->data[2];
	pt=rotation*pt+translation;
}

cv::Mat CornerTag::DrawCoor()
{
  // Check if this ID is present in config/tags.yaml
    // Check if is part of a tag bundle
    for (int i = 0; i < zarray_size(detections_); i++) {
		apriltag_detection_t *det;
        zarray_get(detections_, i, &det);
    	int tagID = det->id;
		Eigen::Vector3d norm_origin,norm_lp,norm_ld,norm_rp,norm_rd;
		norm_origin<<0.0,0.0,-info_.tagsize;
		norm_lp<<-info_.tagsize/2.0,-info_.tagsize/2.0,-info_.tagsize;
		norm_ld<<-info_.tagsize/2.0,info_.tagsize/2.0,-info_.tagsize;
		norm_rp<<info_.tagsize/2.0,-info_.tagsize/2.0,-info_.tagsize;
		norm_rd<<info_.tagsize/2.0,info_.tagsize/2.0,-info_.tagsize;
		CameraCoor2PixelCoor(norm_origin,intrinsics_,poses_[i]);
		CameraCoor2PixelCoor(norm_lp,intrinsics_,poses_[i]);
		CameraCoor2PixelCoor(norm_ld,intrinsics_,poses_[i]);
		CameraCoor2PixelCoor(norm_rp,intrinsics_,poses_[i]);
		CameraCoor2PixelCoor(norm_rd,intrinsics_,poses_[i]);

    // Draw tag outline with edge colors green, blue, blue, red
    // (going counter-clockwise, starting from lower-left corner in
    // tag coords). cv::Scalar(Blue, Green, Red) format for the edge
    // colors!
	
		line(raw_img_, cv::Point((int)det->p[0][0], (int)det->p[0][1]),
			cv::Point((int)det->p[1][0], (int)det->p[1][1]),
			cv::Scalar(0, 0xff, 0),2); // green
		line(raw_img_, cv::Point((int)det->p[0][0], (int)det->p[0][1]),
			cv::Point((int)det->p[3][0], (int)det->p[3][1]),
			cv::Scalar(0, 0, 0xff),2); // red
		line(raw_img_, cv::Point((int)det->p[1][0], (int)det->p[1][1]),
			cv::Point((int)det->p[2][0], (int)det->p[2][1]),
			cv::Scalar(0xff, 0, 0),2); // blue
		line(raw_img_, cv::Point((int)det->p[2][0], (int)det->p[2][1]),
			cv::Point((int)det->p[3][0], (int)det->p[3][1]),
			cv::Scalar(0xff, 0, 0),2); // blue
		
		line(raw_img_, cv::Point((int)det->p[0][0], (int)det->p[0][1]),
			cv::Point((int)norm_ld(0)/norm_ld(2), (int)norm_ld(1)/norm_ld(2)),
			cv::Scalar(0, 0, 0xff)); // red
		line(raw_img_, cv::Point((int)det->p[1][0], (int)det->p[1][1]),
			cv::Point((int)norm_rd(0)/norm_rd(2), (int)norm_rd(1)/norm_rd(2)),
			cv::Scalar(0, 0, 0xff)); // red
		line(raw_img_, cv::Point((int)det->p[2][0], (int)det->p[2][1]),
			cv::Point((int)norm_rp(0)/norm_rp(2), (int)norm_rp(1)/norm_rp(2)),
			cv::Scalar(0, 0, 0xff)); // red
		line(raw_img_, cv::Point((int)det->p[3][0], (int)det->p[3][1]),
			cv::Point((int)norm_lp(0)/norm_lp(2), (int)norm_lp(1)/norm_lp(2)),
			cv::Scalar(0, 0, 0xff)); // red
		line(raw_img_,cv::Point((int)norm_ld(0)/norm_ld(2), (int)norm_ld(1)/norm_ld(2)),
			cv::Point((int)norm_lp(0)/norm_lp(2), (int)norm_lp(1)/norm_lp(2)),
			cv::Scalar(0, 0, 0xff)); // red 3-0
		line(raw_img_,cv::Point((int)norm_rd(0)/norm_rd(2), (int)norm_rd(1)/norm_rd(2)),
			cv::Point((int)norm_ld(0)/norm_ld(2), (int)norm_ld(1)/norm_ld(2)),
			cv::Scalar(0, 0, 0xff)); // red 0-1
		line(raw_img_,cv::Point((int)norm_rd(0)/norm_rd(2), (int)norm_rd(1)/norm_rd(2)),
			cv::Point((int)norm_rp(0)/norm_rp(2), (int)norm_rp(1)/norm_rp(2)),
			cv::Scalar(0, 0, 0xff)); // red 1-2
		line(raw_img_,cv::Point((int)norm_lp(0)/norm_lp(2), (int)norm_lp(1)/norm_lp(2)),
			cv::Point((int)norm_rp(0)/norm_rp(2), (int)norm_rp(1)/norm_rp(2)),
			cv::Scalar(0, 0, 0xff)); // red 2-3

		line(raw_img_,cv::Point((int)norm_rd(0)/norm_rd(2), (int)norm_rd(1)/norm_rd(2)),
			cv::Point((int)norm_lp(0)/norm_lp(2), (int)norm_lp(1)/norm_lp(2)),
			cv::Scalar(0, 0, 0xff)); // red 1-3
		line(raw_img_,cv::Point((int)norm_ld(0)/norm_ld(2), (int)norm_ld(1)/norm_ld(2)),
			cv::Point((int)norm_rp(0)/norm_rp(2), (int)norm_rp(1)/norm_rp(2)),
			cv::Scalar(0, 0, 0xff)); // red 0-2
		// Print tag ID in the middle of the tag
		std::stringstream ss;
		ss << det->id;//<<": "<<poses_[i].t->data[2];
		cv::String text = ss.str();
		int fontface = cv::FONT_HERSHEY_SCRIPT_SIMPLEX;
		double fontscale = 1.0;
		int baseline;
		cv::Size textsize = cv::getTextSize(text, fontface,
											fontscale, 2, &baseline);
		cv::putText(raw_img_, text,
					cv::Point((int)(det->c[0]-textsize.width/2),
							(int)(det->c[1]+textsize.height/2)),
					fontface, fontscale, cv::Scalar(0xff, 0x99, 0), 2);
	}
	return raw_img_;
}

bool CornerTag::GetTagInfo(std::vector<TagInfo> &tagInfo)
{
	if(tagInfos_.size()>0)
	{
		tagInfo=tagInfos_;
		return true;
	}
	return false;

}


/*********************class ExtrinsicCalculator**************************/
ExtrinsicCalculator::ExtrinsicCalculator():
CalResult_(false)
{
    cameraCoor2WorldCoor_<<0,0,1,
                          -1,0,0,
                          0,-1,0;  
	camera_clouds_.resize(3);
	//plane 1
	TagIdDistribution_.clear();
	for(int i=1;i<49;i++)
	{
		//1-16: for plane 1;
		//17-32: for plane 2;
		//33-48: for plane 3;
		TagIdDistribution_.push_back(i);
	}


}



void ExtrinsicCalculator::AddLidarPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_points)
{
	lidar_cloud_=lidar_points;
}
bool ExtrinsicCalculator::AddCameraPointCloud(pcl::PointCloud<pcl::PointXYZ> cameraPoint,bool is_reset)
{
	if(is_reset)
	{
		ROS_INFO("ExtrinsicCalculator: Reset camera points!");
		camera_clouds_.clear();
	}
	if(camera_clouds_.size()>=3)
	{
		ROS_WARN("Can't add more camera point cloud!");
		return false;
	}
	camera_clouds_.push_back(cameraPoint);
	return true;
}

bool ExtrinsicCalculator::AddCameraPointCloud(std::vector<TagInfo> tag_infos)
{
	
	camera_clouds_[0].clear();
	camera_clouds_[1].clear();
	camera_clouds_[2].clear();
	mean_normal[0]<<0,0,0;
	mean_normal[1]<<0,0,0;
	mean_normal[2]<<0,0,0;
	for(int i=0;i<tag_infos.size();i++)
    {
		tag_infos[i].normal=cameraCoor2WorldCoor_*tag_infos[i].normal;
        tag_infos[i].tagCentralCameraCoor=cameraCoor2WorldCoor_*tag_infos[i].tagCentralCameraCoor;
        tag_infos[i].tagCornerCameraCoor[0]=cameraCoor2WorldCoor_*tag_infos[i].tagCornerCameraCoor[0];
        tag_infos[i].tagCornerCameraCoor[1]=cameraCoor2WorldCoor_*tag_infos[i].tagCornerCameraCoor[1];
        tag_infos[i].tagCornerCameraCoor[2]=cameraCoor2WorldCoor_*tag_infos[i].tagCornerCameraCoor[2];
        tag_infos[i].tagCornerCameraCoor[3]=cameraCoor2WorldCoor_*tag_infos[i].tagCornerCameraCoor[3];
        for(int j=0;j<4;j++)
        {
            pcl::PointXYZ pt;
            pt.x=tag_infos[i].tagCornerCameraCoor[j](0);
            pt.y=tag_infos[i].tagCornerCameraCoor[j](1);
            pt.z=tag_infos[i].tagCornerCameraCoor[j](2);
			std::vector<int>::iterator itr=std::find(TagIdDistribution_.begin(),TagIdDistribution_.end(),tag_infos[i].id);
			if(itr==TagIdDistribution_.end())
			{
				ROS_WARN("Can't find tag id from predefined tag array");
				continue;

			}
			auto dis=std::distance(TagIdDistribution_.begin(),itr);
            if(dis<16)
			{
				mean_normal[0]=mean_normal[0]+tag_infos[i].normal;
				camera_clouds_[0].push_back(pt);
			}
            else if(dis>=16&&dis<32)
			{
				mean_normal[1]=mean_normal[1]+tag_infos[i].normal;
                camera_clouds_[1].push_back(pt);

			}
            else if(dis>=32)
			{
                camera_clouds_[2].push_back(pt);
				mean_normal[2]=mean_normal[2]+tag_infos[i].normal;
			}
        }
    }
	mean_normal[0]=mean_normal[0]/camera_clouds_[0].size();
	mean_normal[1]=mean_normal[1]/camera_clouds_[1].size();
	mean_normal[2]=mean_normal[2]/camera_clouds_[2].size();
	// pcl::io::savePCDFile("/home/ramlab/Documents/CornerSeg/dataset/gazebo/1.pcd",camera_clouds_[0]);
	// pcl::io::savePCDFile("/home/ramlab/Documents/CornerSeg/dataset/gazebo/2.pcd",camera_clouds_[1]);
	// pcl::io::savePCDFile("/home/ramlab/Documents/CornerSeg/dataset/gazebo/3.pcd",camera_clouds_[2]);
	// std::cout<<"mean normal 0: "<<mean_normal[0][0]<<","<<mean_normal[0][1]<<","<<mean_normal[0][2]<<std::endl;
	// std::cout<<"mean normal 1: "<<mean_normal[1][0]<<","<<mean_normal[1][1]<<","<<mean_normal[1][2]<<std::endl;
	// std::cout<<"mean normal 2: "<<mean_normal[2][0]<<","<<mean_normal[2][1]<<","<<mean_normal[2][2]<<std::endl;
	if(camera_clouds_[0].size()>=4&&camera_clouds_[1].size()>=4&&camera_clouds_[2].size()>=4)
	{
		return true;
	}else{
		return false;
	}
}
bool ExtrinsicCalculator::AddCameraPointCloud(cv::Mat color_img,apriltag_detection_info_t info,std::string family)
{
    std::vector<TagInfo> tag_infos;
	corn_tag_.init(info,family);
    corn_tag_.Detect(color_img,tag_infos);
	// cv::Mat img = corn_tag_.DrawCoor();
	// cv::imshow("ccc",img);
	// cv::waitKey(0);
	AddCameraPointCloud(tag_infos);
	corn_tag_.Destory();
	return true;
}
/**
 * @brief calculate the extrinsic according to the camera points and lidar points 
 * 		  for each camera plane, each vertical plane should have 12 points at least which means detecting 3 apriltags
 * 		  for horizontal plane, it should have 8 points at least which means detect 2 apriltags
 * 
*/
bool ExtrinsicCalculator::CalExtrinsic(bool verbose)
{
	/*********************calculate normal of camera plane**********************/
	pcl::ModelCoefficients::Ptr plane1_coff(new pcl::ModelCoefficients);
    pcl::ModelCoefficients::Ptr plane2_coff(new pcl::ModelCoefficients);
    pcl::ModelCoefficients::Ptr plane3_coff(new pcl::ModelCoefficients);
	
	if(camera_clouds_[0].size()<=8||camera_clouds_[1].size()<=8)
	{
		ROS_ERROR("No enough points to calculate camera position!");
		CalResult_=false;
		return CalResult_;
	}
	else if(camera_clouds_[2].size()<8)
	{
		ROS_WARN("Camere point is not enough! The groun plane will be assumed lower than lowest point for 1 meter!");
		Point3DRansac(camera_clouds_[0].makeShared(),plane1_coff);
		Point3DRansac(camera_clouds_[1].makeShared(),plane2_coff);

	}else{ 
		Point3DRansac(camera_clouds_[0].makeShared(),plane1_coff);
		Point3DRansac(camera_clouds_[1].makeShared(),plane2_coff);
		Point3DRansac(camera_clouds_[2].makeShared(),plane3_coff);
	}
	//correct the normal direction 
	if(plane1_coff->values[3]<0)
    {
        plane1_coff->values[0]=-plane1_coff->values[0];
        plane1_coff->values[1]=-plane1_coff->values[1];
        plane1_coff->values[2]=-plane1_coff->values[2];
        plane1_coff->values[3]=-plane1_coff->values[3];
    }
    if(plane2_coff->values[3]<0)
    {
        plane2_coff->values[0]=-plane2_coff->values[0];
        plane2_coff->values[1]=-plane2_coff->values[1];
        plane2_coff->values[2]=-plane2_coff->values[2];
        plane2_coff->values[3]=-plane2_coff->values[3];
    }
	//calculate corner origin in camera coordinate
	Eigen::Matrix3d A;
	Eigen::Vector3d D;
	Eigen::Vector3d camera_corner_origin;
    Eigen::Vector3f cam_normal1,cam_normal2,cam_normal3;

	cam_normal1<<plane1_coff->values[0],plane1_coff->values[1],plane1_coff->values[2];
    cam_normal2<<plane2_coff->values[0],plane2_coff->values[1],plane2_coff->values[2];
	if(camera_clouds_[2].size()<8)//no enough ground points because of some camera position and poseture and it can't see tags
	{	
		cam_normal3=cam_normal1.cross(cam_normal2);//calculate the ground normal 
		cam_normal3.normalize();
		cam_normal3=cam_normal3*TAG_HEIGHT;
		if(cam_normal3(2)<0)
			cam_normal3=-cam_normal3;
		pcl::PointCloud<pcl::PointXYZ>::Ptr ground_points(new pcl::PointCloud<pcl::PointXYZ>);
		findLowestPoint(ground_points,cam_normal3);
		if(ground_points->size()<2)
		{
			ROS_WARN("Fail to find any points on the ground");
			CalResult_=false;
			return CalResult_;
		}
		float DD=0.0;
		for(auto pt: *ground_points)
		{
			camera_clouds_[2].push_back(pt);
			DD=DD-(cam_normal3(0)*pt.x+cam_normal3(1)*pt.y+cam_normal3(2)*pt.z);
		}
		DD=DD/ground_points->size();
		plane3_coff->values.clear();
		plane3_coff->values.push_back(cam_normal3(0));
		plane3_coff->values.push_back(cam_normal3(1));
		plane3_coff->values.push_back(cam_normal3(2));
		plane3_coff->values.push_back(DD);
	}
	A<<plane1_coff->values[0],plane1_coff->values[1],plane1_coff->values[2],
	plane2_coff->values[0],plane2_coff->values[1],plane2_coff->values[2],
	plane3_coff->values[0],plane3_coff->values[1],plane3_coff->values[2];

	D<<plane1_coff->values[3],plane2_coff->values[3],plane3_coff->values[3];
	camera_corner_origin=-A.inverse()*D;
	if(plane3_coff->values[3]<0)
	{
		plane3_coff->values[0]=-plane3_coff->values[0];
		plane3_coff->values[1]=-plane3_coff->values[1];
		plane3_coff->values[2]=-plane3_coff->values[2];
		plane3_coff->values[3]=-plane3_coff->values[3];
	}
	cam_normal3<<plane3_coff->values[0],plane3_coff->values[1],plane3_coff->values[2];	

	// normalize normal
    resetCoor(cam_normal1,cam_normal2,cam_normal3);


	/*************************calculate lidar PointCloud normal*******************/
	if(lidar_cloud_->size()<10)
	{
		ROS_WARN("Normal calculator: Please add lidar PointCLoud!");
		CalResult_=false;
		return CalResult_;
	}
	Eigen::Vector3f lidar_origin_corner = Eigen::Vector3f::Zero();
	std::vector<pcl::PointCloud<pcl::PointXYZ> > cloud_planes;
    std::vector<pcl::ModelCoefficients> coefficients;
    std::vector<Eigen::Vector3f> lidar_corner_normal;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_max_plane(new pcl::PointCloud<pcl::PointXYZI>);

    cloud_planes.resize(3);        
    lidar_corner_normal.resize(3);
    coefficients.resize(3);

    PlaneExtraction(lidar_cloud_,cloud_plane,cloud_max_plane,lidar_origin_corner,cloud_planes,coefficients,lidar_corner_normal);
    resetCoor(lidar_corner_normal[0],lidar_corner_normal[1],lidar_corner_normal[2]);

/*************calculate extrinsic parameter***************************/

	Eigen::Matrix3d P; //点云的坐标轴
	P.col(0) = lidar_corner_normal[0].cast<double>() + lidar_origin_corner.cast<double>();
	P.col(1) = lidar_corner_normal[1].cast<double>() + lidar_origin_corner.cast<double>();
	P.col(2) = lidar_corner_normal[2].cast<double>() + lidar_origin_corner.cast<double>();
	
	Eigen::Matrix3d Q;//相机识别到的坐标轴
	
    Q.col(0) = cam_normal1.cast<double>() + camera_corner_origin.cast<double>();
	Q.col(1) = cam_normal2.cast<double>() + camera_corner_origin.cast<double>();
	Q.col(2) = cam_normal3.cast<double>() + camera_corner_origin.cast<double>();

	Eigen::Affine3d affine = Kabsch::Find3DAffineTransform(Q,P, false);
	Eigen::Matrix3d RR = affine.linear();
	Eigen::Vector3d tt = affine.translation();

	R_<<RR(0,0),RR(0,1),RR(0,2),
		RR(1,0),RR(1,1),RR(1,2),
		RR(2,0),RR(2,1),RR(2,2);
	T_<<tt(0),tt(1),tt(2);
	CalResult_=true;
	
	if(verbose==true)
	{	
		pcl::io::savePCDFile("/home/ramlab/Documents/CornerSeg/dataset/corner_aprilTag_point/carla/1.pcd",camera_clouds_[0]);
		pcl::io::savePCDFile("/home/ramlab/Documents/CornerSeg/dataset/corner_aprilTag_point/carla/2.pcd",camera_clouds_[1]);
		pcl::io::savePCDFile("/home/ramlab/Documents/CornerSeg/dataset/corner_aprilTag_point/carla/3.pcd",camera_clouds_[2]);
		std::cout<<"------------------- ExtrinsicCalculator::CalExtrinsic Log begin------------------------"<<std::endl;
		std::cout<<"--------------------plane coefficient---------------------"<<std::endl;
		std::cout<<"plane coff1: "<<plane1_coff->values[0]<<","<<plane1_coff->values[1]<<","<<plane1_coff->values[2]<<","<<plane1_coff->values[3]<<std::endl;
		std::cout<<"plane coff2: "<<plane2_coff->values[0]<<","<<plane2_coff->values[1]<<","<<plane2_coff->values[2]<<","<<plane2_coff->values[3]<<std::endl;
		std::cout<<"plane coff3: "<<plane3_coff->values[0]<<","<<plane3_coff->values[1]<<","<<plane3_coff->values[2]<<","<<plane3_coff->values[3]<<std::endl;
		std::cout<<"--------------------corner origin---------------------"<<std::endl;

		std::cout<<"lidar origin: "<<lidar_origin_corner(0)<<","<<lidar_origin_corner(1)<<","<<lidar_origin_corner(2)<<std::endl;
		std::cout<<"camera origin: "<<camera_corner_origin(0)<<","<<camera_corner_origin(1)<<","<<camera_corner_origin(2)<<std::endl;
		std::cout<<"camera origin - lidar origin: "<<camera_corner_origin(0)+lidar_origin_corner(0)<<","<<camera_corner_origin(1)+lidar_origin_corner(1)<<","<<camera_corner_origin(2)-lidar_origin_corner(2)<<std::endl;
		std::cout<<"--------------------Lidar normal---------------------"<<std::endl;

		std::cout<<"lidar normal 1: "<<lidar_corner_normal[0](0)<<","<<lidar_corner_normal[0](1)<<","<<lidar_corner_normal[0](2)<<std::endl;
		std::cout<<"lidar normal 2: "<<lidar_corner_normal[1](0)<<","<<lidar_corner_normal[1](1)<<","<<lidar_corner_normal[1](2)<<std::endl;
		std::cout<<"lidar normal 3: "<<lidar_corner_normal[2](0)<<","<<lidar_corner_normal[2](1)<<","<<lidar_corner_normal[2](2)<<std::endl;

		std::cout<<"--------------------camera normal---------------------"<<std::endl;
		std::cout<<"camera normal 1: "<<cam_normal1(0)<<","<<cam_normal1(1)<<","<<cam_normal1(2)<<std::endl;
		std::cout<<"camera normal 2: "<<cam_normal2(0)<<","<<cam_normal2(1)<<","<<cam_normal2(2)<<std::endl;
		std::cout<<"camera normal 3: "<<cam_normal3(0)<<","<<cam_normal3(1)<<","<<cam_normal3(2)<<std::endl;

		std::cout<<"-----------R--------------"<<std::endl;
		std::cout<<R_<<std::endl;
		std::cout<<"-----------T--------------"<<std::endl;
		std::cout<<T_<<std::endl;
		Eigen::Vector3f eulerAngle = R_.eulerAngles(2,1,0);//Roll Pitch Yaw
		std::cout<<"----------------------euler angle-----------------------"<<std::endl;
		std::cout<<"Raw: "<<eulerAngle(2)*180/3.1415926<<", Pitch: "<<eulerAngle(1)*180/3.1415926<<", Roll: "<<eulerAngle(0)*180/3.1415926<<std::endl;
		std::cout<<"------------------- ExtrinsicCalculator::CalExtrinsic Log end------------------------"<<std::endl;

	}

	return CalResult_;
}

bool ExtrinsicCalculator::CalExtrinsic2(std::vector<TagInfo> tag_infos, bool verbose)
{
	/*********************calculate normal of camera plane**********************/
	pcl::ModelCoefficients::Ptr plane1_coff(new pcl::ModelCoefficients);
    pcl::ModelCoefficients::Ptr plane2_coff(new pcl::ModelCoefficients);
    pcl::ModelCoefficients::Ptr plane3_coff(new pcl::ModelCoefficients);
	
	if(camera_clouds_[0].size()<=8||camera_clouds_[1].size()<=8)
	{
		ROS_ERROR("No enough points to calculate camera position!");
		CalResult_=false;
		return CalResult_;
	}
	else if(camera_clouds_[2].size()<8)
	{
		ROS_WARN("Camere point is not enough! The groun plane will be assumed lower than lowest point for 1 meter!");
		Point3DRansac(camera_clouds_[0].makeShared(),plane1_coff);
		Point3DRansac(camera_clouds_[1].makeShared(),plane2_coff);

	}else{ 
		Point3DRansac(camera_clouds_[0].makeShared(),plane1_coff);
		Point3DRansac(camera_clouds_[1].makeShared(),plane2_coff);
		Point3DRansac(camera_clouds_[2].makeShared(),plane3_coff);
	}
	//correct the normal direction 
	if(plane1_coff->values[3]<0)
    {
        plane1_coff->values[0]=-plane1_coff->values[0];
        plane1_coff->values[1]=-plane1_coff->values[1];
        plane1_coff->values[2]=-plane1_coff->values[2];
        plane1_coff->values[3]=-plane1_coff->values[3];
		Eigen::Vector3f normal1;
		normal1<<plane1_coff->values[0],plane1_coff->values[1],plane1_coff->values[2];
		normal1.normalize();
		if(normal1.dot(mean_normal[0])>0)
		{
			plane1_coff->values[0]=mean_normal[0][0];
			plane1_coff->values[1]=mean_normal[0][1];
			plane1_coff->values[2]=mean_normal[0][2];
		}else{
			plane1_coff->values[0]=-mean_normal[0][0];
			plane1_coff->values[1]=-mean_normal[0][1];
			plane1_coff->values[2]=-mean_normal[0][2];
		}
    }
    if(plane2_coff->values[3]<0)
    {
        plane2_coff->values[0]=-plane2_coff->values[0];
        plane2_coff->values[1]=-plane2_coff->values[1];
        plane2_coff->values[2]=-plane2_coff->values[2];
        plane2_coff->values[3]=-plane2_coff->values[3];
		Eigen::Vector3f normal2;
		normal2<<plane2_coff->values[0],plane2_coff->values[1],plane2_coff->values[2];
		normal2.normalize();
		if(normal2.dot(mean_normal[1])>0)
		{
			plane2_coff->values[0]=mean_normal[1][0];
			plane2_coff->values[1]=mean_normal[1][1];
			plane2_coff->values[2]=mean_normal[1][2];
		}else{
			plane2_coff->values[0]=-mean_normal[1][0];
			plane2_coff->values[1]=-mean_normal[1][1];
			plane2_coff->values[2]=-mean_normal[1][2];
		}
    }
	//calculate corner origin in camera coordinate
	Eigen::Matrix3d A;
	Eigen::Vector3d D;
	Eigen::Vector3d camera_corner_origin;
    Eigen::Vector3f cam_normal1,cam_normal2,cam_normal3;

	cam_normal1<<plane1_coff->values[0],plane1_coff->values[1],plane1_coff->values[2];
    cam_normal2<<plane2_coff->values[0],plane2_coff->values[1],plane2_coff->values[2];
	if(camera_clouds_[2].size()<8)//no enough ground points because of some camera position and poseture and it can't see tags
	{	
		cam_normal3=cam_normal1.cross(cam_normal2);//calculate the ground normal 
		cam_normal3.normalize();
		cam_normal3=cam_normal3*TAG_HEIGHT;
		if(cam_normal3(2)<0)
			cam_normal3=-cam_normal3;
		pcl::PointCloud<pcl::PointXYZ>::Ptr ground_points(new pcl::PointCloud<pcl::PointXYZ>);
		findLowestPoint(ground_points,cam_normal3);
		if(ground_points->size()<2)
		{
			ROS_WARN("Fail to find any points on the ground");
			CalResult_=false;
			return CalResult_;
		}
		float DD=0.0;
		for(auto pt: *ground_points)
		{
			camera_clouds_[2].push_back(pt);
			DD=DD-(cam_normal3(0)*pt.x+cam_normal3(1)*pt.y+cam_normal3(2)*pt.z);
		}
		DD=DD/ground_points->size();
		plane3_coff->values.clear();
		plane3_coff->values.push_back(cam_normal3(0));
		plane3_coff->values.push_back(cam_normal3(1));
		plane3_coff->values.push_back(cam_normal3(2));
		plane3_coff->values.push_back(DD);
	}
	A<<plane1_coff->values[0],plane1_coff->values[1],plane1_coff->values[2],
	plane2_coff->values[0],plane2_coff->values[1],plane2_coff->values[2],
	plane3_coff->values[0],plane3_coff->values[1],plane3_coff->values[2];

	D<<plane1_coff->values[3],plane2_coff->values[3],plane3_coff->values[3];
	camera_corner_origin=-A.inverse()*D;
	if(plane3_coff->values[3]<0)
	{
		plane3_coff->values[0]=-plane3_coff->values[0];
		plane3_coff->values[1]=-plane3_coff->values[1];
		plane3_coff->values[2]=-plane3_coff->values[2];
		plane3_coff->values[3]=-plane3_coff->values[3];
		Eigen::Vector3f normal3;
		normal3<<plane3_coff->values[0],plane3_coff->values[1],plane3_coff->values[2];
		normal3.normalize();
		if(normal3.dot(mean_normal[2])>0)
		{
			plane3_coff->values[0]=mean_normal[2][0];
			plane3_coff->values[1]=mean_normal[2][1];
			plane3_coff->values[2]=mean_normal[2][2];
		}else{
			plane3_coff->values[0]=-mean_normal[2][0];
			plane3_coff->values[1]=-mean_normal[2][1];
			plane3_coff->values[2]=-mean_normal[2][2];
		}
	}
	cam_normal3<<plane3_coff->values[0],plane3_coff->values[1],plane3_coff->values[2];	

	// normalize normal
    resetCoor(cam_normal1,cam_normal2,cam_normal3);


	/*************************calculate lidar PointCloud normal*******************/
	if(lidar_cloud_->size()<10)
	{
		ROS_WARN("Normal calculator: Please add lidar PointCLoud!");
		CalResult_=false;
		return CalResult_;
	}
	Eigen::Vector3f lidar_origin_corner = Eigen::Vector3f::Zero();
	std::vector<pcl::PointCloud<pcl::PointXYZ> > cloud_planes;
    std::vector<pcl::ModelCoefficients> coefficients;
    std::vector<Eigen::Vector3f> lidar_corner_normal;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_max_plane(new pcl::PointCloud<pcl::PointXYZI>);

    cloud_planes.resize(3);        
    lidar_corner_normal.resize(3);
    coefficients.resize(3);

    PlaneExtraction(lidar_cloud_,cloud_plane,cloud_max_plane,lidar_origin_corner,cloud_planes,coefficients,lidar_corner_normal);
    resetCoor(lidar_corner_normal[0],lidar_corner_normal[1],lidar_corner_normal[2]);

/*************calculate extrinsic parameter***************************/

	Eigen::Matrix3d P; //点云的坐标轴
	P.col(0) = lidar_corner_normal[0].cast<double>() + lidar_origin_corner.cast<double>();
	P.col(1) = lidar_corner_normal[1].cast<double>() + lidar_origin_corner.cast<double>();
	P.col(2) = lidar_corner_normal[2].cast<double>() + lidar_origin_corner.cast<double>();
	
	Eigen::Matrix3d Q;//相机识别到的坐标轴
	
    Q.col(0) = cam_normal1.cast<double>() + camera_corner_origin.cast<double>();
	Q.col(1) = cam_normal2.cast<double>() + camera_corner_origin.cast<double>();
	Q.col(2) = cam_normal3.cast<double>() + camera_corner_origin.cast<double>();

	Eigen::Affine3d affine = Kabsch::Find3DAffineTransform(Q,P, false);
	Eigen::Matrix3d RR = affine.linear();
	Eigen::Vector3d tt = affine.translation();

	R_<<RR(0,0),RR(0,1),RR(0,2),
		RR(1,0),RR(1,1),RR(1,2),
		RR(2,0),RR(2,1),RR(2,2);
	T_<<tt(0),tt(1),tt(2);
	CalResult_=true;
	
	if(verbose==true)
	{	
		pcl::io::savePCDFile("/home/ramlab/Documents/CornerSeg/dataset/corner_aprilTag_point/carla/1.pcd",camera_clouds_[0]);
		pcl::io::savePCDFile("/home/ramlab/Documents/CornerSeg/dataset/corner_aprilTag_point/carla/2.pcd",camera_clouds_[1]);
		pcl::io::savePCDFile("/home/ramlab/Documents/CornerSeg/dataset/corner_aprilTag_point/carla/3.pcd",camera_clouds_[2]);
		std::cout<<"------------------- ExtrinsicCalculator::CalExtrinsic Log begin------------------------"<<std::endl;
		std::cout<<"--------------------plane coefficient---------------------"<<std::endl;
		std::cout<<"plane coff1: "<<plane1_coff->values[0]<<","<<plane1_coff->values[1]<<","<<plane1_coff->values[2]<<","<<plane1_coff->values[3]<<std::endl;
		std::cout<<"plane coff2: "<<plane2_coff->values[0]<<","<<plane2_coff->values[1]<<","<<plane2_coff->values[2]<<","<<plane2_coff->values[3]<<std::endl;
		std::cout<<"plane coff3: "<<plane3_coff->values[0]<<","<<plane3_coff->values[1]<<","<<plane3_coff->values[2]<<","<<plane3_coff->values[3]<<std::endl;
		std::cout<<"--------------------corner origin---------------------"<<std::endl;

		std::cout<<"lidar origin: "<<lidar_origin_corner(0)<<","<<lidar_origin_corner(1)<<","<<lidar_origin_corner(2)<<std::endl;
		std::cout<<"camera origin: "<<camera_corner_origin(0)<<","<<camera_corner_origin(1)<<","<<camera_corner_origin(2)<<std::endl;
		std::cout<<"camera origin - lidar origin: "<<camera_corner_origin(0)-lidar_origin_corner(0)<<","<<camera_corner_origin(1)-lidar_origin_corner(1)<<","<<camera_corner_origin(2)-lidar_origin_corner(2)<<std::endl;
		std::cout<<"--------------------Lidar normal---------------------"<<std::endl;

		std::cout<<"lidar normal 1: "<<lidar_corner_normal[0](0)<<","<<lidar_corner_normal[0](1)<<","<<lidar_corner_normal[0](2)<<std::endl;
		std::cout<<"lidar normal 2: "<<lidar_corner_normal[1](0)<<","<<lidar_corner_normal[1](1)<<","<<lidar_corner_normal[1](2)<<std::endl;
		std::cout<<"lidar normal 3: "<<lidar_corner_normal[2](0)<<","<<lidar_corner_normal[2](1)<<","<<lidar_corner_normal[2](2)<<std::endl;

		std::cout<<"--------------------camera normal---------------------"<<std::endl;
		std::cout<<"camera normal 1: "<<cam_normal1(0)<<","<<cam_normal1(1)<<","<<cam_normal1(2)<<std::endl;
		std::cout<<"camera normal 2: "<<cam_normal2(0)<<","<<cam_normal2(1)<<","<<cam_normal2(2)<<std::endl;
		std::cout<<"camera normal 3: "<<cam_normal3(0)<<","<<cam_normal3(1)<<","<<cam_normal3(2)<<std::endl;

		std::cout<<"-----------R--------------"<<std::endl;
		std::cout<<R_<<std::endl;
		std::cout<<"-----------T--------------"<<std::endl;
		std::cout<<T_<<std::endl;
		Eigen::Vector3f eulerAngle = R_.eulerAngles(2,1,0);//Roll Pitch Yaw
		std::cout<<"----------------------euler angle-----------------------"<<std::endl;
		std::cout<<"Raw: "<<eulerAngle(2)*180/3.1415926<<", Pitch: "<<eulerAngle(1)*180/3.1415926<<", Roll: "<<eulerAngle(0)*180/3.1415926<<std::endl;
		std::cout<<"------------------- ExtrinsicCalculator::CalExtrinsic Log end------------------------"<<std::endl;

	}

	return CalResult_;
}


/**
 * @brief project lidar points to camera image using the transform that just be calculated before
*/
void ExtrinsicCalculator::LidarPorj2Camera(cv::Mat &rawImg,pcl::PointCloud<pcl::PointXYZ>::Ptr inCloud,cv::Mat intrinsics)
{
	if(CalResult_==false)
	{
		ROS_WARN("Please calculate correct R and T and then project point to image!");
		return;
	}
	pcl::PointCloud<pcl::PointXYZ> cloud2camera;
	Eigen::Matrix3f intrinsic_eigen; 
    intrinsic_eigen<<intrinsics.at<float>(0,0),intrinsics.at<float>(0,1),intrinsics.at<float>(0,2)
                    ,intrinsics.at<float>(1,0),intrinsics.at<float>(1,1),intrinsics.at<float>(1,2)
                    ,intrinsics.at<float>(2,0),intrinsics.at<float>(2,1),intrinsics.at<float>(2,2);
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	Eigen::Matrix4f transform;
	transform.block<3,3>(0,0)=R_;
	transform.block<1,3>(3,0)<<0,0,0;
	transform.block<3,1>(0,3) = T_;

	// pcl::transformPointCloud (*inCloud, *transformed_cloud,);
	for(int i=0;i<inCloud->size();i++)
	{
		pcl::PointXYZ pt;
		Eigen::Vector3f cloudpoint;
		Eigen::Vector3f camerapoint;
		Eigen::Vector3f pixelpoint;
		cloudpoint<<inCloud->points[i].x,inCloud->points[i].y,inCloud->points[i].z;
		cloudpoint=R_.inverse()*cloudpoint-R_.inverse()*T_;
		camerapoint=cameraCoor2WorldCoor_.inverse()*cloudpoint;
		pixelpoint=intrinsic_eigen*camerapoint;
		int pixex=pixelpoint(0)/pixelpoint(2);
		int pixey=pixelpoint(1)/pixelpoint(2);

		if(pixex>2&&pixex<rawImg.size[1]&&pixey>2&&pixey<rawImg.size[0]-2&&pixelpoint(2)>0)
		{
			int color_order = int((cloudpoint(0)-7)/0.2);
			if(color_order>20)
				color_order=20;
			else if(color_order<0)
				color_order=0;
			cv::Vec3b ccc= Vec3b(color[color_order][2], color[color_order][1], color[color_order][0]);

			rawImg.at<Vec3b>(pixey,pixex)=ccc;
			rawImg.at<Vec3b>(pixey+1,pixex+1)=ccc;
			rawImg.at<Vec3b>(pixey-1,pixex-1)=ccc;
			rawImg.at<Vec3b>(pixey+1,pixex)=ccc;
			rawImg.at<Vec3b>(pixey,pixex+1)=ccc;
			rawImg.at<Vec3b>(pixey-1,pixex)=ccc;
			rawImg.at<Vec3b>(pixey,pixex-1)=ccc;

		}
	}
}
void ExtrinsicCalculator::findLowestPoint(pcl::PointCloud<pcl::PointXYZ>::Ptr CameraLowestPointExcludeGround,Eigen::Vector3f ground_normal)
{
	CameraLowestPointExcludeGround->clear();
	if(ground_normal(2)<0)
	{
		ROS_WARN("findLowestPoint: Ground normal has wrong direction!");
		ROS_WARN("findLowestPoint: I will inverse it!");
		ground_normal=-ground_normal;
	}
	pcl::PointXYZ plane1LowestPoint(10,10,10);
	pcl::PointXYZ groudPoint;
	//plane 1 step 1: find lowest point
	for(auto pt: camera_clouds_[0])
	{
		if(pt.z<plane1LowestPoint.z)
		{
			plane1LowestPoint=pt;
		}
	}
	
	//step2: find nearst point from lowest point along the ground normal direction.
	for(auto pt: camera_clouds_[0])
	{
		Eigen::Vector3f diff;
		diff<<plane1LowestPoint.x-pt.x,plane1LowestPoint.y-pt.y,plane1LowestPoint.z-pt.z;
		float dis=fabs(diff.dot(ground_normal)); 
		if(dis>0.05) //
			continue;
		else{
			groudPoint.x=pt.x-ground_normal(0);
			groudPoint.y=pt.y-ground_normal(1);
			groudPoint.z=pt.z-ground_normal(2);
			CameraLowestPointExcludeGround->push_back(groudPoint);
		}
	}
//plane 2 step 1: find lowest point
	pcl::PointXYZ plane2LowestPoint(10,10,10);
	for(auto pt: camera_clouds_[1])
	{
		if(pt.z<plane2LowestPoint.z)
		{
			plane2LowestPoint=pt;
		}
	}
	
	//step2: find nearst point from lowest point along the ground normal direction.
	for(auto pt: camera_clouds_[1])
	{
		Eigen::Vector3f diff;
		diff<<plane2LowestPoint.x-pt.x,plane2LowestPoint.y-pt.y,plane2LowestPoint.z-pt.z;
		float dis=fabs(diff.dot(ground_normal)); 
		if(dis>0.05) //
			continue;
		else{
			groudPoint.x=pt.x-ground_normal(0);
			groudPoint.y=pt.y-ground_normal(1);
			groudPoint.z=pt.z-ground_normal(2);
			CameraLowestPointExcludeGround->push_back(groudPoint);
		}
	}
}

Eigen::Vector3f ExtrinsicCalculator::getTranslation()
{
	return T_;
}
Eigen::Matrix3f ExtrinsicCalculator::getRotation()
{
	return R_;
}