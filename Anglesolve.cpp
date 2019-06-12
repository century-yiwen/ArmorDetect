#include"Anglesolve.h"
#include <stdint.h> 
#include "opencv2/opencv.hpp"

using namespace cv;
using namespace std;

bool ismall;
void RectPnPSolver::solvePnP4Points(const std::vector<cv::Point2f> & points2d, cv::Mat & rot, cv::Mat & trans) {
	if (width_target < 10e-5 || height_target < 10e-5) {
		rot = cv::Mat::eye(3, 3, CV_64FC1);
		trans = cv::Mat::zeros(3, 1, CV_64FC1);
		return;
	}
	std::vector<cv::Point3f> point3d;
	double half_x = width_target / 2.0;
	double half_y = height_target / 2.0;

	point3d.push_back(Point3f(-half_x, -half_y, 0));
	point3d.push_back(Point3f(-half_x, half_y, 0));
	point3d.push_back(Point3f(half_x, half_y, 0));
	point3d.push_back(Point3f(half_x, -half_y, 0));


	Mat r;
	solvePnP(point3d, points2d, cam_matrix, distortion_coeff, r, trans);

	Rodrigues(r, rot);
}

void AngleSolver::setRelationPoseCameraPTZ( const cv::Mat & trans_camera_ptz, double y_offset_barrel_ptz) {
	trans_camera_ptz.copyTo(trans_camera2ptz);
	offset_y_barrel_ptz = y_offset_barrel_ptz;
}

 vector<Point2f> AngleSolver::getTarget2dPoinstion(const vector<RotatedRect> & result_rect )
{
		RotatedRect left_light, right_light;
		Point2f  pt1[4], pt2[4];
		vector<Point2f>  pt3;
		if (result_rect.size() != 2)
		{
			return pt3;
		}
		else
		{
			if (result_rect[0].center.x > result_rect[1].center.x)
			{
				left_light = result_rect[1];
				right_light = result_rect[0];
				left_light.points(pt1);
				right_light.points(pt2);

				int ii, jj;
				int flag1 = 1;
				Point2f temp1;

				for (ii = 1; ii < 4 && flag1 == 1; ii++)
				{
					flag1 = 0;
					for (jj = 0; jj < 4 - ii; jj++)
					{
						if (pt1[jj].x > pt1[jj + 1].x)
						{
							flag1 = 1;
							temp1 = pt1[jj];
							pt1[jj] = pt1[jj + 1];
							pt1[jj + 1] = temp1;
						}
					}
				}

				if (pt1[0].y < pt1[1].y)
				{
					pt3.push_back(pt1[0]);//左上角顶点
					pt3.push_back(pt1[1]);//左下角顶点
				}
				else
				{
					pt3.push_back(pt1[1]);
					pt3.push_back(pt1[0]);
				}

				int ii2, jj2;
				int flag2 = 1;
				Point2f temp2;

				for (ii2 = 1; ii2 < 4 && flag2 == 1; ii2++)
				{
					flag2 = 0;
					for (jj2 = 0; jj2 < 4 - ii2; jj2++)
					{
						if (pt2[jj2].x < pt2[jj2 + 1].x)
						{
							flag2 = 1;
							temp2 = pt2[jj2];
							pt2[jj2] = pt2[jj2 + 1];
							pt2[jj2 + 1] = temp2;
						}
					}
				}

				if (pt2[0].y > pt2[1].y)
				{
					pt3.push_back(pt2[0]);
					pt3.push_back(pt2[1]);
				}
				else
				{
					pt3.push_back(pt2[1]);
					pt3.push_back(pt2[0]);
				}



			}
			else
			{
				left_light = result_rect[0];
				right_light = result_rect[1];
				left_light.points(pt1);
				right_light.points(pt2);

				int ii, jj;
				int flag1 = 1;
				Point2f temp1;

				for (ii = 1; ii < 4 && flag1 == 1; ii++)
				{
					flag1 = 0;
					for (jj = 0; jj < 4 - ii; jj++)
					{
						if (pt1[jj].x > pt1[jj + 1].x)
						{
							flag1 = 1;
							temp1 = pt1[jj];
							pt1[jj] = pt1[jj + 1];
							pt1[jj + 1] = temp1;
						}
					}
				}

				if (pt1[0].y < pt1[1].y)
				{
					pt3.push_back(pt1[0]);//左上角顶点
					pt3.push_back(pt1[1]);//左下角顶点
				}
				else
				{
					pt3.push_back(pt1[1]);
					pt3.push_back(pt1[0]);
				}

				int ii2, jj2;
				int flag2 = 1;
				Point2f temp2;

				for (ii2 = 1; ii2 < 4 && flag2 == 1; ii2++)
				{
					flag2 = 0;
					for (jj2 = 0; jj2 < 4 - ii2; jj2++)
					{
						if (pt2[jj2].x < pt2[jj2 + 1].x)
						{
							flag2 = 1;
							temp2 = pt2[jj2];
							pt2[jj2] = pt2[jj2 + 1];
							pt2[jj2 + 1] = temp2;
						}
					}
				}

				if (pt2[0].y > pt2[1].y)
				{
					pt3.push_back(pt2[0]);
					pt3.push_back(pt2[1]);
				}
				else
				{
					pt3.push_back(pt2[1]);
					pt3.push_back(pt2[0]);
				}


			}
			/*line(image, pt3[0], pt3[1], CV_RGB(255, 0, 0), 2, 8, 0);
			line(image, pt3[1], pt3[2], CV_RGB(255, 0, 0), 2, 8, 0);
			line(image, pt3[2], pt3[3], CV_RGB(255, 0, 0), 2, 8, 0);
			line(image, pt3[3], pt3[0], CV_RGB(255, 0, 0), 2, 8, 0);*/

			return pt3;
		}
		return pt3;
}

bool AngleSolver::getAngle(const vector<RotatedRect> & rect, double & angle_x, double & angle_y, double & distance, double bullet_speed) {
	if (rect.size() < 2 )
		return false;

	// get 3D positon in camera coordinate
	//    double wh_ratio = width_target/height_target;
	//    RotatedRect adj_rect(rect.center, Size2f(rect.size.width, rect.size.width/wh_ratio), rect.angle);
	//    vector<Point2f> target2d;
	//    getTarget2dPoinstion(adj_rect, target2d, offset);

	vector<Point2f> target2d;
	target2d=getTarget2dPoinstion(rect);

	cv::Mat r;
	solvePnP4Points(target2d, r, position_in_camera);
	const double *my_xyz = (const double *)position_in_camera.data;
 	 distance = my_xyz[2];
	//position_in_camera.at<double>(2, 0) = 1.4596 * position_in_camera.at<double>(2, 0);  // for camera-2 calibration (unfix center)
	//position_in_camera.at<double>(2, 0) = 1.5348 * position_in_camera.at<double>(2, 0);  // for camera-MH calibration (unfix center)
	position_in_camera.at<double>(2, 0) = scale_z * position_in_camera.at<double>(2, 0);

	/*if (position_in_camera.at<double>(2, 0) < min_distance || position_in_camera.at<double>(2, 0) > max_distance) {
		cout << "out of range: [" << min_distance << ", " << max_distance << "]\n";
		return false;
	}*/

	// translate camera coordinate to PTZ coordinate
	tranformationCamera2PTZ(position_in_camera, position_in_ptz);
	//    cout << "position_in_camera: " << position_in_camera.t() << endl;
	//    cout << "position_in_ptz: " << position_in_ptz.t() << endl;
	// calculte angles to turn, so that make barrel aim at target
	adjustPTZ2Barrel(position_in_ptz, angle_x, angle_y, bullet_speed);

	return true;
}

void AngleSolver::tranformationCamera2PTZ(const cv::Mat & pos, cv::Mat & transed_pos) {
	transed_pos = pos - trans_camera2ptz;
}

void AngleSolver::adjustPTZ2Barrel(const cv::Mat & pos_in_ptz, double & angle_x, double & angle_y, double bullet_speed) {
	const double *_xyz = (const double *)pos_in_ptz.data;
	double down_t = 0.0;
    if (bullet_speed > 10e-3)
		down_t = _xyz[2] / 100.0 / bullet_speed;
    double offset_gravity = 0.5 * 9.8 * down_t * down_t * 100;
	double xyz[3] = { _xyz[0], _xyz[1] - offset_gravity, _xyz[2] };
	double alpha = 0.0, theta = 0.0;

	//alpha = asin(offset_y_barrel_ptz / sqrt(xyz[1] * xyz[1] + xyz[2] * xyz[2]));
	alpha = 0;
	if (xyz[1] < 0) {
		theta = atan(-xyz[1] / xyz[2]);
		angle_y = -(alpha + theta);  // camera coordinate
	}
	else if (xyz[1] < offset_y_barrel_ptz) {
		theta = atan(xyz[1] / xyz[2]);
		angle_y = -(alpha - theta);  // camera coordinate
	}
	else {
		theta = atan(xyz[1] / xyz[2]);
		angle_y = (theta - alpha);   // camera coordinate
	}
	angle_x = atan2(xyz[0], xyz[2]);
	//cout << "angle_x: " << angle_x << "\tangle_y: " << angle_y <<  "\talpha: " << alpha << "\ttheta: " << theta << endl;
	angle_x = angle_x * 180 / 3.1415926;
	angle_y = angle_y * 180 / 3.1415926;
}
