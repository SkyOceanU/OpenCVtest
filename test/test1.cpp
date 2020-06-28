#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include <opencv2/highgui.hpp>
#include <string>
#include <vector>
#include <fstream>
using namespace std;
using namespace cv;
#define PATH "D://OPENCV//left//left" //图片存放目录
#define NUM 15  //图片数量
#define TESTPATH "D://OPENCV//left//left04.jpg" //测试图片文件路径

void myUndistortPoints(const cv::Mat & src, std::vector<cv::Point2d> & dst,
	const cv::Mat & cameraMatrix, const cv::Mat & distortionCoeff);
int main() {
	// 定义用来保存导入的图片
	Mat image_in;
	// 定义用来保存文件路径的容器
	vector<string> filelist;
	// 定义用来保存旋转和平移矩阵的容器
	vector<Mat> rvecs, tvecs;
	// 定义相机矩阵，畸变矩阵
	Mat cameraMatrix;
	Mat distCoeffs;
	int flags = 0;
	// 定义保存图像二维角点的容器
	vector<Point2f> corners;
	// 定义保存图像二维角点的容器
	vector<vector<Point2f> > corners2;
	// 定义保存图像三维和三维角点的容器
	vector<Point3f> worldPoints;
	vector<vector<Point3f> > worldPoints2;
	//***********************生成一组object_points*************************
	for (int j = 0; j < 6; j++) {
		for (int k = 0; k < 9; k++) {
			worldPoints.push_back(Point3f(j*1.0, k*1.0, 0.0f));
			// 世界坐标系的三维vector 放入三维vector
		}
	}
	//***************读取一个文件夹中的所有图片（所有标定图片）**********************
	for (int i = 1; i < NUM; i++) {
		stringstream str;
		if (i != 10) {
			str << PATH << setw(2) << setfill('0') << i << ".jpg";
			// 保存所有图片的路径，放入容器filelist中
			filelist.push_back(str.str());
			image_in = imread(str.str());
		}	
	}

	//***************************找角点**********************************************
	for (int i = 0; i < filelist.size(); i++) {
		cout <<filelist[i]<<endl;
		// 一张张读入图片；
		image_in = imread(filelist[i]);
		// 找图片的角点，参数分别为：
		// 输入图片，图片内角点数（不算棋盘格最外层的角点），输出角点，求解方式
		bool found = findChessboardCorners(image_in, Size(9, 6), corners, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
		// 将找到的角点放入容器中；
		corners2.push_back(corners);
		//画出角点
		drawChessboardCorners(image_in, Size(9, 6), corners, found);
		//显示图像
		imshow("test", image_in);
		// 图像刷新等待时间，单位ms
		waitKey(1000);
		worldPoints2.push_back(worldPoints);
	}
	
	//相机标定
	calibrateCamera(worldPoints2, corners2, image_in.size(), cameraMatrix, distCoeffs,
		rvecs, tvecs, CV_CALIB_FIX_PRINCIPAL_POINT);

	//*************************************查看参数*****************************************
	cout << " Camera intrinsic: " << cameraMatrix.rows << "x" << cameraMatrix.cols << endl;
	cout << cameraMatrix.at<double>(0, 0) << " " << cameraMatrix.at<double>(0, 1) << " " << cameraMatrix.at<double>(0, 2) << endl;
	cout << cameraMatrix.at<double>(1, 0) << " " << cameraMatrix.at<double>(1, 1) << " " << cameraMatrix.at<double>(1, 2) << endl;
	cout << cameraMatrix.at<double>(2, 0) << " " << cameraMatrix.at<double>(2, 1) << " " << cameraMatrix.at<double>(2, 2) << endl;
	cout << distCoeffs.rows << "x" << distCoeffs.cols << endl;
	cout << "distCoeffs : "<< endl << distCoeffs << endl;

	//*********************畸变矫正**************************
	// 导入要矫正的图片
	for (int i = 1; i < NUM; i++) {
		stringstream str;
		if (i != 10) {
			str << PATH << setw(2) << setfill('0') << i << ".jpg";
			Mat test_image2 = imread(str.str());
			Mat show_image;
			undistort(test_image2, show_image, cameraMatrix, distCoeffs);
			imshow("corrected", show_image);
			cout << str.str() << endl;
			waitKey(3000);
			//cout << "输入任意字符跳转到下一张图片"<<endl;
			//getchar();
		}	
	}	
	/*
	//迭代法求解测试
	Mat test_image2 = imread(TESTPATH);
	Mat show_image = imread(TESTPATH);
	undistort(test_image2, show_image, cameraMatrix, distCoeffs);
	imshow("corrected", show_image);
	waitKey(3000);
	vector<cv::Point2d> p;
	myUndistortPoints(test_image2, p, cameraMatrix, distCoeffs);
	show_image = imread(TESTPATH);
	for (int i = 0; i < test_image2.rows; ++i) {
		for (int j = 0; j < test_image2.cols; ++j) {
			if (p[i*test_image2.cols + j].x >= 0 && p[i*test_image2.cols + j].x < test_image2.rows 
				&& p[i*test_image2.cols + j].y < test_image2.cols && p[i*test_image2.cols + j].y >= 0) {
				Vec3b pix = test_image2.at<Vec3b>(i,j); //指针或引用
				show_image.at<Vec3b>(p[i*test_image2.cols + j].x, p[i*test_image2.cols + j].y) = pix;
			}
		}
	}
	imshow("corrected2", show_image);
	waitKey(3000);
	*/
}

//迭代法求解,纠正图像
void myUndistortPoints(const cv::Mat & src, std::vector<cv::Point2d> & dst,
	const cv::Mat & cameraMatrix, const cv::Mat & distortionCoeff)
{

	dst.clear();
	double fx = cameraMatrix.at<double>(0, 0);
	double fy = cameraMatrix.at<double>(1, 1);
	double ux = cameraMatrix.at<double>(0, 2);
	double uy = cameraMatrix.at<double>(1, 2);

	double k1 = distortionCoeff.at<double>(0, 0);
	double k2 = distortionCoeff.at<double>(0, 1);
	double p1 = distortionCoeff.at<double>(0, 2);
	double p2 = distortionCoeff.at<double>(0, 3);
	double k3 = distortionCoeff.at<double>(0, 4);
	double k4 = 0;
	double k5 = 0;
	double k6 = 0;

	for (int i = 0; i < src.rows; i++)
	{
		for (int j = 0; j < src.cols; j++)
		{

			//首先进行坐标转换；
			double xDistortion = (i - ux) / fx;
			double yDistortion = (j - uy) / fy;

			double xCorrected, yCorrected;

			double x0 = xDistortion;
			double y0 = yDistortion;

			//这里使用迭代的方式进行求解，因为根据2中的公式直接求解是困难的，所以通过设定初值进行迭代，这也是OpenCV的求解策略；
			for (int j = 0; j < 10000; j++)
			{
				double r2 = xDistortion*xDistortion + yDistortion*yDistortion;

				double distRadialA = 1 / (1. + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2);
				double distRadialB = 1. + k4 * r2 + k5 * r2 * r2 + k6 * r2 * r2 * r2;

				double deltaX = 2. * p1 * xDistortion * yDistortion + p2 * (r2 + 2. * xDistortion * xDistortion);
				double deltaY = p1 * (r2 + 2. * yDistortion * yDistortion) + 2. * p2 * xDistortion * yDistortion;

				xCorrected = (x0 - deltaX)* distRadialA * distRadialB;
				yCorrected = (y0 - deltaY)* distRadialA * distRadialB;

				xDistortion = xCorrected;
				yDistortion = yCorrected;
			}

			//进行坐标变换；
			xCorrected = xCorrected * fx + ux;
			yCorrected = yCorrected * fy + uy;

			dst.push_back(cv::Point2d((int)(xCorrected+0.5), (int)(yCorrected+0.5)));
		}
	}

}