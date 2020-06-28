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
#define PATH "D://OPENCV//left//left" //ͼƬ���Ŀ¼
#define NUM 15  //ͼƬ����
#define TESTPATH "D://OPENCV//left//left04.jpg" //����ͼƬ�ļ�·��

void myUndistortPoints(const cv::Mat & src, std::vector<cv::Point2d> & dst,
	const cv::Mat & cameraMatrix, const cv::Mat & distortionCoeff);
int main() {
	// �����������浼���ͼƬ
	Mat image_in;
	// �������������ļ�·��������
	vector<string> filelist;
	// ��������������ת��ƽ�ƾ��������
	vector<Mat> rvecs, tvecs;
	// ����������󣬻������
	Mat cameraMatrix;
	Mat distCoeffs;
	int flags = 0;
	// ���屣��ͼ���ά�ǵ������
	vector<Point2f> corners;
	// ���屣��ͼ���ά�ǵ������
	vector<vector<Point2f> > corners2;
	// ���屣��ͼ����ά����ά�ǵ������
	vector<Point3f> worldPoints;
	vector<vector<Point3f> > worldPoints2;
	//***********************����һ��object_points*************************
	for (int j = 0; j < 6; j++) {
		for (int k = 0; k < 9; k++) {
			worldPoints.push_back(Point3f(j*1.0, k*1.0, 0.0f));
			// ��������ϵ����άvector ������άvector
		}
	}
	//***************��ȡһ���ļ����е�����ͼƬ�����б궨ͼƬ��**********************
	for (int i = 1; i < NUM; i++) {
		stringstream str;
		if (i != 10) {
			str << PATH << setw(2) << setfill('0') << i << ".jpg";
			// ��������ͼƬ��·������������filelist��
			filelist.push_back(str.str());
			image_in = imread(str.str());
		}	
	}

	//***************************�ҽǵ�**********************************************
	for (int i = 0; i < filelist.size(); i++) {
		cout <<filelist[i]<<endl;
		// һ���Ŷ���ͼƬ��
		image_in = imread(filelist[i]);
		// ��ͼƬ�Ľǵ㣬�����ֱ�Ϊ��
		// ����ͼƬ��ͼƬ�ڽǵ������������̸������Ľǵ㣩������ǵ㣬��ⷽʽ
		bool found = findChessboardCorners(image_in, Size(9, 6), corners, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_NORMALIZE_IMAGE);
		// ���ҵ��Ľǵ���������У�
		corners2.push_back(corners);
		//�����ǵ�
		drawChessboardCorners(image_in, Size(9, 6), corners, found);
		//��ʾͼ��
		imshow("test", image_in);
		// ͼ��ˢ�µȴ�ʱ�䣬��λms
		waitKey(1000);
		worldPoints2.push_back(worldPoints);
	}
	
	//����궨
	calibrateCamera(worldPoints2, corners2, image_in.size(), cameraMatrix, distCoeffs,
		rvecs, tvecs, CV_CALIB_FIX_PRINCIPAL_POINT);

	//*************************************�鿴����*****************************************
	cout << " Camera intrinsic: " << cameraMatrix.rows << "x" << cameraMatrix.cols << endl;
	cout << cameraMatrix.at<double>(0, 0) << " " << cameraMatrix.at<double>(0, 1) << " " << cameraMatrix.at<double>(0, 2) << endl;
	cout << cameraMatrix.at<double>(1, 0) << " " << cameraMatrix.at<double>(1, 1) << " " << cameraMatrix.at<double>(1, 2) << endl;
	cout << cameraMatrix.at<double>(2, 0) << " " << cameraMatrix.at<double>(2, 1) << " " << cameraMatrix.at<double>(2, 2) << endl;
	cout << distCoeffs.rows << "x" << distCoeffs.cols << endl;
	cout << "distCoeffs : "<< endl << distCoeffs << endl;

	//*********************�������**************************
	// ����Ҫ������ͼƬ
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
			//cout << "���������ַ���ת����һ��ͼƬ"<<endl;
			//getchar();
		}	
	}	
	/*
	//������������
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
				Vec3b pix = test_image2.at<Vec3b>(i,j); //ָ�������
				show_image.at<Vec3b>(p[i*test_image2.cols + j].x, p[i*test_image2.cols + j].y) = pix;
			}
		}
	}
	imshow("corrected2", show_image);
	waitKey(3000);
	*/
}

//���������,����ͼ��
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

			//���Ƚ�������ת����
			double xDistortion = (i - ux) / fx;
			double yDistortion = (j - uy) / fy;

			double xCorrected, yCorrected;

			double x0 = xDistortion;
			double y0 = yDistortion;

			//����ʹ�õ����ķ�ʽ������⣬��Ϊ����2�еĹ�ʽֱ����������ѵģ�����ͨ���趨��ֵ���е�������Ҳ��OpenCV�������ԣ�
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

			//��������任��
			xCorrected = xCorrected * fx + ux;
			yCorrected = yCorrected * fy + uy;

			dst.push_back(cv::Point2d((int)(xCorrected+0.5), (int)(yCorrected+0.5)));
		}
	}

}