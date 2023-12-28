// PCL_Template.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include "pcl/io/pcd_io.h"
#include <pcl/point_types.h>

#include "Kinect.h"
#include <opencv2/opencv.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

HRESULT resultc;
HRESULT resultd;

IKinectSensor* Kinect;

int colorHeight = 0, colorWidth = 0;
int depthHeight = 0, depthWidth = 0;

IDepthFrameSource* depthSource = nullptr;   //�������Դ
IColorFrameSource* colorSource = nullptr;	//RGB����Դ
IDepthFrameReader* depthReader = nullptr;	//������ݵ�Reader
IColorFrameReader* colorReader = nullptr;	//RGB���ݵ�Reader
IColorFrame* colorFrame = nullptr;	//RGB֡����
IDepthFrame* depthFrame = nullptr;		//���֡����
ICoordinateMapper* CoordinateMapper;	//����ӳ����
UINT16* depthData = new UINT16[424 * 512];	//�������
ColorSpacePoint* colorData = new ColorSpacePoint[424 * 512];	//����Ĳ�ɫ����
DepthSpacePoint* color2DepthData = new DepthSpacePoint[1920 * 1080];	//������������
CameraSpacePoint* depth2Cam = new CameraSpacePoint[424 * 512];	//����ϵ����
//ͼ�����
cv::Mat color_img;	//RGBͼ��
cv::Mat color_flip;	//RGB��תͼ��
cv::Mat temp;	//ʮ��λ���ͼ
cv::Mat depth_img;	//��λ���ͼ
cv::Mat depthToRGB;	//�����RGB�����ͼ
cv::Mat depthRGB;

//�������
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;

void initKinect() {
    //��ȡKinect
    resultc = GetDefaultKinectSensor(&Kinect);
    if (SUCCEEDED(resultc))
        std::cout << "�ɹ���ȡ�豸" << std::endl;
    else
        std::cout << "�޷���ȡ�豸" << std::endl;
    //��Kinect
    resultc = Kinect->Open();
    if (SUCCEEDED(resultc))
        std::cout << "�ɹ����豸" << std::endl;
    else
        std::cout << "�޷����豸" << std::endl;
    //��ȡ����Դ
    resultc = Kinect->get_DepthFrameSource(&depthSource);
    resultd = Kinect->get_ColorFrameSource(&colorSource);
    if (SUCCEEDED(resultc) && SUCCEEDED(resultd))
        std::cout << "�ɹ���ȡ����Դ" << std::endl;
    else
        std::cout << "��ȡ����Դʧ��" << std::endl;
    //��������PS:ò�Ʋ��Գ���HRESULTûʲô���ã��������Ͳ��ж��ˣ�������
    //���RGB���ݵķֱ���(��ʼ��)
    IFrameDescription* colorDescription = nullptr;
    colorSource->get_FrameDescription(&colorDescription);
    colorDescription->get_Height(&colorHeight);
    colorDescription->get_Width(&colorWidth);
    colorDescription->Release();	//�ͷ�
    colorSource->OpenReader(&colorReader);
    //ȡ��������ݵķֱ���(��ʼ��)
    IFrameDescription* myDescription = nullptr;
    depthSource->get_FrameDescription(&myDescription);
    myDescription->get_Height(&depthHeight);
    myDescription->get_Width(&depthWidth);
    myDescription->Release();
    depthSource->OpenReader(&depthReader);    //��������ݵ�Reader
    //���ͼ���ʼ��
    color_img = cv::Mat(colorHeight, colorWidth, CV_8UC4);	//RGB
    depth_img = cv::Mat(depthHeight, depthWidth, CV_8UC1);	//���(���ӻ�)
    temp = cv::Mat(depthHeight, depthWidth, CV_16UC1);    //���
    depthToRGB = cv::Mat(depthHeight, depthWidth, CV_8UC4);	//�����ͼ��4ͨ����
    depthRGB = cv::Mat(depthHeight, depthWidth, CV_8UC3);	//�����ͼ��3ͨ����

    Kinect->get_CoordinateMapper(&CoordinateMapper);	//��ȡ����ӳ����
    //��ʼ���������
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud = cloudPtr;
}


int  main(int argc, char** argv)
{
	//��ʼ��Kinect
	initKinect();
	//��ʼ��������ʾ����
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("show"));
	viewer->setBackgroundColor(0.5, 0.5, 0.5);  //���ñ���
	viewer->addCoordinateSystem(1, "Base_link");  //����������ߴ�
	//��¼ʱ���
	auto start = std::chrono::steady_clock::now();
	while (true) {
		//��ȡ���ͼ
		if (depthReader->AcquireLatestFrame(&depthFrame) == S_OK) { //ͨ��Reader���Ի�ȡ���µ�һ֡������ݣ��������֡��,���ж��Ƿ�ɹ���ȡ
			depthFrame->CopyFrameDataToArray(424 * 512, depthData);
			depthFrame->CopyFrameDataToArray(depthHeight * depthWidth, (UINT16*)temp.data); //�Ȱ����ݴ���16λ��ͼ�����
			temp.convertTo(depth_img, CV_8UC1, 255.0 / 4500);   //�ٰ�16λת��Ϊ8λ
			imshow("Depth", depth_img);
			depthFrame->Release();
		}
		//��ȡRGBͼ
		if (colorReader->AcquireLatestFrame(&colorFrame) == S_OK) {
			colorFrame->CopyConvertedFrameDataToArray(colorHeight * colorWidth * 4, (BYTE*)color_img.data, ColorImageFormat::ColorImageFormat_Bgra);	//��ȡ��ɫͼ
			imshow("RGB", color_img);
			colorFrame->Release();
		}
		cv::flip(color_img, color_flip, 1);	//ͼƬ��ת
		//�����RGB����
		CoordinateMapper->MapColorFrameToDepthSpace(512 * 424, reinterpret_cast<UINT16*>(temp.data), 1920 * 1080, color2DepthData);
		CoordinateMapper->MapDepthFrameToColorSpace(512 * 424, reinterpret_cast<UINT16*>(temp.data), 512 * 424, colorData);
		for (int i = 0; i < 512 * 424; i++) {
			ColorSpacePoint p = colorData[i];
			if (p.X != -std::numeric_limits<float>::infinity() && p.Y != -std::numeric_limits<float>::infinity())
			{
				int colorX = static_cast<int>(p.X);
				int colorY = static_cast<int>(p.Y);
				if ((colorX >= 0 && colorX < 1920) && (colorY >= 0 && colorY < 1080))
				{
					depthToRGB.data[i * 4] = color_img.data[(colorY * 1920 + colorX) * 4];
					depthToRGB.data[i * 4 + 1] = color_img.data[(colorY * 1920 + colorX) * 4 + 1];
					depthToRGB.data[i * 4 + 2] = color_img.data[(colorY * 1920 + colorX) * 4 + 2];
					depthToRGB.data[i * 4 + 3] = color_img.data[(colorY * 1920 + colorX) * 4 + 3];
					depthRGB.data[i * 3] = color_img.data[(colorY * 1920 + colorX) * 4];
					depthRGB.data[i * 3 + 1] = color_img.data[(colorY * 1920 + colorX) * 4 + 1];
					depthRGB.data[i * 3 + 2] = color_img.data[(colorY * 1920 + colorX) * 4 + 2];
				}
			}
			else {
				depthRGB.data[i * 3] = 0;
				depthRGB.data[i * 3 + 1] = 0;
				depthRGB.data[i * 3 + 2] = 0;
			}
		}
		//ת����
		CoordinateMapper->MapDepthFrameToCameraSpace(512 * 424, reinterpret_cast<UINT16*>(temp.data), 512 * 424, depth2Cam);
		cloud->width = 512 * 424;
		cloud->height = 1;
		cloud->is_dense = false;
		cloud->points.resize(cloud->width * cloud->height);
		for (size_t i = 0; i < 512; i++) {
			for (size_t j = 0; j < 424; j++) {
				pcl::PointXYZRGB pointTemp;
				if (depth2Cam[i + j * 512].Z > 0.5 && depth2Cam[i + j * 512].Z < 5) {
					pointTemp.x = depth2Cam[i + j * 512].X;
					pointTemp.y = depth2Cam[i + j * 512].Y;
					pointTemp.z = depth2Cam[i + j * 512].Z;
					int X = static_cast<int>(colorData[j * 512 + i].X);
					int Y = static_cast<int>(colorData[j * 512 + i].Y);
					if (X > 0 && Y > 0 && X < 1920 && Y < 1080)
					{
						cv::Vec4b* pixelsRGBImage = color_img.ptr<cv::Vec4b>(Y);
						pointTemp.g = pixelsRGBImage[X][0];
						pointTemp.b = pixelsRGBImage[X][1];
						pointTemp.r = pixelsRGBImage[X][2];
						cloud->points.push_back(pointTemp);
					}
					else continue;
				}
			}
		}
		//���ƿ��ӻ�
		if (cloud->points.size() != 0) {
			viewer->removeAllPointClouds();
			viewer->removeAllShapes();
			viewer->addPointCloud<pcl::PointXYZRGB>(cloud, "origin_cloud");  //��ʾԭʼ����
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.25, "origin_cloud");  //���õ�ߴ�
		}
		viewer->spinOnce();
	}
	depthReader->Release();        //�ͷŲ��õı������ҹرո�Ӧ��
	colorReader->Release();
	colorSource->Release();
	depthSource->Release();
	Kinect->Close();
	Kinect->Release();
}



// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file