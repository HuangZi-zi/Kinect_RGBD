// PCL_Template.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <thread>
#include <time.h>
#include <string.h>

#include "pcl/io/pcd_io.h"
#include <pcl/point_types.h>
#include <pcl/common/impl/io.hpp>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
//#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/voxel_grid.h>
#include "my_region_growing.h"

#include "Kinect.h"
#include <opencv2/opencv.hpp>

#include "main.h"

HRESULT resultc;
HRESULT resultd;

IKinectSensor* Kinect;

//int colorHeight = 0, colorWidth = 0;
//int depthHeight = 0, depthWidth = 0;

IMultiSourceFrameReader* reader;   // Kinect data source
//IDepthFrameSource* depthSource = nullptr;   //�������Դ
//IColorFrameSource* colorSource = nullptr;	//RGB����Դ
//IDepthFrameReader* depthReader = nullptr;	//������ݵ�Reader
//IColorFrameReader* colorReader = nullptr;	//RGB���ݵ�Reader
IMultiSourceFrame* frame = NULL;
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
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

//ɫ������
static std::vector<unsigned char> colors;

void initKinect() {
    //��ȡKinect
    resultc = GetDefaultKinectSensor(&Kinect);
    if (SUCCEEDED(resultc))
        std::cout << "�ɹ���ȡ�豸" << std::endl;
    else
        std::cout << "�޷���ȡ�豸" << std::endl;
    //��Kinect
    resultc = Kinect->Open();
	Kinect->OpenMultiSourceFrameReader(
		FrameSourceTypes::FrameSourceTypes_Depth | FrameSourceTypes::FrameSourceTypes_Color,
		&reader);
    if (SUCCEEDED(resultc))
        std::cout << "�ɹ����豸" << std::endl;
    else
        std::cout << "�޷����豸" << std::endl;
    //��ȡ����Դ
    //resultc = Kinect->get_DepthFrameSource(&depthSource);
    //resultd = Kinect->get_ColorFrameSource(&colorSource);
    //if (SUCCEEDED(resultc) && SUCCEEDED(resultd))
    //    std::cout << "�ɹ���ȡ����Դ" << std::endl;
    //else
    //    std::cout << "��ȡ����Դʧ��" << std::endl;

	

 //   //���RGB���ݵķֱ���(��ʼ��)
 //   IFrameDescription* colorDescription = nullptr;
 //   colorSource->get_FrameDescription(&colorDescription);
 //   colorDescription->get_Height(&colorHeight);
 //   colorDescription->get_Width(&colorWidth);
 //   colorDescription->Release();	//�ͷ�
 //   colorSource->OpenReader(&colorReader);
 //   //ȡ��������ݵķֱ���(��ʼ��)
	//IDepthFrameReference* frameref = NULL;
	//frame->get_DepthFrameReference(&frameref);
	//frameref->AcquireFrame(&depthframe);
	//if (frameref) frameref->Release();
 //   IFrameDescription* myDescription = nullptr;
 //   depthSource->get_FrameDescription(&myDescription);
 //   myDescription->get_Height(&depthHeight);
 //   myDescription->get_Width(&depthWidth);
 //   myDescription->Release();
 //   depthSource->OpenReader(&depthReader);    //��������ݵ�Reader
    //���ͼ���ʼ��
    color_img = cv::Mat(colorHeight, colorWidth, CV_8UC4);	//RGB
    depth_img = cv::Mat(depthHeight, depthWidth, CV_8UC1);	//���(���ӻ�)
    temp = cv::Mat(depthHeight, depthWidth, CV_16UC1);    //���
    depthToRGB = cv::Mat(depthHeight, depthWidth, CV_8UC4);	//�����ͼ��4ͨ����
    depthRGB = cv::Mat(depthHeight, depthWidth, CV_8UC3);	//�����ͼ��3ͨ����

    Kinect->get_CoordinateMapper(&CoordinateMapper);	//��ȡ����ӳ����

	////��¼ʱ���
	//auto start = std::chrono::steady_clock::now();

	while (!SUCCEEDED(reader->AcquireLatestFrame(&frame)))
	{
		cv::waitKey(20);
	}
	cv::waitKey(20);
	while (!SUCCEEDED(reader->AcquireLatestFrame(&frame)))//�ظ��жϣ������֡
	{
		cv::waitKey(20);
	}
	//depthFrame->Release();
	////colorFrame->Release();
	frame->Release();
	std::cout << "initialized!" << endl;
}

void getPointCloud()
{
	//��¼ʱ���
	auto start = std::chrono::steady_clock::now();

	reader->AcquireLatestFrame(&frame);
	//��ȡ���ͼ
	IDepthFrameReference* framerefd = NULL;
	frame->get_DepthFrameReference(&framerefd);
	framerefd->AcquireFrame(&depthFrame);
	if (framerefd) framerefd->Release();
	if (depthFrame) { //ͨ��Reader���Ի�ȡ���µ�һ֡������ݣ��������֡��,���ж��Ƿ�ɹ���ȡ
		depthFrame->CopyFrameDataToArray(424 * 512, depthData);
		depthFrame->CopyFrameDataToArray(depthHeight * depthWidth, (UINT16*)temp.data); //�Ȱ����ݴ���16λ��ͼ�����
		temp.convertTo(depth_img, CV_8UC1, 255.0 / 4500);   //�ٰ�16λת��Ϊ8λ
		//imshow("Depth", depth_img);
		depthFrame->Release();
	}

	//��ȡRGBͼ
	IColorFrameReference* framerefc = NULL;
	frame->get_ColorFrameReference(&framerefc);
	framerefc->AcquireFrame(&colorFrame);
	if (framerefc) framerefc->Release();
	if (colorFrame) {
		colorFrame->CopyConvertedFrameDataToArray(colorHeight * colorWidth * 4, (BYTE*)color_img.data, ColorImageFormat::ColorImageFormat_Bgra);	//��ȡ��ɫͼ
		//imshow("RGB", color_img);
		colorFrame->Release();
	}

	frame->Release();
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

	//��XYZRGBת��XYZ
	pcl::copyPointCloud(*cloud, *cloud_xyz);

}

void randomPointCloud()
{
	// Fill in the cloud data
	cloud_xyz->width = 15;
	cloud_xyz->height = 1;
	cloud_xyz->points.resize(cloud_xyz->width * cloud_xyz->height);

	// Generate the data
	for (auto& point : *cloud_xyz)
	{
		point.x = 1024 * rand() / (RAND_MAX + 1.0f);
		point.y = 1024 * rand() / (RAND_MAX + 1.0f);
		point.z = 1.0;
	}

	// Set a few outliers
	(*cloud_xyz)[0].z = 2.0;
	(*cloud_xyz)[3].z = -2.0;
	(*cloud_xyz)[6].z = 4.0;
	std::cout << "generated" << std::endl;
}

void displayXYZPointCloud(pcl::visualization::PCLVisualizer::Ptr viewer, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, std::string name)
{
	//���ƿ��ӻ�
	if (cloud->points.size() != 0) {
		viewer->removeAllPointClouds();
		viewer->removeAllShapes();
		viewer->addPointCloud<pcl::PointXYZ>(cloud, name);  //��ʾԭʼ����
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.25, name);  //���õ�ߴ�
	}
	viewer->spinOnce();
}

void displayXYZRGBPointCloud(pcl::visualization::PCLVisualizer::Ptr viewer, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string name)
{
	//���ƿ��ӻ�
	if (cloud->points.size() != 0) {
		viewer->removeAllPointClouds();
		viewer->removeAllShapes();
		viewer->addPointCloud<pcl::PointXYZRGB>(cloud, name);  //��ʾԭʼ����
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.25, name);  //���õ�ߴ�
	}
	viewer->spinOnce();
}

int  main(int argc, char** argv)
{
	//��ʼ��Kinect
	initKinect();

	//������ʾ����
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("show origin"));
	viewer->setBackgroundColor(255, 255, 255);  //���ñ���
	//viewer->addCoordinateSystem(1, "Base_link");  //����������ߴ�
	viewer->initCameraParameters();

	//���Ʊ��湤��
	pcl::PCDWriter writer;
	pcl::PCDReader reader;

	//���ɫ��
	for (std::size_t i_segment = 0; i_segment < depthHeight*depthWidth; i_segment++)
	{
		colors.push_back(static_cast<unsigned char> (rand() % 256));
		colors.push_back(static_cast<unsigned char> (rand() % 256));
		colors.push_back(static_cast<unsigned char> (rand() % 256));
	}

	//randomPointCloud();

	//while (cloud_xyz->points.size() == 0)
	while (true)
	{
		getPointCloud();
		if (cloud_xyz->points.size() == 0)
		{
			cv::waitKey(100);
			continue;
		}

		//������
		// Create the filtering object
		pcl::VoxelGrid<pcl::PointXYZ> vg;
		vg.setInputCloud(cloud_xyz);
		vg.setLeafSize(0.01f, 0.01f, 0.01f);
		vg.filter(*cloud_filtered);

		//ȥ����Ⱥ��
		//// �����˲�������ÿ����������ٽ���ĸ�������Ϊ50 ��������׼��ı�������Ϊ1  ����ζ�����һ
		////����ľ��볬����ƽ������һ����׼�����ϣ���õ㱻���Ϊ��Ⱥ�㣬�������Ƴ����洢����
		pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;   //�����˲�������
		sor.setInputCloud(cloud_filtered);                           //���ô��˲��ĵ���
		sor.setMeanK(20);                               //�����ڽ���ͳ��ʱ���ǲ�ѯ���ٽ�����
		sor.setStddevMulThresh(1.0);                      //�����ж��Ƿ�Ϊ��Ⱥ��ķ�ֵ
		sor.filter(*cloud_filtered);                    //�洢
		
		//displayXYZPointCloud(viewer,cloud_filtered, "cloud_filtered");
		/*int key = cv::waitKey(1); 
		if (key != -1)
		{*/
			//std::string u_time = std::to_string(time(nullptr));
			//std::string filename1 = "C:/Users/YawnFun/Pictures/Camera Roll/snapshot_origin" + u_time + ".pcd";
			//std::string filename2 = "C:/Users/YawnFun/Pictures/Camera Roll/snapshot_filtered" + u_time + ".pcd";
			//writer.write<pcl::PointXYZ>(filename1, *cloud_xyz, false);
			//writer.write<pcl::PointXYZ>(filename2, *cloud_filtered, false);
			//std::cout << "snap succeed" << std::endl;
			//key = false;
		//}

		//������������ƽ��������ȡ
		//Create the normal estimation class, and pass the input dataset to it
		pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
		ne.setInputCloud(cloud_filtered);
		// Output datasets
		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
		// Create an empty kdtree representation, and pass it to the normal estimation object.
		// Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
		ne.setSearchMethod(tree);
		// Use all neighbors in a sphere
		ne.setRadiusSearch(0.10);
		// Compute the features
		ne.compute(*cloud_normals);

		pcl::IndicesPtr p_indices(new std::vector <int>);
		pcl::removeNaNFromPointCloud(*cloud_filtered, *p_indices);

		pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
		reg.setMinClusterSize(50);
		reg.setMaxClusterSize(1000000);
		reg.setSearchMethod(tree);
		reg.setNumberOfNeighbours(30);
		reg.setInputCloud(cloud_filtered);
		reg.setIndices(p_indices);//ȥ���������ݵĵ�ĵ���
		reg.setInputNormals(cloud_normals);
		reg.setSmoothnessThreshold(1.0 / 180.0 * M_PI);
		reg.setCurvatureThreshold(1.0);

		std::vector <pcl::PointIndices> clusters;
		reg.extract(clusters);
		pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.mgetColoredCloud();
		displayXYZRGBPointCloud(viewer, colored_cloud, "segmented");
	}

	//depthReader->Release();        //�ͷŲ��õı������ҹرո�Ӧ��
	//colorReader->Release();
	//colorSource->Release();
	//depthSource->Release();
	//Kinect->Close();
	//Kinect->Release();
	//while (!viewer->wasStopped())
	//{
	//	viewer->spinOnce(100);
	//	cv::waitKey(100);
	//}
	std::cin.get();
			return 0;
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
