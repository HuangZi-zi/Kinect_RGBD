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
//IDepthFrameSource* depthSource = nullptr;   //深度数据源
//IColorFrameSource* colorSource = nullptr;	//RGB数据源
//IDepthFrameReader* depthReader = nullptr;	//深度数据的Reader
//IColorFrameReader* colorReader = nullptr;	//RGB数据的Reader
IMultiSourceFrame* frame = NULL;
IColorFrame* colorFrame = nullptr;	//RGB帧数据
IDepthFrame* depthFrame = nullptr;		//深度帧数据
ICoordinateMapper* CoordinateMapper;	//坐标映射器
UINT16* depthData = new UINT16[424 * 512];	//深度数据
ColorSpacePoint* colorData = new ColorSpacePoint[424 * 512];	//对齐的彩色数据
DepthSpacePoint* color2DepthData = new DepthSpacePoint[1920 * 1080];	//对齐的深度数据
CameraSpacePoint* depth2Cam = new CameraSpacePoint[424 * 512];	//坐标系数据
//图像输出
cv::Mat color_img;	//RGB图像
cv::Mat color_flip;	//RGB翻转图像
cv::Mat temp;	//十六位深度图
cv::Mat depth_img;	//八位深度图
cv::Mat depthToRGB;	//深度与RGB对齐的图
cv::Mat depthRGB;

//点云输出
pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

//色彩种子
static std::vector<unsigned char> colors;

void initKinect() {
    //获取Kinect
    resultc = GetDefaultKinectSensor(&Kinect);
    if (SUCCEEDED(resultc))
        std::cout << "成功获取设备" << std::endl;
    else
        std::cout << "无法获取设备" << std::endl;
    //打开Kinect
    resultc = Kinect->Open();
	Kinect->OpenMultiSourceFrameReader(
		FrameSourceTypes::FrameSourceTypes_Depth | FrameSourceTypes::FrameSourceTypes_Color,
		&reader);
    if (SUCCEEDED(resultc))
        std::cout << "成功打开设备" << std::endl;
    else
        std::cout << "无法打开设备" << std::endl;
    //获取数据源
    //resultc = Kinect->get_DepthFrameSource(&depthSource);
    //resultd = Kinect->get_ColorFrameSource(&colorSource);
    //if (SUCCEEDED(resultc) && SUCCEEDED(resultd))
    //    std::cout << "成功获取数据源" << std::endl;
    //else
    //    std::cout << "获取数据源失败" << std::endl;

	

 //   //获得RGB数据的分辨率(初始化)
 //   IFrameDescription* colorDescription = nullptr;
 //   colorSource->get_FrameDescription(&colorDescription);
 //   colorDescription->get_Height(&colorHeight);
 //   colorDescription->get_Width(&colorWidth);
 //   colorDescription->Release();	//释放
 //   colorSource->OpenReader(&colorReader);
 //   //取得深度数据的分辨率(初始化)
	//IDepthFrameReference* frameref = NULL;
	//frame->get_DepthFrameReference(&frameref);
	//frameref->AcquireFrame(&depthframe);
	//if (frameref) frameref->Release();
 //   IFrameDescription* myDescription = nullptr;
 //   depthSource->get_FrameDescription(&myDescription);
 //   myDescription->get_Height(&depthHeight);
 //   myDescription->get_Width(&depthWidth);
 //   myDescription->Release();
 //   depthSource->OpenReader(&depthReader);    //打开深度数据的Reader
    //输出图像初始化
    color_img = cv::Mat(colorHeight, colorWidth, CV_8UC4);	//RGB
    depth_img = cv::Mat(depthHeight, depthWidth, CV_8UC1);	//深度(可视化)
    temp = cv::Mat(depthHeight, depthWidth, CV_16UC1);    //深度
    depthToRGB = cv::Mat(depthHeight, depthWidth, CV_8UC4);	//对齐的图（4通道）
    depthRGB = cv::Mat(depthHeight, depthWidth, CV_8UC3);	//对齐的图（3通道）

    Kinect->get_CoordinateMapper(&CoordinateMapper);	//获取坐标映射器

	////记录时间戳
	//auto start = std::chrono::steady_clock::now();

	while (!SUCCEEDED(reader->AcquireLatestFrame(&frame)))
	{
		cv::waitKey(20);
	}
	cv::waitKey(20);
	while (!SUCCEEDED(reader->AcquireLatestFrame(&frame)))//重复判断，避免假帧
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
	//记录时间戳
	auto start = std::chrono::steady_clock::now();

	reader->AcquireLatestFrame(&frame);
	//获取深度图
	IDepthFrameReference* framerefd = NULL;
	frame->get_DepthFrameReference(&framerefd);
	framerefd->AcquireFrame(&depthFrame);
	if (framerefd) framerefd->Release();
	if (depthFrame) { //通过Reader尝试获取最新的一帧深度数据，放入深度帧中,并判断是否成功获取
		depthFrame->CopyFrameDataToArray(424 * 512, depthData);
		depthFrame->CopyFrameDataToArray(depthHeight * depthWidth, (UINT16*)temp.data); //先把数据存入16位的图像矩阵
		temp.convertTo(depth_img, CV_8UC1, 255.0 / 4500);   //再把16位转换为8位
		//imshow("Depth", depth_img);
		depthFrame->Release();
	}

	//获取RGB图
	IColorFrameReference* framerefc = NULL;
	frame->get_ColorFrameReference(&framerefc);
	framerefc->AcquireFrame(&colorFrame);
	if (framerefc) framerefc->Release();
	if (colorFrame) {
		colorFrame->CopyConvertedFrameDataToArray(colorHeight * colorWidth * 4, (BYTE*)color_img.data, ColorImageFormat::ColorImageFormat_Bgra);	//获取彩色图
		//imshow("RGB", color_img);
		colorFrame->Release();
	}

	frame->Release();
	cv::flip(color_img, color_flip, 1);	//图片翻转
	//深度与RGB对齐
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
	//转点云
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

	//将XYZRGB转成XYZ
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
	//点云可视化
	if (cloud->points.size() != 0) {
		viewer->removeAllPointClouds();
		viewer->removeAllShapes();
		viewer->addPointCloud<pcl::PointXYZ>(cloud, name);  //显示原始点云
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.25, name);  //设置点尺寸
	}
	viewer->spinOnce();
}

void displayXYZRGBPointCloud(pcl::visualization::PCLVisualizer::Ptr viewer, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string name)
{
	//点云可视化
	if (cloud->points.size() != 0) {
		viewer->removeAllPointClouds();
		viewer->removeAllShapes();
		viewer->addPointCloud<pcl::PointXYZRGB>(cloud, name);  //显示原始点云
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.25, name);  //设置点尺寸
	}
	viewer->spinOnce();
}

int  main(int argc, char** argv)
{
	//初始化Kinect
	initKinect();

	//点云显示工具
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("show origin"));
	viewer->setBackgroundColor(255, 255, 255);  //设置背景
	//viewer->addCoordinateSystem(1, "Base_link");  //设置坐标轴尺寸
	viewer->initCameraParameters();

	//点云保存工具
	pcl::PCDWriter writer;
	pcl::PCDReader reader;

	//随机色彩
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

		//降采样
		// Create the filtering object
		pcl::VoxelGrid<pcl::PointXYZ> vg;
		vg.setInputCloud(cloud_xyz);
		vg.setLeafSize(0.01f, 0.01f, 0.01f);
		vg.filter(*cloud_filtered);

		//去除离群点
		//// 创建滤波器，对每个点分析的临近点的个数设置为50 ，并将标准差的倍数设置为1  这意味着如果一
		////个点的距离超出了平均距离一个标准差以上，则该点被标记为离群点，并将它移除，存储起来
		pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;   //创建滤波器对象
		sor.setInputCloud(cloud_filtered);                           //设置待滤波的点云
		sor.setMeanK(20);                               //设置在进行统计时考虑查询点临近点数
		sor.setStddevMulThresh(1.0);                      //设置判断是否为离群点的阀值
		sor.filter(*cloud_filtered);                    //存储
		
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

		//邻域生长法的平面特征提取
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
		reg.setIndices(p_indices);//去除了无数据的点的点云
		reg.setInputNormals(cloud_normals);
		reg.setSmoothnessThreshold(1.0 / 180.0 * M_PI);
		reg.setCurvatureThreshold(1.0);

		std::vector <pcl::PointIndices> clusters;
		reg.extract(clusters);
		pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = reg.mgetColoredCloud();
		displayXYZRGBPointCloud(viewer, colored_cloud, "segmented");
	}

	//depthReader->Release();        //释放不用的变量并且关闭感应器
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
