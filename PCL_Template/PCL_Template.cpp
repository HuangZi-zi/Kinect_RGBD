// PCL_Template.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <thread>
#include <time.h>
#include <string.h>
#include <mutex>
#include <iomanip> // for setw, setfill

#include <pcl/point_types.h>
//#include <vtkPlaneSource.h> 
#include <pcl/ModelCoefficients.h>

#include <pcl/common/impl/io.hpp>
#include <pcl/common/transforms.h>
#include<pcl/common/common_headers.h>

#include <pcl/console/parse.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/range_image_border_extractor.h>

#include <pcl/io/pcd_io.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

//#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/search/kdtree.h>

#include <pcl/range_image/range_image.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/range_image_visualizer.h>

#include "my_region_growing_rgb.h"

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
pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

//色彩种子
static std::vector<unsigned char> colors;

//点云显示工具
pcl::visualization::PCLVisualizer::Ptr p_viewer(new pcl::visualization::PCLVisualizer("show origin"));
//pcl::visualization::PCLVisualizer viewer("3D Viewer");

//深度图显示工具
//pcl::visualization::RangeImageVisualizer range_image_widget("Range image");
//pcl::visualization::RangeImageVisualizer range_image_borders_widget("with border");

std::mutex cloud_mutex;

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
	if (frame==nullptr)
	{
		std::cout << "empty frame! [read]" << std::endl;
		return;
	}
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

void region_growing(pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &out_cloud)
{
	//Create the normal estimation class, and pass the input dataset to it
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(in_cloud);
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
	pcl::removeNaNFromPointCloud(*in_cloud, *p_indices);

	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
	reg.setMinClusterSize(50);
	reg.setMaxClusterSize(1000000);
	reg.setSearchMethod(tree);
	reg.setNumberOfNeighbours(10);
	reg.setInputCloud(in_cloud);
	reg.setIndices(p_indices);//去除了无数据的点的点云
	reg.setInputNormals(cloud_normals);
	reg.setSmoothnessThreshold(1.0 / 180.0 * M_PI);
	reg.setCurvatureThreshold(0.5);

	std::vector <pcl::PointIndices> clusters;
	reg.extract(clusters);
	out_cloud = reg.getColoredCloud();
}

void region_growing_rgb(pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr& out_cloud)
{
	// Create the region growing RGB object
	pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
	reg.setInputCloud(in_cloud);
	reg.setMinClusterSize(100); // Set minimum cluster size (adjust as needed)
	reg.setMaxClusterSize(10000); // Set maximum cluster size (adjust as needed)
	reg.setSearchMethod(pcl::search::Search<pcl::PointXYZRGB>::Ptr(new pcl::search::KdTree<pcl::PointXYZRGB>));

	// Set the parameters for region growing
	reg.setNumberOfNeighbours(30);
	reg.setSmoothnessThreshold(3.0 / 180.0 * M_PI); // Set the smoothness threshold (adjust as needed)
	reg.setCurvatureThreshold(1.0); // Set the curvature threshold (adjust as needed)

	// Starting from the middle of the bottom
	cloud->width = depthWidth;
	cloud->height = depthHeight;
	pcl::PointXYZRGB seed_point = cloud->at(cloud->width / 2, 0);
	reg.setPointColorThreshold(10); // Set the color threshold (adjust as needed)
	reg.setRegionColorThreshold(5); // Set the region color threshold (adjust as needed)

	// Create a placeholder for normals (required for the region growing algorithm)
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
	// Assuming you have computed normals for your input cloud and stored them in 'normals'
	// You may need to replace the next line with the actual method to compute normals
	pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;
	normal_estimator.setInputCloud(cloud);
	normal_estimator.setSearchMethod(pcl::search::Search<pcl::PointXYZRGB>::Ptr(new pcl::search::KdTree<pcl::PointXYZRGB>));
	normal_estimator.setKSearch(50); // Set the number of neighbors to consider for normal estimation
	normal_estimator.compute(*normals);

	// Set the input normals to the region growing object
	reg.setInputNormals(normals);

	// Perform region growing
	std::vector<pcl::PointIndices> clusters;
	reg.extract(clusters);

	// Extract the ground cluster
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr ground(new pcl::PointCloud<pcl::PointXYZRGB>);
	for (const auto& cluster : clusters) {
		if (std::find(cluster.indices.begin(), cluster.indices.end(), cloud->width / 2) != cluster.indices.end()) {
			pcl::ExtractIndices<pcl::PointXYZRGB> extract;
			extract.setInputCloud(cloud);
			pcl::PointIndices::Ptr inliers(new pcl::PointIndices(cluster));
			extract.setIndices(inliers);
			extract.setNegative(false);
			extract.filter(*ground);
			break; // Assuming there's only one ground cluster containing the seed point
		}
	}
	displayXYZRGBPointCloud(p_viewer, ground, "name");
}

//void planar_segmentation()
//{
//	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
//	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
//	// Create the segmentation object
//	pcl::SACSegmentation<pcl::PointXYZ> seg;
//	// Optional
//	seg.setOptimizeCoefficients(true);
//	// Mandatory
//	seg.setModelType(pcl::SACMODEL_PLANE);
//	seg.setMethodType(pcl::SAC_RANSAC);
//	seg.setDistanceThreshold(0.05);
//
//	seg.setInputCloud(cloud_filtered);
//	seg.segment(*inliers, *coefficients);
//
//	if (inliers->indices.size() == 0)
//	{
//		PCL_ERROR("Could not estimate a planar model for the given dataset.\n");
//	}
//
//	float scale[2] = { 5,5 };//显示平面的大小
//
//	pcl::visualization::PCLVisualizer viewer("PCL visualizer");
//	viewer.addPlane(*coefficients, 3, 3, 3, "plane", 0);
//	viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.9, 0.1, 0.1, "plane_1", 0);
//	viewer.setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.6, "plane_1", 0);
//	viewer.addPointCloud(cloud_filtered);
//	viewer.setBackgroundColor(0.1, 0.1, 0.1, 0);
//	viewer.setPosition(800, 400);
//	while (!viewer.wasStopped())
//	{
//		viewer.spinOnce();
//	}
//}

void euclidean_cluster(pcl::visualization::PCLVisualizer::Ptr viewer)
{
	//cloud_filtered = cloud_xyz;
	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloud_temp(new pcl::PointCloud<pcl::PointXYZ>);

	// Create the segmentation object for the planar model and set all the parameters
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>());

	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(100);
	seg.setDistanceThreshold(0.02);

	int nr_points = (int)cloud_filtered->size();
	while (cloud_filtered->size() > 0.3 * nr_points)
	{
		// Segment the largest planar component from the remaining cloud
		seg.setInputCloud(cloud_filtered);
		seg.segment(*inliers, *coefficients);
		if (inliers->indices.size() == 0)
		{
			std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
			break;
		}

		// Extract the planar inliers from the input cloud
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud(cloud_filtered);
		extract.setIndices(inliers);
		extract.setNegative(false);

		// Get the points associated with the planar surface
		extract.filter(*cloud_plane);
		//std::cout << "PointCloud representing the planar component: " << cloud_plane->size() << " data points." << std::endl;

		// Remove the planar inliers, extract the rest
		extract.setNegative(true);
		extract.filter(*cloud_temp);
		*cloud_filtered = *cloud_temp;
	}

	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(cloud_filtered);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance(0.01); // 2cm
	ec.setMinClusterSize(500);
	ec.setMaxClusterSize(25000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(cloud_filtered);
	ec.extract(cluster_indices);
	//std::cout << endl;
	
	//写输出点云
	//int j = 0;
	//for (const auto& cluster : cluster_indices)
	//{
	//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
	//	for (const auto& idx : cluster.indices) {
	//		cloud_cluster->push_back((*cloud_filtered)[idx]);
	//	} //*
	//	cloud_cluster->width = cloud_cluster->size();
	//	cloud_cluster->height = 1;
	//	cloud_cluster->is_dense = true;

	//	std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size() << " data points." << std::endl;

	//	j++;
	//}

	//提取最大的cluster
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
	for (const auto& idx : cluster_indices[0].indices) 
	{
		cloud_cluster->push_back((*cloud_filtered)[idx]);
	} 
	//cloud_cluster->width = cloud_cluster->size();
	//cloud_cluster->height = 1;
	//cloud_cluster->is_dense = true;
	std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size() << " data points." << std::endl;
	displayXYZPointCloud(viewer, cloud_cluster, "cluster");
}

void filteringThread()
{
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
		vg.setLeafSize(0.05f, 0.05f, 0.05f);
		std::lock_guard<std::mutex> lock(cloud_mutex);
		vg.filter(*cloud_filtered);
		//去除离群点
		//// 创建滤波器，对每个点分析的临近点的个数设置为50 ，并将标准差的倍数设置为1  这意味着如果一
		////个点的距离超出了平均距离一个标准差以上，则该点被标记为离群点，并将它移除，存储起来
		pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;   //创建滤波器对象
		sor.setInputCloud(cloud_filtered);                           //设置待滤波的点云
		sor.setMeanK(40);                               //设置在进行统计时考虑查询点临近点数
		sor.setStddevMulThresh(1.0);                      //设置判断是否为离群点的阀值
		sor.filter(*cloud_filtered);                    //存储
	}
}

void featureExtractionThread()
{
	while (true)
	{
		if (cloud_filtered->points.size() == 0)
		{
			cv::waitKey(100);
			continue;
		}
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
		std::lock_guard<std::mutex> lock(cloud_mutex);
		colored_cloud = reg.getColoredCloud();
	}
}

void visualizationThread()
{
	while (true)
	{
		if (colored_cloud->points.size() == 0)
		{
			cv::waitKey(100);
			continue;
		}
		//点云显示工具
		pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("show origin"));
		viewer->setBackgroundColor(255, 255, 255);  //设置背景
		//viewer->addCoordinateSystem(1, "Base_link");  //设置坐标轴尺寸
		viewer->initCameraParameters();
		//点云可视化
		if (colored_cloud->points.size() != 0) {
			viewer->removeAllPointClouds();
			viewer->removeAllShapes();
			viewer->addPointCloud<pcl::PointXYZRGB>(colored_cloud, "name");  //显示原始点云
			viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.25, "name");  //设置点尺寸
		}
		viewer->spinOnce();
	}
	
}

//void borderExtraction()
//{
//	float angularResolution = (float)(1.0f * (M_PI / 180.0f));  //   1.0 degree in radians
//	float maxAngleWidth = (float)(180.0f * (M_PI / 180.0f));  // 360.0 degree in radians
//	float maxAngleHeight = (float)(180.0f * (M_PI / 180.0f));  // 180.0 degree in radians
//	Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(1.0f, 1.0f, 1.0f);
//	pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
//	float noiseLevel = 0.00;
//	float minRange = 0.0f;
//	int borderSize = 1;
//
//	pcl::RangeImage rangeImage;
//	rangeImage.createFromPointCloud(*cloud_filtered, angularResolution, maxAngleWidth, maxAngleHeight,
//		sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);
//
//	rangeImage.setUnseenToMaxRange();//没有数据的点认定为最大值
//	// Extract borders
//	pcl::RangeImageBorderExtractor border_extractor(&rangeImage);
//	pcl::PointCloud<pcl::BorderDescription> border_descriptions;
//	border_extractor.compute(border_descriptions);
//	// Show points in 3D viewer
//	/*pcl::PointCloud<pcl::PointWithRange>::Ptr border_points_ptr(new pcl::PointCloud<pcl::PointWithRange>),
//		veil_points_ptr(new pcl::PointCloud<pcl::PointWithRange>),
//		shadow_points_ptr(new pcl::PointCloud<pcl::PointWithRange>);
//	pcl::PointCloud<pcl::PointWithRange>& border_points = *border_points_ptr,
//		& veil_points = *veil_points_ptr,
//		& shadow_points = *shadow_points_ptr;
//	for (int y = 0; y < (int)range_image.height; ++y)
//	{
//		for (int x = 0; x < (int)range_image.width; ++x)
//		{
//			if (border_descriptions[y * range_image.width + x].traits[pcl::BORDER_TRAIT__OBSTACLE_BORDER])
//				border_points.push_back(range_image[y * range_image.width + x]);
//			if (border_descriptions[y * range_image.width + x].traits[pcl::BORDER_TRAIT__VEIL_POINT])
//				veil_points.push_back(range_image[y * range_image.width + x]);
//			if (border_descriptions[y * range_image.width + x].traits[pcl::BORDER_TRAIT__SHADOW_BORDER])
//				shadow_points.push_back(range_image[y * range_image.width + x]);
//		}
//	}
//	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> border_points_color_handler(border_points_ptr, 0, 255, 0);
//	viewer.addPointCloud<pcl::PointWithRange>(border_points_ptr, border_points_color_handler, "border points");
//	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "border points");
//	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> veil_points_color_handler(veil_points_ptr, 255, 0, 0);
//	viewer.addPointCloud<pcl::PointWithRange>(veil_points_ptr, veil_points_color_handler, "veil points");
//	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "veil points");
//	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> shadow_points_color_handler(shadow_points_ptr, 0, 255, 255);
//	viewer.addPointCloud<pcl::PointWithRange>(shadow_points_ptr, shadow_points_color_handler, "shadow points");
//	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "shadow points");*/
//
//	range_image_borders_widget.visualizeBorders(rangeImage, -std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity(), false,
//		border_descriptions);
//
//	range_image_borders_widget.spinOnce();
//	
//}

int  main(int argc, char** argv)
{
	//初始化Kinect
	initKinect();

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

	//点云可视化设置
	p_viewer->setBackgroundColor(0.5, 0.5, 0.5);  //设置背景
	p_viewer->addCoordinateSystem(1, "Base_link");  //设置坐标轴尺寸
	//p_viewer->initCameraParameters();
	//viewer.setBackgroundColor(1, 1, 1);
	
	//randomPointCloud();

	//while (cloud_xyz->points.size() == 0)
	while (true)
	{
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

		getPointCloud();

		if (cloud_xyz->points.size() == 0)
		{
			std::cout << "empty frame! [main]" << endl;
			cv::waitKey(100);
			continue;
		}
		//displayXYZRGBPointCloud(p_viewer, cloud, "name");
		//std::cout << "size of point cloud: " << cloud_xyz->points.size() << std::endl;
		
		//降采样
		// Create the filtering object
		//pcl::VoxelGrid<pcl::PointXYZ> vg;
		//vg.setInputCloud(cloud_xyz);
		//vg.setLeafSize(0.05f, 0.05f, 0.05f);
		//vg.filter(*cloud_filtered);
		////去除离群点
		//// 创建滤波器，对每个点分析的临近点的个数设置为50 ，并将标准差的倍数设置为1  这意味着如果一
		////个点的距离超出了平均距离一个标准差以上，则该点被标记为离群点，并将它移除，存储起来
		//pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;   //创建滤波器对象
		//sor.setInputCloud(cloud_filtered);                           //设置待滤波的点云
		//sor.setMeanK(10);                               //设置在进行统计时考虑查询点临近点数
		//sor.setStddevMulThresh(3.0);                      //设置判断是否为离群点的阀值
		//sor.filter(*cloud_filtered);                    //存储

		//邻域生长法的平面特征提取
		//region_growing(cloud_filtered, colored_cloud);
		//std::cout << "size of outcome: " << colored_cloud->width * colored_cloud->height<<std::endl;
		//displayXYZRGBPointCloud(viewer, colored_cloud, "segmented");
		//std::cout << colored_cloud->at(1, 1)<<endl;
		region_growing_rgb(cloud, colored_cloud);

		//平面拟合
		//planar_segmentation();

		//聚类分析
		//euclidean_cluster(p_viewer);

		//displayXYZPointCloud(viewer, cloud_filtered, "filtered");

		//点云转为深度图
		// We now want to create a range image from the above point cloud, with a 1deg angular resolution
		//float angularResolution = (float)(0.5f * (M_PI / 180.0f));  //   1.0 degree in radians
		//float maxAngleWidth = (float)(70.0f * (M_PI / 180.0f));  // 360.0 degree in radians
		//float maxAngleHeight = (float)(60.0f * (M_PI / 180.0f));  // 180.0 degree in radians
		//Eigen::Affine3f sensorPose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
		//pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME;
		//float noiseLevel = 0.00;
		//float minRange = 0.0f;
		//int borderSize = 1;

		//pcl::RangeImage::Ptr p_rangeImage(new pcl::RangeImage);
		//pcl::RangeImage& rangeImage= *p_rangeImage;
		//rangeImage.createFromPointCloud(*cloud_xyz, angularResolution, maxAngleWidth, maxAngleHeight,
		//	sensorPose, coordinate_frame, noiseLevel, minRange, borderSize);

		////以图像的形式显示深度图像，深度值作为颜色显示
		//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointWithRange> range_image_color_handler(p_rangeImage, 0, 0, 0);
		//
		////range_image_widget.showRangeImage(rangeImage);
		////range_image_widget.spinOnce();
		//
		//Eigen::Vector3f pos_vector = rangeImage.getTransformationToWorldSystem() * Eigen::Vector3f(0, 0, 0);
		//Eigen::Vector3f look_at_vector = rangeImage.getTransformationToWorldSystem().rotation() * Eigen::Vector3f(0, 0, 1) + pos_vector;
		//Eigen::Vector3f up_vector = rangeImage.getTransformationToWorldSystem().rotation() * Eigen::Vector3f(0, -1, 0);

		//viewer.setCameraPosition(
		//	pos_vector[0], pos_vector[1], pos_vector[2],
		//	look_at_vector[0], look_at_vector[1], look_at_vector[2],
		//	up_vector[0], up_vector[1], up_vector[2]);

		//if (rangeImage.size() != 0) {
		//	viewer.removeAllPointClouds();
		//	viewer.removeAllShapes();
		//	viewer.addPointCloud(p_rangeImage, range_image_color_handler, "range image");
		//	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "range image");
		//}
		//viewer.spinOnce();

		//cout << "range Image refresh!" << endl;

		//边缘检测
		//borderExtraction();
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


	//std::thread filtering_thread(filteringThread);
	//std::thread feature_extraction_thread(featureExtractionThread);
	//std::thread visualization_thread(visualizationThread);

	//filtering_thread.join();
	//feature_extraction_thread.join();
	//visualization_thread.join();

	//std::cin.get();
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

