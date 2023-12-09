#pragma once

#include "main.h"

class c_theKinect
{
public:
	c_theKinect();
	bool GetAndShowDepthData();
	bool GetAndShowColorData();
	~c_theKinect();
private:
	IKinectSensor* m_pKinectSensor;  // 获取Kinect设备
	IDepthFrameSource* pDepthFrameSource;//获得深度信息传感器
	IDepthFrameReader* m_pDepthFrameReader;//打开深度信息帧读取器
	IFrameDescription* depthFrameDescription;
	int nDepthWidth;
	int nDepthHeight;

	IColorFrameSource* pColorFrameSource;//获得彩色信息传感器
	IColorFrameReader* m_pColorFrameReader;//打开彩色信息帧读取器  	
	IFrameDescription* colorFrameDescription;
	int nColorWidth;
	int nColorHeight;

	ICoordinateMapper* coordinateMapper;//坐标数据转换指针

	HRESULT hr;
	Joint aJoints[JointType_Count];

	Mat MatDepth8;
	Mat MatDepth16;
	Mat MatRGB;      //注意：这里必须为4通道的图，Kinect的数据只能以Bgra格式传出
	Mat MatDepthToColor1;
};

extern Point ppp;
extern bool flag;

void GetKinectAllData(void);

HRESULT  GetScreenshotFileName(_Out_writes_z_(nFilePathSize) LPWSTR lpszFilePath, UINT nFilePathSize);
HRESULT  SaveBitmapToFile(BYTE* pBitmapBits, LONG lWidth, LONG lHeight, WORD wBitsPerPixel, LPCWSTR lpszFilePath);
void DrawLine(Mat& Img, const Joint& r1, const Joint& r2, ICoordinateMapper* pMapper);
void onMouse(int event, int x, int y, int flags, void* param);
