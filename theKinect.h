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
	IKinectSensor* m_pKinectSensor;  // ��ȡKinect�豸
	IDepthFrameSource* pDepthFrameSource;//��������Ϣ������
	IDepthFrameReader* m_pDepthFrameReader;//�������Ϣ֡��ȡ��
	IFrameDescription* depthFrameDescription;
	int nDepthWidth;
	int nDepthHeight;

	IColorFrameSource* pColorFrameSource;//��ò�ɫ��Ϣ������
	IColorFrameReader* m_pColorFrameReader;//�򿪲�ɫ��Ϣ֡��ȡ��  	
	IFrameDescription* colorFrameDescription;
	int nColorWidth;
	int nColorHeight;

	ICoordinateMapper* coordinateMapper;//��������ת��ָ��

	HRESULT hr;
	Joint aJoints[JointType_Count];

	Mat MatDepth8;
	Mat MatDepth16;
	Mat MatRGB;      //ע�⣺�������Ϊ4ͨ����ͼ��Kinect������ֻ����Bgra��ʽ����
	Mat MatDepthToColor1;
};

extern Point ppp;
extern bool flag;

void GetKinectAllData(void);

HRESULT  GetScreenshotFileName(_Out_writes_z_(nFilePathSize) LPWSTR lpszFilePath, UINT nFilePathSize);
HRESULT  SaveBitmapToFile(BYTE* pBitmapBits, LONG lWidth, LONG lHeight, WORD wBitsPerPixel, LPCWSTR lpszFilePath);
void DrawLine(Mat& Img, const Joint& r1, const Joint& r2, ICoordinateMapper* pMapper);
void onMouse(int event, int x, int y, int flags, void* param);
