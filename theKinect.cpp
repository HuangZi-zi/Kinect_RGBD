#include "theKinect.h"
#include <vector>
#include "main.h"

c_theKinect::c_theKinect():
	m_pKinectSensor(NULL),
	pDepthFrameSource(NULL),
	m_pDepthFrameReader(NULL),
	depthFrameDescription(NULL),
	nDepthWidth(0),
	nDepthHeight(0),
	pColorFrameSource(NULL),
	m_pColorFrameReader(NULL),
	colorFrameDescription(NULL),
	nColorWidth(0),
	nColorHeight(0),
	coordinateMapper(NULL)
{
	MatRGB.create(1080, 1920, CV_8UC4);
	MatDepth8.create(424, 512, CV_8UC1);
	MatDepth16.create(424, 512, CV_16UC1);
	MatDepthToColor1.create(424, 512, CV_8UC4);
	hr = GetDefaultKinectSensor(&m_pKinectSensor);
	if (m_pKinectSensor) {
		hr = m_pKinectSensor->Open();
	}
	else {
		cout << "��ȡKinect�豸ʧ��" << endl;
		return;
	}
	if (SUCCEEDED(hr)) {
		//��������Ϣ������
		hr = m_pKinectSensor->get_DepthFrameSource(&pDepthFrameSource);
		//�������Ϣ֡��ȡ��
		hr = pDepthFrameSource->OpenReader(&m_pDepthFrameReader);
	}
	if (SUCCEEDED(hr)) {
		//��ò�ɫ��Ϣ������  
		hr = m_pKinectSensor->get_ColorFrameSource(&pColorFrameSource);
		//�򿪲�ɫ��Ϣ֡��ȡ��  
		hr = pColorFrameSource->OpenReader(&m_pColorFrameReader);
	}

}

bool c_theKinect::GetAndShowDepthData()
{
	IDepthFrame* pDepthFrame = NULL;//�����Ϣ����
	//cout << "Waiting\n" << endl;
	while (pDepthFrame == NULL)
		hr = m_pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);//��ȡ�������
	//pDepthFrame->get_FrameDescription(&depthFrameDescription);
	//depthFrameDescription->get_Height(&nDepthHeight);
	//depthFrameDescription->get_Width(&nDepthWidth);

	//cout << "start!\n" << endl;

		hr = pDepthFrame->get_FrameDescription(&depthFrameDescription);

		if (SUCCEEDED(hr))
		{
			hr = depthFrameDescription->get_Width(&nDepthWidth);
		}

		if (SUCCEEDED(hr))
		{
			hr = depthFrameDescription->get_Height(&nDepthHeight);
		}

		if (SUCCEEDED(hr))
			hr = pDepthFrame->CopyFrameDataToArray(nDepthHeight * nDepthWidth, reinterpret_cast<UINT16*>(MatDepth16.data));
		
		UINT16* depthData = new UINT16[512 * 424];
		hr = pDepthFrame->CopyFrameDataToArray(nDepthHeight * nDepthWidth, depthData);
		for (int i = 0; i < nDepthHeight * nDepthWidth; i++) {
			BYTE intensity = static_cast<BYTE>(depthData[i] / 256);
			reinterpret_cast<BYTE*>(MatDepth8.data)[i] = intensity;
		}
		equalizeHist(MatDepth8, MatDepth8);
		equalizeHist(MatDepth16, MatDepth16);
		imshow("MatDepth8", MatDepth8);
		imshow("MatDepth16", MatDepth16);
		pDepthFrame->Release();
		if (waitKey(1) == VK_ESCAPE)
			return false;
		delete[] depthData;
		return true;
	
}

bool c_theKinect::GetAndShowColorData()
{
	IColorFrame* m_pColorFrame = NULL;//��ɫ��Ϣ����
	while (m_pColorFrame == NULL)
		hr = m_pColorFrameReader->AcquireLatestFrame(&m_pColorFrame);//��ȡ��ɫ����
	m_pColorFrame->get_FrameDescription(&colorFrameDescription);
	colorFrameDescription->get_Height(&nColorHeight);
	colorFrameDescription->get_Width(&nColorWidth);
	UINT nColorBufferSize = nColorHeight * nColorWidth * 4;
	if (SUCCEEDED(hr))
		hr = m_pColorFrame->CopyConvertedFrameDataToArray(nColorBufferSize, reinterpret_cast<BYTE*>(MatRGB.data), ColorImageFormat::ColorImageFormat_Bgra);
	Mat MatRGB_resize = MatRGB.clone();       // ��С���㿴
	cv::resize(MatRGB_resize, MatRGB_resize, Size(960, 540));
	//putText(MatRGB_resize, "cdn", Point(100, 100), CV_FONT_HERSHEY_COMPLEX, 1, Scalar(0, 0, 0), 5, 8);
	imshow("MatRGB_resize", MatRGB_resize);
	//Mat channel[4];
	//split(MatRGB_resize, channel);
	setMouseCallback("MatRGB_resize", onMouse, 0);
	if (flag) {
		flag = false;
		cout << MatRGB_resize.at<Vec4b>(ppp) << endl;
		//printf("%d", channel[1].at<uchar>(ppp));
		//cout << channel[0].at<uchar>(ppp)<< endl;
	}
	//cvtColor(MatRGB_resize, MatRGB_resize, COLOR_RGB2YCrCb);
	//imshow("B", channel[0]);
	//imshow("G", channel[1]);
	//imshow("R", channel[2]);
	//putText(MatRGB_resize, "The result is : 1", Point(50, 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 3, 8);
	imshow("MatRGB_resize", MatRGB_resize);
	//if (waitKey(27) == VK_SPACE)
	//	imwrite("E:/abc.bmp", MatRGB);
	m_pColorFrame->Release();
	return true;

}


c_theKinect::~c_theKinect()
{
	cv::destroyAllWindows();
	m_pKinectSensor->Close();
	std::system("pause");
}

void GetKinectAllData(void)
{

}

HRESULT  GetScreenshotFileName(_Out_writes_z_(nFilePathSize) LPWSTR lpszFilePath, UINT nFilePathSize) {
	WCHAR* pszKnownPath = NULL;
	HRESULT hr = SHGetKnownFolderPath(FOLDERID_Pictures, 0, NULL, &pszKnownPath);
	if (SUCCEEDED(hr))
	{
		// Get the time
		WCHAR szTimeString[MAX_PATH];
		GetTimeFormatEx(NULL, 0, NULL, L"hh'-'mm'-'ss", szTimeString, _countof(szTimeString));

		// File name will be KinectScreenshotColor-HH-MM-SS.bmp
		StringCchPrintfW(lpszFilePath, nFilePathSize, L"%s\\KinectScreenshot-Color-%s.bmp", pszKnownPath, szTimeString);
	}

	if (pszKnownPath)
	{
		CoTaskMemFree(pszKnownPath);
	}
	return hr;
}
Point ppp;
bool flag = false;

HRESULT SaveBitmapToFile(BYTE* pBitmapBits, LONG lWidth, LONG lHeight, WORD wBitsPerPixel, LPCWSTR lpszFilePath)
{
	return E_NOTIMPL;
}
void DrawLine(Mat& Img, const Joint& r1, const Joint& r2, ICoordinateMapper* pMapper)
{
	//�������ؽڵ������߶ε����ˣ����ҽ���״̬����
	if (r1.TrackingState == TrackingState_NotTracked || r2.TrackingState == TrackingState_NotTracked)
		return;
	//Ҫ�ѹؽڵ��õ�����������µĵ�ת���ɲ�ɫ�ռ�ĵ�
	ColorSpacePoint p1, p2;
	pMapper->MapCameraPointToColorSpace(r1.Position, &p1);
	pMapper->MapCameraPointToColorSpace(r2.Position, &p2);

	line(Img, Point(p1.X, p1.Y), Point(p2.X, p2.Y), Vec3b(0, 0, 255), 5);
}

void onMouse(int event, int x, int y, int flags, void* param)
{
	if (event == EVENT_LBUTTONDOWN)//������£���ȡ��ʼ���꣬����ͼ���ϸõ㴦��Բ  
	{
		flag = true;
		ppp.x = x;
		ppp.y = y;
		cout << ppp << endl;
	}
}