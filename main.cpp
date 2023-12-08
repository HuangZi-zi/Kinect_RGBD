// Kinect_Camera.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <kinect.h>
#include "main.h"
#include "theKinect.h"

Mat initgraph(Mat input)
{


	int M = input.rows;
	int N = input.cols;

	int W = 16;
	int H = 16;

	for (size_t i = 0; i < M/H; i++)
	{
		for (size_t j = 0; j < N/W; j++)
		{
			
			int k = (i)*H;
		}
	}
}

bool rejectedge

int main()
{
    c_theKinect kinect;
    std::cout << "Hello World!\n";
	while (true)
	{
		kinect.GetAndShowDepthData();
	}
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
