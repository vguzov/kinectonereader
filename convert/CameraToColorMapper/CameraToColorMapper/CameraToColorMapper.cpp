#include <string>
#include <vector>
#include <iostream>
#include <windows.h>
#include "stdafx.h"
#include <Kinect.h>
#include <Shlobj.h>
#include <sys/stat.h>
#include "cnpy/cnpy.h"
typedef float vert_t;
typedef std::pair<vert_t*, bool*> depthmap;
typedef std::pair<CameraSpacePoint*, bool*> pointcloud;
static const int        cDepthWidth = 512;
static const int        cDepthHeight = 424;
ICoordinateMapper*      m_pCoordinateMapper;
IKinectSensor*          m_pKinectSensor = NULL;
IColorFrameReader*      m_pColorFrameReader = NULL;
inline bool file_exist(const std::string& name) {
	struct stat buffer;
	return (stat(name.c_str(), &buffer) == 0);
}
void SetStatusMessage(char *message)
{
	printf("%s", message);
}
int CreateMapper()
{
	HRESULT hr;
	IColorFrameSource* pColorFrameSource = NULL;
	hr = GetDefaultKinectSensor(&m_pKinectSensor);
	if (FAILED(hr))
	{
		return hr;
	}

	if (m_pKinectSensor)
	{

		hr = m_pKinectSensor->Open();

		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->get_ColorFrameSource(&pColorFrameSource);
		}
		if (SUCCEEDED(hr))
		{
			hr = pColorFrameSource->OpenReader(&m_pColorFrameReader);
		}
		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->get_CoordinateMapper(&m_pCoordinateMapper);
		}
		SafeRelease(pColorFrameSource);
	}

	if (!m_pKinectSensor || FAILED(hr))
	{
		SetStatusMessage("No ready Kinect found!");
		return E_FAIL;
	}

	return hr;
}
inline float fillnan(float x)
{
	if (x != x)
		return 0;
	else
		return x;
}
int convert_to_colorspace(cnpy::NpyArray arr, float **converted_arr)
{
	int ptscount = arr.shape[0];
	CameraSpacePoint *camerapts = new CameraSpacePoint[ptscount];
	ColorSpacePoint *colorpts = new ColorSpacePoint[ptscount];
	float *out_arr = new float[ptscount * 2];
	for (int i = 0; i < ptscount; i++)
	{
		camerapts[i].X = reinterpret_cast<double *>(arr.data)[i * 3];
		camerapts[i].Y = reinterpret_cast<double *>(arr.data)[i * 3 + 1];
		camerapts[i].Z = reinterpret_cast<double *>(arr.data)[i * 3 + 2];
	}
	m_pCoordinateMapper->MapCameraPointsToColorSpace(ptscount, camerapts, ptscount, colorpts);
	for (int i = 0; i < ptscount; i++)
	{
		out_arr[i * 2] = colorpts[i].X;
		out_arr[i * 2 + 1] = colorpts[i].Y;
	}
	delete[] camerapts;
	delete[] colorpts;
	*converted_arr = out_arr;
	return ptscount;
}
int main(int argc, char *argv[])
{
	if (argc < 3)
	{
		std::cout << "Usage: " << argv[0] << "input_folder output_folder [frames]" << std::endl;
		return 0;
	}
	int frames = 100;
	bool normalised = false;
	float *color_points;
	int ptscount;
	if (argc > 3)
	{
		if (argv[3][0] == '-')
			normalised = true;
		else
			frames = atoi(argv[3]);
	}
	std::string input_folder(argv[1]), output_folder(argv[2]);
	cnpy::NpyArray arr;
	if (!SUCCEEDED(CreateMapper()))
		return 0;
	Sleep(3000);
	int frames_skipped = 0;
	for (int i = 0; i < frames; i++)
	{
		if (file_exist(input_folder + "/" + std::to_string(i) + ".npy"))
		{
			arr = cnpy::npy_load(input_folder + "/" + std::to_string(i) + ".npy");
			ptscount=convert_to_colorspace(arr,&color_points);
			const unsigned int shape[] = { ptscount, 2 };
			cnpy::npy_save(output_folder+"/"+ std::to_string(i) + ".npy", color_points, shape, 2, "w");
		}
		else
			frames_skipped++;
		std::cout << "\r" << i << " out of " << frames << " frames converted";
		if (frames_skipped>0)
			std::cout << ", " << frames_skipped << " do not exist";
		std::cout << std::flush;
	}
	std::cout << std::endl;
	return 0;
}

