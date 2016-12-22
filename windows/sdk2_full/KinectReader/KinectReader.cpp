//------------------------------------------------------------------------------
// <copyright file="CoordinateMappingBasics.cpp" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

#include "stdafx.h"
#include <math.h>
#include <limits>
#include <Wincodec.h>
#include "KinectReader.h"
#include "rply\rply.h"
#include "cnpy\cnpy.h"
#include "opencv2\opencv.hpp"
#include "opencv2\core\core.hpp"
#include "opencv2\highgui.hpp"
#include <sys/stat.h>
#include <cstdio>
#include <windows.h>
#define WRITE_TO_BUFFER
#define WRITE_NPY
#define NO_PIC
#define REAL_PLY

#ifndef HINST_THISCOMPONENT
EXTERN_C IMAGE_DOS_HEADER __ImageBase;
#define HINST_THISCOMPONENT ((HINSTANCE)&__ImageBase)
#endif
inline bool exists_test(const std::string& name) {
	struct stat buffer;
	return (stat(name.c_str(), &buffer) == 0);
}

int main(int argc, char *argv[])
{
	CCoordinateMappingBasics application;
	if (argc > 1)
	{
		application.framesCount = atoi(argv[1]);
	}
	return application.Run();
}

/// <summary>
/// Constructor
/// </summary>
CCoordinateMappingBasics::CCoordinateMappingBasics() :
m_nLastCounter_Kinect(0),
m_nLastCounter(0),
m_nFramesSinceUpdate(0),
m_fFreq(0),
m_nNextStatusTime(0LL),
m_pKinectSensor(NULL),
m_pCoordinateMapper(NULL),
m_pMultiSourceFrameReader(NULL),
m_pDepthCoordinates(NULL),
m_pColorRGBX(NULL),
framesCount(100),
frameIndex(0),
backgroundCount(200)
{
	hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
	backgroundRemain = backgroundCount;
	LARGE_INTEGER qpf = { 0 };
	if (QueryPerformanceFrequency(&qpf))
	{
		m_fFreq = double(qpf.QuadPart);
	}

	// create heap storage for color pixel data in RGBX format
	m_pColorRGBX = new RGBQUAD[cColorWidth * cColorHeight];

	m_picturebuffer = new Picture[framesCount];
	m_jointbuffer = new Skeleton[framesCount];
	// create heap storage for the coorinate mapping from color to depth
	m_pDepthCoordinates = new DepthSpacePoint[cColorWidth * cColorHeight];
	m_bkg_counter = new int[cDepthSize];
	m_depthbuffer = new Depthmap[framesCount];
	
	for (int i = 0; i < cDepthSize; i++)
	{
		m_bkg_counter[i] = 0;
		m_background.depth[i] = 0;
	}
	std::vector<std::pair<std::string, bool>>launch_params;
#ifdef WRITE_TO_BUFFER
	launch_params.push_back(std::pair<std::string, bool>("WRITE_TO_BUFFER", true));
#else
	launch_params.push_back(std::pair<std::string, bool>("WRITE_TO_BUFFER", false));
#endif
#ifdef WRITE_NPY
	launch_params.push_back(std::pair<std::string, bool>("WRITE_NPY", true));
#else
	launch_params.push_back(std::pair<std::string, bool>("WRITE_NPY", false));
#endif
#ifdef NO_PIC
	launch_params.push_back(std::pair<std::string, bool>("NO_PIC", true));
#else
	launch_params.push_back(std::pair<std::string, bool>("NO_PIC", false));
#endif
#ifdef REAL_PLY
	launch_params.push_back(std::pair<std::string, bool>("REAL_PLY", true));
#else
	launch_params.push_back(std::pair<std::string, bool>("REAL_PLY", false));
#endif
	logger.writeHead(framesCount, backgroundCount,launch_params);
}
int WriteNpy(const char *filename, Depthmap map, bool writebody)
{
	const unsigned int shape[] = { cDepthHeight, cDepthWidth };
	for (int i = 0; i < cDepthSize; i++)
	{
		if (map.getskipped(writebody)[i])
		{
			map.depth[i] = -1;
		}
		else
		{
			if (map.depth[i] < 0)
			{
				map.depth[i] = -map.depth[i];
			}
		}
	}
	cnpy::npy_save(std::string(filename), map.depth, shape, 2, "w");
	return 1;
}
inline float fillnan(float x)
{
	if (x != x)
		return 0;
	else
		return x;
}
int WriteRealPly(const char *filename, Pointcloud map)
{
	p_ply ply_file;
	int *countbuf = new int[cDepthHeight*cDepthWidth];
	int vertex_count = 0, faces_count = 0;

	for (int ind_i = 0; ind_i < cDepthHeight; ind_i++)
	{
		for (int ind_j = 0; ind_j < cDepthWidth; ind_j++)
		{
			if (!map.second[ind_i*cDepthWidth + ind_j])
			{
				vertex_count++;
				if ((ind_i > 0) && (ind_j > 0) && (!map.second[(ind_i - 1)*cDepthWidth + (ind_j - 1)]))
				{
					if (!map.second[ind_i*cDepthWidth + (ind_j - 1)])
						faces_count++;
					if (!map.second[(ind_i - 1)*cDepthWidth + ind_j])
						faces_count++;
				}
			}
		}
	}
	ply_file = ply_create(filename,PLY_LITTLE_ENDIAN, NULL, 0, NULL);
	if (ply_file == NULL)
		return 0;
	ply_add_comment(ply_file, "MSU Graphics and Media Lab, 2016");
	ply_add_element(ply_file, "vertex", vertex_count);
	ply_add_property(ply_file, "x", PLY_FLOAT32, PLY_CHAR, PLY_CHAR);
	ply_add_property(ply_file, "y", PLY_FLOAT32, PLY_CHAR, PLY_CHAR);
	ply_add_property(ply_file, "z", PLY_FLOAT32, PLY_CHAR, PLY_CHAR);
	ply_add_element(ply_file, "face", faces_count);
	ply_add_property(ply_file, "vertex_indices", PLY_LIST, PLY_UCHAR, PLY_INT32);
	ply_write_header(ply_file);
	vertex_count = 0;
	for (int i = 0; i < cDepthHeight; i++)
	{
		for (int j = 0; j < cDepthWidth; j++)
		{
			if (!map.second[i*cDepthWidth + j])
			{
				ply_write(ply_file, fillnan(map.first[i*cDepthWidth + j].X));
				ply_write(ply_file, fillnan(map.first[i*cDepthWidth + j].Y));
				ply_write(ply_file, fillnan(-map.first[i*cDepthWidth + j].Z));
				countbuf[i*cDepthWidth + j] = vertex_count++;
			}
		}
	}
	for (int i = 0; i < cDepthHeight - 1; i++)
	{
		for (int j = 0; j < cDepthWidth - 1; j++)
		{
			if ((!map.second[(i + 1)*cDepthWidth + j + 1]) && (!map.second[i*cDepthWidth + j]))
			{
				if (!map.second[(i + 1)*cDepthWidth + j])
				{
					ply_write(ply_file, 3);
					ply_write(ply_file, countbuf[i*cDepthWidth + j]);
					ply_write(ply_file, countbuf[(i + 1)*cDepthWidth + j]);
					ply_write(ply_file, countbuf[(i + 1)*cDepthWidth + j + 1]);
				}
				if (!map.second[i*cDepthWidth + j + 1])
				{
					ply_write(ply_file, 3);
					ply_write(ply_file, countbuf[i*cDepthWidth + j]);
					ply_write(ply_file, countbuf[(i + 1)*cDepthWidth + j + 1]);
					ply_write(ply_file, countbuf[i*cDepthWidth + j + 1]);
				}
			}
		}
	}
	delete[] countbuf;
	return ply_close(ply_file);
}
int WritePly(const char *filename, Depthmap map, bool writebody)
{
	p_ply ply_file;
	int vertex_count = 0, faces_count = 0, vert_id[cDepthSize];
	for (int ind_i = 0; ind_i < cDepthHeight; ind_i++)
	{
		for (int ind_j = 0; ind_j < cDepthWidth; ind_j++)
		{
			if (!map.getskipped(writebody)[ind_i*cDepthWidth + ind_j])
			{
				vertex_count++;
				if ((ind_i > 0) && (ind_j > 0) && (!map.getskipped(writebody)[(ind_i - 1)*cDepthWidth + (ind_j - 1)]))
				{
					if (!map.getskipped(writebody)[ind_i*cDepthWidth + (ind_j - 1)])
						faces_count++;
					if (!map.getskipped(writebody)[(ind_i - 1)*cDepthWidth + ind_j])
						faces_count++;
				}
			}
		}
	}
	ply_file = ply_create(filename,PLY_LITTLE_ENDIAN, NULL, 0, NULL);
	if (ply_file == NULL)
		return 0;
	ply_add_comment(ply_file, "MSU Graphics and Media Lab, 2016");
	ply_add_element(ply_file, "vertex", vertex_count);
	ply_add_property(ply_file, "x", PLY_FLOAT32, PLY_CHAR, PLY_CHAR);
	ply_add_property(ply_file, "y", PLY_FLOAT32, PLY_CHAR, PLY_CHAR);
	ply_add_property(ply_file, "z", PLY_FLOAT32, PLY_CHAR, PLY_CHAR);
	ply_add_element(ply_file, "face", faces_count);
	ply_add_property(ply_file, "vertex_indices", PLY_LIST, PLY_UCHAR, PLY_INT32);
	ply_write_header(ply_file);
	vertex_count = 0;
	for (int i = 0; i < cDepthHeight; i++)
	{
		for (int j = 0; j < cDepthWidth; j++)
		{
			if (!map.getskipped(writebody)[i*cDepthWidth + j])
			{
				ply_write(ply_file, j);
				ply_write(ply_file, -i);
				ply_write(ply_file, -map.depth[i*cDepthWidth + j]);
				vert_id[i*cDepthWidth + j] = vertex_count++;
			}
		}
	}
	for (int i = 0; i < cDepthHeight - 1; i++)
	{
		for (int j = 0; j < cDepthWidth - 1; j++)
		{
			if ((!map.getskipped(writebody)[(i + 1)*cDepthWidth + j + 1]) && (!map.getskipped(writebody)[i*cDepthWidth + j]))
			{
				if (!map.getskipped(writebody)[(i + 1)*cDepthWidth + j])
				{
					ply_write(ply_file, 3);
					ply_write(ply_file, vert_id[i*cDepthWidth + j]);
					ply_write(ply_file, vert_id[(i + 1)*cDepthWidth + j]);
					ply_write(ply_file, vert_id[(i + 1)*cDepthWidth + j + 1]);
				}
				if (!map.getskipped(writebody)[i*cDepthWidth + j + 1])
				{
					ply_write(ply_file, 3);
					ply_write(ply_file, vert_id[i*cDepthWidth + j]);
					ply_write(ply_file, vert_id[(i + 1)*cDepthWidth + j + 1]);
					ply_write(ply_file, vert_id[i*cDepthWidth + j + 1]);
				}
			}
		}
	}
	return ply_close(ply_file);
}
Pointcloud CCoordinateMappingBasics::ConvertToPointcloud(Depthmap dmap, bool onlybody)
{
	CameraSpacePoint *camerabuf = new CameraSpacePoint[cDepthSize];
	bool *skipped = new bool[cDepthSize];
	UINT16 *depthbuf = new UINT16[cDepthSize];
	for (int i = 0; i < cDepthSize; i++)
	{
		skipped[i] = dmap.getskipped(onlybody);
		depthbuf[i] = static_cast<UINT16>(dmap.depth[i]);
	}
	m_pCoordinateMapper->MapDepthFrameToCameraSpace(cDepthHeight*cDepthWidth, depthbuf, cDepthHeight*cDepthWidth, camerabuf);
	delete[] depthbuf;
	return Pointcloud(camerabuf, skipped);
}
void CCoordinateMappingBasics::FlushBuffer()
{
	cv::Mat pic_matrix(cColorHeight, cColorWidth, CV_8UC3);
	std::string filename_temp;
	printf("\n");
	logger.saveToFile("./output/logfile.txt");
	for (int frame = 0; frame < framesCount; frame++)
	{
		printf("Flushing data to drive... %d/%d \r", frame + 1, framesCount);
		filename_temp = std::string("./output/") + std::to_string(frame);
#ifndef NO_PIC
		for (int i = 0; i < cColorSize; i++)
		{
			pic_matrix.data[i * 3] = m_picturebuffer[frame].data[i].rgbBlue;
			pic_matrix.data[i * 3 + 1] = m_picturebuffer[frame].data[i].rgbGreen;
			pic_matrix.data[i * 3 + 2] = m_picturebuffer[frame].data[i].rgbRed;
		}
		cv::imwrite(filename_temp+".png", pic_matrix);
#endif
#ifdef WRITE_NPY
		WriteNpy((filename_temp + ".npy").c_str(), m_depthbuffer[frame], true);
#else
#ifdef REAL_PLY
		Pointcloud cloud = ConvertToPointcloud(m_depthbuffer[frame], true);
		WriteRealPly((filename_temp + ".ply").c_str(), cloud);
		delete[] cloud.first;
		delete[] cloud.second;
#else
		WritePly((filename_temp + ".ply").c_str(), m_depthbuffer[frame], true);
#endif
#endif
		HandleJoints(filename_temp + ".txt", m_jointbuffer[frame].depth_joints, m_jointbuffer[frame].camera_joints, JointType_Count);
	}

}
/// <summary>
/// Destructor
/// </summary>
CCoordinateMappingBasics::~CCoordinateMappingBasics()
{

	if (m_pColorRGBX)
	{
		delete[] m_pColorRGBX;
		m_pColorRGBX = NULL;
	}

	if (m_pDepthCoordinates)
	{
		delete[] m_pDepthCoordinates;
		m_pDepthCoordinates = NULL;
	}

	delete[] m_depthbuffer;
	delete[] m_bkg_counter;
	delete[] m_picturebuffer;

	// done with frame reader
	SafeRelease(m_pMultiSourceFrameReader);

	// done with coordinate mapper
	SafeRelease(m_pCoordinateMapper);

	// close the Kinect Sensor
	if (m_pKinectSensor)
	{
		m_pKinectSensor->Close();
	}

	SafeRelease(m_pKinectSensor);
}

/// <summary>
/// Creates the main window and begins processing
/// </summary>
/// <param name="hInstance">handle to the application instance</param>
/// <param name="nCmdShow">whether to display minimized, maximized, or normally</param>
int CCoordinateMappingBasics::Run()
{
	if (exists_test("./output/background.ply")|| exists_test("./output/background.npy"))
	{
		char choise;
		printf("There is data in output folder, overwrite? [y/N] ");
		scanf("%c", &choise);
		if ((choise != 'Y') && (choise != 'y'))
			return 0;
	}
	InitializeDefaultSensor();
	// Main message loop
	t_depthstream_state state = DS_BACKGROUND_CAPTURING;
	while (state != DS_COMPLETE)
	{
		state = Update(state);
		if (state == DS_BACKGROUND_COMPLETE)
		{
			//CreateDirectory(L"output", NULL);
			SetConsoleTextAttribute(hConsole, colors[1]);
			printf("Background captured, waiting for your response\n");
			getchar();
			SetConsoleTextAttribute(hConsole, colors[2]);
			for (int seconds = 5; seconds >= 1; seconds--)
			{
				printf("Starting in %d... \r", seconds);
				Sleep(1000);
			}
			printf("\n");
			state = DS_REALTIME_CAPTURING;
			SetConsoleTextAttribute(hConsole, colors[3]);
		}
	}
	SetConsoleTextAttribute(hConsole, colors[4]);
	FlushBuffer();
	
	return 0;
}

void CCoordinateMappingBasics::WriteToBuffer(Depthmap *dm, RGBQUAD *pic, int framenum)
{
	m_depthbuffer[framenum].copyfrom(dm);
	m_picturebuffer[framenum].copyfrom(pic);
}
/// <summary>
/// Main processing function
/// </summary>
t_depthstream_state CCoordinateMappingBasics::Update(t_depthstream_state state)
{
	if (!m_pMultiSourceFrameReader)
	{
		return DS_COMPLETE;
	}

	IMultiSourceFrame* pMultiSourceFrame = NULL;
	IDepthFrame* pDepthFrame = NULL;
	IColorFrame* pColorFrame = NULL;
	IBodyIndexFrame* pBodyIndexFrame = NULL;
	IBodyFrame* pBodyFrame = NULL;

	HRESULT hr = m_pMultiSourceFrameReader->AcquireLatestFrame(&pMultiSourceFrame);

	if (SUCCEEDED(hr))
	{
		IDepthFrameReference* pDepthFrameReference = NULL;

		hr = pMultiSourceFrame->get_DepthFrameReference(&pDepthFrameReference);
		if (SUCCEEDED(hr))
		{
			hr = pDepthFrameReference->AcquireFrame(&pDepthFrame);
		}

		SafeRelease(pDepthFrameReference);
	}

	if (SUCCEEDED(hr))
	{
		IColorFrameReference* pColorFrameReference = NULL;

		hr = pMultiSourceFrame->get_ColorFrameReference(&pColorFrameReference);
		if (SUCCEEDED(hr))
		{
			hr = pColorFrameReference->AcquireFrame(&pColorFrame);
		}

		SafeRelease(pColorFrameReference);
	}
	if (SUCCEEDED(hr))
	{
		IBodyIndexFrameReference* pBodyIndexFrameReference = NULL;

		hr = pMultiSourceFrame->get_BodyIndexFrameReference(&pBodyIndexFrameReference);
		if (SUCCEEDED(hr))
		{
			hr = pBodyIndexFrameReference->AcquireFrame(&pBodyIndexFrame);
		}

		SafeRelease(pBodyIndexFrameReference);
	}
	if (SUCCEEDED(hr))
	{
		IBodyFrameReference* pBodyFrameReference = NULL;

		hr = pMultiSourceFrame->get_BodyFrameReference(&pBodyFrameReference);
		if (SUCCEEDED(hr))
		{
			hr = pBodyFrameReference->AcquireFrame(&pBodyFrame);
		}

		SafeRelease(pBodyFrameReference);
	}

	if (SUCCEEDED(hr))
	{
		INT64 nDepthTime = 0;
		IFrameDescription* pDepthFrameDescription = NULL;
		int nDepthWidth = 0;
		int nDepthHeight = 0;
		UINT nDepthBufferSize = 0;
		UINT16 *pDepthBuffer = NULL;

		IFrameDescription* pColorFrameDescription = NULL;
		int nColorWidth = 0;
		int nColorHeight = 0;
		ColorImageFormat imageFormat = ColorImageFormat_None;
		UINT nColorBufferSize = 0;
		RGBQUAD *pColorBuffer = NULL;

		IFrameDescription* pBodyIndexFrameDescription = NULL;
		int nBodyIndexWidth = 0;
		int nBodyIndexHeight = 0;
		UINT nBodyIndexBufferSize = 0;
		BYTE *pBodyIndexBuffer = NULL;

		USHORT nDepthMinReliableDistance;
		USHORT nDepthMaxDistance;

		// get depth frame data

		hr = pDepthFrame->get_RelativeTime(&nDepthTime);

		if (SUCCEEDED(hr))
		{
			hr = pDepthFrame->get_FrameDescription(&pDepthFrameDescription);
		}

		if (SUCCEEDED(hr))
		{
			hr = pDepthFrameDescription->get_Width(&nDepthWidth);
		}

		if (SUCCEEDED(hr))
		{
			hr = pDepthFrameDescription->get_Height(&nDepthHeight);
		}

		if (SUCCEEDED(hr))
		{
			hr = pDepthFrame->AccessUnderlyingBuffer(&nDepthBufferSize, &pDepthBuffer);
		}
		if (SUCCEEDED(hr))
		{
			hr = pDepthFrame->get_DepthMinReliableDistance(&nDepthMinReliableDistance);
		}

		if (SUCCEEDED(hr))
		{
			// In order to see the full range of depth (including the less reliable far field depth)
			// we are setting nDepthMaxDistance to the extreme potential depth threshold
			nDepthMaxDistance = USHRT_MAX;

			// Note:  If you wish to filter by reliable depth distance, uncomment the following line.
			//// hr = pDepthFrame->get_DepthMaxReliableDistance(&nDepthMaxDistance);
		}

		// get color frame data

		if (SUCCEEDED(hr))
		{
			hr = pColorFrame->get_FrameDescription(&pColorFrameDescription);
		}

		if (SUCCEEDED(hr))
		{
			hr = pColorFrameDescription->get_Width(&nColorWidth);
		}

		if (SUCCEEDED(hr))
		{
			hr = pColorFrameDescription->get_Height(&nColorHeight);
		}

		if (SUCCEEDED(hr))
		{
			hr = pColorFrame->get_RawColorImageFormat(&imageFormat);
		}

		if (SUCCEEDED(hr))
		{
			if (imageFormat == ColorImageFormat_Bgra)
			{
				hr = pColorFrame->AccessRawUnderlyingBuffer(&nColorBufferSize, reinterpret_cast<BYTE**>(&pColorBuffer));
			}
			else if (m_pColorRGBX)
			{
				pColorBuffer = m_pColorRGBX;
				nColorBufferSize = cColorWidth * cColorHeight * sizeof(RGBQUAD);
				hr = pColorFrame->CopyConvertedFrameDataToArray(nColorBufferSize, reinterpret_cast<BYTE*>(pColorBuffer), ColorImageFormat_Bgra);
			}
			else
			{
				hr = E_FAIL;
			}
		}

		// get body index frame data

		if (SUCCEEDED(hr))
		{
			hr = pBodyIndexFrame->get_FrameDescription(&pBodyIndexFrameDescription);
		}

		if (SUCCEEDED(hr))
		{
			hr = pBodyIndexFrameDescription->get_Width(&nBodyIndexWidth);
		}

		if (SUCCEEDED(hr))
		{
			hr = pBodyIndexFrameDescription->get_Height(&nBodyIndexHeight);
		}

		if (SUCCEEDED(hr))
		{
			hr = pBodyIndexFrame->AccessUnderlyingBuffer(&nBodyIndexBufferSize, &pBodyIndexBuffer);
		}

		if (SUCCEEDED(hr))
		{
			INT64 nTime = 0;

			hr = pBodyFrame->get_RelativeTime(&nTime);

			IBody* ppBodies[BODY_COUNT] = { 0 };

			if (SUCCEEDED(hr))
			{
				hr = pBodyFrame->GetAndRefreshBodyData(_countof(ppBodies), ppBodies);
			}

			if (SUCCEEDED(hr))
			{
				if (state == DS_REALTIME_CAPTURING)
				{
					ProcessSkeleton(nTime, BODY_COUNT, ppBodies);
					logger.getLastFrame()->isbodydetected = true;
				}
				
			}

			for (int i = 0; i < _countof(ppBodies); ++i)
			{
				SafeRelease(ppBodies[i]);
			}
		}
		if (SUCCEEDED(hr))
		{
			state = ProcessFrame(state, nDepthTime, pDepthBuffer, nDepthWidth, nDepthHeight,
				pColorBuffer, nColorWidth, nColorHeight,
				pBodyIndexBuffer, nBodyIndexWidth, nBodyIndexHeight,
				nDepthMinReliableDistance, nDepthMaxDistance);
		}
		SafeRelease(pDepthFrameDescription);
		SafeRelease(pColorFrameDescription);
		SafeRelease(pBodyIndexFrameDescription);
	}
	SafeRelease(pBodyFrame);
	SafeRelease(pDepthFrame);
	SafeRelease(pColorFrame);
	SafeRelease(pBodyIndexFrame);
	SafeRelease(pMultiSourceFrame);
	return state;
}

/// <summary>
/// Initializes the default Kinect sensor
/// </summary>
/// <returns>indicates success or failure</returns>
HRESULT CCoordinateMappingBasics::InitializeDefaultSensor()
{
	HRESULT hr;

	hr = GetDefaultKinectSensor(&m_pKinectSensor);
	if (FAILED(hr))
	{
		return hr;
	}

	if (m_pKinectSensor)
	{
		// Initialize the Kinect and get coordinate mapper and the frame reader

		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->get_CoordinateMapper(&m_pCoordinateMapper);
		}

		hr = m_pKinectSensor->Open();

		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->OpenMultiSourceFrameReader(
				FrameSourceTypes::FrameSourceTypes_Depth | FrameSourceTypes::FrameSourceTypes_Color | FrameSourceTypes::FrameSourceTypes_BodyIndex | FrameSourceTypes::FrameSourceTypes_Body,
				&m_pMultiSourceFrameReader);
		}
	}

	if (!m_pKinectSensor || FAILED(hr))
	{
		SetStatusMessage("No ready Kinect found!");
		return E_FAIL;
	}

	return hr;
}

/// <summary>
/// Handle new depth and color data
/// <param name="nTime">timestamp of frame</param>
/// <param name="pDepthBuffer">pointer to depth frame data</param>
/// <param name="nDepthWidth">width (in pixels) of input depth image data</param>
/// <param name="nDepthHeight">height (in pixels) of input depth image data</param>
/// <param name="pColorBuffer">pointer to color frame data</param>
/// <param name="nColorWidth">width (in pixels) of input color image data</param>
/// <param name="nColorHeight">height (in pixels) of input color image data</param>
/// <param name="pBodyIndexBuffer">pointer to body index frame data</param>
/// <param name="nBodyIndexWidth">width (in pixels) of input body index data</param>
/// <param name="nBodyIndexHeight">height (in pixels) of input body index data</param>
/// </summary>
t_depthstream_state CCoordinateMappingBasics::ProcessFrame(t_depthstream_state state, INT64 nTime,
	const UINT16* pDepthBuffer, int nDepthWidth, int nDepthHeight,
	const RGBQUAD* pColorBuffer, int nColorWidth, int nColorHeight,
	const BYTE* pBodyIndexBuffer, int nBodyIndexWidth, int nBodyIndexHeight,
	USHORT nMinDepth, USHORT nMaxDepth)
{

	//if (!m_nStartTime)
	//{
	//	m_nStartTime = nTime;
	//}

	//double fps = 0.0;

	//LARGE_INTEGER qpcNow = { 0 };
	//if (m_fFreq)
	//{
	//	if (QueryPerformanceCounter(&qpcNow))
	//	{
	//		if (m_nLastCounter)
	//		{
	//			m_nFramesSinceUpdate++;
	//			fps = m_fFreq * m_nFramesSinceUpdate / double(qpcNow.QuadPart - m_nLastCounter);
	//		}
	//	}
	//}

	//char szStatusMessage[64];
	//sprintf(szStatusMessage," FPS = %0.2f    Time = %I64d", fps, (nTime - m_nStartTime));

	//SetStatusMessage(szStatusMessage);
	//m_nLastCounter = qpcNow.QuadPart;
	//m_nFramesSinceUpdate = 0;

	// Make sure we've received valid data
	LARGE_INTEGER qpcNow = { 0 };
	QueryPerformanceCounter(&qpcNow);
	double delta_time = double(qpcNow.QuadPart - m_nLastCounter) / m_fFreq;
	//printf(" Time1 = %I64d    Time2 = %I64d    Diff = %I64d ", qpcNow.QuadPart - m_nLastCounter, nTime - m_nLastCounter_Kinect, 
	//	qpcNow.QuadPart - m_nLastCounter - (nTime - m_nLastCounter_Kinect));
	m_nLastCounter = qpcNow.QuadPart;
	m_nLastCounter_Kinect = nTime;
	if (m_pCoordinateMapper && m_pDepthCoordinates &&
		pDepthBuffer && (nDepthWidth == cDepthWidth) && (nDepthHeight == cDepthHeight) &&
		pColorBuffer && (nColorWidth == cColorWidth) && (nColorHeight == cColorHeight) &&
		pBodyIndexBuffer && (nBodyIndexWidth == cDepthWidth) && (nBodyIndexHeight == cDepthHeight))
	{
		for (int i = 0; i < cDepthSize; i++)
		{
			int depth = pDepthBuffer[i];
			if ((depth >= nMinDepth) && (depth <= nMaxDepth))
			{
				m_depthmap.depth[i] = depth;
				BYTE player = pBodyIndexBuffer[i];
				m_depthmap.skipped[i] = false;
				// if we're tracking a player for the current pixel, draw from the color camera
				if (player != 0xff)
				{
					m_depthmap.body_skipped[i] = false;
				}
				else
				{
					m_depthmap.body_skipped[i] = true;
				}
			}
			else
			{
				m_depthmap.skipped[i] = true;
				m_depthmap.body_skipped[i] = true;
			}
		}
		switch (state)
		{
		case DS_BACKGROUND_CAPTURING:
			printf("Capturing background... %d/%d \r", backgroundCount-backgroundRemain, backgroundCount);
			if (delta_time > 1.0f / 25)
			{
				//TODO: repeat frames
			}
			for (int i = 0; i < cDepthSize; i++)
			{
				if (!m_depthmap.skipped[i])
				{
					m_bkg_counter[i]++;
					m_background.depth[i] += m_depthmap.depth[i];
				}
			}
			if (--backgroundRemain == 0)
			{
				state = DS_BACKGROUND_COMPLETE;
				for (int i = 0; i < cDepthSize; i++)
				{
					if (m_bkg_counter[i]>10)
					{
						m_background.depth[i] /= m_bkg_counter[i];
						m_background.skipped[i] = false;
					}
					else
					{
						m_background.skipped[i] = true;
					}
				}
#ifdef WRITE_NPY
				WriteNpy("./output/background.npy", m_background, false);
#else
				WritePly("./output/background.ply", m_background, false);
#endif
			}
			break;
		case DS_REALTIME_CAPTURING:
			logger.logFrame(delta_time, true, nTime);
#ifdef WRITE_TO_BUFFER
			WriteToBuffer(&m_depthmap, m_pColorRGBX, frameIndex);
#endif
			if (++frameIndex == framesCount)
				state = DS_COMPLETE;
			printf("Writing frame %d/%d \r", frameIndex, framesCount);
			break;

		}
	}
	return state;
}

/// <summary>
/// Set the status bar message
/// </summary>
/// <param name="szMessage">message to display</param>
/// <param name="showTimeMsec">time in milliseconds to ignore future status messages</param>
/// <param name="bForce">force status update</param>
void CCoordinateMappingBasics::SetStatusMessage(char* szMessage)
{
	printf("\r%s", szMessage);
}


/// <summary>
/// Save passed in image data to disk as a bitmap
/// </summary>
/// <param name="pBitmapBits">image data to save</param>
/// <param name="lWidth">width (in pixels) of input image data</param>
/// <param name="lHeight">height (in pixels) of input image data</param>
/// <param name="wBitsPerPixel">bits per pixel of image data</param>
/// <param name="lpszFilePath">full file path to output bitmap to</param>
/// <returns>indicates success or failure</returns>
HRESULT CCoordinateMappingBasics::SaveBitmapToFile(BYTE* pBitmapBits, LONG lWidth, LONG lHeight, WORD wBitsPerPixel, LPCWSTR lpszFilePath)
{
	DWORD dwByteCount = lWidth * lHeight * (wBitsPerPixel / 8);

	BITMAPINFOHEADER bmpInfoHeader = { 0 };

	bmpInfoHeader.biSize = sizeof(BITMAPINFOHEADER);  // Size of the header
	bmpInfoHeader.biBitCount = wBitsPerPixel;             // Bit count
	bmpInfoHeader.biCompression = BI_RGB;                    // Standard RGB, no compression
	bmpInfoHeader.biWidth = lWidth;                    // Width in pixels
	bmpInfoHeader.biHeight = -lHeight;                  // Height in pixels, negative indicates it's stored right-side-up
	bmpInfoHeader.biPlanes = 1;                         // Default
	bmpInfoHeader.biSizeImage = dwByteCount;               // Image size in bytes

	BITMAPFILEHEADER bfh = { 0 };

	bfh.bfType = 0x4D42;                                           // 'M''B', indicates bitmap
	bfh.bfOffBits = bmpInfoHeader.biSize + sizeof(BITMAPFILEHEADER);  // Offset to the start of pixel data
	bfh.bfSize = bfh.bfOffBits + bmpInfoHeader.biSizeImage;        // Size of image + headers

	// Create the file on disk to write to
	HANDLE hFile = CreateFileW(lpszFilePath, GENERIC_WRITE, 0, NULL, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, NULL);

	// Return if error opening file
	if (NULL == hFile)
	{
		return E_ACCESSDENIED;
	}

	DWORD dwBytesWritten = 0;

	// Write the bitmap file header
	if (!WriteFile(hFile, &bfh, sizeof(bfh), &dwBytesWritten, NULL))
	{
		CloseHandle(hFile);
		return E_FAIL;
	}

	// Write the bitmap info header
	if (!WriteFile(hFile, &bmpInfoHeader, sizeof(bmpInfoHeader), &dwBytesWritten, NULL))
	{
		CloseHandle(hFile);
		return E_FAIL;
	}

	// Write the RGB Data
	if (!WriteFile(hFile, pBitmapBits, bmpInfoHeader.biSizeImage, &dwBytesWritten, NULL))
	{
		CloseHandle(hFile);
		return E_FAIL;
	}

	// Close the file
	CloseHandle(hFile);
	return S_OK;
}
Point CCoordinateMappingBasics::BodyToScreen(const CameraSpacePoint& bodyPoint, int width, int height)
{
	// Calculate the body's position on the screen
	DepthSpacePoint depthPoint = { 0 };
	m_pCoordinateMapper->MapCameraPointToDepthSpace(bodyPoint, &depthPoint);

	float screenPointX = static_cast<float>(depthPoint.X * width) / cDepthWidth;
	float screenPointY = static_cast<float>(depthPoint.Y * height) / cDepthHeight;

	return Point(screenPointX, screenPointY);
}
void CCoordinateMappingBasics::HandleJoints(const std::string &filename, Point *m_Points, CameraSpacePoint *m_CameraPoints, int points_count)
{
	if (m_Points)
	{
		FILE* skel_file = fopen(filename.c_str(), "w");
		for (int i = 0; i < points_count; ++i)
		{
			fprintf(skel_file, "Depth %d: %.2f %.2f Camera %d: %f %f %f\n", i, m_Points[i].first, m_Points[i].second,
			i, m_CameraPoints[i].X, m_CameraPoints[i].Y, m_CameraPoints[i].Z);
		}
		fclose(skel_file);
	}
	else
	{
		if (exists_test(filename))
		{
			std::remove(filename.c_str());
		}
	}
}
void CCoordinateMappingBasics::ProcessSkeleton(INT64 nTime, int nBodyCount, IBody** ppBodies)
{
	int width = cDepthWidth;
	int height = cDepthHeight;
	HRESULT hr;

	for (int i = 0; i < nBodyCount; ++i)
	{
		IBody* pBody = ppBodies[i];
		if (pBody)
		{
			BOOLEAN bTracked = false;
			hr = pBody->get_IsTracked(&bTracked);

			if (SUCCEEDED(hr) && bTracked)
			{
				Joint joints[JointType_Count];
				Point jointPoints[JointType_Count];
				CameraSpacePoint jointPointsCamera[JointType_Count];

				hr = pBody->GetJoints(_countof(joints), joints);
				if (SUCCEEDED(hr))
				{
					for (int j = 0; j < _countof(joints); ++j)
					{
						jointPoints[j] = BodyToScreen(joints[j].Position, width, height);
						jointPointsCamera[j] = joints[j].Position;
					}
#ifdef WRITE_TO_BUFFER
					m_jointbuffer[frameIndex].copyfrom(jointPoints, jointPointsCamera);
#else
					HandleJoints(std::to_string(framesCount)+".txt",jointPoints, jointPointsCamera, JointType_Count);
#endif
					break;
				}
			}
		}
	}
}
/// <summary>
/// Load an image from a resource into a buffer
/// </summary>
/// <param name="resourceName">name of image resource to load</param>
/// <param name="resourceType">type of resource to load</param>
/// <param name="nOutputWidth">width (in pixels) of scaled output bitmap</param>
/// <param name="nOutputHeight">height (in pixels) of scaled output bitmap</param>
/// <param name="pOutputBuffer">buffer that will hold the loaded image</param>
/// <returns>S_OK on success, otherwise failure code</returns>
HRESULT CCoordinateMappingBasics::LoadResourceImage(PCWSTR resourceName, PCWSTR resourceType, UINT nOutputWidth, UINT nOutputHeight, RGBQUAD* pOutputBuffer)
{
	IWICImagingFactory* pIWICFactory = NULL;
	IWICBitmapDecoder* pDecoder = NULL;
	IWICBitmapFrameDecode* pSource = NULL;
	IWICStream* pStream = NULL;
	IWICFormatConverter* pConverter = NULL;
	IWICBitmapScaler* pScaler = NULL;

	HRSRC imageResHandle = NULL;
	HGLOBAL imageResDataHandle = NULL;
	void *pImageFile = NULL;
	DWORD imageFileSize = 0;

	HRESULT hrCoInit = CoInitialize(NULL);
	HRESULT hr = hrCoInit;

	if (SUCCEEDED(hr))
	{
		hr = CoCreateInstance(CLSID_WICImagingFactory, NULL, CLSCTX_INPROC_SERVER, IID_IWICImagingFactory, (LPVOID*)&pIWICFactory);
	}

	if (SUCCEEDED(hr))
	{
		// Locate the resource
		imageResHandle = FindResourceW(HINST_THISCOMPONENT, resourceName, resourceType);
		hr = imageResHandle ? S_OK : E_FAIL;
	}

	if (SUCCEEDED(hr))
	{
		// Load the resource
		imageResDataHandle = LoadResource(HINST_THISCOMPONENT, imageResHandle);
		hr = imageResDataHandle ? S_OK : E_FAIL;
	}

	if (SUCCEEDED(hr))
	{
		// Lock it to get a system memory pointer.
		pImageFile = LockResource(imageResDataHandle);
		hr = pImageFile ? S_OK : E_FAIL;
	}

	if (SUCCEEDED(hr))
	{
		// Calculate the size.
		imageFileSize = SizeofResource(HINST_THISCOMPONENT, imageResHandle);
		hr = imageFileSize ? S_OK : E_FAIL;
	}

	if (SUCCEEDED(hr))
	{
		// Create a WIC stream to map onto the memory.
		hr = pIWICFactory->CreateStream(&pStream);
	}

	if (SUCCEEDED(hr))
	{
		// Initialize the stream with the memory pointer and size.
		hr = pStream->InitializeFromMemory(
			reinterpret_cast<BYTE*>(pImageFile),
			imageFileSize);
	}

	if (SUCCEEDED(hr))
	{
		// Create a decoder for the stream.
		hr = pIWICFactory->CreateDecoderFromStream(
			pStream,
			NULL,
			WICDecodeMetadataCacheOnLoad,
			&pDecoder);
	}

	if (SUCCEEDED(hr))
	{
		// Create the initial frame.
		hr = pDecoder->GetFrame(0, &pSource);
	}

	if (SUCCEEDED(hr))
	{
		// Convert the image format to 32bppPBGRA
		// (DXGI_FORMAT_B8G8R8A8_UNORM + D2D1_ALPHA_MODE_PREMULTIPLIED).
		hr = pIWICFactory->CreateFormatConverter(&pConverter);
	}

	if (SUCCEEDED(hr))
	{
		hr = pIWICFactory->CreateBitmapScaler(&pScaler);
	}

	if (SUCCEEDED(hr))
	{
		hr = pScaler->Initialize(
			pSource,
			nOutputWidth,
			nOutputHeight,
			WICBitmapInterpolationModeCubic
			);
	}

	if (SUCCEEDED(hr))
	{
		hr = pConverter->Initialize(
			pScaler,
			GUID_WICPixelFormat32bppPBGRA,
			WICBitmapDitherTypeNone,
			NULL,
			0.f,
			WICBitmapPaletteTypeMedianCut);
	}

	UINT width = 0;
	UINT height = 0;
	if (SUCCEEDED(hr))
	{
		hr = pConverter->GetSize(&width, &height);
	}

	// make sure the image scaled correctly so the output buffer is big enough
	if (SUCCEEDED(hr))
	{
		if ((width != nOutputWidth) || (height != nOutputHeight))
		{
			hr = E_FAIL;
		}
	}

	if (SUCCEEDED(hr))
	{
		hr = pConverter->CopyPixels(NULL, width * sizeof(RGBQUAD), nOutputWidth * nOutputHeight * sizeof(RGBQUAD), reinterpret_cast<BYTE*>(pOutputBuffer));
	}

	SafeRelease(pScaler);
	SafeRelease(pConverter);
	SafeRelease(pSource);
	SafeRelease(pDecoder);
	SafeRelease(pStream);
	SafeRelease(pIWICFactory);

	if (SUCCEEDED(hrCoInit))
	{
		CoUninitialize();
	}

	return hr;
}

