// FirstApprox.cpp: определяет точку входа для консольного приложения.
//
// Windows Header Files
#include <vector>
#include <windows.h>


#include <Shlobj.h>
#include <string>
#include "stdafx.h"
#include <Kinect.h>
#include "rply\rply.h"
#include "cnpy\cnpy.h"
#define WRITE_TO_BUFFER
#define WRITE_NPY
typedef std::pair<int*, bool*> depthmap;
static const int        cScreenWidth = 320;
static const int        cScreenHeight = 240;
static const int        cDepthWidth = 512;
static const int        cDepthHeight = 424;
static const int        cBytesPerPixel = 4;

static const int        cStatusMessageMaxLen = MAX_PATH * 2;
// Current Kinect
IKinectSensor*          m_pKinectSensor = NULL;

// Depth reader
IDepthFrameReader*      m_pDepthFrameReader = NULL;

ICoordinateMapper*      m_pCoordinateMapper;

// Body reader
bool                    m_bFloorDetected = false;
Vector4                 m_fFloorClipPlane;
CameraSpacePoint*       m_pCameraSpacePoints;
IBodyFrameReader*       m_pBodyFrameReader;
int*                    m_depth;
int*                    m_background;
int*                    m_holded;
int*                    m_holded_vertex_nonskipped_counter;
int*                    m_background_vertex_nonskipped_counter;
bool*					m_vertex_skipped;
bool*					m_background_vertex_skipped;
HANDLE                  m_pSkeletonStreamHandle;
HANDLE                  m_hNextSkeletonEvent;
HANDLE                  m_pDepthStreamHandle;
HANDLE                  m_hNextDepthFrameEvent;
UINT16 *pDepthCopy;
int frame_counter = 0;
int holding_counter = 0;
static const int frame_counter_all = 300;
static const int holding_counter_all = 3;
int background_capture_all = 200;
int background_capture_remain = background_capture_all;
depthmap m_depth_buffer;
enum t_depthstream_state {
	DS_BACKGROUND_CAPTURING,
	DS_BACKGROUND_COMPLETE,
	DS_REALTIME_CAPTURING,
	DS_REALTIME_STOPPED,
	DS_COMPLETE
};
t_depthstream_state Update(t_depthstream_state);
int CreateFirstConnected(void);
void Draw(void);
void WriteBufToPly(const char *filename_start);
int _tmain(int argc, _TCHAR* argv[])
{
	pDepthCopy = new UINT16[cDepthWidth*cDepthHeight];
	m_pCameraSpacePoints = new CameraSpacePoint[cDepthWidth*cDepthHeight];
	m_depth = new int[cDepthWidth*cDepthHeight];
	m_background = new int[cDepthWidth*cDepthHeight];
	m_vertex_skipped = new bool[cDepthWidth*cDepthHeight];
	m_background_vertex_skipped = new bool[cDepthWidth*cDepthHeight];
	m_background_vertex_nonskipped_counter = new int[cDepthWidth*cDepthHeight];
#ifdef WRITE_TO_BUFFER
	m_depth_buffer.first = new int[cDepthWidth*cDepthHeight*frame_counter_all];
	m_depth_buffer.second = new bool[cDepthWidth*cDepthHeight*frame_counter_all];
#endif
	CreateFirstConnected();
	for (int i = 0; i < cDepthHeight; i++)
	{
		for (int j = 0; j < cDepthWidth; j++)
		{
			m_background_vertex_nonskipped_counter[i*cDepthWidth + j] = 0;
			m_background_vertex_skipped[i*cDepthWidth + j] = false;
			m_background[i*cDepthWidth + j] = 0;
		}
	}
	printf("Ready to capture...\n");
	t_depthstream_state state = DS_BACKGROUND_CAPTURING;
	while (state != DS_COMPLETE)
	{
		state = Update(state);
		if (state == DS_BACKGROUND_COMPLETE)
		{
			getchar();
			for (int seconds = 10; seconds >= 1; seconds--)
			{
				printf("Starting in %d... \r",seconds);
				Sleep(1000);
			}
			
		}
	}
#ifdef WRITE_TO_BUFFER
	WriteBufToPly(".\\output\\output");
	delete[] m_depth_buffer.first;
	delete[] m_depth_buffer.second;
#endif
	delete[] pDepthCopy;
	delete[] m_pCameraSpacePoints;
	delete[] m_background_vertex_nonskipped_counter;
	delete[] m_background_vertex_skipped;
	delete[] m_depth;
	delete[] m_vertex_skipped;
	delete[] m_background;
	return 0;
}
void SetStatusMessage(char *message)
{
	printf("%s", message);
}
int CreateFirstConnected()
{
	HRESULT hr;

	hr = GetDefaultKinectSensor(&m_pKinectSensor);
	if (FAILED(hr))
	{
		return hr;
	}

	if (m_pKinectSensor)
	{
		// Initialize the Kinect and get the depth reader
		IDepthFrameSource* pDepthFrameSource = NULL;
		IBodyFrameSource* pBodyFrameSource = NULL;

		hr = m_pKinectSensor->Open();

		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->get_DepthFrameSource(&pDepthFrameSource);
		}

		if (SUCCEEDED(hr))
		{
			hr = pDepthFrameSource->OpenReader(&m_pDepthFrameReader);
		}
		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->get_CoordinateMapper(&m_pCoordinateMapper);
		}

		if (SUCCEEDED(hr))
		{
			hr = m_pKinectSensor->get_BodyFrameSource(&pBodyFrameSource);
		}

		if (SUCCEEDED(hr))
		{
			hr = pBodyFrameSource->OpenReader(&m_pBodyFrameReader);
		}

		SafeRelease(pBodyFrameSource);
		SafeRelease(pDepthFrameSource);
	}

	if (!m_pKinectSensor || FAILED(hr))
	{
		SetStatusMessage("No ready Kinect found!");
		return E_FAIL;
	}

	return hr;
}

void NaiveBackgroundRemove(depthmap map, depthmap background, double average_noise)
{
	for (int i = 0; i < cDepthHeight; i++)
	{
		for (int j = 0; j < cDepthWidth; j++)
		{
			if ((!background.second[i*cDepthWidth + j]) &&
				(abs(map.first[i*cDepthWidth + j] - background.first[i*cDepthWidth + j]) <= average_noise))
			{
				map.second[i*cDepthWidth + j] = true;
			}
		}
	}
}

int WriteNpy(const char *filename, depthmap map)
{
	const unsigned int shape[] = { cDepthHeight, cDepthWidth };
	for (int i = 0; i < cDepthHeight*cDepthWidth; i++)
	{
		if (map.second[i])
		{
			map.first[i] = -1;
		}
		else
		{
			if (map.first[i] < 0)
			{
				map.first[i] = -map.first[i];
			}
		}
	}
	cnpy::npy_save(std::string(filename), map.first, shape, 2, "w");
	return 1;
}

int WritePly(const char *filename, depthmap map)
{
	p_ply ply_file;
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
	ply_file = ply_create(filename,/*PLY_ASCII*/PLY_LITTLE_ENDIAN, NULL, 0, NULL);
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
				ply_write(ply_file, j);
				ply_write(ply_file, -i);
				ply_write(ply_file, -map.first[i*cDepthWidth + j]);
				map.first[i*cDepthWidth + j] = vertex_count++;
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
					ply_write(ply_file, map.first[i*cDepthWidth + j]);
					ply_write(ply_file, map.first[(i + 1)*cDepthWidth + j]);
					ply_write(ply_file, map.first[(i + 1)*cDepthWidth + j + 1]);
				}
				if (!map.second[i*cDepthWidth + j + 1])
				{
					ply_write(ply_file, 3);
					ply_write(ply_file, map.first[i*cDepthWidth + j]);
					ply_write(ply_file, map.first[(i + 1)*cDepthWidth + j + 1]);
					ply_write(ply_file, map.first[i*cDepthWidth + j + 1]);
				}
			}
		}
	}
	return ply_close(ply_file);
}
void WriteBuf(int frame, depthmap map)
{
	for (int i = 0; i < cDepthHeight; i++)
	{
		for (int j = 0; j < cDepthWidth; j++)
		{
			m_depth_buffer.first[(frame*cDepthHeight + i)*cDepthWidth + j] = map.first[i*cDepthWidth + j];
			m_depth_buffer.second[(frame*cDepthHeight + i)*cDepthWidth + j] = map.second[i*cDepthWidth + j];
		}
	}
}
void WriteBufToPly(const char *filename_start)
{
	for (int frame = 0; frame < frame_counter_all; frame++)
	{
		printf("Flushing data to drive... %d/%d \r", frame+1,frame_counter_all);
		for (int i = 0; i < cDepthHeight; i++)
		{
			for (int j = 0; j < cDepthWidth; j++)
			{
				m_depth[i*cDepthWidth + j] = m_depth_buffer.first[(frame*cDepthHeight + i)*cDepthWidth + j];
				m_vertex_skipped[i*cDepthWidth + j] = m_depth_buffer.second[(frame*cDepthHeight + i)*cDepthWidth + j];
			}
		}
#ifdef WRITE_NPY
		WriteNpy((std::string(filename_start) + std::to_string(frame + 1) + std::string(".npy")).c_str(), depthmap(m_depth, m_vertex_skipped));
#else
		WritePly((std::string(filename_start) + std::to_string(frame+1) + std::string(".ply")).c_str(), depthmap(m_depth, m_vertex_skipped));
#endif
	}
}
t_depthstream_state ProcessDepth(t_depthstream_state state, INT64 nTime, const UINT16* pBuffer, int nWidth, int nHeight, USHORT nMinDepth, USHORT nMaxDepth)
{
	HRESULT hr;
	long size = nWidth * nHeight;
	// end pixel is start + width*height - 1
	const UINT16* pBufferEnd = pBuffer + (nWidth * nHeight);
	

	int ind_i = 0, ind_j = 0;
	memcpy(pDepthCopy, pBuffer, size*sizeof(UINT16));
	if (m_bFloorDetected)
	{
		//Map entire frame from depth space to camera space
		m_pCoordinateMapper->MapDepthFrameToCameraSpace(size, (UINT16*)pDepthCopy, size, m_pCameraSpacePoints);

		Vector4 fcp = m_fFloorClipPlane;

		float divisor = sqrtf(fcp.x * fcp.x + fcp.y * fcp.y + fcp.z * fcp.z);

		CameraSpacePoint s;

		for (int i = 0; i < size; i++)
		{
			s = m_pCameraSpacePoints[i];

			//Calculate distance from floor plane to current camera space point             
			float dist = (fcp.x * s.X + fcp.y * s.Y + fcp.z * s.Z + fcp.w) / divisor;

			//distance comparison in meters 0.02f = 2 cm
			if (dist < 0.02f)
			{
				m_vertex_skipped[i] = true;
			}
			else
			{
				m_vertex_skipped[i] = false;
			}
		}
	}


	while (pBuffer < pBufferEnd)
	{
		USHORT depth = *pBuffer;
		if ((depth >= nMinDepth) && (depth <= nMaxDepth))
		{
			m_depth[ind_i*cDepthWidth + ind_j] = depth;
			if (!m_bFloorDetected)
				m_vertex_skipped[ind_i*cDepthWidth + ind_j] = false;
		}
		else
		{
			m_vertex_skipped[ind_i*cDepthWidth + ind_j] = true;
		}
		ind_j++;
		if (ind_j >= cDepthWidth)
		{
			ind_i++;
			ind_j = 0;
		}

		// Increment our index into the Kinect's depth buffer
		++pBuffer;
	}
	if (background_capture_remain > 0)
	{
		printf("Capturing background... %d/%d \r", background_capture_all - (--background_capture_remain), background_capture_all);
		for (int i = 0; i < cDepthHeight; i++)
		{
			for (int j = 0; j < cDepthWidth; j++)
			{
				if (!m_vertex_skipped[i*cDepthWidth + j])
				{
					m_background[i*cDepthWidth + j] += m_depth[i*cDepthWidth + j];
					m_background_vertex_nonskipped_counter[i*cDepthWidth + j]++;
				}
			}
		}
		state = DS_BACKGROUND_CAPTURING;
	}
	else if (background_capture_remain == 0)
	{
		printf("Background captured, waiting for your response\n");
		background_capture_remain--;
		for (int i = 0; i < cDepthHeight; i++)
		{
			for (int j = 0; j < cDepthWidth; j++)
			{
				if (m_background_vertex_nonskipped_counter[i*cDepthWidth + j]>2)
					m_background[i*cDepthWidth + j] /= m_background_vertex_nonskipped_counter[i*cDepthWidth + j];
				else
					m_background_vertex_skipped[i*cDepthWidth + j] = true;
			}
		}
#ifdef WRITE_NPY
		WriteNpy(".\\output\\background.npy", depthmap(m_background, m_background_vertex_skipped));
#else
		WritePly(".\\output\\background.ply", depthmap(m_background, m_background_vertex_skipped));
#endif
		state = DS_BACKGROUND_COMPLETE;
	}
	else
	{
		//NaiveBackgroundRemove(depthmap(m_depth, m_vertex_skipped), 
		//	depthmap(m_background, m_background_vertex_skipped),20);
		Draw();
#ifdef WRITE_TO_BUFFER
		WriteBuf(frame_counter - 1, depthmap(m_depth, m_vertex_skipped));
#else
#ifdef WRITE_NPY
		WriteNpy((std::string(".\\output\\output") + std::to_string(frame_counter) + std::string(".npy")).c_str(), 
			depthmap(m_depth, m_vertex_skipped));
#else
		WritePly((std::string(".\\output\\output") + std::to_string(frame_counter) + std::string(".ply")).c_str(), 
			depthmap(m_depth, m_vertex_skipped));
#endif
#endif
		state = DS_REALTIME_CAPTURING;
	}

	return state;
}
void Draw(void)
{
	printf("Writing frame %d/%d \r", frame_counter, frame_counter_all);
}
typedef std::pair<float, float> Point;

Point BodyToScreen(const CameraSpacePoint& bodyPoint, int width, int height)
{
	// Calculate the body's position on the screen
	DepthSpacePoint depthPoint = { 0 };
	m_pCoordinateMapper->MapCameraPointToDepthSpace(bodyPoint, &depthPoint);

	float screenPointX = static_cast<float>(depthPoint.X * width) / cDepthWidth;
	float screenPointY = static_cast<float>(depthPoint.Y * height) / cDepthHeight;

	return Point(screenPointX, screenPointY);
}
void HandleJoints(const Point *m_Points, int points_count)
{
	FILE* skel_file = fopen((std::string(".\\output\\output") + std::to_string(frame_counter) + std::string(".txt")).c_str(), "w");
	for (int i = 0; i < points_count; ++i)
	{
		fprintf(skel_file,"Point %d: %.2f %.2f\n", i, m_Points[i].first, m_Points[i].second);
	}
	fclose(skel_file);
}

void ProcessSkeleton(INT64 nTime, int nBodyCount, IBody** ppBodies)
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

				hr = pBody->GetJoints(_countof(joints), joints);
				if (SUCCEEDED(hr))
				{
					for (int j = 0; j < _countof(joints); ++j)
					{
						jointPoints[j] = BodyToScreen(joints[j].Position, width, height);
					}

					HandleJoints(jointPoints, JointType_Count);
					break;
				}
			}
		}
	}
}
t_depthstream_state Update(t_depthstream_state state)
{
	if ((NULL == m_pDepthFrameReader) || (NULL == m_pBodyFrameReader))
	{
		return DS_COMPLETE;
	}
	IBodyFrame* pBodyFrame = NULL;


	HRESULT hr = m_pBodyFrameReader->AcquireLatestFrame(&pBodyFrame);

	if (SUCCEEDED(hr))
	{
		INT64 nTime = 0;

		hr = pBodyFrame->get_RelativeTime(&nTime);

		IBody* ppBodies[BODY_COUNT] = { 0 };
		Vector4 fcp;
		hr = pBodyFrame->get_FloorClipPlane(&fcp);

		if (SUCCEEDED(hr))
		{
			m_fFloorClipPlane = fcp;
			m_bFloorDetected = true;
		}
		if (SUCCEEDED(hr))
		{
			hr = pBodyFrame->GetAndRefreshBodyData(_countof(ppBodies), ppBodies);
		}
		
		if (SUCCEEDED(hr))
		{
			ProcessSkeleton(nTime, BODY_COUNT, ppBodies);
		}

		for (int i = 0; i < _countof(ppBodies); ++i)
		{
			SafeRelease(ppBodies[i]);
		}
	}

	SafeRelease(pBodyFrame);

	IDepthFrame* pDepthFrame = NULL;

	 hr = m_pDepthFrameReader->AcquireLatestFrame(&pDepthFrame);

	if (SUCCEEDED(hr))
	{
		INT64 nTime = 0;
		IFrameDescription* pFrameDescription = NULL;
		int nWidth = 0;
		int nHeight = 0;
		USHORT nDepthMinReliableDistance = 0;
		USHORT nDepthMaxDistance = 0;
		UINT nBufferSize = 0;
		UINT16 *pBuffer = NULL;

		hr = pDepthFrame->get_RelativeTime(&nTime);

		if (SUCCEEDED(hr))
		{
			hr = pDepthFrame->get_FrameDescription(&pFrameDescription);
		}

		if (SUCCEEDED(hr))
		{
			hr = pFrameDescription->get_Width(&nWidth);
		}

		if (SUCCEEDED(hr))
		{
			hr = pFrameDescription->get_Height(&nHeight);
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

		if (SUCCEEDED(hr))
		{
			hr = pDepthFrame->AccessUnderlyingBuffer(&nBufferSize, &pBuffer);
		}

		if (SUCCEEDED(hr))
		{
			state = ProcessDepth(state, nTime, pBuffer, nWidth, nHeight, nDepthMinReliableDistance, nDepthMaxDistance);
			if (frame_counter == frame_counter_all)
				return DS_COMPLETE;
			else if (background_capture_remain<0) frame_counter++;
		}

		SafeRelease(pFrameDescription);
	}

	SafeRelease(pDepthFrame);
	return state;

}