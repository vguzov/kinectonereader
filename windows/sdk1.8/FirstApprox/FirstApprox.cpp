// FirstApprox.cpp: определяет точку входа для консольного приложения.
//
// Windows Header Files
#include <vector>
#include <windows.h>


#include <Shlobj.h>
#include <string>
#include "stdafx.h"
#include "NuiApi.h"
#include "rply\rply.h"
#define WRITE_TO_BUFFER
typedef std::pair<int*, bool*> depthmap;
static const int        cScreenWidth = 320;
static const int        cScreenHeight = 240;
static const int        cDepthWidth = 640;
static const int        cDepthHeight = 480;
static const int        cBytesPerPixel = 4;

static const int        cStatusMessageMaxLen = MAX_PATH * 2;
INuiSensor*             m_pNuiSensor;
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
int frame_counter = 0;
int holding_counter = 0;
static const int frame_counter_all = 200;
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
			m_background[i*cDepthWidth + j] = false;
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
	INuiSensor * pNuiSensor;
	
	int iSensorCount = 0;
	HRESULT hr = NuiGetSensorCount(&iSensorCount);
	if (FAILED(hr))
	{
		return hr;
	}

	// Look at each Kinect sensor
	for (int i = 0; i < iSensorCount; ++i)
	{
		// Create the sensor so we can check status, if we can't create it, move on to the next
		hr = NuiCreateSensorByIndex(i, &pNuiSensor);
		if (FAILED(hr))
		{
			continue;
		}

		// Get the status of the sensor, and if connected, then we can initialize it
		hr = pNuiSensor->NuiStatus();
		if (S_OK == hr)
		{
			m_pNuiSensor = pNuiSensor;
			break;
		}

		// This sensor wasn't OK, so release it since we're not using it
		pNuiSensor->Release();
	}
	
	if (NULL != m_pNuiSensor)
	{
		// Initialize the Kinect and specify that we'll be using skeleton
		hr = m_pNuiSensor->NuiInitialize(NUI_INITIALIZE_FLAG_USES_SKELETON | NUI_INITIALIZE_FLAG_USES_DEPTH);
		if (SUCCEEDED(hr))
		{
			// Create an event that will be signaled when skeleton data is available
			m_hNextSkeletonEvent = CreateEventW(NULL, TRUE, FALSE, NULL);
			// Create an event that will be signaled when depth data is available
			m_hNextDepthFrameEvent = CreateEvent(NULL, TRUE, FALSE, NULL);

			// Open a skeleton stream to receive skeleton data
			hr = m_pNuiSensor->NuiSkeletonTrackingEnable(m_hNextSkeletonEvent, 0);
			// Open a depth image stream to receive depth frames
			hr = m_pNuiSensor->NuiImageStreamOpen(
				NUI_IMAGE_TYPE_DEPTH,
				NUI_IMAGE_RESOLUTION_640x480,
				0,
				2,
				m_hNextDepthFrameEvent,
				&m_pDepthStreamHandle);
		}
	}

	if (NULL == m_pNuiSensor || FAILED(hr))
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
	//printf("Vertex=%d, faces=%d\n", vertex_count, faces_count);
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
		WritePly((std::string(filename_start) + std::to_string(frame+1) + std::string(".ply")).c_str(), depthmap(m_depth, m_vertex_skipped));
	}
}
t_depthstream_state ProcessDepth(t_depthstream_state state)
{
	HRESULT hr;
	NUI_IMAGE_FRAME imageFrame;

	// Attempt to get the depth frame
	hr = m_pNuiSensor->NuiImageStreamGetNextFrame(m_pDepthStreamHandle, 0, &imageFrame);
	if (FAILED(hr))
	{
		return state;
	}

	BOOL nearMode;
	INuiFrameTexture* pTexture;

	// Get the depth image pixel texture
	hr = m_pNuiSensor->NuiImageFrameGetDepthImagePixelFrameTexture(
		m_pDepthStreamHandle, &imageFrame, &nearMode, &pTexture);
	if (FAILED(hr))
	{
		goto ReleaseFrame;
	}

	NUI_LOCKED_RECT LockedRect;

	// Lock the frame data so the Kinect knows not to modify it while we're reading it
	pTexture->LockRect(0, &LockedRect, NULL, 0);

	// Make sure we've received valid data
	if (LockedRect.Pitch != 0)
	{
		// Get the min and max reliable depth for the current frame
		int minDepth = (nearMode ? NUI_IMAGE_DEPTH_MINIMUM_NEAR_MODE : NUI_IMAGE_DEPTH_MINIMUM) >> NUI_IMAGE_PLAYER_INDEX_SHIFT;
		int maxDepth = (nearMode ? NUI_IMAGE_DEPTH_MAXIMUM_NEAR_MODE : NUI_IMAGE_DEPTH_MAXIMUM) >> NUI_IMAGE_PLAYER_INDEX_SHIFT;

		const NUI_DEPTH_IMAGE_PIXEL * pBufferRun = reinterpret_cast<const NUI_DEPTH_IMAGE_PIXEL *>(LockedRect.pBits);

		// end pixel is start + width*height - 1
		const NUI_DEPTH_IMAGE_PIXEL * pBufferEnd = pBufferRun + (cDepthWidth * cDepthHeight);

		int ind_i = 0, ind_j = 0;

		while (pBufferRun < pBufferEnd)
		{
			// discard the portion of the depth that contains only the player index
			USHORT depth = pBufferRun->depth;

			// To convert to a byte, we're discarding the most-significant
			// rather than least-significant bits.
			// We're preserving detail, although the intensity will "wrap."
			// Values outside the reliable depth range are mapped to 0 (black).

			// Note: Using conditionals in this loop could degrade performance.
			// Consider using a lookup table instead when writing production code.
			if (depth >= minDepth && depth <= maxDepth)
			{
				m_depth[ind_i*cDepthWidth + ind_j] = depth;
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
			++pBufferRun;
		}
		if (background_capture_remain > 0)
		{
			printf("Capturing background... %d/%d \r", background_capture_all-(--background_capture_remain),background_capture_all);
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
			WritePly(".\\output\\background.ply", depthmap(m_background, m_background_vertex_skipped));
			state = DS_BACKGROUND_COMPLETE;
		}
		else
		{
			//NaiveBackgroundRemove(depthmap(m_depth, m_vertex_skipped), 
			//	depthmap(m_background, m_background_vertex_skipped),20);
			Draw();
#ifdef WRITE_TO_BUFFER
			WriteBuf(frame_counter-1, depthmap(m_depth, m_vertex_skipped));
#else
			WritePly((std::string(".\\output\\output") + std::to_string(frame_counter) + std::string(".ply")).c_str(), 
				depthmap(m_depth, m_vertex_skipped));
#endif
			state = DS_REALTIME_CAPTURING;
		}
	}

	// We're done with the texture so unlock it
	pTexture->UnlockRect(0);

	pTexture->Release();

ReleaseFrame:
	// Release the frame
	m_pNuiSensor->NuiImageStreamReleaseFrame(m_pDepthStreamHandle, &imageFrame);
	return state;
}
void Draw(void)
{
	printf("Writing frame %d/%d \r", frame_counter, frame_counter_all);
}
typedef std::pair<float, float> Point;

Point SkeletonToScreen(Vector4 skeletonPoint, int width, int height)
{
	LONG x, y;
	USHORT depth;

	// Calculate the skeleton's position on the screen
	// NuiTransformSkeletonToDepthImage returns coordinates in NUI_IMAGE_RESOLUTION_320x240 space
	NuiTransformSkeletonToDepthImage(skeletonPoint, &x, &y, &depth);

	float screenPointX = static_cast<float>(x * width) / cScreenWidth;
	float screenPointY = static_cast<float>(y * height) / cScreenHeight;

	return Point(screenPointX, screenPointY);
}

void HandleSkeleton(const NUI_SKELETON_DATA & skel)
{
	FILE* skel_file = fopen((std::string("./output/output") + std::to_string(frame_counter) + std::string(".txt")).c_str(), "w");
	Point *m_Points = new Point[NUI_SKELETON_POSITION_COUNT];
	for (int i = 0; i < NUI_SKELETON_POSITION_COUNT; ++i)
	{
		m_Points[i] = SkeletonToScreen(skel.SkeletonPositions[i], cScreenWidth, cScreenHeight);
		fprintf(skel_file,"Point %d: %.2f %.2f\n", i, m_Points[i].first, m_Points[i].second);
	}
	fclose(skel_file);
}

void ProcessSkeleton()
{
	
	NUI_SKELETON_FRAME skeletonFrame = { 0 };

	HRESULT hr = m_pNuiSensor->NuiSkeletonGetNextFrame(0, &skeletonFrame);
	if (FAILED(hr))
	{
		return;
	}

	// smooth out the skeleton data
	m_pNuiSensor->NuiTransformSmooth(&skeletonFrame, NULL);

	//printf("F");

	for (int i = 0; i < NUI_SKELETON_COUNT; ++i)
	{
		NUI_SKELETON_TRACKING_STATE trackingState = skeletonFrame.SkeletonData[i].eTrackingState;

		if (NUI_SKELETON_TRACKED == trackingState)
		{
			// We're tracking the skeleton, draw it
			HandleSkeleton(skeletonFrame.SkeletonData[i]);

		}
	}
}
t_depthstream_state Update(t_depthstream_state state)
{
	if (NULL == m_pNuiSensor)
	{
		return DS_COMPLETE;
	}

	// Wait for 0ms, just quickly test if it is time to process a skeleton
	if (WAIT_OBJECT_0 == WaitForSingleObject(m_hNextSkeletonEvent, 0))
	{
		ProcessSkeleton();
	}
	if (WAIT_OBJECT_0 == WaitForSingleObject(m_hNextDepthFrameEvent, 0))
	{
		state = ProcessDepth(state);
		if (frame_counter == frame_counter_all)
			return DS_COMPLETE;
		else if (background_capture_remain<0) frame_counter++;
	}
	return state;

}