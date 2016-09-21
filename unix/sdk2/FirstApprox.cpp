#include <vector>
#include <string>
#include <fstream>
#include <cstdlib>
#include <iostream>
#include "rply/rply.h"
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>
#include <libfreenect2/logger.h>
#include <cmath>
#include <limits>
#include <unistd.h>
#define WRITE_TO_BUFFER
typedef std::pair<float*, bool*> depthmap;
static const int        cScreenWidth = 320;
static const int        cScreenHeight = 240;
static const int        cDepthWidth = 512;
static const int        cDepthHeight = 424;
static const int        cBytesPerPixel = 4;
libfreenect2::Freenect2 freenect2;
libfreenect2::Freenect2Device *dev = 0;
libfreenect2::PacketPipeline *pipeline = 0;
libfreenect2::FrameMap frames;
libfreenect2::Frame undistorted(512, 424, 4), registered(512, 424, 4);
libfreenect2::Registration* registration;
libfreenect2::SyncMultiFrameListener listener(libfreenect2::Frame::Depth);
float*                    m_depth;
float*                    m_background;
float*                    m_holded;
int*                    m_holded_vertex_nonskipped_counter;
int*                    m_background_vertex_nonskipped_counter;
bool*					m_vertex_skipped;
bool*					m_background_vertex_skipped;
int frame_counter = 0;
int holding_counter = 0;
std::string output_dir("");
static const int frame_counter_all = 300;
static const int holding_counter_all = 3;
int background_capture_all = 200;
int background_capture_remain = background_capture_all;
float inf_float = std::numeric_limits<float>::infinity();
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
class MyFileLogger: public libfreenect2::Logger
{
private:
  std::ofstream logfile_;
public:
  MyFileLogger(const char *filename)
  {
if (filename)
      logfile_.open(filename);
    level_ = Debug;
  }
bool good()
  {
return logfile_.is_open() && logfile_.good();
  }
virtual void log(Level level, const std::string &message)
  {
    logfile_ << "[" << libfreenect2::Logger::level2str(level) << "] " << message << std::endl;
  }
};
int main(int argc, char* argv[])
{
	if (argc<2)
	{
		printf("Usage: %s output_dir\n",argv[0]);
		return 1;
	}
	output_dir = std::string(argv[1]);
    MyFileLogger *filelogger = new MyFileLogger("./log.txt");
  if (filelogger->good())
  libfreenect2::setGlobalLogger(filelogger);
  else
  delete filelogger;
	m_depth = new float[cDepthWidth*cDepthHeight];
	m_background = new float[cDepthWidth*cDepthHeight];
	m_vertex_skipped = new bool[cDepthWidth*cDepthHeight];
	m_background_vertex_skipped = new bool[cDepthWidth*cDepthHeight];
	m_background_vertex_nonskipped_counter = new int[cDepthWidth*cDepthHeight];
#ifdef WRITE_TO_BUFFER
	m_depth_buffer.first = new float[cDepthWidth*cDepthHeight*frame_counter_all];
	m_depth_buffer.second = new bool[cDepthWidth*cDepthHeight*frame_counter_all];
#endif
	//pipeline = new libfreenect2::CpuPacketPipeline();
	pipeline = new libfreenect2::OpenCLPacketPipeline(-1);
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
			//getchar();
			for (int seconds = 4; seconds >= 1; seconds--)
			{
				printf("Starting in %d... \n",seconds);
				sleep(1);
			}
			
		}
	}
    dev->stop();
    dev->close();
#ifdef WRITE_TO_BUFFER
	WriteBufToPly((output_dir+std::string("/output")).c_str());
	delete[] m_depth_buffer.first;
	delete[] m_depth_buffer.second;
#endif
	delete[] m_background_vertex_nonskipped_counter;
	delete[] m_background_vertex_skipped;
	delete[] m_depth;
	delete[] m_vertex_skipped;
	delete[] m_background;
	delete filelogger;
	return 0;
}
void SetStatusMessage(char *message)
{
	printf("%s", message);
}
int CreateFirstConnected()
{
    if(freenect2.enumerateDevices() == 0)
    {
      std::cout << "No device connected!" << std::endl;
      return -1;
    }
	std::string serial = freenect2.getDefaultDeviceSerialNumber();
	dev = freenect2.openDevice(serial, pipeline);
    if(dev == 0)
    {
      std::cout << "Failure opening device!" << std::endl;
      return -1;
    }
    dev->setIrAndDepthFrameListener(&listener);
	dev->startStreams(false, true);
	return 0;
}

void NaiveBackgroundRemove(depthmap map, depthmap background, double average_noise)
{
	for (int i = 0; i < cDepthHeight; i++)
	{
		for (int j = 0; j < cDepthWidth; j++)
		{
			if ((!background.second[i*cDepthWidth + j]) &&
				(std::abs(map.first[i*cDepthWidth + j] - background.first[i*cDepthWidth + j]) <= average_noise))
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
				ply_write(ply_file, map.first[i*cDepthWidth + j]);
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
		std::cout<<"Flushing data to drive... " << (frame+1) << "/" << frame_counter_all<<"\r"<<std::flush;
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
	libfreenect2::Frame *depth_frame = frames[libfreenect2::Frame::Depth];

	// Make sure we've received valid data

		int ind_i = 0, ind_j = 0;
		float* depth_data = reinterpret_cast<float *>(depth_frame->data);

		while (ind_i<cDepthHeight)
		{
			float depth = depth_data[ind_i*cDepthWidth+ind_j];
			if ((depth >=0) && (depth<inf_float) && !std::isnan(depth))
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
			WritePly((output_dir+std::string("/background.ply")).c_str(), depthmap(m_background, m_background_vertex_skipped));
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
			WritePly((output_dir+std::string("/output") + std::to_string(frame_counter) + std::string(".ply")).c_str(), 
				depthmap(m_depth, m_vertex_skipped));
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

// Point SkeletonToScreen(Vector4 skeletonPoint, int width, int height)
// {
// 	LONG x, y;
// 	USHORT depth;
//
// 	// Calculate the skeleton's position on the screen
// 	// NuiTransformSkeletonToDepthImage returns coordinates in NUI_IMAGE_RESOLUTION_320x240 space
// 	NuiTransformSkeletonToDepthImage(skeletonPoint, &x, &y, &depth);
//
// 	float screenPointX = static_cast<float>(x * width) / cScreenWidth;
// 	float screenPointY = static_cast<float>(y * height) / cScreenHeight;
//
// 	return Point(screenPointX, screenPointY);
// }
//
// void HandleSkeleton(const NUI_SKELETON_DATA & skel)
// {
// 	FILE* skel_file = fopen((std::string("./output/output") + std::to_string(frame_counter) + std::string(".txt")).c_str(), "w");
// 	Point *m_Points = new Point[NUI_SKELETON_POSITION_COUNT];
// 	for (int i = 0; i < NUI_SKELETON_POSITION_COUNT; ++i)
// 	{
// 		m_Points[i] = SkeletonToScreen(skel.SkeletonPositions[i], cScreenWidth, cScreenHeight);
// 		fprintf(skel_file,"Point %d: %.2f %.2f\n", i, m_Points[i].first, m_Points[i].second);
// 	}
// 	fclose(skel_file);
// }
//
// void ProcessSkeleton()
// {
//
// 	NUI_SKELETON_FRAME skeletonFrame = { 0 };
//
// 	HRESULT hr = m_pNuiSensor->NuiSkeletonGetNextFrame(0, &skeletonFrame);
// 	if (FAILED(hr))
// 	{
// 		return;
// 	}
//
// 	// smooth out the skeleton data
// 	m_pNuiSensor->NuiTransformSmooth(&skeletonFrame, NULL);
//
// 	//printf("F");
//
// 	for (int i = 0; i < NUI_SKELETON_COUNT; ++i)
// 	{
// 		NUI_SKELETON_TRACKING_STATE trackingState = skeletonFrame.SkeletonData[i].eTrackingState;
//
// 		if (NUI_SKELETON_TRACKED == trackingState)
// 		{
// 			// We're tracking the skeleton, draw it
// 			HandleSkeleton(skeletonFrame.SkeletonData[i]);
//
// 		}
// 	}
// }
t_depthstream_state Update(t_depthstream_state state)
{
	if (!dev)
	{
		return DS_COMPLETE;
	}

	// if (WAIT_OBJECT_0 == WaitForSingleObject(m_hNextSkeletonEvent, 0))
	// {
	// 	ProcessSkeleton();
	// }
    if (!listener.waitForNewFrame(frames, 10*1000)) // 10 sconds
    {
      std::cout << "Frame waiting timeout" << std::endl;
      return DS_COMPLETE;
    }
	state = ProcessDepth(state);
	listener.release(frames);
	if (frame_counter == frame_counter_all)
		return DS_COMPLETE;
	else if (background_capture_remain<0) frame_counter++;
	return state;

}