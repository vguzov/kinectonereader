//------------------------------------------------------------------------------
// <copyright file="CoordinateMappingBasics.h" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

#pragma once
#include <vector>
#include <sstream>
#include <fstream>
#include <iomanip>
#include <list>
#include <Kinect.h>
#include <windows.h>
#include <chrono>
#include <thread>
#include "opencv2\opencv.hpp"
#include "opencv2\core\core.hpp"
#include "opencv2\highgui.hpp"
static const int        cDepthWidth = 512;
static const int        cDepthHeight = 424;
static const int        cColorWidth = 1920;
static const int        cColorHeight = 1080;
static const int        cDepthSize = cDepthWidth*cDepthHeight;
static const int        cColorSize = cColorWidth*cColorHeight;
const WORD colors[] =
{
	0x0F, 0x0A, 0x0C, 0xCF, 0x9F
};
enum t_depthstream_state {
	DS_BACKGROUND_CAPTURING,
	DS_BACKGROUND_COMPLETE,
	DS_REALTIME_CAPTURING,
	DS_REALTIME_STOPPED,
	DS_COMPLETE
};
typedef std::pair<float, float> Point;
typedef std::pair<CameraSpacePoint*, bool*> Pointcloud;
struct Depthmap
{
	int *depth;
	bool *skipped;
	bool *body_skipped;
	int *link_count;
	Depthmap()
	{
		link_count = new int(1);
		depth = new int[cDepthSize];
		skipped = new bool[cDepthSize];
		body_skipped = new bool[cDepthSize];
	}
	Depthmap(const Depthmap &map)
	{
		*(map.link_count) += 1;
		link_count = map.link_count;
		depth = map.depth;
		skipped = map.skipped;
		body_skipped = map.body_skipped;
	}
	~Depthmap()
	{
		*link_count -= 1;
		if (*link_count == 0)
		{
			delete[] depth;
			delete[] skipped;
			delete[] body_skipped;
			delete link_count;
		}
	}
	const bool *getskipped(bool isbody)
	{
		return isbody ? body_skipped : skipped;
	}
	void copyfrom(Depthmap *dmptr)
	{
		memcpy(depth, dmptr->depth, cDepthSize*sizeof(int));
		memcpy(skipped, dmptr->skipped, cDepthSize*sizeof(bool));
		memcpy(body_skipped, dmptr->body_skipped, cDepthSize*sizeof(bool));
	}
};

struct Picture
{
	RGBQUAD *data;
	int *link_count;
	Picture()
	{
		data = new RGBQUAD[cColorSize];
		link_count = new int(1);
	}
	Picture(const Picture &pic)
	{
		*(pic.link_count) += 1;
		link_count = pic.link_count;
		data = pic.data;
	}
	~Picture()
	{
		*link_count -= 1;
		if (*link_count == 0)
		{
			delete[] data;
			delete link_count;
		}
	}
	void copyfrom(RGBQUAD *dataptr)
	{
		memcpy(data, dataptr, cColorSize*sizeof(RGBQUAD));
	}
};
class EncodedPicture
{
	std::vector<uchar> data;
	std::thread compression_thread;
public:
	void encode(const RGBQUAD *picdata)
	{
		cv::Mat pic_matrix(cColorHeight, cColorWidth, CV_8UC3);
		for (int i = 0; i < cColorSize; i++)
		{
			pic_matrix.data[i * 3] = picdata[i].rgbBlue;
			pic_matrix.data[i * 3 + 1] = picdata[i].rgbGreen;
			pic_matrix.data[i * 3 + 2] = picdata[i].rgbRed;
		}
		std::vector<int> cparams;
		cparams.push_back(cv::IMWRITE_PNG_STRATEGY);
		cparams.push_back(cv::IMWRITE_PNG_STRATEGY_RLE);
		cv::imencode(".png", pic_matrix, data,cparams);
	}
	cv::Mat decode()
	{
		return cv::imdecode(data, cv::IMREAD_COLOR);
	}

	void copyfrom(RGBQUAD *dataptr)
	{
		compression_thread = std::thread(&EncodedPicture::encode, this, dataptr);
		//encode(dataptr);
	}
	~EncodedPicture()
	{
		compression_thread.detach();
	}
	EncodedPicture(const Picture &pic)
	{
		encode(pic.data);
	}
	EncodedPicture() {}
	//operator Picture()
	//{
	//	cv::Mat pic_matrix = decode();
	//	Picture pic;
	//	for (int i = 0; i < cColorSize; i++)
	//	{
	//		pic.data[i].rgbBlue = pic_matrix.data[i * 3];
	//		pic.data[i].rgbGreen = pic_matrix.data[i * 3 + 1];
	//		pic.data[i].rgbRed = pic_matrix.data[i * 3 + 2];
	//	}
	//	return pic;
	//}
	operator cv::Mat()
	{
		return decode();
	}
};
struct Skeleton
{
	Point *depth_joints;
	CameraSpacePoint *camera_joints;
	int *link_count;
	Skeleton()
	{
		depth_joints = NULL;
		camera_joints = NULL;
		link_count = new int(1);
	}
	Skeleton(const Skeleton &sk)
	{
		*(sk.link_count) += 1;
		link_count = sk.link_count;
		depth_joints = sk.depth_joints;
		camera_joints = sk.camera_joints;
	}
	void zeroinit()
	{
		if (!depth_joints)
		{
			depth_joints = new Point[JointType_Count];
			camera_joints = new CameraSpacePoint[JointType_Count];
		}
		for (int i = 0; i < JointType_Count; i++)
		{
			depth_joints[i] = Point(0,0);
			camera_joints[i].X = 0;
			camera_joints[i].Y = 0;
			camera_joints[i].Z = 0;
		}
	}
	void copyfrom(Point *depth_data, CameraSpacePoint *camera_data)
	{
		if (!depth_joints)
		{
			depth_joints = new Point[JointType_Count];
			camera_joints = new CameraSpacePoint[JointType_Count];
		}
		for (int i = 0; i < JointType_Count; i++)
		{
			depth_joints[i] = depth_data[i];
			camera_joints[i] = camera_data[i];
		}
	}
	~Skeleton()
	{
		*link_count -= 1;
		if (*link_count == 0)
		{
			if (depth_joints)
			{
				delete[] depth_joints;
				delete[] camera_joints;
			}
			delete link_count;
		}
	}
};
class LogLine
{
public:
	float delta;
	bool isorig;
	bool isbodydetected;
	INT64 kinect_timestamp;
	int framenum;
	LogLine(int framenum, float delta, bool isorig, bool isbodydetected, INT64 kinect_timestamp)
	{
		this->delta = delta;
		this->isorig = isorig;
		this->isbodydetected = isbodydetected;
		this->kinect_timestamp = kinect_timestamp;
		this->framenum = framenum;
	}
	friend std::ostream &operator<< (std::ostream &str, const LogLine &line);
};
std::ostream &operator<< (std::ostream &str, const LogLine &line)
{
	str << std::setfill('0') << std::setw(4)
		<< line.framenum << "; " << (line.isorig ? "orig" : "copy")
		<< "; " << (line.isbodydetected ? "skel" : "none") << "; "
		<< std::setprecision(2) << 1 / line.delta << "; "
		<< line.delta << "; " << line.kinect_timestamp << std::endl;
	return str;
}
class Logger
{
	std::vector<LogLine> logtext;
	std::string headerstr;
	int framenum;
public:
	Logger():framenum(0) {}
	LogLine *getLastFrame()
	{
		if (framenum == 0)
		{
			return NULL;
		}
		else
		{
			return &logtext[framenum - 1];
		}
	}
	LogLine *getFrame(int frame)
	{
		return &logtext[frame];
	}
	void logFrame(float delta, bool isorig, bool isbodydetected, INT64 kinect_timestamp)
	{
		logtext.push_back(LogLine(framenum++, delta, isorig, isbodydetected, kinect_timestamp));
	}
	void writeHead(int framescount, int skipped_frames, int bkgframescount, const std::vector<std::pair<std::string, bool>> &launch_params)
	{
		std::stringstream logline;
		std::time_t time = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
		logline << "# Created at " << std::ctime(&time) << std::endl;
		logline << "# Videoset frames: " << framescount << ", " << skipped_frames << " skipped" << std::endl;
		logline << "# Background frames: " << bkgframescount << std::endl;
		logline << "# Launch parameters:" << std::endl;
		for (auto it = launch_params.cbegin(); it != launch_params.cend(); ++it)
		{
			logline << "#     " << it->first << ": " << (it->second ? "TRUE" : "FALSE") << std::endl;
		}
		logline << "# Number; Original or copied; Is skeleton detected; FPS; time from previous frame; Kinect timestamp" << std::endl;
		headerstr = logline.str();
	}
	void saveToFile(const std::string &filepath)
	{
		std::ofstream logfile;
		bool lastskel = false;
		logfile.open(filepath, std::ios::out);
		logfile << headerstr;
		for (int i=0; i<logtext.size(); i++)
		{
			//if ((!logtext[i].isorig) && lastskel)
			//{
			//	int j = i + 1;
			//	bool res = true;
			//	while ((j < logtext.size()) && (!logtext[j].isorig))
			//	{
			//		j++;
			//	}
			//	if ((j >= logtext.size())||(!logtext[j].isbodydetected))
			//	{
			//		res = false;
			//	}
			//	for (int k = i + 1; k < j; k++)
			//	{
			//		logtext[k].isbodydetected = res;
			//		logfile << logtext[k];
			//	}
			//	i = j;
			//}
			//if (logtext[i].isorig)
			//{
			//	logfile << logtext[i];
			//	lastskel = logtext[i].isbodydetected;
			//}
			logfile << logtext[i];
		}
		logfile.close();
	}
};
class CCoordinateMappingBasics
{
public:
	
	HANDLE  hConsole;

	int                     framesCount;
	int                     frameIndex;
	int                     backgroundRemain;
	int                     backgroundCount;
    /// <summary>
    /// Constructor
    /// </summary>
    CCoordinateMappingBasics();

    /// <summary>
    /// Destructor
    /// </summary>
    ~CCoordinateMappingBasics();

    /// <summary>
    /// Creates the main window and begins processing
    /// </summary>
    int                     Run(void);

private:
	int                    m_skipped_frames_counter;
	std::list<int>         doubled_frames;
	Logger                  logger;
	
    INT64                   m_nLastCounter_Kinect;
    INT64                   m_nLastCounter;
    double                  m_fFreq;
    INT64                   m_nNextStatusTime;
    DWORD                   m_nFramesSinceUpdate;

	Depthmap                m_depthmap;
	Depthmap                m_background;
	Depthmap                *m_depthbuffer;
	int                     *m_bkg_counter;


    // Current Kinect
    IKinectSensor*          m_pKinectSensor;
    ICoordinateMapper*      m_pCoordinateMapper;
    DepthSpacePoint*        m_pDepthCoordinates;

    // Frame reader
    IMultiSourceFrameReader*m_pMultiSourceFrameReader;

    // Direct2D
    RGBQUAD*                m_pColorRGBX;
	Picture*                m_picturebuffer;

	Skeleton*                  m_jointbuffer;

	void WriteLogHead();
	void WriteToBuffer(Depthmap *dm, RGBQUAD *pic, int framenum);
	void FlushBuffer();
	Pointcloud ConvertToPointcloud(Depthmap dmap, bool onlybody);
    /// <summary>
    /// Main processing function
    /// </summary>
	t_depthstream_state Update(t_depthstream_state);

    /// <summary>
    /// Initializes the default Kinect sensor
    /// </summary>
    /// <returns>S_OK on success, otherwise failure code</returns>
    HRESULT                 InitializeDefaultSensor();

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
	t_depthstream_state                 ProcessFrame(t_depthstream_state state, INT64 nTime,
                                         const UINT16* pDepthBuffer, int nDepthHeight, int nDepthWidth, 
                                         const RGBQUAD* pColorBuffer, int nColorWidth, int nColorHeight,
                                         const BYTE* pBodyIndexBuffer, int nBodyIndexWidth, int nBodyIndexHeight, 
										 USHORT nMinDepth, USHORT nMaxDepth);

	bool ProcessSkeleton(INT64 nTime, int nBodyCount, IBody** ppBodies);
	Point BodyToScreen(const CameraSpacePoint& bodyPoint, int width, int height);
	void HandleJoints(const std::string &filename, Point *m_Points, CameraSpacePoint *m_CameraPoints, int points_count);
	Skeleton InterpolateJoints(const Skeleton &skel_from, const Skeleton &skel_to);
    /// <summary>
    /// Set the status bar message
    /// </summary>
    /// <param name="szMessage">message to display</param>
	void SetStatusMessage(char* szMessage);


    /// <summary>
    /// Save passed in image data to disk as a bitmap
    /// </summary>
    /// <param name="pBitmapBits">image data to save</param>
    /// <param name="lWidth">width (in pixels) of input image data</param>
    /// <param name="lHeight">height (in pixels) of input image data</param>
    /// <param name="wBitsPerPixel">bits per pixel of image data</param>
    /// <param name="lpszFilePath">full file path to output bitmap to</param>
    /// <returns>indicates success or failure</returns>

    HRESULT                 SaveBitmapToFile(BYTE* pBitmapBits, LONG lWidth, LONG lHeight, WORD wBitsPerPixel, LPCWSTR lpszFilePath);
    /// <summary>
    /// Load an image from a resource into a buffer
    /// </summary>
    /// <param name="resourceName">name of image resource to load</param>
    /// <param name="resourceType">type of resource to load</param>
    /// <param name="nOutputWidth">width (in pixels) of scaled output bitmap</param>
    /// <param name="nOutputHeight">height (in pixels) of scaled output bitmap</param>
    /// <param name="pOutputBuffer">buffer that will hold the loaded image</param>
    /// <returns>S_OK on success, otherwise failure code</returns>
    HRESULT                 LoadResourceImage(PCWSTR resourceName, PCWSTR resourceType, UINT nOutputWidth, UINT nOutputHeight, RGBQUAD* pOutputBuffer);
};

