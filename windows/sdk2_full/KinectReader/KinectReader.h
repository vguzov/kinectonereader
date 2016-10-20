//------------------------------------------------------------------------------
// <copyright file="CoordinateMappingBasics.h" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

#pragma once
#include <vector>
#include <Kinect.h>
static const int        cDepthWidth = 512;
static const int        cDepthHeight = 424;
static const int        cDepthSize = cDepthWidth*cDepthHeight;
enum t_depthstream_state {
	DS_BACKGROUND_CAPTURING,
	DS_BACKGROUND_COMPLETE,
	DS_REALTIME_CAPTURING,
	DS_REALTIME_STOPPED,
	DS_COMPLETE
};
typedef std::pair<float, float> Point;
struct Depthmap
{
	int *depth;
	bool *skipped;
	bool *body_skipped;
	Depthmap()
	{
		depth = new int[cDepthSize];
		skipped = new bool[cDepthSize];
		body_skipped = new bool[cDepthSize];
	}
	~Depthmap()
	{
		delete[] depth;
		delete[] skipped;
		delete[] body_skipped;
	}
};

class CCoordinateMappingBasics
{
public:
	

	static const int        cColorWidth = 1920;
	static const int        cColorHeight = 1080;
	int                     framesCount;
	int                     frameIndex;
	int                     backgroundRemain;
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
    INT64                   m_nStartTime;
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
    RGBQUAD*                m_pOutputRGBX; 
    RGBQUAD*                m_pBackgroundRGBX; 
    RGBQUAD*                m_pColorRGBX;

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

	void ProcessSkeleton(INT64 nTime, int nBodyCount, IBody** ppBodies);
	Point BodyToScreen(const CameraSpacePoint& bodyPoint, int width, int height);
	void HandleJoints(const Point *m_Points, int points_count);
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

