#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <windows.h>
#include "stdafx.h"
#include <Kinect.h>
#include <Shlobj.h>
#include <sys/stat.h>
#include "rply/rply.h"
#include "cnpy/cnpy.h"
typedef float vert_t;
typedef std::pair<vert_t*, bool*> depthmap;
typedef std::pair<CameraSpacePoint*, bool*> pointcloud;
static const int        cDepthWidth = 512;
static const int        cDepthHeight = 424;
ICoordinateMapper*      m_pCoordinateMapper;
IKinectSensor*          m_pKinectSensor = NULL;
IDepthFrameReader*      m_pDepthFrameReader = NULL;
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
	IDepthFrameSource* pDepthFrameSource = NULL;
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
		SafeRelease(pDepthFrameSource);
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
int WritePly(const char *filename, pointcloud map)
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
void WriteNpyCloud(const std::string &filename, pointcloud map)
{
	std::vector<float> clouddata;
	int cloudsize = 0;
	for (int i = 0; i < cDepthHeight*cDepthWidth; i++)
	{
		if (!map.second[i])
		{
			clouddata.push_back(map.first[i].X);
			clouddata.push_back(map.first[i].Y);
			clouddata.push_back(map.first[i].Z);
			cloudsize++;
		}
	}
	const unsigned int shape[] = { cloudsize, 3 };
	cnpy::npy_save(filename, &clouddata[0], shape, 2, "w");
}
void npy_to_cloud(cnpy::NpyArray arr, pointcloud map, UINT16 *depthbuf, bool normalised = false)
{
	for (int i = 0; i < cDepthHeight*cDepthWidth; i++)
	{
		if (normalised && (reinterpret_cast<double *>(arr.data)[i] < 0.1) || (!normalised) && (reinterpret_cast<int *>(arr.data)[i]<0))
		{
			map.second[i] = true;
			depthbuf[i] = 1000;
		}
		else
		{
			map.second[i] = false;
			depthbuf[i] = normalised ? (reinterpret_cast<double *>(arr.data)[i] * 5000 + 1000) : reinterpret_cast<int *>(arr.data)[i];
		}
	}
	m_pCoordinateMapper->MapDepthFrameToCameraSpace(cDepthHeight*cDepthWidth, depthbuf, cDepthHeight*cDepthWidth, map.first);
	//for (int i = 0; i < 10; i++)
	//{
	//	std::cout << depthbuf[i]<<": "<<map.first[i].X << ", " << map.first[i].Y << ", " << map.first[i].Z << std::endl;
	//}
}
void save_depth_camera_intrinsics(std::string filename)
{
	std::ofstream outfile;
	CameraIntrinsics ci;
	m_pCoordinateMapper->GetDepthCameraIntrinsics(&ci);
	outfile.open(filename, std::ios::out);
	outfile << "FOCAL_X: " << ci.FocalLengthX <<std::endl;
	outfile << "FOCAL_Y: " << ci.FocalLengthY << std::endl;
	outfile << "PRINCIPAL_X: " << ci.PrincipalPointX << std::endl;
	outfile << "PRINCIPAL_Y: " << ci.PrincipalPointY << std::endl;
	outfile << "RAD_DIST_2: " << ci.RadialDistortionSecondOrder << std::endl;
	outfile << "RAD_DIST_4: " << ci.RadialDistortionFourthOrder << std::endl;
	outfile << "RAD_DIST_6: " << ci.RadialDistortionSixthOrder << std::endl;
	outfile.close();
}
int main(int argc, char *argv[])
{
	if (argc < 3)
	{
		std::cout << "Usage: " <<argv[0]<< "input_folder output_folder [frames] [-norm|-npy]"<<std::endl;
		return 0;
	}
	int frames = 100;
	bool normalised = false;
	bool write_npy = false;
	for (int argi = 3; argi<argc; argi++)
	{
		if (argv[argi][0] == '-')
		{
			if (std::string(argv[argi]) == "-npy")
			{
				write_npy = true;
			}
			else
			{
				normalised = true;
			}
		}
		else
		{
			frames = atoi(argv[argi]);
		}
	}
	std::string input_folder(argv[1]), output_folder(argv[2]);
	cnpy::NpyArray arr;
	vert_t *vertex = new vert_t[cDepthHeight*cDepthWidth];
	bool *vertex_skipped = new bool[cDepthHeight*cDepthWidth];
	UINT16 *depthbuf = new UINT16[cDepthHeight*cDepthWidth];
	CameraSpacePoint *camerabuf = new CameraSpacePoint[cDepthHeight*cDepthWidth];
	//depthmap dmap(vertex, vertex_skipped);
	pointcloud cloud(camerabuf, vertex_skipped);

	if (!SUCCEEDED(CreateMapper()))
		return 0;
	Sleep(2000);
	save_depth_camera_intrinsics(output_folder + "/depth_intrinsics.txt");
	int frames_skipped = 0;
	if (file_exist(input_folder + "/background.npy"))
	{
		arr = cnpy::npy_load(input_folder + "/background.npy");
		npy_to_cloud(arr, cloud, depthbuf, normalised);
		if (write_npy)
		{
			WriteNpyCloud(output_folder + "/background.npy", cloud);
		}
		else
		{
			WritePly((output_folder + "/background.ply").c_str(), cloud);
		}
	}
	for (int i = 0; i < frames; i++)
	{
		if (file_exist(input_folder + "/" + std::to_string(i) + ".npy"))
		{
			arr = cnpy::npy_load(input_folder + "/" + std::to_string(i) + ".npy");
			npy_to_cloud(arr, cloud, depthbuf, normalised);
			if (write_npy)
			{
				WriteNpyCloud(output_folder + "/" + std::to_string(i) + ".npy", cloud);
			}
			else
			{
				WritePly((output_folder + "/" + std::to_string(i) + ".ply").c_str(), cloud);
			}
			if (file_exist(input_folder + "/" + std::to_string(i) + "_full.npy"))
			{
				arr = cnpy::npy_load(input_folder + "/" + std::to_string(i) + "_full.npy");
				npy_to_cloud(arr, cloud, depthbuf, normalised);
				if (write_npy)
				{
					WriteNpyCloud(output_folder + "/" + std::to_string(i) + "_full.npy", cloud);
				}
				else
				{
					WritePly((output_folder + "/" + std::to_string(i) + "_full.ply").c_str(), cloud);
				}
			}
		}
		else
			frames_skipped++;
		std::cout << "\r" << i+1 << " out of " << frames << " frames converted";
		if (frames_skipped>0)
			std::cout << ", " << frames_skipped << " do not exist";
		std::cout << std::flush;
	}
	std::cout << std::endl;
	delete[] vertex;
	delete[] vertex_skipped;
	delete[] depthbuf;
	delete[] camerabuf;
	return 0;
}

