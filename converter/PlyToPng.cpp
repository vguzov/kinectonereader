#include <string>
#include <vector>
#include <iostream>
#include <sys/stat.h>
#include "rply/rply.h"
#include "cnpy.h"

int        cDepthWidth = 512;
int        cDepthHeight = 424;
float *data;
int coord_stack[2] = {-1, -1};
static int vertex_cb(p_ply_argument argument) {
	long eol;
	ply_get_argument_user_data(argument, NULL, &eol);
	float st = ply_get_argument_value(argument);
	if (coord_stack[0] < 0)
	{
		coord_stack[0] = static_cast<int>(st);
		if (coord_stack[0] < 0) coord_stack[0] = -coord_stack[0];
	}
	else if (coord_stack[1] < 0)
	{
		coord_stack[1] = static_cast<int>(st);
		if (coord_stack[1] < 0) coord_stack[1] = -coord_stack[1];
	}
	else
	{
		
		data[coord_stack[1] * cDepthWidth + coord_stack[0]] = st;
		for (int i = 0; i < 2; i++) coord_stack[i] = -1;
	}
	//std::cout<<"("<<coord_stack[0]<<","<<coord_stack[1]<<")"<<std::endl;
	return 1;
}
inline bool exist_test (const std::string& name) {
  struct stat buffer;   
  return (stat (name.c_str(), &buffer) == 0); 
}
int ReadPly(std::string filename)
{
	p_ply ply_file = ply_open(filename.c_str(), NULL, 0, NULL);
	if (!ply_file)
		return 0;
	ply_read_header(ply_file);
	ply_set_read_cb(ply_file, "vertex", "x", vertex_cb, NULL, 0);
	ply_set_read_cb(ply_file, "vertex", "y", vertex_cb, NULL, 0);
	ply_set_read_cb(ply_file, "vertex", "z", vertex_cb, NULL, 1);
	if (!ply_read(ply_file)) return 0;
	ply_close(ply_file);
	return 1;
}
int main(int argc, char* argv[])
{
	if (argc < 3)
	{
		std::cout << "Usage: " << argv[0] << " input_folder output_folder [height width]" << std::endl;
		return 1;
	}
	for (int i=1; i<3; i++)
	{
		if (argv[i][std::string(argv[i]).length()-1] == '/')
			argv[i][std::string(argv[i]).length()-1]='\0';
	}
	if (argc >= 5)
	{
		cDepthHeight = std::stoi(argv[3]);
		cDepthWidth = std::stoi(argv[4]);
	}
	int index = 1;
	data = new float[cDepthHeight*cDepthWidth];
	const unsigned int shape[] = {cDepthHeight,cDepthWidth};
	std::string in_path = std::string(argv[1]) +"/output"+ std::to_string(index) + ".ply";
	if (exist_test(std::string(argv[1]) + "/background.ply"))
	{
		ReadPly(std::string(argv[1]) + "/background.ply");
		cnpy::npy_save(std::string(argv[2]) + "/background.npy", data, shape, 2, "w");
	}
	while (exist_test(in_path))
	{
		std::cout << "Processing " << index << "\r" << std::flush;
		ReadPly(in_path);
		cnpy::npy_save(std::string(argv[2]) +"/output"+ std::to_string(index) + ".npy", data, shape, 2, "w");
		index++;
		in_path = std::string(argv[1]) +"/output"+ std::to_string(index) + ".ply";
	}
	std::cout<<"Converted "<<index-1<<" frames                 "<<std::endl;
	return 0;
}

