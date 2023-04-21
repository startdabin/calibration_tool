#include <iostream>
#include "src/stereo_calibration.h"
#include "src/utils.h"

int main() {
	std::string src_param_path = "G:\\data\\3d_reconstruction\\calibrate\\t0409\\stereo_param.yml";
	std::string dst_param_path = "G:\\data\\3d_reconstruction\\calibrate\\t0409\\dst_param.yml";

	cv::Size board_size{ 11,8 };
	float square_length = 10;

	std::string chessboard_image_name = "G:\\data\\3d_reconstruction\\calibrate\\t0409\\cs.png";
	auto chessboard_image = cv::imread(chessboard_image_name, cv::IMREAD_GRAYSCALE);
	
	calibrateQ(chessboard_image, { 8,5 }, 10, src_param_path, dst_param_path);
	
	//calibrateQ()

	return 0;
}