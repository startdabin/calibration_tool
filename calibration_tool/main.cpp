#include <iostream>
#include "src/stereo_calibration.h"
#include "src/utils.h"

int main() {
	std::string src_param_path = "F:\\data\\calibrate\\stereo_param.yml";
	std::string dst_param_path = "F:\\data\\calibrate\\new_param.yml";

	cv::Size board_size{ 8,6 };
	float square_length = 10;

	std::string chessboard_image_name = "F:\\data\\calibrate\\cc.png";
	auto chessboard_image = cv::imread(chessboard_image_name, cv::IMREAD_GRAYSCALE);
	
	calibrateRT(chessboard_image, board_size, square_length, src_param_path, dst_param_path);

	return 0;
}