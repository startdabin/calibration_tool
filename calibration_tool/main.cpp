#include <iostream>
#include "src/stereo_calibration.h"
#include "src/utils.h"

int main() {
	std::string left_imgs_path = "D:/PROJECT/work/stereo_calibration/data/stereo_imgs0/left/";
	std::string right_imgs_path = "D:/PROJECT/work/stereo_calibration/data/stereo_imgs0/right/";
	auto imgs_l = loadImgs(left_imgs_path, cv::IMREAD_GRAYSCALE);
	auto imgs_r = loadImgs(right_imgs_path, cv::IMREAD_GRAYSCALE);

	cv::Size board_size{ 11,8 };
	float square_length = 10;
	std::string clibrate_file_name = "D:/PROJECT/work/stereo_calibration/data/stereo_imgs0/stereo_param.xml";

	// stereo clibrate
	stereoClibrate(imgs_l, imgs_r, board_size, square_length, clibrate_file_name);

	return 0;
}