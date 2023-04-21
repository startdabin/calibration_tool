#pragma once
#include <iostream>
#include <opencv2/core.hpp>

// 查找角点
void findCorner(std::vector<cv::Mat> imgs, cv::Size board_size,
				std::vector<std::vector<cv::Point2f>>& imgs_points,
				bool isSubCorner = true);

// 双目标定
void stereoClibrate(const std::vector<cv::Mat>& imgs_l,
			const std::vector<cv::Mat>& imgs_r,
			cv::Size board_size,
			float square_length,
			std::string parameters_file,
			bool isSubCorner=true);

// 标定到特定坐标系
void calibrateRT(const cv::Mat& chessboard_image, cv::Size board_size, float board_length, std::string src_param, std::string dst_param);

// test demo  
void stereoCalibrationDemo(std::string left_img_dir, std::string right_img_dir, std::string param_path);