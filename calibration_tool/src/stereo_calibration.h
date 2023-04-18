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

// 标定重投影矩阵到特定坐标系
void calibrateQ();