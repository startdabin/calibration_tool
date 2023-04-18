#pragma once
#include <iostream>
#include <opencv2/core.hpp>

// ���ҽǵ�
void findCorner(std::vector<cv::Mat> imgs, cv::Size board_size,
				std::vector<std::vector<cv::Point2f>>& imgs_points,
				bool isSubCorner = true);

// ˫Ŀ�궨
void stereoClibrate(const std::vector<cv::Mat>& imgs_l,
			const std::vector<cv::Mat>& imgs_r,
			cv::Size board_size,
			float square_length,
			std::string parameters_file,
			bool isSubCorner=true);

// �궨��ͶӰ�����ض�����ϵ
void calibrateQ();