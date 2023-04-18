#include "stereo_calibration.h"
#include <iostream>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "utils.h"

void findCorner(std::vector<cv::Mat> imgs, cv::Size board_size,
	std::vector<std::vector<cv::Point2f>>& imgs_points,
	bool isSubCorner) {
	bool found_corner = false;
	for (auto img : imgs) {
		std::vector<cv::Point2f> corners;
		found_corner = cv::findChessboardCorners(img, board_size, corners);

		if (found_corner) {
			if (isSubCorner) {
				cv::cornerSubPix(img, corners, cv::Size(5, 5), cv::Size(-1, -1),
					cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
						30, 0.01));
			}
			imgs_points.emplace_back(corners);
		}
		else
		{
			std::cout << "found corners failure!!!" << std::endl;
		}
	}
}

// 求相机初始内参矩阵
void findObjectPoints(std::vector<std::vector<cv::Point2f>>& imgs_points,
							cv::Size board_size, cv::Size img_size, float square_length,
							std::vector<std::vector<cv::Point3f>>& object_points)
{
	// 物理角点世界坐标系
	for (int i = 0; i < imgs_points.size();i++) {
		std::vector<cv::Point3f> points;
		for (int h = 0; h < board_size.height; h++) {
			for (int w = 0; w < board_size.width; w++) {
				points.emplace_back(cv::Point3f(w * square_length, h * square_length, 0));
			}
		}
		object_points.emplace_back(points);
	}
}


void stereoClibrate(const std::vector<cv::Mat>& imgs_l,
	const std::vector<cv::Mat>& imgs_r,
	cv::Size board_size,
	float square_length,
	std::string parameters_file,
	bool isSubCorner) {

	cv::Mat camera_matrix_l, camera_matrix_r;
	cv::Mat dist_coeffs_l, dist_coeffs_r;

	auto image_size = imgs_l[0].size();
	std::vector<std::vector<cv::Point2f>> imgs_l_corners;
	std::vector<std::vector<cv::Point2f>> imgs_r_corners;
	findCorner(imgs_l, board_size, imgs_l_corners, isSubCorner);
	findCorner(imgs_r, board_size, imgs_r_corners, isSubCorner);


	std::vector<std::vector<cv::Point3f>> object_points;
	findObjectPoints(imgs_l_corners, board_size, image_size, square_length, object_points);

	// 
	//单相机标定
	cv::Mat mtx_L, R_L, T_L, mtx_R, R_R, T_R;
	// 左相机标定
	cv::calibrateCamera(object_points, imgs_l_corners, image_size,
		mtx_L, dist_coeffs_l, R_L, T_L);
	camera_matrix_l = getOptimalNewCameraMatrix(mtx_L, dist_coeffs_l,
		image_size, 1,
		image_size, 0);
	// 右相机标定
	cv::calibrateCamera(object_points, imgs_r_corners, image_size,
		mtx_R, dist_coeffs_r, R_R, T_R);
	camera_matrix_r = getOptimalNewCameraMatrix(mtx_R, dist_coeffs_r,
		image_size, 1,
		image_size, 0);
	
	//std::cout << "Left Camera Matrix:   " << camera_matrix_l << std::endl;
	//std::cout << "Right Camera Matrix:   " << camera_matrix_r << std::endl;


	// 立体标定
	cv::Mat R, T, E, F;
	auto rms = cv::stereoCalibrate(object_points, imgs_l_corners, imgs_r_corners,
		camera_matrix_l, dist_coeffs_l,
		camera_matrix_r, dist_coeffs_r,
		image_size, R, T, E, F,
		cv::CALIB_FIX_INTRINSIC,
		cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 100, 1e-6));
	cv::Mat R1, R2, P1, P2, Q;
	stereoRectify(camera_matrix_l, dist_coeffs_l,
		camera_matrix_r, dist_coeffs_r,
		image_size, R, T, R1, R2, P1, P2, Q,
		1);

	std::cout << "re-projection error:	" << rms <<std::endl;
	std::cout << std::endl;
	// save param
	cv::FileStorage fs(parameters_file, cv::FileStorage::WRITE);
	if (fs.isOpened())
	{
		fs << "M1" << camera_matrix_l << "D1" << dist_coeffs_l
			<< "M2" << camera_matrix_r << "D2" << dist_coeffs_r
			<< "R" << R << "T" << T << "R1" << R1
			<< "R2" << R2 << "P1" << P1 << "P2" << P2
			<< "Q" << Q;
		fs.release();
	}
	std::cout << "stereo clibrate finish!!!" << std::endl;
}

// test demo  
void stereoCalibrationDemo(std::string left_img_dir, std::string right_img_dir, std::string param_path) {
	std::string left_imgs_path = left_img_dir;
	std::string right_imgs_path = right_img_dir;
	auto imgs_l = loadImgs(left_imgs_path, cv::IMREAD_GRAYSCALE);
	auto imgs_r = loadImgs(right_imgs_path, cv::IMREAD_GRAYSCALE);

	cv::Size board_size{ 11,8 };
	float square_length = 10;
	std::string clibrate_file_name = param_path;

	// stereo clibrate
	stereoClibrate(imgs_l, imgs_r, board_size, square_length, clibrate_file_name);
}