#pragma once
#pragma once
#include <iostream>
#include <vector>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <chrono>

/**
*	@brief	����Ŀ���ļ�������ͼƬ
*	@param	img_dir	Ŀ���ļ���
*	@param	imread_mode	ͼ���ȡģʽ
*
*	@return	imgs
*/
std::vector<cv::Mat> loadImgs(std::string imgs_dir, cv::ImreadModes imread_mode);

// ͼƬ��ʾ
void displayImage(const cv::Mat& image, std::string windows_name = "Display window", float h = 512);

// ��ʱ��
class Timer {
public:
    Timer() : start_(std::chrono::high_resolution_clock::now()) {}

    void reset() {
        start_ = std::chrono::high_resolution_clock::now();
    }

    double elapsed() const {
        auto now = std::chrono::high_resolution_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - start_);
        return elapsed.count() / 1000.0;
    }

private:
    std::chrono::time_point<std::chrono::high_resolution_clock> start_;
};