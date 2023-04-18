#pragma once
#pragma once
#include <iostream>
#include <vector>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <chrono>

/**
*	@brief	加载目标文件夹所有图片
*	@param	img_dir	目标文件夹
*	@param	imread_mode	图像读取模式
*
*	@return	imgs
*/
std::vector<cv::Mat> loadImgs(std::string imgs_dir, cv::ImreadModes imread_mode);

// 图片显示
void displayImage(const cv::Mat& image, std::string windows_name = "Display window", float h = 512);

// 计时器
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