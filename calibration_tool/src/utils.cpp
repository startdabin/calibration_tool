#include "utils.h"


/**
*	@brief	����Ŀ���ļ�������ͼƬ
*	@param	img_dir	Ŀ���ļ���
*	@param	imread_mode	ͼ���ȡģʽ
*
*	@return	imgs
*/
std::vector<cv::Mat> loadImgs(std::string imgs_dir, cv::ImreadModes imread_mode) {
    std::vector<cv::Mat> imgs;
    std::vector<std::string> imgs_path;
    cv::glob(imgs_dir, imgs_path);
    for (auto img_path : imgs_path) {
        imgs.push_back(cv::imread(img_path, imread_mode));
    }
    return imgs;
}

// ͼƬ��ʾ
void displayImage(const cv::Mat& image, std::string windows_name, float h) {

    cv::namedWindow(windows_name, cv::WINDOW_NORMAL);
    int w = int(h / image.rows * image.cols);
    cv::resizeWindow(windows_name, w, h);
    cv::imshow(windows_name, image);
    cv::waitKey();
}

