#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <vector>


class calibration
{
public:
    calibration();
    std::vector<cv::Mat> convertCSVToMat(std::string filename);
    std::vector <cv::Mat> chessboardPose(cv::Mat img_rgb);
    std::vector<cv::Mat> handToEyeCalibration(std::string img_folder, std::string rtvec_folder);
    cv::Mat euler2rot(const cv::Mat & euler);
};

#endif 
