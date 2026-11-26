#ifndef CALIBRATION_HPP
#define CALIBRATION_HPP

#include <opencv2/core.hpp>
#include "futbot/Video.hpp"

#include <string>
#include <vector>

class Calibration {
public:
    // Lê o tamanho real do campo (seção `field` do config.yaml)
    explicit Calibration(const std::string& configFile);

    // If `recalibrate` is false, reuses the saved calibration when it exists
    void calibrate(Video& video, bool recalibrate = false);
    static bool hasSavedFile();
    cv::Point2f uvToXy(const cv::Point2f& uv) const;

private:
    void computeHomography(const std::vector<cv::Point2f>& uvPoints);

    float m_fieldSizeX = 0.0f;   // cm, entre o 1o e o 2o canto clicados
    float m_fieldSizeY = 0.0f;   // cm, entre o 2o e o 3o canto clicados
    cv::Mat m_homographyMatrix;
};

#endif // CALIBRATION_HPP
