#ifndef BALL_HPP
#define BALL_HPP

#include <opencv2/core.hpp>
#include <optional>
#include <string>
#include "futbot/Color.hpp"

class Ball {
public:
    Ball(const Color& color, const std::string& configFile);
    virtual ~Ball();

    std::optional<std::tuple<cv::Point2f, float, double>> findCentroid(
        const cv::Mat& frame) const;

private:
    const Color& m_color;
    double m_minArea;
    double m_minCircularity;
    double m_maxRadius;
};

#endif // BALL_HPP
