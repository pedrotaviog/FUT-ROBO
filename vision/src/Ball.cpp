#include "futbot/Ball.hpp"

#include <format>
#include <stdexcept>
#include <opencv2/imgproc.hpp>

Ball::Ball(const Color& color, const std::string& configFile) : m_color{color}
{
    cv::FileStorage fs(configFile, cv::FileStorage::READ);

    if (!fs.isOpened()) {
        auto errorMsg = std::format("Couldn't read file `{}`", configFile);
        throw std::runtime_error(errorMsg);
    }

    fs["color"]["ballMinArea"] >> m_minArea;
    fs["color"]["ballMinCircularity"] >> m_minCircularity;
    fs["color"]["ballMaxRadius"] >> m_maxRadius;

    fs.release();
}

Ball::~Ball()
{
}

std::optional<std::tuple<cv::Point2f, float, double>> Ball::findCentroid(
    const cv::Mat& frame) const
{
    std::vector<std::vector<cv::Point>> contours;
    m_color.findContours(frame, contours);

    std::optional<std::tuple<cv::Point2f, float, double>> ball = std::nullopt;
    double bestCircularity = m_minCircularity;

    // Pick the roundest blob instead of the largest one: hands, skin and wood
    // may share the ball hue, but they don't fill their enclosing circle
    for (const auto& contour : contours) {
        double contourArea = cv::contourArea(contour);
        if (contourArea < m_minArea) continue;

        cv::Point2f centroid;
        float radius;
        cv::minEnclosingCircle(contour, centroid, radius);
        if (radius <= 0.0f) continue;

        // `ballMaxRadius` is disabled when set to zero
        if (m_maxRadius > 0.0 && radius > m_maxRadius) continue;

        // How much of the enclosing circle the contour actually fills
        double circularity = contourArea / (CV_PI * radius * radius);
        if (circularity < bestCircularity) continue;

        bestCircularity = circularity;
        ball = std::make_tuple(centroid, radius, contourArea);
    }

    return ball;
}
