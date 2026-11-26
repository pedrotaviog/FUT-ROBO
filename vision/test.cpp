#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

constexpr int FPS = 30;
static const std::string FILENAME{"test/output.avi"};
static const std::string WINNAME{"window"};

int main() {
    cv::VideoCapture cap(FILENAME);
    cv::Mat frame, frameHsv, frameS, frameBin, frameMask;
    int key = 0;

    if (!cap.isOpened()) {
        throw std::runtime_error(std::format("Couldn't open `{}`", FILENAME));
    }

    cv::namedWindow(WINNAME);

    do {
        cap.read(frame);

        if (frame.empty()) {
            // Rewind video if it ends
            cap.set(cv::CAP_PROP_POS_FRAMES, 0);
            cap.read(frame);
        }

        // Convert BGR to HSV
        cv::cvtColor(frame, frameHsv, cv::COLOR_BGR2HSV);

        // Extracts channel 1 (Saturation)
        cv::extractChannel(frameHsv, frameS, 1);

        // double thresh = 40;
        // cv::threshold(frameS, frameBin, thresh, 255, cv::THRESH_BINARY);

        // // Apply the binary mask to the original BGR frame
        // cv::bitwise_and(frame, frame, frameMask, frameBin);

        cv::imshow(WINNAME, frameS);
        key = cv::waitKey(1000 / FPS);
    } while (key != 27); // ESC

    cv::destroyWindow(WINNAME);

    return 0;
}
