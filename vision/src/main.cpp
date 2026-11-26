#include <cstdlib>
#include <format>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <print>
#include <string>
#include <cmath>
#include <array>
#include <cstdio>
#include <functional>
#include <string_view>
#include <unistd.h>
#include "futbot/Video.hpp"
#include "futbot/Calibration.hpp"
#include "futbot/Team.hpp"
#include "futbot/Publisher.hpp"
#include "futbot/Ball.hpp"
#include "common/protocol.h"

constexpr float RAD2DEG = 180.0f / M_PIf;
constexpr std::string CONFIG_FILE = "config.yaml";

constexpr char MULTICAST_GROUP[] = "224.1.1.1";
constexpr int PORT = 5007;

static inline float getTheta(const cv::Point2f pt1, const cv::Point2f pt2)
{
    return std::atan2(pt1.y - pt2.y, pt1.x - pt2.x);
}

// What to do with a saved calibration found on startup
enum class CalibMode { Ask, Recalibrate, Reuse };

static bool askYesNo(const std::string& question)
{
    std::print("{} [s/N] ", question);
    std::fflush(stdout);

    std::string answer;
    if (!std::getline(std::cin, answer)) {
        return false;
    }
    return !answer.empty() && (answer[0] == 's' || answer[0] == 'S');
}

// Decides whether to recalibrate something that has a saved calibration
static bool shouldRecalibrate(CalibMode mode, bool hasSaved, const std::string& question)
{
    if (!hasSaved || mode == CalibMode::Recalibrate) return true;
    if (mode == CalibMode::Reuse) return false;
    return askYesNo(question);
}

int main(int argc, char* argv[])
{
    CalibMode mode = CalibMode::Ask;
    for (int i = 1; i < argc; i++) {
        std::string_view arg = argv[i];
        if (arg == "--calibrar") {
            mode = CalibMode::Recalibrate;
        } else if (arg == "--aproveitar") {
            mode = CalibMode::Reuse;
        } else {
            std::println("Uso: {} [--calibrar | --aproveitar]", argv[0]);
            std::println("  (sem opcao)    pergunta se quer recalibrar a calibracao salva");
            std::println("  --calibrar     refaz a calibracao do campo e das cores");
            std::println("  --aproveitar   usa a calibracao salva sem perguntar");
            return arg == "-h" || arg == "--help" ? 0 : 1;
        }
    }

    // Without a terminal to answer (e.g. started by the IHM), keep the saved calibration
    if (mode == CalibMode::Ask && !(isatty(STDIN_FILENO) && isatty(STDOUT_FILENO))) {
        mode = CalibMode::Reuse;
    }

    Color orange("laranja", CONFIG_FILE), green("verde", CONFIG_FILE),
        blue("azul", CONFIG_FILE), pink("rosa", CONFIG_FILE),
        yellow("amarelo", CONFIG_FILE);
    std::array<std::reference_wrapper<Color>, 5> colors{orange, green, blue, pink, yellow};
    Calibration calib(CONFIG_FILE);

    bool hasSavedColors = false;
    for (const Color& color : colors) {
        hasSavedColors = hasSavedColors || color.hasFile();
    }

    bool recalibrateField = shouldRecalibrate(mode, Calibration::hasSavedFile(),
        "Calibracao do campo encontrada. Recalibrar?");
    bool recalibrateColors = shouldRecalibrate(mode, hasSavedColors,
        "Calibracao das cores encontrada. Recalibrar?");

    Video video(CONFIG_FILE);
    Ball ball(orange, CONFIG_FILE);
    std::array<Team, 2> teams{
        Team{green, pink, yellow, CONFIG_FILE},
        Team{blue, pink, yellow, CONFIG_FILE}
    };

    Publisher publisher(MULTICAST_GROUP, PORT);
    FieldPoses field{};

    auto buffer = reinterpret_cast<const uint8_t*>(&field);
    size_t bufsize = sizeof(FieldPoses);

    try {
        int key;

        calib.calibrate(video, recalibrateField);

        for (Color& color : colors) {
            if (recalibrateColors || !color.readFile()) color.select(video);
        }

        do {
            video.updateFrame();

            // Find the ball pose
            auto ballResult = ball.findCentroid(video.frameHsv());
            if (ballResult.has_value()) {
                const auto& [ballCentroid, ballRadius, ballArea] = ballResult.value();
                cv::Point2f ballRealCentroid = calib.uvToXy(ballCentroid);
                std::string ballMsg = std::format("{:.0f},{:.0f}\nArea={:.0f}",
                    ballRealCentroid.x, ballRealCentroid.y, ballArea);
                // std::println("ball radius: {}", ballRadius); // rascunho
                // publisher.publish(ballMsg, "BALL");
                video.drawCircle(ballCentroid, ballRadius, Color::CYAN);
                video.putText(ballMsg, ballCentroid);

                field.ball.x = ballRealCentroid.x;
                field.ball.y = ballRealCentroid.y;
            } else {
                std::println("Ball wasn't found!");
            }

            // Find the four robots; there's two teams, and each team has two robots
            for (int i = 0; i < 2; i++) {
                Team& team = teams[i];
                team.findPoses(video);

                for (int j = 0; j < 2; j++) {
                    const Team::Player& player = team.players()[j];
                    cv::Point2f playerRealRectCentroid = calib.uvToXy(player.centroidRect);
                    cv::Point2f playerRealCircleCentroid = calib.uvToXy(player.centroidCircle);
                    float playerAngle = getTheta(playerRealCircleCentroid, playerRealRectCentroid);
                    std::string playerMsg = std::format("{:.0f},{:.0f},{:.0f}\nArea={:.2f}",
                        playerRealRectCentroid.x, playerRealRectCentroid.y, playerAngle * RAD2DEG, player.rectArea);
                    video.putText(playerMsg, player.centroidRect);

                    uint8_t robot_id = i + (j << 1);
                    field.robots[robot_id].id = robot_id;
                    field.robots[robot_id].position.x = playerRealRectCentroid.x;
                    field.robots[robot_id].position.y = playerRealRectCentroid.y;
                    field.robots[robot_id].angle = playerAngle;
                }
            }

            publisher.send(buffer, bufsize);

            key = video.showFrame();
        } while (key != 27);
    } catch (const std::exception& e) {
        std::println(std::cerr, "Error: {}", e.what());
        return 1;
    }

    return 0;
}
