#include "futbot/Calibration.hpp"
#include <array>
#include <filesystem>
#include <format>
#include <print>
#include <stdexcept>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>

constexpr std::string CALIB_CONFIG_FILE = "data/calib.yaml";

// Ordem em que os cantos são clicados, como aparecem na imagem.
static const std::array<const char*, 4> CORNER_NAMES = {
    "superior esquerdo", "superior direito", "inferior direito", "inferior esquerdo"
};

Calibration::Calibration(const std::string& configFile)
{
    cv::FileStorage fs(configFile, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        throw std::runtime_error(std::format("Couldn't open file `{}`", configFile));
    }
    fs["field"]["sizeX_cm"] >> m_fieldSizeX;
    fs["field"]["sizeY_cm"] >> m_fieldSizeY;
    fs.release();

    if (m_fieldSizeX <= 0.0f || m_fieldSizeY <= 0.0f) {
        throw std::runtime_error(std::format(
            "`{}` precisa de field.sizeX_cm e field.sizeY_cm (tamanho real do campo em cm)", configFile));
    }
}

bool Calibration::hasSavedFile()
{
    return std::filesystem::exists(CALIB_CONFIG_FILE);
}

void Calibration::computeHomography(const std::vector<cv::Point2f>& uvPoints)
{
    // Cantos do campo em centímetros reais, origem no centro, na ordem dos
    // cliques. As duas escalas precisam ser as reais: escala diferente em
    // cada eixo distorce todos os ângulos medidos.
    const float halfX = m_fieldSizeX / 2.0f;
    const float halfY = m_fieldSizeY / 2.0f;
    const std::vector<cv::Point2f> xyPoints = {
        {-halfX,  halfY},
        { halfX,  halfY},
        { halfX, -halfY},
        {-halfX, -halfY},
    };
    m_homographyMatrix = cv::findHomography(uvPoints, xyPoints);
    std::println("Campo: {:g} x {:g} cm", m_fieldSizeX, m_fieldSizeY);
}

void Calibration::calibrate(Video& video, bool recalibrate)
{
    cv::FileStorage fs;

    // Reaproveita os cantos clicados antes; a homografia é recalculada com o
    // tamanho atual do campo, então mudar o config.yaml não exige clicar de novo.
    if (!recalibrate) {
        fs.open(CALIB_CONFIG_FILE, cv::FileStorage::READ);
        if (fs.isOpened()) {
            std::vector<cv::Point2f> savedPoints;
            fs["uvPoints"] >> savedPoints;
            fs.release();
            if (savedPoints.size() == CORNER_NAMES.size()) {
                computeHomography(savedPoints);
                return;
            }
            std::println("`{}` nao tem os cantos clicados; recalibrando o campo.", CALIB_CONFIG_FILE);
        }
    }

    std::vector<cv::Point> uvPoints;

    // Sets mouse callback function
    cv::setMouseCallback(
        video.windowName(),
        [](int event, int x, int y, int flags, void* userdata) -> void {
            (void)flags; // Prevent unused parameter warning
            auto& uvPointsRef =
                *static_cast<std::vector<cv::Point>*>(userdata);

            if (event == cv::EVENT_LBUTTONDOWN) {
                uvPointsRef.emplace_back(x, y);
                std::println("Ponto {} selecionado: {}, {}", uvPointsRef.size(), x, y);
            }
        },
        &uvPoints
    );

    std::println("Clique os 4 cantos do campo em sentido horario, comecando pelo superior esquerdo.");
    std::println("O lado entre o 1o e o 2o canto mede field.sizeX_cm ({:g} cm) no config.yaml.", m_fieldSizeX);

    // Loop until user selects all points
    while (uvPoints.size() < CORNER_NAMES.size()) {
        video.updateFrame();
        video.putText(std::format("Clique o canto {} ({}/4)", CORNER_NAMES[uvPoints.size()], uvPoints.size() + 1));
        int key = video.showFrame();
        if (key == 27) { // key == ESC
            exit(EXIT_SUCCESS);
        }
    }

    // Unsets mouse callback function
    cv::setMouseCallback(video.windowName(), nullptr);

    const std::vector<cv::Point2f> uvPointsF(uvPoints.begin(), uvPoints.end());
    computeHomography(uvPointsF);

    // Salva os cantos clicados (a homografia vai junto só para referência)
    fs.open(CALIB_CONFIG_FILE, cv::FileStorage::WRITE);
    fs << "uvPoints" << uvPointsF;
    fs << "homographyMatrix" << m_homographyMatrix;
    fs.release();
}

cv::Point2f Calibration::uvToXy(const cv::Point2f& uv) const
{
    // Although we only want to transform a single point, the
    // `cv::perspectiveTransform` function only accepts vectors of points
    std::vector<cv::Point2f> src = { uv };
    std::vector<cv::Point2f> dst;

    cv::perspectiveTransform(src, dst, m_homographyMatrix);

    return dst.front();
}
