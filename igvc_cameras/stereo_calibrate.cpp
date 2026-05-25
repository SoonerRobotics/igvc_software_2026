#include <iostream>
#include <iomanip>
#include <vector>
#include <string>
#include <chrono>

#include <opencv2/opencv.hpp>

static constexpr int    INNER_COLS     = 7;
static constexpr int    INNER_ROWS     = 9;
static constexpr float  SQUARE_SIZE    = 25.4f;
static constexpr double REQUIRED_SECS  = 3.0;
static constexpr int    MIN_FRAMES     = 20;
static constexpr int    MISS_TOLERANCE = 8;

int main(int argc, char** argv)
{
    if (argc < 3) {
        std::cerr << "Usage: stereo_calibrate <left_camera> <right_camera>\n";
        return 1;
    }

    auto openCam = [](const std::string& arg) {
        cv::VideoCapture cap;
        try {
            int idx = std::stoi(arg);
            cap.open(idx, cv::CAP_V4L2);
        } catch (...) {
            cap.open(arg, cv::CAP_V4L2);
        }
        cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
        cap.set(cv::CAP_PROP_FRAME_WIDTH,  640);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);
        return cap;
    };

    cv::VideoCapture leftCap  = openCam(argv[1]);
    cv::VideoCapture rightCap = openCam(argv[2]);

    if (!leftCap.isOpened() || !rightCap.isOpened()) {
        std::cerr << "Failed to open one or both cameras.\n";
        return 1;
    }

    cv::Size boardSize(INNER_COLS, INNER_ROWS);

    std::vector<cv::Point3f> objTemplate;
    for (int r = 0; r < INNER_ROWS; r++)
        for (int c = 0; c < INNER_COLS; c++)
            objTemplate.emplace_back(c * SQUARE_SIZE, r * SQUARE_SIZE, 0.f);

    std::vector<std::vector<cv::Point3f>> allObjPoints;
    std::vector<std::vector<cv::Point2f>> allLeftPoints;
    std::vector<std::vector<cv::Point2f>> allRightPoints;

    cv::Size imageSize;
    std::chrono::steady_clock::time_point firstDetection;
    bool timerRunning = false;
    int  missCount    = 0;

    std::cout << "Show the chessboard to BOTH cameras simultaneously.\n"
              << "Hold steady for " << REQUIRED_SECS << "s to collect "
              << MIN_FRAMES << " frames...\n\n";

    cv::Mat leftFrame, rightFrame, leftGray, rightGray, leftBlur, rightBlur;

    while (true)
    {
        if (!leftCap.read(leftFrame)   || leftFrame.empty() ||
            !rightCap.read(rightFrame) || rightFrame.empty()) {
            std::cerr << "Failed to read from cameras.\n";
            break;
        }

        imageSize = leftFrame.size();

        cv::cvtColor(leftFrame,  leftGray,  cv::COLOR_BGR2GRAY);
        cv::cvtColor(rightFrame, rightGray, cv::COLOR_BGR2GRAY);
        cv::bilateralFilter(leftGray,  leftBlur,  5, 75, 75);
        cv::bilateralFilter(rightGray, rightBlur, 5, 75, 75);

        std::vector<cv::Point2f> leftCorners, rightCorners;
        bool foundLeft  = cv::findChessboardCorners(leftBlur,  boardSize, leftCorners,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);
        bool foundRight = cv::findChessboardCorners(rightBlur, boardSize, rightCorners,
            cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);

        if (!foundLeft || !foundRight) {
            missCount++;
            if (missCount >= MISS_TOLERANCE) {
                timerRunning = false;
                missCount    = 0;
                allObjPoints.clear();
                allLeftPoints.clear();
                allRightPoints.clear();
                std::cout << "\r[--] "
                          << (!foundLeft  ? "Left missing   " : "               ")
                          << (!foundRight ? "Right missing  " : "               ")
                          << "— resetting...     " << std::flush;
            } else {
                std::cout << "\r[~~] Brief miss (" << missCount << "/" << MISS_TOLERANCE << ")                       " << std::flush;
            }
            continue;
        }

        missCount = 0;

        cv::cornerSubPix(leftGray,  leftCorners,  {11,11}, {-1,-1},
            {cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.1});
        cv::cornerSubPix(rightGray, rightCorners, {11,11}, {-1,-1},
            {cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.1});

        if (!timerRunning) {
            firstDetection = std::chrono::steady_clock::now();
            timerRunning   = true;
        }

        double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - firstDetection).count();

        allObjPoints.push_back(objTemplate);
        allLeftPoints.push_back(leftCorners);
        allRightPoints.push_back(rightCorners);

        std::cout << "\r[OK] Both boards found. Hold still... "
                  << std::fixed << std::setprecision(1) << elapsed
                  << "/" << REQUIRED_SECS << "s  ("
                  << allObjPoints.size() << " frames)     " << std::flush;

        if (elapsed >= REQUIRED_SECS && (int)allObjPoints.size() >= MIN_FRAMES)
            break;
    }

    leftCap.release();
    rightCap.release();

    if ((int)allObjPoints.size() < MIN_FRAMES) {
        std::cerr << "\nNot enough frames collected.\n";
        return 1;
    }

    std::cout << "\nRunning stereo calibration...\n";

    cv::Mat K1 = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat K2 = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat D1, D2, R, T, E, F;

    double rms = cv::stereoCalibrate(
        allObjPoints,
        allLeftPoints,
        allRightPoints,
        K1, D1, K2, D2,
        imageSize,
        R, T, E, F,
        cv::CALIB_FIX_ASPECT_RATIO |
        cv::CALIB_ZERO_TANGENT_DIST |
        cv::CALIB_SAME_FOCAL_LENGTH,
        {cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 100, 1e-5});

    cv::Mat R1, R2, P1, P2, Q;
    cv::stereoRectify(
        K1, D1, K2, D2,
        imageSize, R, T,
        R1, R2, P1, P2, Q,
        cv::CALIB_ZERO_DISPARITY,
        0,
        imageSize);

    std::cout << "\n╔══════════════════════════════════════════════════════════╗\n";
    std::cout <<   "║           STEREO CALIBRATION COMPLETE                    ║\n";
    std::cout <<   "╠══════════════════════════════════════════════════════════╣\n";
    std::cout <<   "║  Frames used : " << std::setw(42) << std::left << allObjPoints.size() << "║\n";
    std::cout <<   "║  RMS error   : " << std::setw(42) << std::left << std::setprecision(6) << std::fixed << rms << "║\n";

    auto printMat3x3 = [](const std::string& label, const cv::Mat& m) {
        std::cout << "╠══════════════════════════════════════════════════════════╣\n";
        std::cout << "║  " << label << "\n";
        for (int r = 0; r < 3; r++) {
            std::cout << "║  [";
            for (int c = 0; c < 3; c++)
                std::cout << std::setw(13) << std::fixed << std::setprecision(5)
                          << m.at<double>(r, c) << (c < 2 ? "  " : "");
            std::cout << " ]  ║\n";
        }
    };

    printMat3x3("Left  Camera Matrix (K1):", K1);
    printMat3x3("Right Camera Matrix (K2):", K2);
    printMat3x3("Rotation (R) — right relative to left:", R);

    std::cout << "╠══════════════════════════════════════════════════════════╣\n";
    std::cout << "║  Translation (T) — right relative to left (mm):         ║\n";
    std::cout << "║  [ " << std::setw(10) << std::fixed << std::setprecision(4) << T.at<double>(0)
              <<   "  "  << std::setw(10) << T.at<double>(1)
              <<   "  "  << std::setw(10) << T.at<double>(2) << " ]            ║\n";
    std::cout << "╠══════════════════════════════════════════════════════════╣\n";
    std::cout << "║  Baseline: " << std::setprecision(2) << cv::norm(T) << " mm\n";

    std::cout << "╠══════════════════════════════════════════════════════════╣\n";
    std::cout << "║  Copy-pasteable values:                                  ║\n";
    std::cout << "╚══════════════════════════════════════════════════════════╝\n\n";

    auto printRow = [](const std::string& name, const cv::Mat& m) {
        std::cout << name << " = {";
        cv::Mat flat = m.reshape(1, 1);
        for (int i = 0; i < flat.cols; i++) {
            if (i) std::cout << ", ";
            std::cout << std::setprecision(8) << std::fixed << flat.at<double>(i);
        }
        std::cout << "};\n";
    };

    printRow("K1", K1);
    printRow("D1", D1);
    printRow("K2", K2);
    printRow("D2", D2);
    printRow("R",  R);
    printRow("T",  T);
    printRow("R1", R1);
    printRow("R2", R2);
    printRow("P1", P1);
    printRow("P2", P2);

    return 0;
}