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
static constexpr int    MIN_FRAMES     = 10;
static constexpr int    MISS_TOLERANCE = 8;

int main(int argc, char** argv)
{
    if (argc < 2) {
        std::cerr << "Usage: calibrate <camera_path_or_index>\n";
        return 1;
    }

    cv::VideoCapture cap;
    std::string arg = argv[1];
    try {
        int idx = std::stoi(arg);
        cap.open(idx, cv::CAP_V4L2);
    } catch (...) {
        cap.open(arg, cv::CAP_V4L2);
    }

    if (!cap.isOpened()) {
        std::cerr << "Failed to open camera: " << arg << "\n";
        return 1;
    }

    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
    cap.set(cv::CAP_PROP_FRAME_WIDTH,  640);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480);

    cv::Size boardSize(INNER_COLS, INNER_ROWS);

    std::vector<cv::Point3f> objTemplate;
    for (int r = 0; r < INNER_ROWS; r++)
        for (int c = 0; c < INNER_COLS; c++)
            objTemplate.emplace_back(c * SQUARE_SIZE, r * SQUARE_SIZE, 0.f);

    std::vector<std::vector<cv::Point3f>> allObjPoints;
    std::vector<std::vector<cv::Point2f>> allImgPoints;

    cv::Size imageSize;
    std::chrono::steady_clock::time_point firstDetection;
    bool timerRunning = false;
    bool calibrated   = false;
    int  missCount    = 0;

    std::cout << "Point camera at chessboard (" << INNER_COLS << "x" << INNER_ROWS
              << " inner corners, " << SQUARE_SIZE << "mm squares).\n"
              << "Hold steady for " << REQUIRED_SECS << "s...\n";

    cv::Mat frame, gray, blurred;

    while (!calibrated)
    {
        if (!cap.read(frame) || frame.empty()) {
            std::cerr << "Failed to read frame.\n";
            break;
        }

        imageSize = frame.size();
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        cv::bilateralFilter(gray, blurred, 5, 75, 75);

        std::vector<cv::Point2f> corners;
        bool found = cv::findChessboardCorners(
            blurred, boardSize, corners,
            cv::CALIB_CB_ADAPTIVE_THRESH |
            cv::CALIB_CB_NORMALIZE_IMAGE |
            cv::CALIB_CB_FAST_CHECK);

        if (!found || corners.empty()) {
            missCount++;
            if (missCount >= MISS_TOLERANCE) {
                timerRunning = false;
                missCount    = 0;
                allObjPoints.clear();
                allImgPoints.clear();
                std::cout << "\r[--] Board lost, resetting...                    " << std::flush;
            } else {
                std::cout << "\r[~~] Brief miss (" << missCount << "/" << MISS_TOLERANCE << ")                  " << std::flush;
            }
            continue;
        }

        missCount = 0;

        cv::cornerSubPix(
            gray, corners, cv::Size(11, 11), cv::Size(-1, -1),
            cv::TermCriteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.1));

        if (!timerRunning) {
            firstDetection = std::chrono::steady_clock::now();
            timerRunning   = true;
        }

        double elapsed = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - firstDetection).count();

        allObjPoints.push_back(objTemplate);
        allImgPoints.push_back(corners);

        std::cout << "\r[OK] Hold still... "
                  << std::fixed << std::setprecision(1) << elapsed
                  << "/" << REQUIRED_SECS << "s  ("
                  << allObjPoints.size() << " frames)          " << std::flush;

        if (elapsed >= REQUIRED_SECS && (int)allObjPoints.size() >= MIN_FRAMES)
        {
            std::cout << "\nRunning calibration...\n";

            cv::Mat cameraMatrix, distCoeffs;
            std::vector<cv::Mat> rvecs, tvecs;

            double rms = cv::calibrateCamera(
                allObjPoints, allImgPoints, imageSize,
                cameraMatrix, distCoeffs, rvecs, tvecs);

            std::cout << "\n╔══════════════════════════════════════════════════╗\n";
            std::cout <<   "║          CALIBRATION COMPLETE                    ║\n";
            std::cout <<   "╠══════════════════════════════════════════════════╣\n";
            std::cout <<   "║  Frames used      : " << std::setw(29) << std::left << allObjPoints.size() << "║\n";
            std::cout <<   "║  RMS reprojection : " << std::setw(29) << std::left << std::setprecision(6) << std::fixed << rms << "║\n";
            std::cout <<   "╠══════════════════════════════════════════════════╣\n";
            std::cout <<   "║  Camera Matrix:                                  ║\n";
            for (int r = 0; r < 3; r++) {
                std::cout << "║  [";
                for (int c = 0; c < 3; c++)
                    std::cout << std::setw(12) << std::setprecision(4) << std::fixed
                              << cameraMatrix.at<double>(r, c)
                              << (c < 2 ? "  " : "");
                std::cout << " ]  ║\n";
            }
            std::cout <<   "╠══════════════════════════════════════════════════╣\n";
            std::cout <<   "║  fx=" << std::setprecision(2) << cameraMatrix.at<double>(0,0)
                      <<   "  fy=" << cameraMatrix.at<double>(1,1)
                      <<   "  cx=" << cameraMatrix.at<double>(0,2)
                      <<   "  cy=" << cameraMatrix.at<double>(1,2) << "\n";
            std::cout <<   "╠══════════════════════════════════════════════════╣\n";
            std::cout <<   "║  Distortion (k1 k2 p1 p2 k3):                   ║\n";
            std::cout <<   "║  [";
            for (int i = 0; i < distCoeffs.cols; i++)
                std::cout << std::setw(11) << std::setprecision(6) << std::fixed
                          << distCoeffs.at<double>(0, i)
                          << (i < distCoeffs.cols - 1 ? "  " : "");
            std::cout << " ]  ║\n";
            std::cout <<   "╚══════════════════════════════════════════════════╝\n";

            calibrated = true;
        }
    }

    cap.release();
    return calibrated ? 0 : 1;
}