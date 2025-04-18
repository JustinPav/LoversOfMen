#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/videoio.hpp>
#include <iostream>
#include <vector>

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <ref_image_path> [camera_index]\n";
        return -1;
    }
    std::string refPath = argv[1];
    int camIndex = 0;
    if (argc >= 3) camIndex = std::stoi(argv[2]);

    // Load and preprocess reference image
    cv::Mat refImg = cv::imread(refPath, cv::IMREAD_COLOR);
    if (refImg.empty()) {
        std::cerr << "Error: could not load reference image from " << refPath << std::endl;
        return -1;
    }
    cv::Mat refGray;
    cv::cvtColor(refImg, refGray, cv::COLOR_BGR2GRAY);

    // Initialize ORB detector and BFMatcher
    cv::Ptr<cv::ORB> orb = cv::ORB::create(1000);
    std::vector<cv::KeyPoint> kpRef;
    cv::Mat desRef;
    orb->detectAndCompute(refGray, cv::noArray(), kpRef, desRef);
    cv::BFMatcher matcher(cv::NORM_HAMMING);

    // Open camera
    cv::VideoCapture cap(camIndex);
    if (!cap.isOpened()) {
        std::cerr << "Error: could not open camera index " << camIndex << std::endl;
        return -1;
    }

    // Reference image corner points
    std::vector<cv::Point2f> refCorners = {
        cv::Point2f(0, 0),
        cv::Point2f((float)refGray.cols, 0),
        cv::Point2f((float)refGray.cols, (float)refGray.rows),
        cv::Point2f(0, (float)refGray.rows)
    };

    while (true) {
        cv::Mat frame, gray;
        if (!cap.read(frame)) break;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        // Detect and compute features in frame
        std::vector<cv::KeyPoint> kpFrame;
        cv::Mat desFrame;
        orb->detectAndCompute(gray, cv::noArray(), kpFrame, desFrame);

        if (!desFrame.empty() && kpFrame.size() >= 10) {
            // KNN match and ratio test
            std::vector<std::vector<cv::DMatch>> knnMatches;
            matcher.knnMatch(desRef, desFrame, knnMatches, 2);
            std::vector<cv::DMatch> goodMatches;
            for (auto& m : knnMatches) {
                if (m.size() == 2 && m[0].distance < 0.75f * m[1].distance) {
                    goodMatches.push_back(m[0]);
                }
            }

            // Homography if enough matches
            if (goodMatches.size() > 15) {
                std::vector<cv::Point2f> ptsRef, ptsFrame;
                for (auto& m : goodMatches) {
                    ptsRef.push_back(kpRef[m.queryIdx].pt);
                    ptsFrame.push_back(kpFrame[m.trainIdx].pt);
                }
                cv::Mat mask;
                cv::Mat H = cv::findHomography(ptsRef, ptsFrame, cv::RANSAC, 5.0, mask);
                if (!H.empty()) {
                    // Project and draw polygon
                    std::vector<cv::Point2f> dstCorners;
                    cv::perspectiveTransform(refCorners, dstCorners, H);
                    std::vector<cv::Point> dstCornersInt;
                    for (auto& p : dstCorners) {
                        dstCornersInt.emplace_back(cv::Point(std::round(p.x), std::round(p.y)));
                    }
                    cv::polylines(frame, dstCornersInt, true, cv::Scalar(0, 255, 0), 3);
                }
            }
        }

        // Display
        cv::imshow("Object Detection", frame);
        char key = (char)cv::waitKey(1);
        if (key == 'q' || key == 27) break;
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}
