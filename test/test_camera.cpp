#include <gtest/gtest.h>

#include "trialSlam/common.h"
#include "trialSlam/camera.h"

TEST(TrialSlamTestSuite, TestCamera) {
    std::shared_ptr<trialSlam::Camera> camera(new trialSlam::Camera(
        816.011676, 811.145514, 349.590470, 230.759915,
        0.040870, 0.181320, -0.003507, 0.006012, 0.000000,
        2
    ));

    camera -> open();
    cv::namedWindow("Camera Test (RGB)", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Camera Test (GRAY)", cv::WINDOW_AUTOSIZE);

    cv::Mat shot;
    while (camera -> capture(shot)) {
        cv::imshow("Camera Test (RGB)", shot);

        cv::Mat gray;
        cv::cvtColor(shot, gray, cv::COLOR_RGB2GRAY);
        cv::imshow("Camera Test (GRAY)", gray);

        if (cv::waitKey(1) == 27) break;
    }

    camera -> close();
}

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
