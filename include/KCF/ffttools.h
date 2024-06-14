#pragma once

#include <opencv2/core.hpp>

namespace FFTTools
{
    cv::Mat fftd(cv::Mat img, bool backwards = false);
    cv::Mat real(cv::Mat img);
    cv::Mat imag(cv::Mat img);
    cv::Mat magnitude(cv::Mat img);
    cv::Mat complexMultiplication(cv::Mat a, cv::Mat b);
    cv::Mat complexDivision(cv::Mat a, cv::Mat b);
    void rearrange(cv::Mat &img);
    void normalizedLogTransform(cv::Mat &img);
}

