#ifndef RECTTOOLS_H
#define RECTTOOLS_H

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace RectTools
{

template <typename t>
cv::Vec<t, 2> center(const cv::Rect_<t> &rect);

template <typename t>
t x2(const cv::Rect_<t> &rect);

template <typename t>
t y2(const cv::Rect_<t> &rect);

template <typename t>
void resize(cv::Rect_<t> &rect, float scalex, float scaley = 0);

template <typename t>
void limit(cv::Rect_<t> &rect, cv::Rect_<t> limit);

template <typename t>
void limit(cv::Rect_<t> &rect, t width, t height, t x = 0, t y = 0);

template <typename t>
cv::Rect getBorder(const cv::Rect_<t> &original, cv::Rect_<t> &limited);

cv::Mat subwindow(const cv::Mat &in, const cv::Rect &window, int borderType = cv::BORDER_CONSTANT);

cv::Mat getGrayImage(cv::Mat img);

} // namespace RectTools

#endif // RECTTOOLS_H
