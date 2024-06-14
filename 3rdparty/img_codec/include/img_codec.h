/*************************************************************************************************************************
 * Copyright 2024 Grifcc
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 *of this software and associated documentation files (the “Software”), to deal
 *in the Software without restriction, including without limitation the rights
 *to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *copies of the Software, and to permit persons to whom the Software is
 *furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 *all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *SOFTWARE.
 *************************************************************************************************************************/
#pragma once

#include <atomic>
#include <cstdint>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

extern "C"
{
#include <libavcodec/avcodec.h>
#include <libavdevice/avdevice.h>
#include <libavformat/avformat.h>
#include <libavutil/error.h>
#include <libavutil/imgutils.h>
#include <libavutil/log.h>
#include <libavutil/mem.h>
#include <libavutil/time.h>
#include <libswscale/swscale.h>
}

struct img_Label
{
    unsigned short id;
    unsigned short x;
    unsigned short y;
    unsigned short w;
    unsigned short h;
    float conf;
    unsigned char cls;
};

struct WinInfo
{
    unsigned int longitude;
    unsigned int latitude;
    unsigned int altitude;
    unsigned short x;
    unsigned short y;
    unsigned char win_idx;
};

class ImgEncode
{
public:
    ImgEncode(const std::string codec = "hevc_nvmpi", const int bit_rate = 500000,
              const int witdh = 512, const int height = 512,
              const int log_level = AV_LOG_QUIET);

    ~ImgEncode();

    bool encode(const cv::Mat &img, const std::vector<img_Label> &labels,
                const WinInfo win_info, unsigned char **dst, int &size);

private:
    void mat2frame(const cv::Mat &, AVFrame *frame);

    const AVPixelFormat fmt_ = AV_PIX_FMT_NV12;

    const std::string des_codec_;
    const int64_t bit_rate_;
    const int width_;
    const int height_;

    int64_t pts_ = 0;
    int64_t dts_ = 0;

    AVCodecContext *encodeContext_ = nullptr;
    SwsContext *swsContext_ = nullptr;
    AVCodec *encodec_ = nullptr;
    AVPacket *packet_;
    AVFrame *frame_;
};

class ImgDecode
{
public:
    ImgDecode(const std::string codec = "hevc_nvmpi", const int witdh = 512,
              const int height = 512, const int log_level = AV_LOG_QUIET);

    ~ImgDecode();

    bool decode(unsigned char *src, int size, cv::Mat &img,
                std::vector<img_Label> &labels, WinInfo &win_info);

private:
    void frame2mat(AVFrame *&frame, cv::Mat &mat);

    const AVPixelFormat fmt_ = AV_PIX_FMT_YUV420P;

    const std::string des_codec_;
    const int width_;
    const int height_;

    int64_t pts_ = 0;
    int64_t dts_ = 0;

    AVCodecContext *decodeContext_ = nullptr;
    SwsContext *swsContext_ = nullptr;
    AVCodec *decodec_ = nullptr;
    AVPacket *packet_;
    AVFrame *frame_;
};
