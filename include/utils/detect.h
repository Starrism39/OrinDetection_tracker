#include "NvInfer.h"
#include <cuda_runtime_api.h>
#include <NvOnnxParser.h>

//#include "util.h"
//#include "logger.h"
#include <cassert>
#include <cfloat>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>

#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

#include <queue>
#include<thread>
#include<mutex>

#include<vector>
#include<algorithm>
//#include "yolotrt.h"

#include <cmath>
#include<sys/time.h>

//#include<format>
//using namespace cv;

#include "common.h"
#include "ByteTrack/STrack.h"
#include "ByteTrack/BYTETracker.h"
#include "ByteTrack/Object.h"
#include "ByteTrack/Rect.h"

using namespace nvinfer1;
using namespace std;



double what_time_is_it_now();
cv::Scalar color_cho(int id);
std::vector<std::vector<int>> nms(
                                std::vector<float>& scores,
                                std::vector<std::vector<std::vector<float>>>& boexs2,
                                nvinfer1::Dims32 output_dims,
                                float* output_buffer,
                                int nc,
                                const float score_threshold = 0.25 , // 分数阈值        //0.00474
                                const float nms_threshold = 0.45);

void plot_img(cv::Mat* img,int d,int t,std::vector<std::vector<float>> bb,int frame_id,int& th_id);

std::vector<byte_track::BYTETracker::STrackPtr> track(int nc,std::vector<std::vector<float>>& pred,std::vector<byte_track::BYTETracker>& trackerm,cv::Mat* img);
nvinfer1::ICudaEngine* InitEngine(const std::string engineFilename);
void warmup(nvinfer1::IExecutionContext* context,
            cudaStream_t& stream,
            cudaGraphExec_t& graph_vec,
            void* input_buffer,
            void* output_buffer,
            void* input_mem,
            void* output_mem,
            size_t input_size,
            size_t output_size);
void run_graph(
               cudaStream_t& stream,
               cudaGraphExec_t& graph_exec                             
               );
void infout(void* output_buffer,
            int nc,
            cv::Mat* img,
            nvinfer1::Dims32 output_dims,
            std::vector<byte_track::BYTETracker>& trackerm,
            int& frame_id,
            std::vector<std::vector<float>>& pred_tracked
            );
void send_img(cv::Mat* img,
                int d,int t,
                std::vector<std::vector<float>> bb,
                int frame_id,
                int& th_id);


