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
#include "utils/timer.h"
//#include"decter.h"

//#include"nms_cuda.cuh"

static Logger gLogger(ILogger::Severity::kINFO);


using namespace nvinfer1;
using namespace std;

int clip_ind[2][4]={{0,590,1180,1408},    ////x
                      {0,590,1180,1408}, /////y
                    };

#include<stdlib.h>
#include<string.h>
#include<errno.h>
#include<unistd.h>
#include<sys/types.h>
#include<sys/socket.h>
#include<arpa/inet.h>
#include<iostream>





// double what_time_is_it_now()
// {
//     // 单位: ms
//     struct timeval time;
//     if (gettimeofday(&time, NULL)) {
//         return 0;
//     }
//     return (double)time.tv_sec * 1000 + (double)time.tv_usec * .001;
// }



cv::Scalar color_cho(int id)
{
    cv::Scalar color;
    if(id==0)
    {
        color[0]=0;color[1]=0;color[2]=255;
        return color;
    } 
    if(id==1)
    {
        color[0]=0;color[1]=255;color[2]=0;
        return color;
    } 
    if(id==2)
    {
        color[0]=255;color[1]=0;color[2]=0;
        return color;
    } 
    if(id==3)
    {
        color[0]=230;color[1]=224;color[2]=176;
        return color;
    } 
    if(id==4)
    {
        color[0]=205;color[1]=90;color[2]=106;
        return color;
    } 
    if(id==5)
    {
        color[0]=255;color[1]=255;color[2]=0;
        return color;
    } 
    if(id==6)
    {
        color[0]=0;color[1]=0;color[2]=0;
        return color;
    } 
     if(id==7)
    {
        color[0]=84;color[1]=46;color[2]=8;
        return color;
    } 
     if(id==8)
    {
        color[0]=240;color[1]=32;color[2]=160;
        return color;
    } 
     if(id==9)
    {
        color[0]=64;color[1]=125;color[2]=255;
        return color;
    } 

    if(id==-1)
    {
        color[0]=0;color[1]=0;color[2]=0;
        return color;
    } 
     
}




std::vector<std::vector<int>> nms(
                                std::vector<float>& scores,
                                std::vector<std::vector<std::vector<float>>>& boexs2,
                                nvinfer1::Dims32 output_dims,
                                float* output_buffer,
                                int nc,
                                const float score_threshold = 0.25 , // 分数阈值        //0.00474
                                const float nms_threshold = 0.45)
{
    std::vector<std::vector<int>> indices(output_dims.d[0]);  //////    要根据batch的大小决定
    std::vector<float> nums(nc);
    int max_wh=10000;
    std::vector<cv::Rect> boexs;
    for(int k=0;k<output_dims.d[0];k++)
    {
        std::vector<std::vector<float>> boexsm;
        for (int i=0;i<output_dims.d[1];i++)
            {
               
                if (output_buffer[k*output_dims.d[1]*output_dims.d[2]+i*output_dims.d[2]+4]>score_threshold)
                {
                      std::vector<float> ltwh(6);
                    for(int j=0;j<output_dims.d[2];j++)
                    {
                        if (j<5)
                        {
                        ltwh[j]=output_buffer[k*output_dims.d[1]*output_dims.d[2]+i*output_dims.d[2]+j];
                        }
                        else
                        {
                            nums[j-5]=output_buffer[k*output_dims.d[1]*output_dims.d[2]+i*output_dims.d[2]+j];
                                
                        }
                    }
                    int max_num_index = max_element(nums.begin(), nums.end()) - nums.begin();
                    ltwh[0]=ltwh[0]-ltwh[2]/2.0;
                    ltwh[1]=ltwh[1]-ltwh[3]/2.0;

                    int x=round(ltwh[0])+max_num_index*max_wh;
                    int y=round(ltwh[1])+max_num_index*max_wh;
                    int width=round(ltwh[2]);
                    int heigth=round(ltwh[3]);
                    cv::Rect rect2(x,y,width,heigth);
                    boexs.push_back(rect2);
                    scores.push_back(ltwh[4]);

                    ltwh[0]=ltwh[0]; 
                    ltwh[1]=ltwh[1];
                    ltwh[4]=ltwh[4]*nums[max_num_index];
                    ltwh[5]=(float)max_num_index;
                    boexsm.push_back(ltwh);
                }                    
            }

        boexs2.push_back(boexsm); 
        boexs2[k]=boexsm;

        std::vector<int> indices_b;   
        cv::dnn::NMSBoxes(boexs,scores, score_threshold, nms_threshold,indices_b);
        indices[k]=indices_b;

        std::vector <float>().swap(scores);
        std::vector <cv::Rect>().swap(boexs);
    }
        return indices;
}




void send_img(cv::Mat* img,int d,int t,std::vector<std::vector<float>> bb,int frame_id,int& th_id)
{
     cv::Mat img_raw;
    cv::Rect rect(0,0,0,0);
    cv::Rect rect2(0,0,0,0);

    int isColor = 1;
    int fps = 24;

    if (t==1)
    {
        //std::cout<<online_targets.size()<<std::endl;
        
                
                //cv::cvtColor(*img,*img,cv::COLOR_RGB2BGR);
                //std::cout<<ot.time_stamp<<"|"<<od.time_stamp<<std::endl;
                img->convertTo(*img,CV_8UC1);
                for(int i=0;i<bb.size();i++)
                {
                    rect2.x=round(bb[i][0]);
                    rect2.y=round(bb[i][1]);
                    rect2.width=round(bb[i][2]);
                    rect2.height=round(bb[i][3]);
                    std::stringstream ss2;

                    ss2<<bb[i][6];
                    //ss << boexs2[indices[i]][4];
                    ss2<<"|";
                    ss2<<int(bb[i][5]);
                    std::string text2 =ss2.str();
                    cv::Point pp=cv::Point(rect2.x,rect2.y);
                    double font_scale=0.5;
                    int font_face = cv::FONT_HERSHEY_SIMPLEX;
                    int thickness = 1;
                    cv::Scalar color=color_cho(int(bb[i][5]));
                    cv::rectangle(*img,rect2, color,1, cv::LINE_8,0);
                    cv::putText(*img,text2,pp,font_face,font_scale,color,thickness);
                }


                double font_scale2=0.8;
                int font_face2 = cv::FONT_HERSHEY_SIMPLEX;
                int thickness2 = 1;
                std::stringstream ss4;
                ss4<<"frame_id= ";
                ss4<<frame_id;
                //ss << boexs2[indices[i]][4];
                std::string fra =ss4.str();
                cv::Point pp2=cv::Point(20,20);
                cv::putText(*img,fra,pp2,font_face2,font_scale2,cv::Scalar(0, 255, 255),thickness2);
                //img->convertTo(*img,CV_32FC1);
                // cv::imshow("sjdj",*img);
                // cv::waitKey(2);
		
    }
}





void plot_img(cv::Mat* img,int d,int t,std::vector<std::vector<float>> bb,int frame_id,int& th_id)
{
     cv::Mat img_raw;
    cv::Rect rect(0,0,0,0);
    cv::Rect rect2(0,0,0,0);

    int isColor = 1;
    int fps = 24;

    
    if (d==1)
    {
        for (int i=0;i<bb.size();i++)
        {

        
                    std::stringstream ss;
	                ss << bb[i][5];
                    ss << "|";
	                ss << bb[i][4];
                    //std::cout<<boexs2[indices[i]][5]<<std::endl;
                    
                    rect.x=round(bb[i][0]);
                    rect.y=round(bb[i][1]);
                    rect.width=round(bb[i][2]);
                    rect.height=round(bb[i][3]);
                    //std::cout<<image.time_stamp<<"|"<<od.time_stamp<<std::endl;
                    std::string text =ss.str();
                    cv::Point pp=cv::Point(rect.x,rect.y);
                    double font_scale=0.5;
                    int font_face = cv::FONT_HERSHEY_SIMPLEX;
                    int thickness = 1;

                    cv::rectangle(*img,rect, cv::Scalar(255, 0, 0),1, cv::LINE_8,0);   
                    cv::putText(*img,text,pp,font_face,font_scale,cv::Scalar(0, 255, 255),thickness);
        }
        std::stringstream ss3;
        ss3<<int(frame_id);
        std::string fra_i =ss3.str();
        //if (frame_id<10)
        //cv::imwrite("/root/ws/OrinDetection/result/dectre/dout"+fra_i+".jpg",*img);

    }
    if (t==1)
    {
        //std::cout<<online_targets.size()<<std::endl;
        
                
                //cv::cvtColor(*img,*img,cv::COLOR_RGB2BGR);
                //std::cout<<ot.time_stamp<<"|"<<od.time_stamp<<std::endl;
                img->convertTo(*img,CV_8UC1);
                for(int i=0;i<bb.size();i++)
                {
                    rect2.x=round(bb[i][0]);
                    rect2.y=round(bb[i][1]);
                    rect2.width=round(bb[i][2]);
                    rect2.height=round(bb[i][3]);
                    std::stringstream ss2;

                    ss2<<bb[i][6];
                    //ss << boexs2[indices[i]][4];
                    ss2<<"|";
                    ss2<<int(bb[i][5]);
                    std::string text2 =ss2.str();
                    cv::Point pp=cv::Point(rect2.x,rect2.y);
                    double font_scale=0.5;
                    int font_face = cv::FONT_HERSHEY_SIMPLEX;
                    int thickness = 1;
                    cv::Scalar color=color_cho(int(bb[i][5]));
                    cv::rectangle(*img,rect2, color,1, cv::LINE_8,0);
                    cv::putText(*img,text2,pp,font_face,font_scale,color,thickness);
                }


                double font_scale2=0.8;
                int font_face2 = cv::FONT_HERSHEY_SIMPLEX;
                int thickness2 = 1;
                std::stringstream ss4;
                ss4<<"frame_id= ";
                ss4<<frame_id;
                //ss << boexs2[indices[i]][4];
                std::string fra =ss4.str();
                cv::Point pp2=cv::Point(20,20);
                cv::putText(*img,fra,pp2,font_face2,font_scale2,cv::Scalar(0, 255, 255),thickness2);

                std::string savepath= "/root/ws/OrinDetection/result/out";
                std::stringstream ss3;
                ss3<<frame_id;
                std::string fra_i =ss3.str();



                std::stringstream ss5;
                ss5<<"trackr result";
                ss5<<th_id;
                std::string trar =ss5.str();
                //std::cout<<trar<<std::endl;
                //cv::resize(*img,*img,cv::Size(640,640));
                cv::imshow(trar,*img);
                cv::waitKey(2);

                // if (frame_id<100){
                // cv::imwrite(savepath+fra_i+".jpg",*img);
                //writer.write(img_raw); 
                // }

    }
}

void clip_img(cv::Mat& image,std::vector<cv::Mat>* imgs)
{
    int over_lap=50;
    int idx;
    int idy;
    for (int i=0;i<16;i++)
    {
        idx=i%4;
        idy=i/4;
        cv::Rect roi(clip_ind[0][idx], clip_ind[1][idy],640,640);
        imgs->push_back(image(roi));
    }
    
}


std::vector<byte_track::BYTETracker::STrackPtr> track(int nc,std::vector<std::vector<float>>& pred,std::vector<byte_track::BYTETracker>& trackerm,cv::Mat* img)
{
    std::vector<byte_track::BYTETracker::STrackPtr> targets_all;
                    std::vector<byte_track::Object> objes;
                    for(int j=0;j<pred.size();j++)
                    {
                        //std::cout<<boexs2[indices[j]][5]<<std::endl;
                        if(int(pred[j][5])==0 or int(pred[j][5])==1 or int(pred[j][5])==2 )
                        {
                            byte_track::Object obje;
                            obje.rect.tlwh[0,0]=pred[j][0];//-pred[j][2]/2;
                            obje.rect.tlwh[0,1]=pred[j][1];//-pred[j][3]/2;
                            obje.rect.tlwh[0,2]=pred[j][2];                      
                            obje.rect.tlwh[0,3]=pred[j][3];
                            obje.prob=pred[j][4];
                            obje.label=pred[j][5];
                            ////0-19119 19200-23999 24000-25199
                            

                            objes.push_back(obje);

                        }

                    }
                    //st3=std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
                    std::vector<byte_track::BYTETracker::STrackPtr> online_targets = trackerm[0].update(objes,img);
    return online_targets;
            
}

nvinfer1::ICudaEngine* InitEngine(const std::string engineFilename) 
{   
    nvinfer1::ICudaEngine* mEngine = nullptr;
    
    std::ifstream enginefile(engineFilename,std::ios::binary);
    if (enginefile.fail())
    {
        std::cout<<"enginefile is fail"<<std::endl;
        
    }
    enginefile.seekg(0,std::ifstream::end);
    auto fsize=enginefile.tellg();
    enginefile.seekg(0,std::ifstream::beg);

    std::vector<char> engineData(fsize);
    enginefile.read(engineData.data(),fsize);
    nvinfer1::IRuntime* runtime{nvinfer1::createInferRuntime(gLogger)};
    
    mEngine=runtime->deserializeCudaEngine(engineData.data(),fsize,nullptr);


    return mEngine;
}

void warmup(nvinfer1::IExecutionContext* context,
            cudaStream_t& stream,
            cudaGraphExec_t& graph_vec,
            void* input_buffer,
            void* output_buffer,
            void* input_mem,
            void* output_mem,
            size_t input_size,
            size_t output_size)

{
    context->enqueueV3(stream);
    cudaGraph_t graph;
    //void* bindings[] = {input_mem, output_mem};
    cudaStreamBeginCapture(stream,cudaStreamCaptureModeGlobal);
    cudaMemcpyAsync(input_mem,input_buffer,input_size, cudaMemcpyHostToDevice, stream);
    //bool status = context->enqueueV2(bindings, stream, nullptr);
    context->enqueueV3(stream);
    cudaMemcpyAsync(output_buffer,output_mem,output_size, cudaMemcpyDeviceToHost, stream);
    cudaStreamSynchronize(stream);
    cudaStreamEndCapture(stream,&graph);
    cudaGraphInstantiate(&graph_vec, graph,0);
    cudaGraphDestroy(graph);
    for (int i = 0; i < 100; i++) {
        cudaGraphLaunch(graph_vec, stream);
    }


}


void run_graph(
               cudaStream_t& stream,
               cudaGraphExec_t& graph_exec                             
               )
{
    cudaGraphLaunch(graph_exec, stream);
    cudaStreamSynchronize(stream);
    //std::cout << "time:" << end_time - start_time  << std::endl;
    
}

void infout(void* output_buffer,
            int nc,
            cv::Mat* img,
            nvinfer1::Dims32 output_dims,
            std::vector<byte_track::BYTETracker>& trackerm,
            int& frame_id,
            std::vector<std::vector<float>>& pred_tracked)
{           
           std::vector<float> scores;
            std::vector<std::vector<std::vector<float>>> boexs2(output_dims.d[0]);
            int cl_num[nc];
            std::vector<std::vector<float>> pred;
            //std::vector<std::vector<float>> pred_tracked;
            std::vector<byte_track::BYTETracker::STrackPtr> online_targets;
    
                std::vector<std::vector<int>> indices=nms(scores,boexs2,output_dims,(float*)output_buffer,nc); 
                int c_num=0;
                for (int k=0;k<indices.size();k++)
                    for (int i=0;i<indices[k].size();i++)
                    {
                        pred.push_back(boexs2[k][indices[k][i]]);
                        c_num=c_num+1;
                    
                    }
                //std::cout<<c_num<<std::endl;
                //outre.obj_num=cl_num; 
                online_targets=track(nc,pred,trackerm,img);
                //output_tracked outtracked;

                for(int j=0;j<online_targets.size();j++)
                {
                    std::vector<float> bb(7);
                    bb[0]=online_targets[j]->getRect().tlwh[0,0];
                    bb[1]=online_targets[j]->getRect().tlwh[0,1];
                    bb[2]=online_targets[j]->getRect().tlwh[0,2];
                    bb[3]=online_targets[j]->getRect().tlwh[0,3];
                    bb[4]=online_targets[j]->getScore();
                    bb[5]=online_targets[j]->getlabel();
                    bb[6]=online_targets[j]->getTrackId();
                    pred_tracked.push_back(bb);
                }
                int th_id=0;
                //plot_img(img,0,1,pred_tracked,frame_id,th_id);

            std::vector <std::vector<float>>().swap(pred);
            
            std::vector <std::vector<std::vector<float>>>().swap(boexs2);
            std::vector <float>().swap(scores);
                  
}

