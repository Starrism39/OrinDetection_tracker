#include <chrono>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>
#include <cstring>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <queue>
#include <mutex>
#include <thread>
#include <string>
#include <functional>
#include <numeric>
#include <typeinfo>
#include <dirent.h>
#include <cassert>
#include <cfloat>
#include <memory>
#include <sstream>
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <cmath>
#include <cuda_runtime_api.h>

#include "img_codec.h"
#include "KCF/kcftracker.h"
#include "utils/logger.h"
#include "utils/threadsafe_queue.h"
#include "NvInfer.h"
#include "common.h"
#include "ByteTrack/STrack.h"
#include "ByteTrack/BYTETracker.h"
#include "imgmemorymanager.h"
#include "vps4mpcap.h"
#include "ByteTrack/Object.h"
#include "ByteTrack/Rect.h"
#include "utils/detect.h"
#include "rtsp_stream.h"
#include "utils/logger.h"
#include "utils/udp_operation.h"

#define MAXLINE 512 * 512
#define MAX_BUFFER_SIZE 1024
#define PACKET_SIZE 1038 // 13 bytes header + 1024 bytes data + 1 byte trailer
#define PAYLOAD_SIZE 1024

int id, label; // 点选时传进的id号和类别
uint8_t work_mode, if_track;
bool flag_run = true, flag_auto = false, flag_manual = false, flag_initMot = true, flag_end = false, flag_exit = false;
unsigned int longitude = 0, latitude = 0, altitude = 0;
std::mutex state_mtx, initMot_mtx, location_mtx;
std::mutex mtx;

// 结构体来保存视频的状态
struct VideoState
{
    bool ifBox = false;  // 是否存在目标框
    bool ifinit = false; // 是否需要初始化
    cv::Rect boundingBox;
};

// 下位机参数队列缓存
struct Task {
    // 编码
    cv::Mat result;
    std::vector<image_Label> labels;
    WinInfo win_info;
    unsigned char *endata;
    int ensize;
    int winX;
    int winY;
    int frame_id;
    bool flag_auto_task;
    bool flag_manual_task;

    // UDP
    std::vector<Label> bounding;
    UDPOperation *remote_server;
    UDPOperation *send_server;
    UDPOperation *image_server;
    int win_idx;
    bool if_lost;
};

// 编码器线程控制
std::queue<Task> task_queue;
std::mutex queue_mtx;
std::condition_variable queue_cv;
bool stop_processing = false;


bool write_file(const std::string &path, void *data, size_t len)
{
    FILE *file = fopen(path.c_str(), "wb");
    if (file == nullptr)
    {
        fprintf(stderr, "can't open file: %s\n", path.c_str());
        return false;
    }
    fwrite(data, sizeof(unsigned char), len, file);
    fclose(file);
    return true;
}

inline int unit8ToInt(uint8_t highByte, uint8_t lowByte)
{
    std::stringstream ss;
    ss << std::hex << static_cast<int>(highByte) << std::setw(2) << std::setfill('0')
       << static_cast<int>(lowByte);

    // 从字符串流中读取拼接后的字符串
    std::string resultStr;
    ss >> resultStr;

    // 将字符串转换为整数
    int result = std::stoi(resultStr, nullptr, 16);
    return result;
}

// 上位机
void receive_location(UDPOperation *server, int win_id)
{
    int x = 0, y = 0, w = 0, h = 0;
    int x_bias = 0, y_bias = 0;
    char buffer[16];
    while (true)
    {
        int n = server->recv_buffer(reinterpret_cast<char *>(buffer), 1024);
        if (n == 132)
        {
            MLOG_DEBUG("数据包长为 %d", n);
            if (buffer[0] == 0x7E && buffer[1] == 0x14 && buffer[2] == 0x02 && buffer[131] == 0xE7)
            {
                // 计算经度的起始
                int start_longitude = 3 + win_id * 16 + 4;
                
                // 计算纬度的起始
                int start_latitude = 3 + win_id * 16 + 4 + 4;
                
                // 计算高程的起始
                int start_altitude = 3 + win_id * 16 + 4 + 4 + 4;

                unsigned int longi, lati, alti;
                
                // 从数据包中提取4个字节，并将其转换为无符号整数              
                for (int i = 0; i < 4; ++i) {
                    longi |= buffer[start_longitude + i] << ((3 - i) * 8);
                }

                for (int i = 0; i < 4; ++i) {
                    lati |= buffer[start_latitude + i] << ((3 - i) * 8);
                }

                for (int i = 0; i < 4; ++i) {
                    alti|= buffer[start_altitude + i] << ((3 - i) * 8);
                }
                MLOG_DEBUG("经纬高为： %f %f %f", static_cast<float>(longi)/std::pow(10, 7), static_cast<float>(lati)/std::pow(10, 7), static_cast<float>(alti)/100);
                location_mtx.lock();
                longitude = longi;
                latitude = lati;
                altitude = alti;
                location_mtx.unlock();
            }
        }
    }
    return;
}


// 上位机
void receive(UDPOperation *server, VideoState &state, int win_id)
{
    int x = 0, y = 0, w = 0, h = 0;
    int x_bias = 0, y_bias = 0;
    char buffer[16];
    while (true)
    {
        int n = server->recv_buffer(reinterpret_cast<char *>(buffer), 1024);
        if (n == 16)
        {
            MLOG_DEBUG("数据包长为 %d", n);
            if (buffer[0] == 0x7E && buffer[1] == 0x19 && buffer[15] == 0xE7 && win_id == static_cast<int>(buffer[3]))
            {
                if (buffer[2] == 0x03)
                { // 跟踪
                    if (buffer[14] == 0x01)
                    {
                        work_mode = 0x01;
                        // 手动跟踪
                        x = unit8ToInt(buffer[4], buffer[5]);
                        y = unit8ToInt(buffer[6], buffer[7]);
                        w = unit8ToInt(buffer[8], buffer[9]);
                        h = unit8ToInt(buffer[10], buffer[11]);
                        flag_auto = false;
                        flag_manual = true;
                        MLOG_INFO("转为手动跟踪模式");
                        MLOG_INFO("开始时的x, y, w, h: %d %d %d %d", x, y, w, h);
                    }
                    else if (buffer[14] == 0x02)
                    {
                        work_mode = 0x02;
                        // 自动跟踪
                        id = unit8ToInt(buffer[12], buffer[13]);
                        x = unit8ToInt(buffer[4], buffer[5]);
                        y = unit8ToInt(buffer[6], buffer[7]);
                        w = unit8ToInt(buffer[8], buffer[9]);
                        h = unit8ToInt(buffer[10], buffer[11]);
                        flag_auto = true;
                        flag_manual = false;
                        MLOG_INFO("转为自动跟踪模式");
                        MLOG_DEBUG("id: %d", id); 
                    }
                }
                else if (buffer[2] == 0x04)
                { // 解除跟踪，转为Mot模式
                    work_mode = 0x00;
                    flag_auto = false;
                    flag_manual = false;
                    MLOG_INFO("解除跟踪模式");  
                }
                else if (buffer[2] == 0x05)
                { // 跟踪微调
                    if (flag_auto || flag_manual)
                    { // KCF模式下才能微调
                        int direction = static_cast<int>(buffer[4]);
                        int stride = static_cast<int>(buffer[5]);
                        switch (direction)
                        {
                        case 1:
                            x_bias = 0;
                            y_bias = -stride;
                            MLOG_INFO("向上%d个像素", stride);
                            break;
                        case 2:
                            x_bias = 0;
                            y_bias = stride;
                            MLOG_INFO("向下%d个像素", stride);
                            break;
                        case 3:
                            x_bias = -stride;
                            y_bias = 0;
                            MLOG_INFO("向左%d个像素", stride);
                            break;
                        case 4:
                            x_bias = stride;
                            y_bias = 0;
                            MLOG_INFO("向右%d个像素", stride);
                            break;
                        default:
                            MLOG_INFO("无效的方向 请输入1到4", stride);
                            break;
                        }
                        state_mtx.lock();
                        x = state.boundingBox.x + x_bias;
                        y = state.boundingBox.y + y_bias;
                        w = state.boundingBox.width;
                        h = state.boundingBox.height;
                        state_mtx.unlock();
                        flag_auto = false;
                        flag_manual = true;
                        MLOG_INFO("转为跟踪微调模式");
                        MLOG_DEBUG("x, y, w, h: %d %d %d %d", x, y, w, h);
                        if (x < 0 || x + w > 512)
                        {
                            x = x - x_bias;
                            MLOG_INFO("x超出边界");
                        }
                        if (y < 0 || y + h > 512)
                        {
                            y = y - y_bias;
                            MLOG_INFO("y超出边界");
                        }
                    }
                    else
                    {
                        MLOG_INFO("非KCF模式下禁止跟踪微调");
                    }
                }
                if (flag_auto || flag_manual)
                {   
                    initMot_mtx.lock(); // 从Mot转KCF，Mot需要清除历史记录
                    flag_initMot = true;
                    initMot_mtx.unlock();

                    if (x < 0 || x + w >= 512){
                        flag_auto = false;
                        flag_manual = false;
                        x = x - x_bias;
                        MLOG_INFO("x超出边界");
                        continue;
                    }
                    else if (y < 0 || y + h >= 512){
                        flag_auto = false;
                        flag_manual = false;
                        y = y - y_bias;
                        MLOG_INFO("y超出边界");
                        continue;
                    }
                    state_mtx.lock(); // 从Mot转KCF、KCF选取新目标，KCF都需要初始化
                    state.ifinit = true;
                    state.ifBox = true;
                    state.boundingBox = cv::Rect(x , y , w, h);
                    state_mtx.unlock();
                }
            }
        }
    }
    return;
}



void send_KCF(UDPOperation *server, vector<Label> bounding, int win_id, uint16_t win_x, uint16_t win_y, uint8_t frame_id, bool if_lost)
{
    int16_t x_bias, y_bias;
    std::vector<uint8_t> message;
    // 帧头
    message.push_back(0x7E);
    // UID
    message.push_back(0x91);
    // 窗口号
    message.push_back(win_id & 0xFF);
    // 模组成像模式
    message.push_back(0x05);

    // 窗口坐标
    message.push_back(win_x >> 8);
    message.push_back(win_x & 0xff);
    message.push_back(win_y >> 8);
    message.push_back(win_y & 0xff);

    // 预留位
    for (int i = 0; i < 9; ++i)
    {
        message.push_back(0xFF);
    }

    // 图像帧数
    message[16] = frame_id;
    
    // 手动或自动跟踪
    if(bounding.empty()){
        work_mode = 0x00;
    }
    message.push_back(work_mode && 0xFF);

    // 是否处于跟踪模式
    message.push_back(if_track && 0xFF);
    
    // 目标个数
    message.push_back(bounding.size() & 0xFF);

    // 添加bounding中的每个Label
    for (const auto &label : bounding)
    {
        // 跟踪状态
        if(if_lost){
            message.push_back(0x03);
        }
        else{
            message.push_back(0x01);
        }
        

        // 目标类别
        message.push_back(label.label & 0xFF);

        // 目标置信度
        message.push_back(label.score & 0xFF);

        // 脱靶量x
        x_bias = static_cast<int16_t>(label.x + label.w / 2) - 256;
        message.push_back((x_bias >> 8) & 0xFF);
        message.push_back(x_bias & 0xFF);

        // 脱靶量y
        y_bias = static_cast<int16_t>(label.y + label.h / 2) - 256;
        message.push_back((y_bias >> 8) & 0xFF);
        message.push_back(y_bias & 0xFF);

        MLOG_INFO("脱靶量为：%d %d", x_bias, y_bias);

        // 目标宽度
        message.push_back((label.w >> 8) & 0xFF); // w 的高位
        message.push_back(label.w & 0xFF);        // w 的低位

        // 目标高度
        message.push_back((label.h >> 8) & 0xFF); // h 的高位
        message.push_back(label.h & 0xFF);        // h 的低位

        // 跟踪ID
        message.push_back((label.id >> 8) & 0xFF); // h 的高位
        message.push_back(label.id & 0xFF);        // h 的低位

        // 预留
        for (int i = 0; i < 3; ++i)
        {
            message.push_back(0xFF);
        }
    }

    // 帧尾
    message.push_back(0xE7);

    // 将消息发送出去
    server->send_buffer(reinterpret_cast<char *>(message.data()), message.size());
    // printMessageHex(message);
}

void send_MOT(UDPOperation *server, vector<Label> bounding, int win_id, uint16_t win_x, uint16_t win_y, uint8_t frame_id)
{
    int16_t x_bias, y_bias;
    std::vector<uint8_t> message;
    // 帧头
    message.push_back(0x7E);
    // UID
    message.push_back(0x51);
    // SID
    message.push_back(0x05);
    // 窗口号
    message.push_back(win_id & 0xFF);
    // 模组成像模式
    message.push_back(0x05);

    // 窗口坐标
    message.push_back(win_x >> 8);
    message.push_back(win_x & 0xff);
    message.push_back(win_y >> 8);
    message.push_back(win_y & 0xff);

    // 预留位
    for (int i = 0; i < 10; ++i)
    {
        message.push_back(0xFF);
    }

    // 图像帧数
    message[16] = frame_id;

    // 目标个数
    message.push_back(bounding.size() & 0xFF);

    // 添加bounding中的每个Label
    for (const auto &label : bounding)
    {
        // 目标编号
        message.push_back(label.id & 0xFF);
        message.push_back((label.id >> 8) & 0xFF);
        

        // 目标类别
        message.push_back(label.label & 0xFF);

        // 目标置信度
        message.push_back(label.score & 0xFF);

        // x
        message.push_back(label.x & 0xFF);
        message.push_back((label.x >> 8) & 0xFF);
        

        // y
        message.push_back(label.y & 0xFF);
        message.push_back((label.y >> 8) & 0xFF);
        

        // 目标宽度
        message.push_back(label.w & 0xFF);        // w 的低位
        message.push_back((label.w >> 8) & 0xFF); // w 的高位
        

        // 目标高度
        message.push_back(label.h & 0xFF);        // h 的低位
        message.push_back((label.h >> 8) & 0xFF); // h 的高位
        

        // 预留
        for (int i = 0; i < 4; ++i)
        {
            message.push_back(0xFF);
        }
    }

    // 帧尾
    message.push_back(0xE7);

    // 将消息发送出去
    server->send_buffer(reinterpret_cast<char *>(message.data()), message.size());

    // printMessageHex(message);
}

// 绘制十字的函数
void drawCross(cv::Mat& frame, const cv::Scalar& color = cv::Scalar(0, 0, 0), int thickness = 1) {
    // 获取图像的中心点
    cv::Point center(frame.cols / 2, frame.rows / 2);

    // 画垂直线
    cv::line(frame, 
             cv::Point(center.x, center.y - frame.rows / 20), 
             cv::Point(center.x, center.y + frame.rows / 20), 
             color, thickness, cv::LINE_AA);

    // 画水平线
    cv::line(frame, 
             cv::Point(center.x - frame.cols / 20, center.y), 
             cv::Point(center.x + frame.cols / 20, center.y), 
             color, thickness, cv::LINE_AA);
}

// 在图像右上角绘制文本的函数
void drawText(cv::Mat& frame, const std::string& text, const cv::Scalar& color = cv::Scalar(0, 0, 0), int thickness = 1, double scale = 1.0) {
    int fontFace = cv::FONT_HERSHEY_SIMPLEX;
    int baseline = 0;
    int margin = 10;

    // 获取文本大小
    cv::Size textSize = cv::getTextSize(text, fontFace, scale, thickness, &baseline);
    baseline += thickness;

    // 计算文本位置：右上角，留出一定的边距
    cv::Point textOrg(frame.cols - textSize.width - margin, margin + textSize.height);

    // 绘制文本
    cv::putText(frame, text, textOrg, fontFace, scale, color, thickness, cv::LINE_AA);
}

void send_image(unsigned char *endata, size_t data_size, UDPOperation *server, unsigned char mode, int win_id, uint16_t win_x, uint16_t win_y, uint8_t frame_id)
{
    int total_packets = (data_size + PAYLOAD_SIZE - 1) / PAYLOAD_SIZE; // Calculate total packets
    unsigned char packet[PACKET_SIZE];
    MLOG_DEBUG("data len: %d", (int)data_size);

    for (int i = 0; i < total_packets; i++)
    {
        memset(packet, 0, PACKET_SIZE);

        // 帧头
        packet[0] = 0x7E;

        // UID
        packet[1] = 0x40;

        // 窗口号
        packet[2] = win_id & 0xFF;

        // 0x01：跟踪  0x02：识别
        packet[3] = mode;

        // 窗口坐标
        packet[4] = win_x >> 8;
        packet[5] = win_x & 0xff;
        packet[6] = win_y >> 8;
        packet[7] = win_y & 0xff;

        // 帧IDX
        packet[8] = (unsigned char)frame_id;

        // 码流当前包数
        packet[9] = (unsigned char)i;

        // 码流总包数
        packet[10] = (unsigned char)total_packets;

        // 当前包的码流有效数据长度
        size_t packet_data_length = (i == total_packets - 1) ? (data_size % PAYLOAD_SIZE) : PAYLOAD_SIZE;
        packet[11] = (unsigned char)(packet_data_length >> 8);
        packet[12] = (unsigned char)(packet_data_length & 0xFF);

        // 填充数据
        memcpy(packet + 13, endata + i * PAYLOAD_SIZE, packet_data_length);

        // 帧尾
        packet[PACKET_SIZE - 1] = 0xE7;

        server->send_buffer(reinterpret_cast<char *>(packet), PACKET_SIZE);
    }
    MLOG_DEBUG("frame %d send successful", frame_id);
}

// 下位机
void send_thread(ImgEncode* encode) {
    while (true) {
        std::unique_lock<std::mutex> lock(queue_mtx);
        queue_cv.wait(lock, [] { return !task_queue.empty() || stop_processing; });

        if (stop_processing && task_queue.empty()) {
            break;
        }

        Task task = task_queue.front();
        task_queue.pop();
        lock.unlock();

        encode->encode(task.result, task.labels, task.win_info, &task.endata, task.ensize);

        MLOG_INFO("窗口坐标为： %d %d ", task.win_info.x, task.win_info.y);
        MLOG_INFO("经纬高为： %d %d %d", task.win_info.longitude, task.win_info.latitude, task.win_info.altitude);
        MLOG_INFO("pack size: %.2fKB", task.ensize / 1024.0);

        if (task.flag_auto_task || task.flag_manual_task) {
            if (!task.bounding.empty()) {
                send_KCF(task.remote_server, task.bounding, task.win_idx, task.winX, task.winY, task.frame_id, task.if_lost);
            }
            send_image(task.endata, task.ensize, task.image_server, 0x01, task.win_idx, task.winX, task.winY, task.frame_id);
        } else {
            std::vector<Label> empty;
            send_MOT(task.send_server, task.bounding, task.win_idx, task.winX, task.winY, task.frame_id);
            send_KCF(task.remote_server, empty, task.win_idx, task.winX, task.winY, task.frame_id, task.if_lost);
            send_image(task.endata, task.ensize, task.image_server, 0x02, task.win_idx, task.winX, task.winY, task.frame_id);
        }

        delete[] task.endata;
    }
}

void add_multicast_group(const std::string &multicast_address)
{
    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd == -1)
    {
        perror("socket");
        exit(0);
    }
    struct ip_mreq mreq; // 多播地址结构体
    mreq.imr_multiaddr.s_addr = inet_addr(multicast_address.c_str());
    mreq.imr_interface.s_addr = htonl(INADDR_ANY);
    setsockopt(fd, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq));
}

void packet_handler_t(u_char *param, const struct pcap_pkthdr *header, const u_char *pkt_data)
{
    void *out_data = nullptr;
    if (!pkt_data || header->len < 60 || header->len > 1514)
        return;
    if (header->len == 1083 && pkt_data[42] == 0x81)
    {
        if (recvCheck((uint8_t *)pkt_data + 42, 1041))
        {
            out_data = VPS4MPcap::instance().recvPack((uint8_t *)pkt_data + 42, NULL);
            if (out_data)
            {
                reinterpret_cast<threadsafe_queue<void *> *>(param)->push(out_data);
            }
        }
    }
}

// KCF核心算法
void KCF(cv::Mat frame, VideoState &state, bool init, KCFTracker &tracker)
{
    // cosine distance between template of previous and next frames
    double corr_distance;
    cv::Rect tmp = state.boundingBox;

    // 第一帧初始化
    if (init)
    {
        tracker.init(tmp, frame);

        state_mtx.lock();
        state.boundingBox = tmp;
        state_mtx.unlock();
    }

    // 更新判断
    else
    {
        tmp = tracker.update(frame, corr_distance);

        state_mtx.lock();
        state.boundingBox = tmp;
        state_mtx.unlock();
    }
}

cv::Mat runKCF(cv::Mat frame, VideoState &state, KCFTracker &tracker)
{
    cv::Mat display = frame.clone();
    state_mtx.lock();
    bool ifbox = state.ifBox, ifinit = state.ifinit;
    state_mtx.unlock();

    if (ifbox)
    {
        if (ifinit)
        {
            KCF(frame, state, true, tracker);
            state_mtx.lock();
            state.ifinit = false;
            state_mtx.unlock();
        }

        KCF(frame, state, false, tracker);

        // 在图像上绘制目标框
        cv::rectangle(display, state.boundingBox, cv::Scalar(255, 255, 255), 2);
    }

    return display;
}

void run_MOT(cv::Mat &img,
             void *input_buffer,
             void *output_buffer,
             Dims32 output_dims,
             size_t input_size,
             cudaStream_t &stream,
             cudaGraphExec_t &graph_vec,
             std::vector<byte_track::BYTETracker> &trackerm,
             std::vector<std::vector<float>> &pred_tracked,
             int nc,
             int &frame_id)
{
    cv::Mat img_g;
    img.convertTo(img_g, CV_32FC1);
    memcpy(input_buffer, reinterpret_cast<float *>(img_g.data), input_size);
    run_graph(stream, graph_vec);
    // std::vector<std::vector<float>> pred_tracked;
    infout(output_buffer,
           nc,
           &img_g,
           output_dims,
           trackerm,
           frame_id,
           pred_tracked);
    int th_id = 0;
    send_img(&img, 0, 1, pred_tracked, frame_id, th_id);

    frame_id = frame_id + 1;
}

int main(int argc, char *argv[])
{

    if (argc < 12)
    {
        std::cout << "Usage: main <tcp_port> <udp_port> <engine_path> <rtsp_address> <stream_id> <win_idx> \n"
                  << "\t multicast address: the address of data receiving\n"
                  << "\t multicast port: the port of data receiving\n"
                  << "\t engine_path: the path of data engine\n"
                  << "\t remote address: send to address"
                  << "\t remote port: send to port"
                  << "\t win_idx: win_idx"
                  << "\t data receive address: udp_address"
                  << "\t data receive port: udp_port"
                  << "Examples:\n"
                  << "\t ./orindectection_tracker 9005 model/yolov5s_data1_green_255_simplify.engine 1 /root/xjy/OrinDeteciton_tracker_test/test.mp4\n"
                  << std::endl;
        return -1;
    }

    std::string multicast_address = argv[1];
    int multicast_port = atoi(argv[2]);
    std::string engineFilename = argv[3];
    std::string remote_address = argv[4];
    int remote_port = atoi(argv[5]);
    int win_idx = atoi(argv[6]);
    std::string udp_address = argv[7];
    int udp_port = atoi(argv[8]);
    std::string image_address = argv[9];
    int image_port = atoi(argv[10]);
    std::string codec = argv[11];
    int bit_rate = atoi(argv[12]);

    // 定义参数
    VideoState state;
    cv::Mat frame, result;
    string frameInfoStr;
    int frame_id = 0;

    // 定义MOT类
    int nc = 4;
    std::vector<byte_track::BYTETracker> trackerm;
    for (int r = 0; r < nc; r++)
    {
        int distance_mode = 1;
        byte_track::BYTETracker trackerb(distance_mode);
        trackerm.push_back(trackerb);
    }

    // Detection
    nvinfer1::ICudaEngine *mEngine = InitEngine(engineFilename);
    nvinfer1::IExecutionContext *context = mEngine->createExecutionContext();

    auto input_dims = nvinfer1::Dims{4, {1, 1, 512, 512}};
    context->setInputShape("images", input_dims);
    size_t input_size = input_dims.d[0] * input_dims.d[1] * input_dims.d[2] * input_dims.d[3] * sizeof(float);

    auto output_dims = nvinfer1::Dims{3, {input_dims.d[0], 16128, 5 + nc}};
    size_t output_size = output_dims.d[0] * output_dims.d[1] * output_dims.d[2] * sizeof(float);

    void *input_buffer;
    void *output_buffer;

    void *input_mem;
    void *output_mem;

    cudaHostAlloc(&input_buffer, input_size, cudaHostAllocMapped);
    cudaHostAlloc(&output_buffer, output_size, cudaHostAllocMapped);

    cudaMalloc(&input_mem, input_size);
    cudaMalloc(&output_mem, output_size);

    context->setInputTensorAddress("images", input_mem);
    context->setTensorAddress("output0", output_mem);

    cudaStream_t stream;
    if (cudaStreamCreate(&stream) != cudaSuccess)
    {
        MLOG_ERROR("ERROR: cuda stream creation failed.");
    }
    cudaGraphExec_t graph_vec = nullptr;
    MLOG_DEBUG("Start warmup");
    warmup(context,
           stream,
           graph_vec,
           input_buffer,
           output_buffer,
           input_mem,
           output_mem,
           input_size,
           output_size);
    cudaStreamSynchronize(stream);
    MLOG_DEBUG("warmup end");

    // 创建KCF类
    bool MULTISCALE = true;
    bool HOG = false;
    bool FIXEDWINDOW = false;
    bool LAB = false;
    int flag_mode = 0; // 0代表上一帧在运行MOT，1代表上一帧在运行KCF
    double k = 0.15;  // 范围阈值系数
    float multiple = 3.5;  // 搜索范围倍数
    KCFTracker tracker(HOG, FIXEDWINDOW, MULTISCALE, LAB, k, multiple);
    std::vector<std::vector<float>> MOT_previous_tracked; // t,l,w,h,score,label,id，点选时根据label和id判断是否从MOT取框

    add_multicast_group(udp_address);

    // UDP接收数据
    threadsafe_queue<void *> q;
    bool ret = VPS4MPcap::instance().init10g();
    if (ret)
    {
        ret = ImgMemoryManager::instance().mallocMemoryPool(512 * 512 *
                                                            static_cast<uint64_t>(1000));
        if (!ret)
        {
            MLOG_ERROR("can't allocate mem");
            throw std::runtime_error("can't allocate mem");
        }
        else
        {
            MLOG_INFO("allocate mem success");
        }
    }
    else
    {
        MLOG_ERROR("init pcap err.");
        throw std::runtime_error("init pcap err");
    }
    VPS4MPcap::instance().pcapSetHandler(packet_handler_t);
    VPS4MPcap::instance().set_WinIdx(win_idx);
    MLOG_INFO("开始捕获相机数据");
    std::thread([&]()
                { VPS4MPcap::instance().openCapture(("udp and port " + std::to_string(udp_port)).c_str(),
                                                    reinterpret_cast<uint8_t *>(&(q))); })
        .detach();

    // 图像编码
    WinInfo win_info;
    auto encode = new ImgEncode(codec, bit_rate, 512, 512);

    // 创建组播通信
    UDPOperation receive_server(multicast_address.c_str(), multicast_port, "eth0");
    receive_server.create_client();
    UDPOperation receive_location_server(multicast_address.c_str(), multicast_port, "eth0");
    receive_location_server.create_client();
    UDPOperation send_server(multicast_address.c_str(), multicast_port, "eth0");
    send_server.create_server();
    UDPOperation image_server(image_address.c_str(), image_port, "eth0");
    image_server.create_server();
    UDPOperation remote_server(remote_address.c_str(), remote_port, "eth0");
    remote_server.create_server();

    // 创建上位机线程
    std::thread informReceive(receive, &receive_server, std::ref(state), std::ref(win_idx)); // 上位机不停接收数据
    informReceive.detach();
    std::thread informReceive_location(receive_location, &receive_location_server, std::ref(win_idx)); // 上位机不停接收数据
    informReceive_location.detach();

    // 创建下位机线程
    std::thread consumer(send_thread, encode);
    consumer.detach();

    // 处理图像并发回客户端
    auto start = std::chrono::steady_clock::now();
    while (true)
    {
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count();
        MLOG_INFO("代码运行时间: %d 毫秒", duration);
        start = std::chrono::steady_clock::now();
        vector<Label> bounding;
        std::vector<image_Label> labels;

        AirImg *Airpack = reinterpret_cast<AirImg *>(q.wait_and_pop()); // 必须
        cv::Mat frame = cv::Mat(512, 512, CV_8UC1, Airpack->frame).clone();
        ImgMemoryManager::instance().setFree(Airpack->mempool_point);

        if (frame.empty())
        {
            continue; // 如果 frame 为空，则跳过当前循环
        }
        if (flag_run)
        {
            if (flag_initMot)
            {
                for (int r = 0; r < nc; r++)
                {
                    trackerm[r].clear_all_tracks();
                }

                frame_id = 0;
                initMot_mtx.lock();
                flag_initMot = false;
                initMot_mtx.unlock();
                // MLOG_DEBUG("Mot历史记录已清除");
            }

            if (!flag_auto && !flag_manual)
            {
                std::vector<std::vector<float>> pred_tracked; // t,l,w,h,score,label,id
                if (frame.channels() == 3)
                {
                    cv::cvtColor(frame, frame, cv::COLOR_BGR2GRAY);
                }
                run_MOT(frame,
                        input_buffer,
                        output_buffer,
                        output_dims,
                        input_size,
                        stream,
                        graph_vec,
                        trackerm,
                        pred_tracked,
                        nc,
                        frame_id);
                MOT_previous_tracked = pred_tracked;

                for (size_t i = 0; i < pred_tracked.size(); i++)
                {
                    Label object;
                    // 提取数据并存入结构体
                    object.id = static_cast<uint16_t>(MOT_previous_tracked[i][6]);
                    object.label = static_cast<uint8_t>(MOT_previous_tracked[i][5]);
                    object.score = static_cast<uint8_t>(std::round(MOT_previous_tracked[i][4] * 100));
                    object.x = static_cast<uint16_t>(MOT_previous_tracked[i][0]);
                    object.y = static_cast<uint16_t>(MOT_previous_tracked[i][1]);
                    object.w = static_cast<uint16_t>(MOT_previous_tracked[i][2]);
                    object.h = static_cast<uint16_t>(MOT_previous_tracked[i][3]);
                    // 注意：这里 score 没有存入信息中，因为要求只存储 x、y、w、h、label 和 id
                    // MLOG_DEBUG("MOT的输出框是 id: %d, label: %d, x: %d, y: %d, w: %d, h: %d, score: %d", object.id, object.label, object.x, object.y, object.w, object.h, object.score);
                    bounding.push_back(object);
                    labels.push_back({Airpack->frame_id, object.x, object.y, object.w, object.h, object.score, object.label});
                }

                result = frame.clone();
                flag_mode = 0;
                work_mode = 0x00;
                if_track = 0x00;
            }
            else if (flag_auto && !flag_manual)
            {
                // *****************************************************************************************
                if (flag_mode == 0)
                {
                    for (size_t i = 0; i < MOT_previous_tracked.size(); i++)
                    {
                        int x = static_cast<int>(MOT_previous_tracked[i][0]);
                        int y = static_cast<int>(MOT_previous_tracked[i][1]);
                        int w = static_cast<int>(MOT_previous_tracked[i][2]);
                        int h = static_cast<int>(MOT_previous_tracked[i][3]);
                        // 注意：这里 score 没有存入信息中，因为要求只存储 x、y、w、h、label 和 id
                        int label_now = static_cast<int>(MOT_previous_tracked[i][5]);
                        int id_now = static_cast<int>(MOT_previous_tracked[i][6]);
                        // MLOG_DEBUG("MOT保存的数据是 %d %d %d %d %d %d", id, label, x, y, w, h);

                        if (id == id_now && label == label_now)
                        {
                            MLOG_DEBUG("目标存在上一帧MOT中");
                            state_mtx.lock();
                            state.boundingBox = cv::Rect(x, y, w, h);
                            state_mtx.unlock();
                        }
                    }
                }
                // *****************************************************************************************
                if (frame.channels() == 1)
                {
                    cv::cvtColor(frame, frame, cv::COLOR_GRAY2BGR);
                }
                result = runKCF(frame, state, tracker);
                Label object;
                object.id = 0;
                object.label = 0;
                object.score = 0;
                state_mtx.lock();
                object.x = state.boundingBox.x;
                object.y = state.boundingBox.y;
                object.w = state.boundingBox.width;
                object.h = state.boundingBox.height;
                state_mtx.unlock();
                MLOG_INFO("自动模式的输出框是 id: %d, label: %d, x: %d, y: %d, w: %d, h: %d, score: %d", object.id, object.label, object.x, object.y, object.w, object.h, object.score);
                bounding.push_back(object);
                labels.push_back({Airpack->frame_id, object.x, object.y, object.w, object.h, 0.0, 0});
                flag_mode = 1;
                work_mode = 0x02;
                if_track = 0x01;
            }
            else if (!flag_auto && flag_manual)
            {
                if (frame.channels() == 1)
                {
                    cv::cvtColor(frame, frame, cv::COLOR_GRAY2BGR);
                }
                result = runKCF(frame, state, tracker);
                Label object;
                object.id = 0;
                object.label = 0;
                object.score = 0;
                state_mtx.lock();
                object.x = state.boundingBox.x;
                object.y = state.boundingBox.y;
                object.w = state.boundingBox.width;
                object.h = state.boundingBox.height;
                state_mtx.unlock();
                MLOG_INFO("手动模式的输出框是 id: %d, label: %d, x: %d, y: %d, w: %d, h: %d, score: %d", object.id, object.label, object.x, object.y, object.w, object.h, object.score);
                bounding.push_back(object);
                labels.push_back({Airpack->frame_id, object.x, object.y, object.w, object.h, 0.0, 0});
                flag_mode = 1;
                work_mode = 0x01;
                if_track = 0x01;
            }
            else if (flag_auto && flag_manual)
            {
                MLOG_DEBUG("错误，点选和框选不能同时进行");
            }
        }

        // 保存图像
        if (Airpack->frame_id <= 255)
        {
            std::string filename = "/home/nvidia/OrinDetection_tracker_code/data" + std::to_string(win_idx % 2) + "/" + std::to_string(Airpack->frame_id) + ".png";
            cv::imwrite(filename, result);
            std::cout << "Saved " << filename << std::endl;
        }

        // 图像编码
        if (result.channels() == 3)
        {
            cv::cvtColor(result, result, cv::COLOR_BGR2GRAY);
        }
        if (flag_auto || flag_manual){
            drawCross(result, cv::Scalar(255, 255, 255), 1);
            drawText(result, "KCF", cv::Scalar(255, 255, 255), 1, 1.0);
        }
        else if (!flag_auto && !flag_manual){
            drawCross(result, cv::Scalar(255, 255, 255), 1);
            drawText(result, "MOT", cv::Scalar(255, 255, 255), 1, 1.0);
        }

        // cv::imshow("frame", frame);
        // cv::waitKey(1);
        
        location_mtx.lock();
        win_info.longitude = longitude;
        win_info.latitude = latitude;
        win_info.altitude = altitude;
        location_mtx.unlock();
        win_info.x = (unsigned short)Airpack->winX;
        win_info.y = (unsigned short)Airpack->winY;
        win_info.win_idx = win_idx;

        Task task;
        task.result = result.clone();
        task.labels = labels;
        task.win_info = win_info;
        task.endata = nullptr;
        task.ensize = 0;
        task.winX = Airpack->winX;
        task.winY = Airpack->winY;
        task.frame_id = Airpack->frame_id;
        task.flag_auto_task = flag_auto;
        task.flag_manual_task = flag_manual;

        task.bounding = bounding;
        task.remote_server = &remote_server;
        task.send_server = &send_server;
        task.image_server = &image_server;
        task.win_idx = win_idx;
        task.if_lost = tracker.if_lost;
        
        {
            std::lock_guard<std::mutex> lock(queue_mtx);
            task_queue.push(task);
        }
        queue_cv.notify_one();

        delete Airpack;

        if (flag_end || flag_exit)
        {
            {
                std::lock_guard<std::mutex> lock(queue_mtx);
                stop_processing = true;
            }
            queue_cv.notify_all();
            consumer.join();
            break;
        }             
    }
    VPS4MPcap::instance().pcapClose();
    delete encode;
    receive_server.destory();
    remote_server.destory();
    send_server.destory();
    MLOG_INFO("退出");
    return 0;
}