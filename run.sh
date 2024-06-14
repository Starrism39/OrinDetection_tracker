# 第一个输入是控制信息接收地址（中控机到跟踪模块） 第二是控制信息接收端口 第三是引擎路径 第四是pcap目标组播地址(跟踪模块到中控机) 第五是pcap目标端口 第六是窗口号 
# 第七是视频接收地址 第八是视频接收端口 第九是推流地址 第十是推流端口 第十一是编码器 第十二是编码比特率
cd install
cwd=`pwd`
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:`pwd`/lib/
./orindectection_tracker 224.0.104.10 10390 model/yolov5s_data1_green_255_simplify.engine  224.0.104.10 8189 1 224.0.100.10 5031 224.0.104.10 8190 hevc_nvmpi 240000