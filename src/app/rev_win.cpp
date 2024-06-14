#include <thread>

#include "utils/logger.h"
#include "imgmemorymanager.h"
#include "vps4mpcap.h"
#include <opencv2/video.hpp>
#include <opencv2/opencv.hpp>
#include "utils/threadsafe_queue.h"

void add_multicast_group(const std::string &multicast_address) {
  int fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (fd == -1) {
    perror("socket");
    exit(0);
  }
  struct ip_mreq mreq;  // 多播地址结构体
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

int main()
{
  int port = 5031;
  std::string udp_adderss = "224.0.100.10";
  int win_idx = 1;
  threadsafe_queue<void *> q;
  bool ret = VPS4MPcap::instance().init10g();
  add_multicast_group(udp_adderss);
  if (ret)
  {
    ret = ImgMemoryManager::instance().mallocMemoryPool(512 * 512 *
                                                        static_cast<uint64_t>(10000));
    if (!ret)
    {
      MLOG_ERROR("can't allocate mem");
      throw std::runtime_error("can't allocate mem");
    }
    else
    {
      MLOG_INFO(" allocate mem success");
    }
  }
  else
  {
    MLOG_ERROR("init pcap err.");
    throw std::runtime_error("init pcap err");
  }
  MLOG_INFO("start to capture udp pack.");
  VPS4MPcap::instance().pcapSetHandler(packet_handler_t);
  VPS4MPcap::instance().set_WinIdx(win_idx);

  std::thread([&]()
              { VPS4MPcap::instance().openCapture(("udp and port " + std::to_string(port)).c_str(),
                                                  reinterpret_cast<uint8_t *>(&(q))); }).detach();
  while (true)
  {
    if (!q.empty())
    {
      AirImg *Airpack = reinterpret_cast<AirImg *>(q.wait_and_pop());  // 必须

      //process
      MLOG_INFO("rev paket:%d", Airpack->frame_id);
      cv::Mat img =cv::Mat(512, 512, CV_8UC1, Airpack->frame).clone();
      // cv::imwrite("rev_img/" + std::to_string(Airpack->frame_id) + "_" + std::to_string(Airpack->win_idx) + ".jpg", img);    
      ImgMemoryManager::instance().setFree(Airpack->mempool_point);  //必须的
      delete Airpack;  // 必须的
      cv::imshow("result", img);
      cv::waitKey(1);
    }
  }
  
}
