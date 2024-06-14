#pragma once

#include <pcap/pcap.h>

#include <atomic>

#include "airimgrecv.h"
#include "singleton.h"

typedef void VPSMPcapHandler(unsigned char *param, const struct pcap_pkthdr *header, const unsigned char *pkt_data);

#define VPS2M_ETHERNET_CNT 20
#define VPS2M_ETHERNET_LEN 1024

typedef struct TAG_EthernetInfo
{
  char Ethernet[VPS2M_ETHERNET_CNT][VPS2M_ETHERNET_LEN];
  unsigned int NetMask[VPS2M_ETHERNET_CNT];
  int EthernetCnt = -1;
} VPSEthernetInfo;

class VPS4MPcap : public vpskit::Singleton<VPS4MPcap>
{
  friend vpskit::Singleton<VPS4MPcap>;

private:
  VPS4MPcap();
  ~VPS4MPcap();

public:
  bool init10g();


public:
  /// 查询网卡数量和名字\n
  /// 返回false表示未找到网卡
  bool pcapQueryEthernet();
  /// 返回查询到的网卡数量，供上层遍历使用
  int getEthernetCnt();

  /// 打开并初始化某个网卡\n
  /// 自动开启一个线程, 死循环处理网卡接收的数据\n
  /// 回调由上层注入，并且注意这个回调处理必须快，否则会丢失数据\n
  void pcapSetHandler(VPSMPcapHandler *packet_handler);
  /// 网卡index（从0开始），调用前需要先设置handler; 如果m_ethHandle不为0则强制reset\n
  /// 返回false，表示网卡打开失败
  bool pcapInit(int index);

  /// 退出loop及loop线程, 如果存在
  void pcapClose();

  pcap_t *getEthHandle() { return m_ethHandle; }

  VPSEthernetInfo getEthernetScanResult();

  void closeAll();
  int openCapture(const char *filter_exp, uint8_t *custom_param);
  void *recvPack(void *vdata, void *out_data);

  void set_WinIdx(uint8_t idx)
  {
    imgrev_->set_WinIdx(idx);
  }

private:
  VPSEthernetInfo *m_info;
  pcap_t *volatile m_ethHandle; // 跨线程调用了
  int volatile m_ethIndex;      // 网卡索引，第xx张网卡

  VPSMPcapHandler *m_packet_handler;
  AirImgRecv *imgrev_;
  
  bool opened_=false;
};

bool recvCheck(uint8_t *buf, int len);
