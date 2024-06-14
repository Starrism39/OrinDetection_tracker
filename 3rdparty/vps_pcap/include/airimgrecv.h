#pragma once

#include <atomic>

struct AirImgPacket
{
  uint8_t fun;
  uint8_t model;
  uint8_t frame;
  uint8_t quad;     // 象限号、窗口号
  uint16_t line;    // line, scan serial num, or reserve
  uint16_t pkt;     // pkt
  uint16_t winX;    // winX, or reserve
  uint16_t winY;    // winY, or reserve
  uint32_t reserve; // 4 Bytes
};

enum ScanAreaColor
{
  scanAreaYellow = 0,
  scanAreaBlue,
  scanAreaGreen,
  scanAreaRed,
  scanAreaColorMax
};

union AirLastPacket_Union
{
  AirImgPacket normal[4]; // 4 quad
  AirImgPacket win8_16[8];
  //    AirImgPacket win16[4];
  AirImgPacket scan[scanAreaColorMax];
};

struct AirLastPacketCheck
{
  AirLastPacket_Union model[7];
};

struct AirImg
{
  uint8_t frame_id;
  uint8_t *frame;
  uint8_t win_idx; // 象限号、窗口号
  uint16_t winX;   // winX, or reserve
  uint16_t winY;   // winY, or reserve
  int mempool_point;
};

class AirImgRecv
{
public:
  explicit AirImgRecv();
  ~AirImgRecv();
  void *imgRecv(uint8_t *data, void *out_data);

  void startRecv() { m_shouldRecv = true; }
  void stopRecv() { m_shouldRecv = false; }

  void set_WinIdx(uint8_t idx)
  {
    win_idx_ = idx;
  }

private:
  bool isLastPacket(uint8_t *data);

private:
  bool m_shouldRecv = true;
  bool m_win8Model = true;
  bool m_win1Model = true; // only one win
  bool m_save = false;
  bool m_view = false;
  uint8_t win_idx_ = 0;
  uint8_t *mempool;
};
