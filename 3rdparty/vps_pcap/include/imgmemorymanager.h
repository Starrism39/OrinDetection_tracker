#pragma once

#include <atomic>
#include <cstdint>
#include <cstddef>

#include "singleton.h"

#define WINMODEWIDTH 512
#define WINMODEHEIGHT 512


class ImgMemoryManager : public vpskit::Singleton<ImgMemoryManager>
{
  friend class vpskit::Singleton<ImgMemoryManager>;

public:
  ImgMemoryManager() {}
  ~ImgMemoryManager();

  bool mallocMemoryPool(size_t mem_pool_size); // 仅仅分配1张10bits原始图片的内存
  void releaseMemory();

  unsigned char *getFreeMem(int offset);

  void setBusy();
  void setFree(int point);

private:
  unsigned char **m_pool; // 分配内存池

  int platform_;
  size_t mem_pool_size_;
  std::atomic<bool> *busy_flags_;
  int using_point_ = -1;
  int img_nums_;
};
