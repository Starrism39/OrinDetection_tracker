#ifndef VPSKIT_CLASS_COMMON_SINGLETON_H
#define VPSKIT_CLASS_COMMON_SINGLETON_H


namespace vpskit {

class NonCopyable {
 protected:
  NonCopyable() = default;
  ~NonCopyable() = default;

 public:
  NonCopyable(const NonCopyable &) = delete;             // Copy construct
  NonCopyable(NonCopyable &&) = delete;                  // Move construct
  NonCopyable &operator=(const NonCopyable &) = delete;  // Copy assign
  NonCopyable &operator=(NonCopyable &&) = delete;       // Move assign
};

template <typename T>
class Singleton : public NonCopyable {
 public:
  static T &instance() {
    static T _instance;
    return _instance;
  }
};

}  // namespace vpskit

#endif  // VPSKIT_CLASS_COMMON_SINGLETON_H
