#pragma once

#include <chrono>
#include <cstdint>
#include <mutex>
#include <thread>

#ifdef _WIN32
extern "C" __declspec(dllimport) void * __stdcall CreateMutexA(void *, int, const char *);
extern "C" __declspec(dllimport) unsigned long __stdcall WaitForSingleObject(void *, unsigned long);
extern "C" __declspec(dllimport) int __stdcall ReleaseMutex(void *);
extern "C" __declspec(dllimport) int __stdcall CloseHandle(void *);
extern "C" __declspec(dllimport) void __stdcall Sleep(unsigned long milliseconds);
extern "C" uintptr_t __cdecl _beginthreadex(void *, unsigned int,
                                            unsigned int (__stdcall *)(void *),
                                            void *, unsigned int, unsigned int *);
#endif

#if defined(_WIN32) && !defined(_GLIBCXX_HAS_GTHREADS)
namespace std {
class mutex {
public:
  mutex() : handle_(CreateMutexA(nullptr, 0, nullptr)) {}
  ~mutex() {
    if (handle_) CloseHandle(handle_);
  }
  mutex(const mutex &) = delete;
  mutex &operator=(const mutex &) = delete;
  void lock() { WaitForSingleObject(handle_, 0xFFFFFFFFul); }
  void unlock() { ReleaseMutex(handle_); }

private:
  void *handle_ = nullptr;
};

class thread {
public:
  thread() = default;
  template <typename T>
  thread(void (T::*fn)(), T *object) {
    Start(new ThreadCall<T>(fn, object));
  }
  ~thread() = default;
  thread(const thread &) = delete;
  thread &operator=(const thread &) = delete;
  thread(thread &&other) noexcept : handle_(other.handle_) {
    other.handle_ = 0;
  }
  thread &operator=(thread &&other) noexcept {
    if (this != &other) {
      if (handle_) CloseHandle(reinterpret_cast<void *>(handle_));
      handle_ = other.handle_;
      other.handle_ = 0;
    }
    return *this;
  }
  bool joinable() const { return handle_ != 0; }
  void join() {
    if (!handle_) return;
    WaitForSingleObject(reinterpret_cast<void *>(handle_), 0xFFFFFFFFul);
    CloseHandle(reinterpret_cast<void *>(handle_));
    handle_ = 0;
  }

private:
  struct ThreadBase {
    virtual ~ThreadBase() = default;
    virtual void Run() = 0;
  };
  template <typename T>
  struct ThreadCall : ThreadBase {
    ThreadCall(void (T::*fnIn)(), T *objectIn) : fn(fnIn), object(objectIn) {}
    void Run() override { (object->*fn)(); }
    void (T::*fn)();
    T *object;
  };
  static unsigned int __stdcall Entry(void *arg) {
    ThreadBase *call = static_cast<ThreadBase *>(arg);
    call->Run();
    delete call;
    return 0;
  }
  void Start(ThreadBase *call) {
    unsigned int id = 0;
    handle_ = _beginthreadex(nullptr, 0, &thread::Entry, call, 0, &id);
    if (!handle_) delete call;
  }
  uintptr_t handle_ = 0;
};

namespace this_thread {
template <typename Rep, typename Period>
void sleep_for(const chrono::duration<Rep, Period> &duration) {
  const auto millis = chrono::duration_cast<chrono::milliseconds>(duration);
  Sleep(static_cast<unsigned long>(millis.count() > 0 ? millis.count() : 1));
}
} // namespace this_thread
} // namespace std
#endif
