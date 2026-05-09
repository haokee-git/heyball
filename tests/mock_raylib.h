#pragma once
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
inline double GetTime(void) {
  return static_cast<double>(GetTickCount64()) / 1000.0;
}
#else
#include <sys/time.h>
inline double GetTime(void) {
  struct timeval tv;
  gettimeofday(&tv, nullptr);
  return tv.tv_sec + tv.tv_usec / 1000000.0;
}
#endif
