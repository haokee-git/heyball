#include "text_utils.hpp"

#include <algorithm>

#ifdef _WIN32
extern "C" __declspec(dllimport) int __stdcall MultiByteToWideChar(
    unsigned int codePage, unsigned long flags, const char *multiByteStr,
    int multiByteCount, wchar_t *wideCharStr, int wideCharCount);
#endif

void Utf8PopBack(std::string &s) {
  if (s.empty()) return;
  size_t i = s.size() - 1;
  while (i > 0 && (s[i] & 0xC0) == 0x80) --i;
  s.resize(i);
}

std::vector<int> BuildFontCodepoints() {
  std::vector<int> codepoints;
  auto addRange = [&](int first, int last) {
    for (int c = first; c <= last; ++c) codepoints.push_back(c);
  };
  addRange(32, 126);
  addRange(161, 255);
  addRange(256, 383);
  addRange(0x2000, 0x206F);
  addRange(0x2100, 0x214F);
  addRange(0x2190, 0x21FF);
  addRange(0x2460, 0x24FF);
  addRange(0x25A0, 0x25FF);
  addRange(0x3000, 0x303F);
  addRange(0xFE10, 0xFE1F);
  addRange(0xFE30, 0xFE4F);
  addRange(0xFF00, 0xFFEF);
#ifdef _WIN32
  constexpr unsigned int kGbCodePage = 936;
  constexpr unsigned long kErrInvalidChars = 0x00000008;
  for (int high = 0xA1; high <= 0xF7; ++high) {
    for (int low = 0xA1; low <= 0xFE; ++low) {
      const char bytes[2] = {static_cast<char>(high), static_cast<char>(low)};
      wchar_t wide[2]{};
      if (MultiByteToWideChar(kGbCodePage, kErrInvalidChars, bytes, 2, wide, 2) == 1) {
        codepoints.push_back(static_cast<int>(wide[0]));
      }
    }
  }
#else
  addRange(0x4E00, 0x9FFF);
#endif
  std::sort(codepoints.begin(), codepoints.end());
  codepoints.erase(std::unique(codepoints.begin(), codepoints.end()), codepoints.end());
  return codepoints;
}

void AppendUtf8(std::string &s, int codepoint) {
  if (codepoint < 0x80) {
    s += static_cast<char>(codepoint);
  } else if (codepoint < 0x800) {
    s += static_cast<char>(0xC0 | (codepoint >> 6));
    s += static_cast<char>(0x80 | (codepoint & 0x3F));
  } else if (codepoint < 0x10000) {
    s += static_cast<char>(0xE0 | (codepoint >> 12));
    s += static_cast<char>(0x80 | ((codepoint >> 6) & 0x3F));
    s += static_cast<char>(0x80 | (codepoint & 0x3F));
  } else if (codepoint < 0x110000) {
    s += static_cast<char>(0xF0 | (codepoint >> 18));
    s += static_cast<char>(0x80 | ((codepoint >> 12) & 0x3F));
    s += static_cast<char>(0x80 | ((codepoint >> 6) & 0x3F));
    s += static_cast<char>(0x80 | (codepoint & 0x3F));
  }
}

int Utf8PrevPos(const std::string &s, int pos) {
  if (pos <= 0 || s.empty()) return 0;
  size_t i = static_cast<size_t>(pos);
  do { --i; } while (i > 0 && (s[i] & 0xC0) == 0x80);
  return static_cast<int>(i);
}

int Utf8NextPos(const std::string &s, int pos) {
  if (pos < 0 || static_cast<size_t>(pos) >= s.size()) return static_cast<int>(s.size());
  size_t i = static_cast<size_t>(pos);
  if ((s[i] & 0x80) == 0) return static_cast<int>(i + 1);
  size_t len = s.size();
  if ((s[i] & 0xE0) == 0xC0) {
    if (i + 1 < len) return static_cast<int>(i + 2);
  } else if ((s[i] & 0xF0) == 0xE0) {
    if (i + 2 < len) return static_cast<int>(i + 3);
  } else if ((s[i] & 0xF8) == 0xF0) {
    if (i + 3 < len) return static_cast<int>(i + 4);
  }
  return static_cast<int>(i + 1);
}

int Utf8CodepointCount(const std::string &s) {
  int count = 0;
  for (int i = 0; i < static_cast<int>(s.size()); i = Utf8NextPos(s, i)) {
    ++count;
  }
  return count;
}

void TextInsertAt(std::string &s, int &cursor, int codepoint) {
  if (codepoint < 0x80) {
    s.insert(static_cast<size_t>(cursor), 1, static_cast<char>(codepoint));
    ++cursor;
  } else if (codepoint < 0x800) {
    s.insert(static_cast<size_t>(cursor), 1, static_cast<char>(0xC0 | (codepoint >> 6)));
    s.insert(static_cast<size_t>(cursor) + 1, 1, static_cast<char>(0x80 | (codepoint & 0x3F)));
    cursor += 2;
  } else if (codepoint < 0x10000) {
    s.insert(static_cast<size_t>(cursor), 1, static_cast<char>(0xE0 | (codepoint >> 12)));
    s.insert(static_cast<size_t>(cursor) + 1, 1, static_cast<char>(0x80 | ((codepoint >> 6) & 0x3F)));
    s.insert(static_cast<size_t>(cursor) + 2, 1, static_cast<char>(0x80 | (codepoint & 0x3F)));
    cursor += 3;
  } else if (codepoint < 0x110000) {
    s.insert(static_cast<size_t>(cursor), 1, static_cast<char>(0xF0 | (codepoint >> 18)));
    s.insert(static_cast<size_t>(cursor) + 1, 1, static_cast<char>(0x80 | ((codepoint >> 12) & 0x3F)));
    s.insert(static_cast<size_t>(cursor) + 2, 1, static_cast<char>(0x80 | ((codepoint >> 6) & 0x3F)));
    s.insert(static_cast<size_t>(cursor) + 3, 1, static_cast<char>(0x80 | (codepoint & 0x3F)));
    cursor += 4;
  }
}

void TextEraseBefore(std::string &s, int &cursor) {
  if (cursor <= 0) return;
  int prev = Utf8PrevPos(s, cursor);
  s.erase(static_cast<size_t>(prev), static_cast<size_t>(cursor - prev));
  cursor = prev;
}

void TextEraseAfter(std::string &s, int &cursor) {
  if (cursor >= static_cast<int>(s.size())) return;
  int next = Utf8NextPos(s, cursor);
  s.erase(static_cast<size_t>(cursor), static_cast<size_t>(next - cursor));
}

int TextCursorFromMouseX(Font font, const std::string &text, float mouseX,
                         float textX, float fontSize) {
  float relX = mouseX - textX;
  if (relX <= 0) return 0;
  float totalW = 0;
  int cursor = 0;
  while (cursor < static_cast<int>(text.size())) {
    int next = Utf8NextPos(text, cursor);
    std::string ch = text.substr(static_cast<size_t>(cursor), static_cast<size_t>(next - cursor));
    float w = MeasureTextEx(font, ch.c_str(), fontSize, 0.0f).x;
    if (totalW + w * 0.5f > relX) return cursor;
    totalW += w;
    cursor = next;
  }
  return static_cast<int>(text.size());
}

bool BackspaceRepeat::tick(bool down, double now) {
  if (down) {
    if (!wasDown) {
      wasDown = true;
      pressTime = now;
      lastRepeat = now;
      repeating = false;
      return true;
    }
    if (!repeating) {
      if (now - pressTime >= 0.42) {
        repeating = true;
        lastRepeat = now;
        return true;
      }
    } else {
      if (now - lastRepeat >= 0.055) {
        lastRepeat = now;
        return true;
      }
    }
  } else {
    wasDown = false;
  }
  return false;
}
