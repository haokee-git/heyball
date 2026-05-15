#pragma once

#include <raylib.h>
#include <string>
#include <vector>

void Utf8PopBack(std::string &s);
std::vector<int> BuildFontCodepoints();
void AppendUtf8(std::string &s, int codepoint);
int Utf8PrevPos(const std::string &s, int pos);
int Utf8NextPos(const std::string &s, int pos);
int Utf8CodepointCount(const std::string &s);
void TextInsertAt(std::string &s, int &cursor, int codepoint);
void TextEraseBefore(std::string &s, int &cursor);
void TextEraseAfter(std::string &s, int &cursor);
int TextCursorFromMouseX(Font font, const std::string &text, float mouseX,
                         float textX, float fontSize);

struct BackspaceRepeat {
  bool wasDown = false;
  double pressTime = 0.0;
  double lastRepeat = 0.0;
  bool repeating = false;

  bool tick(bool down, double now);
};
