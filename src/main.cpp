#include "core.hpp"
#include "network.hpp"
#include <raylib.h>
#include <raymath.h>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>
#ifdef _WIN32
extern "C" __declspec(dllimport) int __stdcall AllocConsole(void);
struct HeyballWinPoint {
  long x;
  long y;
};
extern "C" __declspec(dllimport) int __stdcall GetCursorPos(HeyballWinPoint *point);
extern "C" __declspec(dllimport) short __stdcall GetAsyncKeyState(int key);
extern "C" __declspec(dllimport) int __stdcall MultiByteToWideChar(
    unsigned int codePage, unsigned long flags, const char *multiByteStr,
    int multiByteCount, wchar_t *wideCharStr, int wideCharCount);
extern "C" __declspec(dllimport) int __stdcall SetWindowPos(void *hwnd, void *insertAfter,
                                                           int x, int y, int cx, int cy,
                                                           unsigned int flags);
#endif
namespace {
using hb::Ball;
using hb::BallGroup;
using hb::Phase;
using hb::ShotEvents;
using hb::ShotParams;
using hb::Vec2;
constexpr int kUiFontSize = 96;
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
  if ((s[i] & 0xE0) == 0xC0) { if (i + 1 < len) return static_cast<int>(i + 2); }
  else if ((s[i] & 0xF0) == 0xE0) { if (i + 2 < len) return static_cast<int>(i + 3); }
  else if ((s[i] & 0xF8) == 0xF0) { if (i + 3 < len) return static_cast<int>(i + 4); }
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
int TextCursorFromMouseX(Font font, const std::string &text, float mouseX, float textX, float fontSize) {
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
struct BackspaceRepeat {
  bool wasDown = false;
  double pressTime = 0.0;
  double lastRepeat = 0.0;
  bool repeating = false;
  // Returns true when one character should be deleted this frame.
  bool tick(bool down, double now) {
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
};
struct View {
  Rectangle play{};
  double scale = 1.0;
  float uiScale = 1.0f;
};
enum class AppMode { Single, Lobby, RoomHost, RoomClient, PlayingOnline };
struct Game {
  hb::PhysicsWorld world;
  hb::RulesEngine rules;
  Phase phase = Phase::Aiming;
  ShotEvents shotEvents;
  Vec2 aim{1.0, 0.0};
  double power = 0.0;
  double tipX = 0.0;
  double tipY = 0.0;
  Font font{};
  bool customFont = false;
  bool draggingTitle = false;
  int resizeMode = 0;
  Vector2 windowActionMouseStart{};
  Vector2 windowActionPosStart{};
  int windowActionWidthStart = 0;
  int windowActionHeightStart = 0;
  bool requestClose = false;
  bool helpOpen = false;
  double splashStartTime = 0.0;
  bool splashDone = false;
  RenderTexture2D splashTexture{};
  // Saved single-player state for returning from lobby
  hb::PhysicsWorld savedWorld;
  hb::RulesEngine savedRules;
  Phase savedPhase = Phase::Aiming;
  Vec2 savedAim{1.0, 0.0};
  double savedPower = 0.0;
  // Network
  AppMode appMode = AppMode::Single;
  hb::NetworkHost host;
  hb::NetworkClient client;
  std::vector<hb::RoomInfo> roomList;
  std::string roomName;
  std::string roomPassword;
  bool hostIsPlayer1 = true;
  bool amReady = false;
  bool opReady = false;
  std::vector<std::string> chatHistory;
  bool chatInputActive = false;
  std::string chatInputBuf;
  int chatInputCursor = 0;
  BackspaceRepeat bsRepeat;
  BackspaceRepeat leftRepeat;
  BackspaceRepeat rightRepeat;
  BackspaceRepeat delRepeat;
  bool textAllSelected = false;
  double lastAimSend = 0.0;
  double lastPosSend = 0.0;
  bool showCreateDlg = false;
  bool showJoinPwdDlg = false;
  std::string createRoomName;
  int createNameCursor = 0;
  std::string createRoomPwd;
  int createPwdCursor = 0;
  std::string createRoomNotice;
  bool createHostIsP1 = true;
  std::string pendingJoinRoomName;
  std::string pendingJoinHostIP;
  std::string joinPwdInput;
  int joinPwdCursor = 0;
  bool showPwd = false;
  int onlineTurn = -1;  // 0 or 1, who plays on this machine (host perspective)
  int assignedPlayer = -1;  // received from host via ASSIGN message
  std::array<hb::Ball, 16> syncedBalls{};
  double opAimTipX = 0.0, opAimTipY = 0.0, opPower = 0.0, opAimX = 1.0, opAimY = 0.0;
  int createDlgFocus = 0;  // 0=room name, 1=password
};
enum ResizeEdge {
  ResizeNone = 0,
  ResizeLeft = 1,
  ResizeRight = 2,
  ResizeTop = 4,
  ResizeBottom = 8
};
Vector2 DesktopMousePosition() {
#ifdef _WIN32
  HeyballWinPoint point{};
  if (GetCursorPos(&point)) {
    return {static_cast<float>(point.x), static_cast<float>(point.y)};
  }
#endif
  return GetMousePosition();
}
bool IsPrimaryMouseDown() {
#ifdef _WIN32
  constexpr int kVkLButton = 0x01;
  return (GetAsyncKeyState(kVkLButton) & 0x8000) != 0;
#else
  return IsMouseButtonDown(MOUSE_LEFT_BUTTON);
#endif
}
void ApplyWindowBounds(int x, int y, int width, int height) {
#ifdef _WIN32
  constexpr unsigned int kNoZOrder = 0x0004;
  constexpr unsigned int kNoActivate = 0x0010;
  if (void *handle = GetWindowHandle()) {
    SetWindowPos(handle, nullptr, x, y, width, height, kNoZOrder | kNoActivate);
    return;
  }
#endif
  SetWindowPosition(x, y);
  SetWindowSize(width, height);
}
void BeginWindowAction(Game &game, int resizeMode) {
  game.resizeMode = resizeMode;
  game.draggingTitle = resizeMode == ResizeNone;
  game.windowActionMouseStart = DesktopMousePosition();
  game.windowActionPosStart = GetWindowPosition();
  game.windowActionWidthStart = GetScreenWidth();
  game.windowActionHeightStart = GetScreenHeight();
}
View MakeView(int sw, int sh) {
  const float uiScale = static_cast<float>(
      hb::Clamp(std::min(sw / 1360.0, sh / 820.0), 0.72, 1.55));
  const float marginX = 78.0f * uiScale;
  const float uiReserve = 285.0f * uiScale;
  const float framePad = 16.0f * uiScale;
  const float topClearance = 86.0f * uiScale;
  const float bottomClearance = 80.0f * uiScale;
  const float availableW =
      std::max(1.0f, static_cast<float>(sw) - marginX * 2.0f - uiReserve);
  const float tableTop = topClearance + framePad;
  const float tableBottom =
      std::max(tableTop + 1.0f, static_cast<float>(sh) - bottomClearance - framePad);
  const float availableH = tableBottom - tableTop;
  const float tableAspect = static_cast<float>(hb::kTableWidth / hb::kTableHeight);
  float w = availableW;
  float h = w / tableAspect;
  if (h > availableH) {
    h = availableH;
    w = h * tableAspect;
  }
  Rectangle r{marginX, tableTop + (availableH - h) * 0.5f, w, h};
  return {r, w / hb::kTableWidth, uiScale};
}
Vector2 WorldToScreen(const View &view, Vec2 p) {
  return {view.play.x + view.play.width * 0.5f + static_cast<float>(p.x * view.scale),
          view.play.y + view.play.height * 0.5f + static_cast<float>(p.y * view.scale)};
}
Vec2 ScreenToWorld(const View &view, Vector2 p) {
  return {(p.x - (view.play.x + view.play.width * 0.5f)) / view.scale,
          (p.y - (view.play.y + view.play.height * 0.5f)) / view.scale};
}
Vec2 ClampCuePlacement(Vec2 pos) {
  const double left = -hb::kTableWidth * 0.5 + hb::kCushionNoseInset + hb::kBallRadius;
  const double right = hb::kTableWidth * 0.5 - hb::kCushionNoseInset - hb::kBallRadius;
  const double top = -hb::kTableHeight * 0.5 + hb::kCushionNoseInset + hb::kBallRadius;
  const double bottom = hb::kTableHeight * 0.5 - hb::kCushionNoseInset - hb::kBallRadius;
  return {hb::Clamp(pos.x, left, right), hb::Clamp(pos.y, top, bottom)};
}
Vec2 RayToTableEdge(Vec2 origin, Vec2 dir) {
  const double left = -hb::kTableWidth * 0.5 + hb::kCushionNoseInset;
  const double right = hb::kTableWidth * 0.5 - hb::kCushionNoseInset;
  const double top = -hb::kTableHeight * 0.5 + hb::kCushionNoseInset;
  const double bottom = hb::kTableHeight * 0.5 - hb::kCushionNoseInset;
  double best = 1e9;
  if (std::abs(dir.x) > 1e-8) {
    const double tLeft = (left - origin.x) / dir.x;
    const double yLeft = origin.y + dir.y * tLeft;
    if (tLeft > 0.0 && yLeft >= top && yLeft <= bottom) {
      best = std::min(best, tLeft);
    }
    const double tRight = (right - origin.x) / dir.x;
    const double yRight = origin.y + dir.y * tRight;
    if (tRight > 0.0 && yRight >= top && yRight <= bottom) {
      best = std::min(best, tRight);
    }
  }
  if (std::abs(dir.y) > 1e-8) {
    const double tTop = (top - origin.y) / dir.y;
    const double xTop = origin.x + dir.x * tTop;
    if (tTop > 0.0 && xTop >= left && xTop <= right) {
      best = std::min(best, tTop);
    }
    const double tBottom = (bottom - origin.y) / dir.y;
    const double xBottom = origin.x + dir.x * tBottom;
    if (tBottom > 0.0 && xBottom >= left && xBottom <= right) {
      best = std::min(best, tBottom);
    }
  }
  return origin + dir * (best == 1e9 ? 1.0 : best);
}
int FirstAimTarget(const hb::PhysicsWorld &world, Vec2 origin, Vec2 dir, Vec2 *ghost) {
  double best = 1e9;
  int target = -1;
  for (const Ball &ball : world.Balls()) {
    if (ball.number == 0 || ball.pocketed || ball.sinking) {
      continue;
    }
    const Vec2 rel = ball.pos - origin;
    const double forward = Dot(rel, dir);
    if (forward <= 0.0) {
      continue;
    }
    const double closestSq = LengthSq(rel) - forward * forward;
    const double hitRadius = hb::kBallDiameter;
    if (closestSq > hitRadius * hitRadius) {
      continue;
    }
    const double offset = std::sqrt(std::max(0.0, hitRadius * hitRadius - closestSq));
    const double travel = forward - offset;
    if (travel > 0.0 && travel < best) {
      best = travel;
      target = ball.number;
      if (ghost) {
        *ghost = origin + dir * travel;
      }
    }
  }
  return target;
}
bool PointInRect(Vector2 p, Rectangle r) {
  return p.x >= r.x && p.x <= r.x + r.width && p.y >= r.y &&
         p.y <= r.y + r.height;
}
Font LoadGameFont(std::vector<int> &codepoints) {
  Font font = LoadFontEx("PingFangSC.otf", kUiFontSize,
                         codepoints.data(), static_cast<int>(codepoints.size()));
  if (font.texture.id != 0) return font;
  font = LoadFontEx("C:/Windows/Fonts/msyh.ttc", kUiFontSize,
                    codepoints.data(), static_cast<int>(codepoints.size()));
  if (font.texture.id != 0) return font;
  font = LoadFontEx("C:/Windows/Fonts/msyhbd.ttc", kUiFontSize,
                    codepoints.data(), static_cast<int>(codepoints.size()));
  if (font.texture.id != 0) return font;
  return Font{};
}
void ConfigureFontTexture(Font &font) {
  if (font.texture.id == 0) return;
  SetTextureFilter(font.texture, TEXTURE_FILTER_BILINEAR);
  SetTextureWrap(font.texture, TEXTURE_WRAP_CLAMP);
}
void DrawTextF(Font font, const char *text, Vector2 pos, float size, Color color) {
  const Vector2 p{std::round(pos.x), std::round(pos.y)};
  const float fs = std::max(1.0f, std::round(size));
  DrawTextEx(font, text, p, fs, 0.0f, color);
}
void DrawRoundedFillWithBorder(Rectangle r, float roundness, int segments,
                               Color fill, Color border) {
  Rectangle px{std::round(r.x), std::round(r.y), std::round(r.width),
               std::round(r.height)};
  DrawRectangleRounded(px, roundness, segments, border);
  const Rectangle inner{px.x + 1.0f, px.y + 1.0f, px.width - 2.0f,
                        px.height - 2.0f};
  DrawRectangleRounded(inner, roundness, segments, fill);
}
Color BallColor(int n) {
  switch (n) {
  case 1:
  case 9:
    return {222, 188, 46, 255};
  case 2:
  case 10:
    return {38, 91, 185, 255};
  case 3:
  case 11:
    return {185, 48, 46, 255};
  case 4:
  case 12:
    return {94, 57, 145, 255};
  case 5:
  case 13:
    return {212, 113, 38, 255};
  case 6:
  case 14:
    return {36, 127, 82, 255};
  case 7:
  case 15:
    return {115, 54, 36, 255};
  case 8:
    return {18, 18, 18, 255};
  default:
    return {235, 232, 215, 255};
  }
}
void DrawBackdrop() {
  DrawRectangle(0, 0, GetScreenWidth(), GetScreenHeight(), {0, 0, 0, 140});
}
Color MixColor(Color a, Color b, float t) {
  t = static_cast<float>(hb::Clamp(t, 0.0, 1.0));
  return {
      static_cast<unsigned char>(a.r * (1.0f - t) + b.r * t),
      static_cast<unsigned char>(a.g * (1.0f - t) + b.g * t),
      static_cast<unsigned char>(a.b * (1.0f - t) + b.b * t),
      static_cast<unsigned char>(a.a * (1.0f - t) + b.a * t)};
}
Vector3 BallLocalFromWorld(const Ball &ball, Vector3 world) {
  const auto &m = ball.orientation;
  return {static_cast<float>(world.x * m[0] + world.y * m[3] + world.z * m[6]),
          static_cast<float>(world.x * m[1] + world.y * m[4] + world.z * m[7]),
          static_cast<float>(world.x * m[2] + world.y * m[5] + world.z * m[8])};
}
Vector3 BallWorldFromLocal(const Ball &ball, Vector3 local) {
  const auto &m = ball.orientation;
  return {static_cast<float>(local.x * m[0] + local.y * m[1] + local.z * m[2]),
          static_cast<float>(local.x * m[3] + local.y * m[4] + local.z * m[5]),
          static_cast<float>(local.x * m[6] + local.y * m[7] + local.z * m[8])};
}
std::array<Vector3, 2> NumberSpotLocals() {
  return {{{0.0f, 0.0f, 1.0f},
           {0.0f, 0.0f, -1.0f}}};
}
struct NumberSpot {
  Vector3 local{};
  Vector3 world{};
  float visibility = 0.0f;
};
float NumberSpotVisibility(float facing) {
  const float t = static_cast<float>(hb::Clamp((facing - 0.01f) / 0.58f, 0.0, 1.0));
  return t * t * (3.0f - 2.0f * t);
}
std::array<NumberSpot, 2> NumberSpotsForBall(const Ball &ball) {
  const auto spots = NumberSpotLocals();
  std::array<NumberSpot, 2> out{};
  for (int i = 0; i < 2; ++i) {
    const Vector3 world = BallWorldFromLocal(ball, spots[i]);
    const NumberSpot candidate{spots[i], world, NumberSpotVisibility(world.z)};
    if (candidate.visibility > out[0].visibility) {
      out[1] = out[0];
      out[0] = candidate;
    } else if (candidate.visibility > out[1].visibility) {
      out[1] = candidate;
    }
  }
  return out;
}
Color ShadeBallPixel(Color base, Vector3 n, float edgeAlpha) {
  const Vector3 light = Vector3Normalize({-0.45f, -0.58f, 0.82f});
  const float diffuse = std::max(0.0f, Vector3DotProduct(n, light));
  const float rim = std::pow(std::max(0.0f, 1.0f - n.z), 1.6f);
  const float highlight = std::pow(std::max(0.0f, diffuse), 28.0f);
  const float shade = 0.44f + diffuse * 0.58f - rim * 0.16f;
  Color out{
      static_cast<unsigned char>(hb::Clamp(base.r * shade + 255.0f * highlight * 0.32f, 0.0, 255.0)),
      static_cast<unsigned char>(hb::Clamp(base.g * shade + 255.0f * highlight * 0.32f, 0.0, 255.0)),
      static_cast<unsigned char>(hb::Clamp(base.b * shade + 255.0f * highlight * 0.32f, 0.0, 255.0)),
      static_cast<unsigned char>(hb::Clamp(edgeAlpha * 255.0f, 0.0, 255.0))};
  return out;
}
void DrawBall(Font font, const View &view, const Ball &ball) {
  if ((ball.pocketed || ball.sinking) && ball.pocketFade >= 1.0) {
    return;
  }
  const Vector2 c = WorldToScreen(view, ball.pos);
  float r = static_cast<float>(hb::kBallRadius * view.scale);
  float visualAlpha = 1.0f;
  if (ball.sinking || ball.pocketed) {
    const float drop = static_cast<float>(hb::Clamp(ball.pocketFade, 0.0, 1.0));
    r *= 1.0f - drop * 0.72f;
    visualAlpha = 1.0f - drop * 0.46f;
  }
  DrawCircleV({c.x + r * 0.18f, c.y + r * 0.22f}, r * 1.02f,
              {0, 0, 0, static_cast<unsigned char>(115 * visualAlpha)});
  const bool stripe = hb::IsStripe(ball.number);
  const Color base = BallColor(ball.number);
  const Color ivory{246, 243, 226, 255};
  const auto numberSpots = NumberSpotsForBall(ball);
  const int numberSpotCount = ball.number > 0 ? 2 : 0;
  const float plaqueAngle = stripe ? 0.395f : 0.430f;
  const float plaqueCos = std::cos(plaqueAngle);
  const float plaqueFeather = 0.040f;
  const int minX = static_cast<int>(std::floor(c.x - r - 1.0f));
  const int maxX = static_cast<int>(std::ceil(c.x + r + 1.0f));
  const int minY = static_cast<int>(std::floor(c.y - r - 1.0f));
  const int maxY = static_cast<int>(std::ceil(c.y + r + 1.0f));
  for (int py = minY; py <= maxY; ++py) {
    for (int px = minX; px <= maxX; ++px) {
      const float nx = (px + 0.5f - c.x) / r;
      const float ny = (py + 0.5f - c.y) / r;
      const float d2 = nx * nx + ny * ny;
      if (d2 > 1.0f) {
        continue;
      }
      const float nz = std::sqrt(std::max(0.0f, 1.0f - d2));
      Vector3 normal{nx, ny, nz};
      Vector3 local = BallLocalFromWorld(ball, normal);
      Color material = ball.number == 0 ? ivory : base;
      if (stripe) {
        const float stripeWidth = 0.43f;
        const float feather = 0.055f;
        const float band = std::abs(local.y);
        if (band > stripeWidth + feather) {
          material = ivory;
        } else if (band > stripeWidth) {
          const float t = (band - stripeWidth) / feather;
          material = MixColor(base, ivory, t);
        }
      }
      for (int i = 0; i < numberSpotCount; ++i) {
        const NumberSpot &spot = numberSpots[i];
        const float spotWeight = i == 0 ? 1.0f : 0.45f;
        if (spot.visibility <= (i == 0 ? 0.01f : 0.22f)) {
          continue;
        }
        const float d = Vector3DotProduct(local, spot.local);
        if (d >= plaqueCos - plaqueFeather) {
          const float t = (d - (plaqueCos - plaqueFeather)) / plaqueFeather;
          material = MixColor(material, ivory, t * spot.visibility * spotWeight);
        }
      }
      const float edgeAlpha = d2 > 0.90f ? (1.0f - d2) / 0.10f : 1.0f;
      Color pixel = ShadeBallPixel(material, normal, edgeAlpha);
      pixel.a = static_cast<unsigned char>(pixel.a * visualAlpha);
      DrawPixel(px, py, pixel);
    }
  }
  DrawCircleV({c.x - r * 0.32f, c.y - r * 0.34f}, r * 0.18f,
              {255, 255, 255, static_cast<unsigned char>(110 * visualAlpha)});
  DrawCircleLines(static_cast<int>(c.x), static_cast<int>(c.y), r,
                  {25, 25, 25, static_cast<unsigned char>(150 * visualAlpha)});
  if (ball.number > 0) {
    char label[4]{};
    std::snprintf(label, sizeof(label), "%d", ball.number);
    for (int i = 0; i < numberSpotCount; ++i) {
      const NumberSpot &spot = numberSpots[i];
      const float spotWeight = i == 0 ? 1.0f : 0.45f;
      if (spot.visibility <= (i == 0 ? 0.06f : 0.26f)) {
        continue;
      }
      const Vector2 d{c.x + spot.world.x * r, c.y + spot.world.y * r};
      Vector3 tangentLocal =
          Vector3CrossProduct({0.0f, 1.0f, 0.0f}, spot.local);
      if (Vector3Length(tangentLocal) < 0.01f) {
        tangentLocal = Vector3CrossProduct({1.0f, 0.0f, 0.0f}, spot.local);
      }
      tangentLocal = Vector3Normalize(tangentLocal);
      const Vector3 tangentWorld = BallWorldFromLocal(ball, tangentLocal);
      const float rotation =
          std::atan2(tangentWorld.y, tangentWorld.x) * 57.2957795f;
      const float faceScale = 0.76f + 0.24f * spot.visibility;
      const float fs = std::max((ball.number >= 10 ? 7.0f : 8.0f) * view.uiScale,
                                (ball.number >= 10 ? r * 0.58f : r * 0.74f)) *
                       faceScale;
      const Vector2 sz = MeasureTextEx(font, label, fs, 0.0f);
      const Vector2 origin{sz.x * 0.5f, sz.y * 0.5f};
      const Color textColor{
          10, 10, 10,
          static_cast<unsigned char>(255 * spot.visibility * spotWeight * visualAlpha)};
      const float bold = std::max(0.35f, 0.55f * view.uiScale);
      DrawTextPro(font, label, {d.x + bold, d.y}, origin, rotation, fs, 0.0f,
                  textColor);
      DrawTextPro(font, label, {d.x, d.y + bold}, origin, rotation, fs, 0.0f,
                  textColor);
      DrawTextPro(font, label, {d.x, d.y}, origin, rotation, fs, 0.0f,
                  textColor);
    }
  }
}
Vector2 BezierCubic(Vector2 a, Vector2 b, Vector2 c, Vector2 d, float t) {
  const float u = 1.0f - t;
  return {u * u * u * a.x + 3.0f * u * u * t * b.x +
              3.0f * u * t * t * c.x + t * t * t * d.x,
          u * u * u * a.y + 3.0f * u * u * t * b.y +
              3.0f * u * t * t * c.y + t * t * t * d.y};
}
void AppendBezier(std::vector<Vector2> &points, Vector2 a, Vector2 b,
                  Vector2 c, Vector2 d) {
  constexpr int segments = 14;
  for (int i = 1; i <= segments; ++i) {
    points.push_back(BezierCubic(a, b, c, d,
                                 static_cast<float>(i) / segments));
  }
}
void FillPolygonFan(const std::vector<Vector2> &points, Color color) {
  if (points.size() < 3) {
    return;
  }
  Vector2 center{};
  for (Vector2 p : points) {
    center.x += p.x;
    center.y += p.y;
  }
  center.x /= static_cast<float>(points.size());
  center.y /= static_cast<float>(points.size());
  for (size_t i = 0; i < points.size(); ++i) {
    DrawTriangle(center, points[i], points[(i + 1) % points.size()], color);
  }
}
void DrawPolyline(const std::vector<Vector2> &points, float thick,
                  Color color) {
  for (size_t i = 1; i < points.size(); ++i) {
    DrawLineEx(points[i - 1], points[i], thick, color);
  }
}
void DrawHorizontalCushion(float x1, float x2, float outerY, float depth,
                           bool top) {
  if (x2 <= x1 + depth * 2.0f) {
    return;
  }
  const float jaw = std::min(depth * 2.45f, (x2 - x1) * 0.42f);
  const float innerY = outerY + (top ? depth : -depth);
  const float sy = top ? 1.0f : -1.0f;
  const Color rubber{26, 73, 59, 255};
  const Color nose{7, 31, 25, 205};
  std::vector<Vector2> body;
  body.push_back({x1, outerY});
  body.push_back({x2, outerY});
  AppendBezier(body, {x2, outerY},
               {x2 - jaw * 0.08f, outerY + sy * depth * 0.16f},
               {x2 - jaw * 0.52f, innerY}, {x2 - jaw, innerY});
  body.push_back({x1 + jaw, innerY});
  AppendBezier(body, {x1 + jaw, innerY},
               {x1 + jaw * 0.52f, innerY},
               {x1 + jaw * 0.08f, outerY + sy * depth * 0.16f},
               {x1, outerY});
  FillPolygonFan(body, rubber);
  std::vector<Vector2> noseLine{{x1, outerY}};
  AppendBezier(noseLine, {x1, outerY},
               {x1 + jaw * 0.08f, outerY + sy * depth * 0.16f},
               {x1 + jaw * 0.52f, innerY}, {x1 + jaw, innerY});
  noseLine.push_back({x2 - jaw, innerY});
  AppendBezier(noseLine, {x2 - jaw, innerY},
               {x2 - jaw * 0.52f, innerY},
               {x2 - jaw * 0.08f, outerY + sy * depth * 0.16f},
               {x2, outerY});
  DrawPolyline(noseLine, 1.35f, nose);
}
void DrawVerticalCushion(float outerX, float y1, float y2, float depth,
                         bool left) {
  if (y2 <= y1 + depth * 2.0f) {
    return;
  }
  const float jaw = std::min(depth * 2.45f, (y2 - y1) * 0.42f);
  const float innerX = outerX + (left ? depth : -depth);
  const float sx = left ? 1.0f : -1.0f;
  const Color rubber{26, 73, 59, 255};
  const Color nose{7, 31, 25, 205};
  std::vector<Vector2> body;
  body.push_back({outerX, y1});
  body.push_back({outerX, y2});
  AppendBezier(body, {outerX, y2},
               {outerX + sx * depth * 0.16f, y2 - jaw * 0.08f},
               {innerX, y2 - jaw * 0.52f}, {innerX, y2 - jaw});
  body.push_back({innerX, y1 + jaw});
  AppendBezier(body, {innerX, y1 + jaw},
               {innerX, y1 + jaw * 0.52f},
               {outerX + sx * depth * 0.16f, y1 + jaw * 0.08f},
               {outerX, y1});
  FillPolygonFan(body, rubber);
  std::vector<Vector2> noseLine{{outerX, y1}};
  AppendBezier(noseLine, {outerX, y1},
               {outerX + sx * depth * 0.16f, y1 + jaw * 0.08f},
               {innerX, y1 + jaw * 0.52f}, {innerX, y1 + jaw});
  noseLine.push_back({innerX, y2 - jaw});
  AppendBezier(noseLine, {innerX, y2 - jaw},
               {innerX, y2 - jaw * 0.52f},
               {outerX + sx * depth * 0.16f, y2 - jaw * 0.08f},
               {outerX, y2});
  DrawPolyline(noseLine, 1.35f, nose);
}
void DrawPocketShape(const View &view, int index, Vector2 p) {
  const bool side = index == 1 || index == 4;
  const double mouth = side ? hb::kSidePocketMouth : hb::kCornerPocketMouth;
  const float mouthPx = static_cast<float>(mouth * view.scale);
  const float r = std::max(14.0f * view.uiScale, mouthPx * (side ? 0.54f : 0.58f));
  const Color rim{42, 39, 32, 240};
  const Color innerRim{6, 8, 7, 255};
  DrawCircleV(p, r + 4.0f * view.uiScale, rim);
  DrawCircleV(p, r + 1.5f * view.uiScale, innerRim);
  DrawCircleV(p, r, BLACK);
}
void DrawPockets(const View &view) {
  const Vector2 pockets[6] = {
      WorldToScreen(view, {-hb::kTableWidth * 0.5, -hb::kTableHeight * 0.5}),
      WorldToScreen(view, {0.0, -hb::kTableHeight * 0.5}),
      WorldToScreen(view, {hb::kTableWidth * 0.5, -hb::kTableHeight * 0.5}),
      WorldToScreen(view, {-hb::kTableWidth * 0.5, hb::kTableHeight * 0.5}),
      WorldToScreen(view, {0.0, hb::kTableHeight * 0.5}),
      WorldToScreen(view, {hb::kTableWidth * 0.5, hb::kTableHeight * 0.5}),
  };
  for (int i = 0; i < 6; ++i) {
    DrawPocketShape(view, i, pockets[i]);
  }
}
void DrawTable(const View &view) {
  const float frameW = 16.0f * view.uiScale;
  const float cushionD = static_cast<float>(hb::kCushionNoseInset * view.scale);
  const float sideGap =
      static_cast<float>(hb::kSidePocketMouth * 0.74 * view.scale);
  const float cornerGap =
      static_cast<float>(hb::kCornerPocketMouth * 0.74 * view.scale);
  Rectangle rail{view.play.x - frameW, view.play.y - frameW,
                 view.play.width + frameW * 2, view.play.height + frameW * 2};
  DrawRectangleRounded(rail, 0.018f, 12, {17, 19, 18, 255});
  DrawRectangleRec(view.play, {31, 86, 72, 255});
  const float left = view.play.x;
  const float right = view.play.x + view.play.width;
  const float top = view.play.y;
  const float bottom = view.play.y + view.play.height;
  const float midX = view.play.x + view.play.width * 0.5f;
  DrawHorizontalCushion(left + cornerGap, midX - sideGap, top, cushionD,
                        true);
  DrawHorizontalCushion(midX + sideGap, right - cornerGap, top, cushionD,
                        true);
  DrawHorizontalCushion(left + cornerGap, midX - sideGap, bottom, cushionD,
                        false);
  DrawHorizontalCushion(midX + sideGap, right - cornerGap, bottom, cushionD,
                        false);
  DrawVerticalCushion(left, top + cornerGap, bottom - cornerGap, cushionD,
                      true);
  DrawVerticalCushion(right, top + cornerGap, bottom - cornerGap, cushionD,
                      false);
  DrawRectangleLinesEx(view.play, 1.0f, {9, 32, 26, 130});
  const float spotR = 3.0f;
  DrawCircleV(WorldToScreen(view, {hb::kHeadSpotX, 0.0}), spotR,
              {210, 220, 205, 85});
  DrawCircleV(WorldToScreen(view, {hb::kFootSpotX, 0.0}), spotR,
              {210, 220, 205, 85});
}
void DrawDetailedCue(Vector2 cueBall, Vector2 aimDir, float ballR,
                     float power, const View &view);
void DrawTaperedRect(Vector2 a, Vector2 b, float wA, float wB, Color color);
std::string LocalPlayerLabel(const Game &game, int player);
std::string LocalStatusMessage(const Game &game);
void ApplySyncedBalls(Game &game);
void DrawCueShadow(const Game &game, const View &view) {
  if (game.phase != Phase::Aiming || game.world.CueBall().pocketed ||
      game.world.CueBall().sinking) return;
  const Vector2 cue = WorldToScreen(view, game.world.CueBall().pos);
  const Vector2 dir{static_cast<float>(game.aim.x), static_cast<float>(game.aim.y)};
  const Vector2 forward = Vector2Normalize(dir);
  if (forward.x == 0.0f && forward.y == 0.0f) return;
  const float s = static_cast<float>(view.scale);
  const float us = view.uiScale;
  const float cueLen = std::min(s * 1.30f, 1700.0f);
  const float pullback = (10.0f + static_cast<float>(game.power) * 118.0f) * us;
  const float ballR = static_cast<float>(hb::kBallRadius * view.scale);
  const float gap = ballR + 5.0f * us + pullback;
  const float tipX = cue.x - forward.x * gap;
  const float tipY = cue.y - forward.y * gap;
  const float buttX = tipX - forward.x * cueLen;
  const float buttY = tipY - forward.y * cueLen;
  const float tipW = std::max(0.005f * s, 2.5f * us);
  const float buttW = std::max(0.016f * s, 7.5f * us);
  const float jt = 0.35f;
  auto P = [&](float t) -> Vector2 {
    return Vector2{buttX + (tipX - buttX) * t, buttY + (tipY - buttY) * t};
  };
  auto W = [&](float t) -> float {
    if (t <= jt) return buttW;
    if (t <= 0.96f) return buttW + (tipW - buttW) * ((t - jt) / (0.96f - jt));
    return tipW;
  };
  const float sx = 4.5f * us;
  const float sy = 5.0f * us;
  // Shadow butt: cylinder section
  DrawLineEx({P(0.0f).x + sx, P(0.0f).y + sy},
             {P(jt).x + sx, P(jt).y + sy},
             buttW * 1.05f, {0, 0, 0, 72});
  // Shadow shaft: from joint to tip
  const Vector2 sTip = {P(1.0f).x + sx, P(1.0f).y + sy};
  DrawTaperedRect({P(jt).x + sx, P(jt).y + sy}, sTip,
                  W(jt) * 1.05f, tipW * 0.8f, {0, 0, 0, 72});
}
void DrawCueAndAim(const Game &game, const View &view) {
  if (game.phase != Phase::Aiming || game.world.CueBall().pocketed ||
      game.world.CueBall().sinking) {
    return;
  }
  const Vector2 cue = WorldToScreen(view, game.world.CueBall().pos);
  const Vec2 aim = hb::Normalize(game.aim);
  Vec2 ghostPos{};
  const bool hasGhost = FirstAimTarget(game.world, game.world.CueBall().pos, aim, &ghostPos) > 0;
  const Vec2 aimStop =
      hasGhost ? ghostPos : RayToTableEdge(game.world.CueBall().pos, aim);
  const Vector2 aimEnd = WorldToScreen(view, aimStop);
  const Vector2 dir{static_cast<float>(aim.x), static_cast<float>(aim.y)};
  const float ballR = static_cast<float>(hb::kBallRadius * view.scale);
  DrawDetailedCue(cue, dir, ballR, static_cast<float>(game.power), view);
  DrawLineEx(cue, aimEnd, 1.0f,
             {214, 225, 210, 70});
  if (hasGhost) {
    const Vector2 ghost = WorldToScreen(view, ghostPos);
    const float ghostR = static_cast<float>(hb::kBallRadius * view.scale);
    DrawCircleLines(static_cast<int>(ghost.x), static_cast<int>(ghost.y), ghostR,
                    {236, 241, 228, 205});
  }
}
Rectangle Button(Font font, Rectangle r, const char *text, bool active) {
  DrawRoundedFillWithBorder(
      r, 0.12f, 8, active ? Color{57, 79, 72, 255} : Color{27, 30, 31, 255},
      active ? Color{88, 111, 101, 230} : Color{58, 66, 63, 210});
  const float fs = std::max(15.0f, std::min(22.0f, r.height * 0.72f));
  const Vector2 sz = MeasureTextEx(font, text, fs, 0.0f);
  DrawTextF(font, text, {r.x + (r.width - sz.x) * 0.5f, r.y + (r.height - sz.y) * 0.5f}, fs,
            {222, 224, 215, 255});
  return r;
}
void DrawTopStatus(Game &game) {
  const float s = static_cast<float>(
      hb::Clamp(std::min(GetScreenWidth() / 1360.0, GetScreenHeight() / 820.0),
                0.72, 1.55));
  const auto &rs = game.rules.State();
  char line[192]{};
  if (game.appMode == AppMode::PlayingOnline && game.onlineTurn >= 0) {
    std::snprintf(line, sizeof(line), "目前%s击球    目标%s",
                  LocalPlayerLabel(game, rs.currentPlayer).c_str(),
                  hb::GroupName(rs.players[rs.currentPlayer].group));
  } else {
    std::snprintf(line, sizeof(line), "目前玩家%d击球    目标%s",
                  rs.currentPlayer + 1,
                  hb::GroupName(rs.players[rs.currentPlayer].group));
  }
  const float fs = 26.0f * s;
  const Vector2 sz = MeasureTextEx(game.font, line, fs, 0.0f);
  const Vector2 pos{(GetScreenWidth() - sz.x) * 0.5f, 47.0f * s};
  DrawTextF(game.font, line, pos, fs, {236, 240, 230, 255});
  DrawLineEx({pos.x, pos.y + sz.y + 3.0f * s},
             {pos.x + sz.x, pos.y + sz.y + 3.0f * s},
             1.0f, {93, 112, 101, 130});
}
float UiScale() {
  return static_cast<float>(
      hb::Clamp(std::min(GetScreenWidth() / 1360.0, GetScreenHeight() / 820.0),
                0.72, 1.55));
}
float OnlineUiScale() {
  return static_cast<float>(
      hb::Clamp(std::min(GetScreenWidth() / 960.0, GetScreenHeight() / 600.0),
                1.0, 1.65));
}
float OnlineChatWidth(float s) {
  return 220.0f * s;
}
std::string ReplaceAll(std::string text, const std::string &from,
                       const std::string &to) {
  if (from.empty()) return text;
  size_t pos = 0;
  while ((pos = text.find(from, pos)) != std::string::npos) {
    text.replace(pos, from.size(), to);
    pos += to.size();
  }
  return text;
}
std::string LocalPlayerLabel(const Game &game, int player) {
  if (game.appMode == AppMode::PlayingOnline && game.onlineTurn >= 0) {
    return player == game.onlineTurn ? "你" : "对手";
  }
  return std::to_string(player + 1) + "号玩家";
}
std::string LocalStatusMessage(const Game &game) {
  std::string msg = game.rules.State().message;
  if (game.appMode == AppMode::PlayingOnline && game.onlineTurn >= 0) {
    msg = ReplaceAll(msg, "1号玩家", LocalPlayerLabel(game, 0));
    msg = ReplaceAll(msg, "2号玩家", LocalPlayerLabel(game, 1));
  }
  return msg;
}
void ApplySyncedBalls(Game &game) {
  for (int i = 0; i < 16; ++i) {
    Ball &dst = game.world.Balls()[i];
    const Ball &src = game.syncedBalls[i];
    dst.pos = src.pos;
    dst.pocketed = src.pocketed;
    dst.sinking = src.sinking;
    dst.pocketFade = src.pocketed ? 1.0 : src.pocketFade;
    dst.vel = {};
    if (dst.pocketed || dst.sinking) {
      dst.rollOmega = {};
      dst.sideOmega = 0.0;
    }
  }
}
Rectangle ControlPanelRect(float s) {
  return {static_cast<float>(GetScreenWidth()) - 258.0f * s, 46.0f * s,
          224.0f * s, 390.0f * s};
}
Vector2 TipControlCenter(Rectangle panel, float s) {
  return {panel.x + 112.0f * s, panel.y + 146.0f * s};
}
constexpr float kTitleBarHeight = 40.0f;
constexpr float kTitleWindowButtonWidth = 46.0f;
Rectangle HelpButtonRect() {
  return {204.0f, 0.0f, 78.0f, kTitleBarHeight};
}
Rectangle HelpWindowRect(float s) {
  return {108.0f * s, 54.0f * s, 520.0f * s, 314.0f * s};
}
constexpr int kMaxRoomNameChars = 24;
constexpr int kMaxRoomPasswordChars = 20;
struct CreateRoomDialogLayout {
  Rectangle panel{};
  Rectangle nameBox{};
  Rectangle pwdBox{};
  Rectangle showPwd{};
  Rectangle p1{};
  Rectangle p2{};
  Rectangle ok{};
  Rectangle cancel{};
  float scale = 1.0f;
};
CreateRoomDialogLayout CreateRoomLayout() {
  const float sw = static_cast<float>(GetScreenWidth());
  const float sh = static_cast<float>(GetScreenHeight());
  const float s = OnlineUiScale();
  const float cw = hb::Clamp(sw * 0.52f, 520.0f * s, 760.0f * s);
  const float ch = hb::Clamp(sh * 0.58f, 390.0f * s, 500.0f * s);
  const float cx = sw * 0.5f - cw * 0.5f;
  const float cy = sh * 0.5f - ch * 0.5f;
  const float pad = 24.0f * s;
  const float boxH = 34.0f * s;
  const float showW = 104.0f * s;
  CreateRoomDialogLayout l{};
  l.panel = {cx, cy, cw, ch};
  l.nameBox = {cx + pad, cy + 82.0f * s, cw - pad * 2.0f, boxH};
  l.pwdBox = {cx + pad, cy + 164.0f * s, cw - pad * 2.0f - showW - 10.0f * s, boxH};
  l.showPwd = {l.pwdBox.x + l.pwdBox.width + 10.0f * s, l.pwdBox.y, showW, boxH};
  l.p1 = {cx + pad, cy + 242.0f * s, 96.0f * s, 32.0f * s};
  l.p2 = {l.p1.x + l.p1.width + 14.0f * s, l.p1.y, 112.0f * s, 32.0f * s};
  l.ok = {cx + cw * 0.5f - 92.0f * s, cy + ch - 58.0f * s, 82.0f * s, 34.0f * s};
  l.cancel = {cx + cw * 0.5f + 14.0f * s, l.ok.y, 82.0f * s, 34.0f * s};
  l.scale = s;
  return l;
}
void ResetCreateRoomDialog(Game &game) {
  game.host.Stop();
  game.showCreateDlg = true;
  game.showJoinPwdDlg = false;
  game.createDlgFocus = 0;
  game.createRoomName = "我的房间";
  game.createNameCursor = static_cast<int>(game.createRoomName.size());
  game.createRoomPwd.clear();
  game.createPwdCursor = 0;
  game.createRoomNotice.clear();
  game.showPwd = false;
  game.createHostIsP1 = true;
  game.textAllSelected = false;
}
void CloseCreateRoomDialog(Game &game) {
  game.showCreateDlg = false;
  game.createRoomNotice.clear();
  game.createRoomPwd.clear();
  game.showPwd = false;
  game.textAllSelected = false;
  game.bsRepeat.wasDown = false;
  game.leftRepeat.wasDown = false;
  game.rightRepeat.wasDown = false;
  game.delRepeat.wasDown = false;
}
void OpenJoinPasswordDialog(Game &game, const hb::RoomInfo &room) {
  game.showCreateDlg = false;
  game.showJoinPwdDlg = true;
  game.pendingJoinRoomName = room.name;
  game.pendingJoinHostIP = room.hostIP;
  game.joinPwdInput.clear();
  game.joinPwdCursor = 0;
  game.textAllSelected = false;
  game.bsRepeat.wasDown = false;
  game.leftRepeat.wasDown = false;
  game.rightRepeat.wasDown = false;
  game.delRepeat.wasDown = false;
}
void CloseJoinPasswordDialog(Game &game) {
  game.showJoinPwdDlg = false;
  game.pendingJoinRoomName.clear();
  game.pendingJoinHostIP.clear();
  game.joinPwdInput.clear();
  game.joinPwdCursor = 0;
  game.bsRepeat.wasDown = false;
}
Rectangle LobbyCreateButtonRect() {
  const float s = OnlineUiScale();
  const float w = 154.0f * s;
  const float h = 40.0f * s;
  return {static_cast<float>(GetScreenWidth()) - 30.0f * s - w,
          static_cast<float>(GetScreenHeight()) - 28.0f * s - h, w, h};
}
Rectangle LobbyRefreshButtonRect() {
  const float s = OnlineUiScale();
  const Rectangle create = LobbyCreateButtonRect();
  return {create.x - 18.0f * s - 132.0f * s, create.y, 132.0f * s, create.height};
}
void DrawTaperedRect(Vector2 a, Vector2 b, float wA, float wB, Color color) {
  const float dx = b.x - a.x, dy = b.y - a.y;
  const float len = sqrtf(dx * dx + dy * dy);
  if (len < 0.5f) return;
  const float angle = atan2f(dy, dx) * RAD2DEG;
  const int segs = std::max(1, static_cast<int>(len / 14.0f));
  for (int i = 0; i < segs; i++) {
    const float t0 = static_cast<float>(i) / segs;
    const float t1 = static_cast<float>(i + 1) / segs;
    const float w = (wA + (wB - wA) * (t0 + t1) * 0.5f);
    const Vector2 p0 = {a.x + dx * t0, a.y + dy * t0};
    const Vector2 p1 = {a.x + dx * t1, a.y + dy * t1};
    const float sl = sqrtf((p1.x - p0.x) * (p1.x - p0.x) + (p1.y - p0.y) * (p1.y - p0.y));
    const Vector2 center = {(p0.x + p1.x) * 0.5f, (p0.y + p1.y) * 0.5f};
    DrawRectanglePro({center.x, center.y, sl + 0.5f, w},
                     {sl * 0.5f + 0.25f, w * 0.5f}, angle, color);
  }
}
void DrawDetailedCue(Vector2 cueBall, Vector2 aimDir, float ballR,
                     float power, const View &view) {
  const Vector2 forward = Vector2Normalize(aimDir);
  if (forward.x == 0.0f && forward.y == 0.0f) return;
  BeginBlendMode(BLEND_ALPHA);
  EndBlendMode();
  const float s = static_cast<float>(view.scale);
  const float us = view.uiScale;
  const float cueLen = std::min(s * 1.30f, 1700.0f);
  const float pullback = (10.0f + power * 118.0f) * us;
  const float gap = ballR + 5.0f * us + pullback;
  const float tipX = cueBall.x - forward.x * gap;
  const float tipY = cueBall.y - forward.y * gap;
  const float buttX = tipX - forward.x * cueLen;
  const float buttY = tipY - forward.y * cueLen;
  const float tipW = std::max(0.005f * s, 2.5f * us);
  const float buttW = std::max(0.016f * s, 7.5f * us);
  const float jt = 0.35f;
  const auto P = [&](float t) -> Vector2 {
    return {buttX + (tipX - buttX) * t, buttY + (tipY - buttY) * t};
  };
  const auto W = [&](float t) -> float {
    if (t <= jt) return buttW;
    if (t <= 0.96f) return buttW + (tipW - buttW) * ((t - jt) / (0.96f - jt));
    return tipW;
  };
  DrawLineEx(P(0.0f), P(jt), buttW, {92, 48, 18, 255});
  DrawTaperedRect(P(jt), P(0.96f), W(jt), W(0.96f), {195, 165, 110, 255});
  DrawLineEx(P(0.0f), P(0.025f), W(0.0f) + 1.0f * us, {16, 14, 12, 255});
  DrawLineEx(P(0.023f), P(0.036f), W(0.023f) + 1.5f * us, {180, 148, 95, 255});
  DrawTaperedRect(P(jt), P(jt + 0.015f), W(jt) + 1.5f * us, W(jt + 0.015f) + 1.5f * us, {156, 163, 171, 255});
  DrawLineEx(P(jt - 0.008f), P(jt), W(jt - 0.008f) + 1.0f * us, {195, 190, 180, 255});
  auto drawRing = [&](float t1, float t2, float extra, Color color) {
    DrawTaperedRect(P(t1), P(t2), W(t1) + extra * us, W(t2) + extra * us, color);
  };
  drawRing(0.07f, 0.082f, 1.5f, {140, 145, 152, 255});
  drawRing(0.086f, 0.094f, 0.6f, {200, 195, 185, 255});
  drawRing(0.11f, 0.122f, 1.5f, {140, 145, 152, 255});
  drawRing(0.126f, 0.134f, 0.6f, {200, 195, 185, 255});
  drawRing(0.14f, 0.33f, 0.0f, {18, 20, 22, 255});
  drawRing(0.14f, 0.148f, 1.2f, {160, 152, 140, 255});
  drawRing(0.322f, 0.33f, 1.2f, {160, 152, 140, 255});
  DrawTaperedRect(P(0.96f), P(0.984f), W(0.96f) + 0.3f * us, W(0.984f), {230, 228, 222, 255});
  DrawTaperedRect(P(0.984f), P(1.0f), W(0.984f) * 0.85f, tipW * 0.5f, {28, 52, 82, 255});
  const Vector2 perp{-forward.y, forward.x};
  DrawLineEx({P(0.01f).x + perp.x * W(0.01f) * 0.36f, P(0.01f).y + perp.y * W(0.01f) * 0.36f},
             {P(0.96f).x + perp.x * tipW * 0.28f, P(0.96f).y + perp.y * tipW * 0.28f},
             1.0f * us, {255, 255, 255, 85});
  DrawLineEx({P(0.01f).x - perp.x * W(0.01f) * 0.40f, P(0.01f).y - perp.y * W(0.01f) * 0.40f},
             {P(0.96f).x - perp.x * tipW * 0.36f, P(0.96f).y - perp.y * tipW * 0.36f},
             0.9f * us, {0, 0, 0, 32});
  for (int g = -1; g <= 1; g += 2) {
    const float midW = (tipW + buttW) * 0.5f;
    const float gOff = g * midW * 0.16f;
    DrawLineEx({P(0.45f).x + perp.x * gOff, P(0.45f).y + perp.y * gOff},
               {P(0.92f).x + perp.x * gOff * 0.4f, P(0.92f).y + perp.y * gOff * 0.4f},
               0.8f * us, {130, 100, 45, 45});
  }
}
void DrawUI(Game &game) {
  const float s = UiScale();
  Rectangle panel = ControlPanelRect(s);
  DrawRoundedFillWithBorder(panel, 0.045f, 12, {12, 13, 14, 232},
                            {50, 58, 55, 190});
  DrawTextF(game.font, "击球设置", {panel.x + 18 * s, panel.y + 17 * s}, 27 * s,
            {236, 238, 228, 255});
  DrawTextF(game.font, "击点", {panel.x + 18 * s, panel.y + 61 * s}, 22 * s,
            {205, 212, 204, 255});
  const Vector2 center = TipControlCenter(panel, s);
  const float padR = 50.0f * s;
  DrawCircleV(center, padR, {25, 29, 29, 255});
  DrawCircleLines(static_cast<int>(center.x), static_cast<int>(center.y), padR, {113, 128, 121, 180});
  DrawLineEx({center.x - padR, center.y}, {center.x + padR, center.y}, 1.0f, {110, 119, 115, 125});
  DrawLineEx({center.x, center.y - padR}, {center.x, center.y + padR}, 1.0f, {110, 119, 115, 125});
  const Vector2 mouse = GetMousePosition();
  if (Vector2Distance(mouse, center) <= padR) {
    DrawCircleV(mouse, 8.0f * s, {228, 232, 218, 62});
    DrawCircleLines(static_cast<int>(mouse.x), static_cast<int>(mouse.y),
                    8.0f * s, {228, 232, 218, 145});
  }
  DrawCircleV({center.x + static_cast<float>(game.tipX * padR),
               center.y - static_cast<float>(game.tipY * padR)},
              7.0f, {228, 232, 218, 255});
  DrawTextF(game.font, "左", {center.x - padR - 27 * s, center.y - 10 * s}, 18 * s,
            {177, 186, 178, 255});
  DrawTextF(game.font, "右", {center.x + padR + 10 * s, center.y - 10 * s}, 18 * s,
            {177, 186, 178, 255});
  DrawTextF(game.font, "上", {center.x - 9 * s, center.y - padR - 29 * s}, 18 * s,
            {177, 186, 178, 255});
  DrawTextF(game.font, "下", {center.x - 9 * s, center.y + padR + 13 * s}, 18 * s,
            {177, 186, 178, 255});
  Button(game.font, {panel.x + 18 * s, panel.y + 238 * s, 188 * s, 30 * s},
         "还原", true);
  char powerText[64]{};
  std::snprintf(powerText, sizeof(powerText), "力度 %3d%%", static_cast<int>(game.power * 100.0));
  DrawTextF(game.font, powerText, {panel.x + 18 * s, panel.y + 292 * s}, 22 * s,
            {226, 231, 221, 255});
  DrawRectangleRounded({panel.x + 18 * s, panel.y + 322 * s, 188 * s, 12 * s}, 0.4f, 8, {28, 31, 31, 255});
  DrawRectangleRounded({panel.x + 18 * s, panel.y + 322 * s, static_cast<float>(188.0 * game.power * s), 12 * s},
                       0.4f, 8, {143, 170, 151, 255});
  Button(game.font, {panel.x + 18 * s, panel.y + 354 * s, 188 * s, 30 * s},
         "重新摆球", true);
  float msgX = 48.0f * s;
  float msgW = static_cast<float>(GetScreenWidth()) - 96.0f * s;
  if (game.appMode == AppMode::PlayingOnline) {
    const float os = OnlineUiScale();
    const float chatRight = 12.0f * os + OnlineChatWidth(os);
    msgX = chatRight + 20.0f * os;
    msgW = static_cast<float>(GetScreenWidth()) - msgX - 48.0f * s;
  }
  Rectangle msg{msgX, static_cast<float>(GetScreenHeight()) - 58.0f * s,
                msgW, 34.0f * s};
  DrawRectangleRounded(msg, 0.08f, 8, {9, 10, 10, 210});
  const std::string msgText = LocalStatusMessage(game);
  DrawTextF(game.font, msgText.c_str(), {msg.x + 14 * s, msg.y + 6 * s}, 22 * s, {221, 224, 216, 255});
  if (game.phase == Phase::GroupChoice) {
    DrawBackdrop();
    Rectangle modal{static_cast<float>(GetScreenWidth()) * 0.5f - 170.0f,
                    static_cast<float>(GetScreenHeight()) * 0.5f - 62.0f, 340.0f, 124.0f};
    DrawRoundedFillWithBorder(modal, 0.04f, 12, {13, 15, 15, 245},
                              {72, 84, 78, 220});
    DrawTextF(game.font, "选择球组", {modal.x + 108, modal.y + 17}, 26, {236, 238, 228, 255});
    Button(game.font, {modal.x + 34, modal.y + 69, 126, 34}, "全色球", true);
    Button(game.font, {modal.x + 180, modal.y + 69, 126, 34}, "半色球", true);
  }
}
Rectangle TitleButton(Rectangle r, const char *label, Font font, bool hot) {
  DrawRectangleRec(r, hot ? Color{30, 33, 33, 255} : Color{0, 0, 0, 0});
  const float fs = 22.0f;
  const Vector2 sz = MeasureTextEx(font, label, fs, 0.0f);
  DrawTextF(font, label, {r.x + (r.width - sz.x) * 0.5f,
                          r.y + (r.height - sz.y) * 0.5f - 1.0f},
            fs,
            {198, 204, 197, 235});
  return r;
}
void DrawTitleTextItem(Font font, Rectangle r, const char *label, float fs,
                       Color color, bool hot) {
  DrawRectangleRec(r, hot ? Color{20, 22, 22, 255} : Color{0, 0, 0, 0});
  const Vector2 sz = MeasureTextEx(font, label, fs, 0.0f);
  DrawTextF(font, label, {r.x + (r.width - sz.x) * 0.5f,
                          r.y + (r.height - sz.y) * 0.5f - 1.0f},
            fs, color);
}
enum class WindowButtonKind {
  Minimize,
  Maximize,
  Restore,
  Close
};
void DrawWindowButton(Rectangle r, WindowButtonKind kind, bool hot) {
  DrawRectangleRec(r, hot ? Color{30, 33, 33, 255} : Color{0, 0, 0, 0});
  const Color c = kind == WindowButtonKind::Close && hot
                      ? Color{236, 210, 205, 255}
                      : Color{198, 204, 197, 235};
  const Vector2 center{r.x + r.width * 0.5f, r.y + r.height * 0.5f};
  const float s = 9.2f;
  if (kind == WindowButtonKind::Minimize) {
    DrawLineEx({center.x - s * 0.75f, center.y + 4.0f},
               {center.x + s * 0.75f, center.y + 4.0f}, 1.5f, c);
  } else if (kind == WindowButtonKind::Maximize) {
    DrawRectangleLinesEx({center.x - s * 0.65f, center.y - s * 0.65f,
                          s * 1.3f, s * 1.3f},
                         1.5f, c);
  } else if (kind == WindowButtonKind::Restore) {
    DrawRectangleLinesEx({center.x - s * 0.35f, center.y - s * 0.75f,
                          s * 1.05f, s * 1.05f},
                         1.3f, c);
    DrawRectangleLinesEx({center.x - s * 0.70f, center.y - s * 0.35f,
                          s * 1.05f, s * 1.05f},
                         1.3f, c);
  } else {
    DrawLineEx({center.x - s * 0.65f, center.y - s * 0.65f},
               {center.x + s * 0.65f, center.y + s * 0.65f}, 1.5f, c);
    DrawLineEx({center.x + s * 0.65f, center.y - s * 0.65f},
               {center.x - s * 0.65f, center.y + s * 0.65f}, 1.5f, c);
  }
}
  // Lobby / Room / Chat UI
void DrawTitleBar(Game &game);
void DrawDetailedCue(Vector2 cueBall, Vector2 aimDir, float ballR,
                     float power, const View &view);
void DrawTextInputWithCursor(Font font, const std::string &text, int cursor,
                              Vector2 pos, float fontSize, Color color,
                              bool showCursor, bool allSelected = false) {
  std::string before = text.substr(0, static_cast<size_t>(cursor));
  float cursorX = MeasureTextEx(font, before.c_str(), fontSize, 0.0f).x;
  if (allSelected && !text.empty()) {
    const Vector2 textSize = MeasureTextEx(font, text.c_str(), fontSize, 0.0f);
    DrawRectangle(static_cast<int>(std::round(pos.x - 1.0f)),
                  static_cast<int>(std::round(pos.y - 1.0f)),
                  static_cast<int>(std::ceil(textSize.x + 3.0f)),
                  static_cast<int>(std::ceil(fontSize * 1.12f)),
                  {77, 99, 88, 210});
    DrawTextF(font, text.c_str(), pos, fontSize, {248, 250, 242, 255});
    return;
  }
  DrawTextF(font, text.c_str(), pos, fontSize, color);
  if (showCursor) {
    float barX = std::round(pos.x + cursorX);
    float barH = fontSize * 1.1f;
    float barY = std::round(pos.y + (fontSize - barH) * 0.5f);
    DrawRectangle(static_cast<int>(barX), static_cast<int>(barY), 2,
                  static_cast<int>(barH), color);
  }
}
void DrawChatWindow(Game &game) {
  const float sh = static_cast<float>(GetScreenHeight());
  const float us = OnlineUiScale();
  const float cw = OnlineChatWidth(us);
  const float ch = 150.0f * us;
  const float cx = 12.0f * us;
  const float cy = sh - ch - 12.0f * us;
  DrawRectangleRounded({cx - 2, cy - 2, cw + 4, ch + 4}, 0.04f, 8, {8, 9, 10, 200});
  DrawRectangleRounded({cx, cy, cw, ch}, 0.04f, 8, {15, 17, 18, 220});
  int visible = game.chatInputActive ? 5 : 7;
  int start = std::max(0, static_cast<int>(game.chatHistory.size()) - visible);
  float y = cy + 8.0f * us;
  for (int i = start; i < static_cast<int>(game.chatHistory.size()); ++i) {
    DrawTextF(game.font, game.chatHistory[i].c_str(), {cx + 8.0f * us, y},
              14.0f * us, {210, 215, 205, 255});
    y += 18.0f * us;
  }
  if (game.chatInputActive) {
    DrawRectangleRounded({cx + 4, cy + ch - 28.0f * us, cw - 8, 24.0f * us},
                         0.12f, 6, {25, 27, 28, 230});
    bool showCursor = (static_cast<int>(GetTime() * 2.0) % 2) != 0;
    DrawTextInputWithCursor(game.font, game.chatInputBuf, game.chatInputCursor,
                            {cx + 10.0f * us, cy + ch - 26.0f * us},
                            15.0f * us, {238, 240, 232, 255}, showCursor,
                            game.textAllSelected);
  } else {
    DrawTextF(game.font, "Enter 输入消息", {cx + 8.0f * us, cy + ch - 22.0f * us},
              13.0f * us, {120, 128, 122, 200});
  }
}
void DrawCreateRoomDialog(Game &game) {
  {
    const CreateRoomDialogLayout l = CreateRoomLayout();
    const float us = l.scale;
    const Vector2 mouse = GetMousePosition();
    const float pad = 24.0f * us;
    DrawRectangleRounded(l.panel, 0.035f, 12, {12, 14, 14, 245});
    DrawRectangleRoundedLines(l.panel, 0.035f, 12, {70, 82, 76, 210});
    DrawTextF(game.font, "创建房间", {l.panel.x + pad, l.panel.y + 20.0f * us},
              29.0f * us, {236, 238, 228, 255});
    DrawTextF(game.font, "房间名", {l.panel.x + pad, l.panel.y + 61.0f * us},
              20.0f * us, {190, 200, 188, 255});

    Color nameBoxColor = game.createDlgFocus == 0 ? Color{35, 39, 36, 240}
                                                  : Color{25, 27, 28, 220};
    DrawRectangleRounded(l.nameBox, 0.06f, 6, nameBoxColor);
    if (game.createDlgFocus == 0) {
      DrawRectangleRoundedLines(l.nameBox, 0.06f, 6, {110, 140, 125, 180});
    }
    const bool nameShowCursor =
        game.createDlgFocus == 0 && (static_cast<int>(GetTime() * 2.5) % 2) == 0;
    DrawTextInputWithCursor(game.font, game.createRoomName, game.createNameCursor,
                            {l.nameBox.x + 10.0f * us, l.nameBox.y + 5.0f * us},
                            21.0f * us, {236, 240, 230, 255}, nameShowCursor,
                            game.createDlgFocus == 0 && game.textAllSelected);
    const int roomNameLen = Utf8CodepointCount(game.createRoomName);
    char countText[64]{};
    std::snprintf(countText, sizeof(countText), "%d/%d", roomNameLen, kMaxRoomNameChars);
    const Vector2 countSize = MeasureTextEx(game.font, countText, 16.0f * us, 0.0f);
    DrawTextF(game.font, countText,
              {l.nameBox.x + l.nameBox.width - countSize.x - 6.0f * us,
               l.nameBox.y + l.nameBox.height + 5.0f * us},
              16.0f * us,
              roomNameLen >= kMaxRoomNameChars ? Color{221, 184, 116, 255}
                                               : Color{126, 139, 130, 230});
    if (!game.createRoomNotice.empty()) {
      DrawTextF(game.font, game.createRoomNotice.c_str(),
                {l.nameBox.x, l.nameBox.y + l.nameBox.height + 5.0f * us},
                16.0f * us, {221, 184, 116, 255});
    }

    DrawTextF(game.font, "密码（可选）", {l.panel.x + pad, l.panel.y + 143.0f * us},
              20.0f * us, {190, 200, 188, 255});
    Color pwdBoxColor = game.createDlgFocus == 1 ? Color{35, 39, 36, 240}
                                                 : Color{25, 27, 28, 220};
    DrawRectangleRounded(l.pwdBox, 0.06f, 6, pwdBoxColor);
    if (game.createDlgFocus == 1) {
      DrawRectangleRoundedLines(l.pwdBox, 0.06f, 6, {110, 140, 125, 180});
    }
    const std::string pwdDisplay =
        game.showPwd ? game.createRoomPwd : std::string(game.createRoomPwd.size(), '*');
    const bool pwdShowCursor =
        game.createDlgFocus == 1 && (static_cast<int>(GetTime() * 2.5) % 2) == 0;
    DrawTextInputWithCursor(game.font, pwdDisplay, game.createPwdCursor,
                            {l.pwdBox.x + 10.0f * us, l.pwdBox.y + 5.0f * us},
                            21.0f * us, {236, 240, 230, 255}, pwdShowCursor,
                            game.createDlgFocus == 1 && game.textAllSelected);
    const bool showPwdH = CheckCollisionPointRec(mouse, l.showPwd);
    DrawRectangleRounded(l.showPwd, 0.08f, 6,
                         showPwdH ? Color{45, 49, 48, 255} : Color{30, 33, 33, 255});
    const char *showPwdLabel = game.showPwd ? "隐藏密码" : "显示密码";
    const Vector2 showPwdSize = MeasureTextEx(game.font, showPwdLabel, 17.0f * us, 0.0f);
    DrawTextF(game.font, showPwdLabel,
              {l.showPwd.x + (l.showPwd.width - showPwdSize.x) * 0.5f,
               l.showPwd.y + (l.showPwd.height - showPwdSize.y) * 0.5f},
              17.0f * us, {214, 219, 211, 255});

    DrawTextF(game.font, "先手", {l.panel.x + pad, l.panel.y + 221.0f * us},
              20.0f * us, {190, 200, 188, 255});
    Color p1Col = game.createHostIsP1 ? Color{57, 79, 72, 255} : Color{25, 27, 28, 220};
    Color p2Col = game.createHostIsP1 ? Color{25, 27, 28, 220} : Color{57, 79, 72, 255};
    if (CheckCollisionPointRec(mouse, l.p1)) {
      p1Col.r = std::min(255, p1Col.r + 15);
      p1Col.g = std::min(255, p1Col.g + 15);
      p1Col.b = std::min(255, p1Col.b + 10);
    }
    if (CheckCollisionPointRec(mouse, l.p2)) {
      p2Col.r = std::min(255, p2Col.r + 15);
      p2Col.g = std::min(255, p2Col.g + 15);
      p2Col.b = std::min(255, p2Col.b + 10);
    }
    DrawRectangleRounded(l.p1, 0.08f, 6, p1Col);
    DrawTextF(game.font, "我", {l.p1.x + 36.0f * us, l.p1.y + 6.0f * us},
              18.0f * us, {236, 240, 230, 255});
    DrawRectangleRounded(l.p2, 0.08f, 6, p2Col);
    DrawTextF(game.font, "客人", {l.p2.x + 34.0f * us, l.p2.y + 6.0f * us},
              18.0f * us, {236, 240, 230, 255});

    const bool okH = CheckCollisionPointRec(mouse, l.ok);
    const bool cancelH = CheckCollisionPointRec(mouse, l.cancel);
    DrawRectangleRounded(l.ok, 0.08f, 6,
                         okH ? Color{77, 99, 88, 255} : Color{57, 79, 72, 255});
    DrawTextF(game.font, "确定", {l.ok.x + 22.0f * us, l.ok.y + 7.0f * us},
              18.0f * us, {236, 240, 230, 255});
    DrawRectangleRounded(l.cancel, 0.08f, 6,
                         cancelH ? Color{35, 39, 38, 240} : Color{25, 27, 28, 220});
    DrawTextF(game.font, "取消", {l.cancel.x + 22.0f * us, l.cancel.y + 7.0f * us},
              18.0f * us, {200, 205, 198, 255});
    return;
  }
  const float sw = static_cast<float>(GetScreenWidth());
  const float sh = static_cast<float>(GetScreenHeight());
  const float us = UiScale();
  const Vector2 mouse = GetMousePosition();
  float cw = 360.0f * us, ch = 280.0f * us, cx = sw * 0.5f - cw * 0.5f, cy = sh * 0.5f - ch * 0.5f;
  DrawRectangleRounded({cx, cy, cw, ch}, 0.05f, 12, {12, 14, 14, 245});
  DrawRectangleRoundedLines({cx, cy, cw, ch}, 0.05f, 12, {70, 82, 76, 210});
  DrawTextF(game.font, "创建房间", {cx + 16, cy + 14}, 24 * us, {236, 238, 228, 255});
  DrawTextF(game.font, "房间名", {cx + 16, cy + 58}, 18 * us, {180, 190, 178, 255});
  Rectangle nameBox{cx + 16, cy + 80, cw - 32, 28};
  Color nameBoxColor = game.createDlgFocus == 0 ? Color{35, 39, 36, 240} : Color{25, 27, 28, 220};
  DrawRectangleRounded(nameBox, 0.08f, 6, nameBoxColor);
  if (game.createDlgFocus == 0) {
    DrawRectangleRoundedLines(nameBox, 0.08f, 6, {110, 140, 125, 180});
  }
  bool nameShowCursor = game.createDlgFocus == 0 && (static_cast<int>(GetTime() * 2.5) % 2) == 0;
  DrawTextInputWithCursor(game.font, game.createRoomName, game.createNameCursor,
                          {cx + 22, cy + 84}, 18 * us, {236, 240, 230, 255}, nameShowCursor);
  DrawTextF(game.font, "密码 (可选)", {cx + 16, cy + 120}, 18 * us, {180, 190, 178, 255});
  float pwdBoxW = cw - 32 - 46 * us;
  Rectangle pwdBox{cx + 16, cy + 142, pwdBoxW, 28};
  Color pwdBoxColor = game.createDlgFocus == 1 ? Color{35, 39, 36, 240} : Color{25, 27, 28, 220};
  DrawRectangleRounded(pwdBox, 0.08f, 6, pwdBoxColor);
  if (game.createDlgFocus == 1) {
    DrawRectangleRoundedLines(pwdBox, 0.08f, 6, {110, 140, 125, 180});
  }
  std::string pwdDisplay = game.showPwd ? game.createRoomPwd : std::string(game.createRoomPwd.size(), '*');
  bool pwdShowCursor = game.createDlgFocus == 1 && (static_cast<int>(GetTime() * 2.5) % 2) == 0;
  DrawTextInputWithCursor(game.font, pwdDisplay, game.createPwdCursor,
                          {cx + 22, cy + 146}, 18 * us, {236, 240, 230, 255}, pwdShowCursor);
  Rectangle showPwdR{cx + 16 + pwdBoxW + 6, cy + 142, 40 * us, 28};
  bool showPwdH = CheckCollisionPointRec(mouse, showPwdR);
  DrawRectangleRounded(showPwdR, 0.12f, 6, showPwdH ? Color{45, 49, 48, 255} : Color{30, 33, 33, 255});
  DrawTextF(game.font, game.showPwd ? "隐" : "显", {showPwdR.x + 10 * us, showPwdR.y + 5}, 16 * us, {200, 205, 198, 255});
  DrawTextF(game.font, "先手", {cx + 16, cy + 182}, 18 * us, {180, 190, 178, 255});
  Rectangle p1R{cx + 16, cy + 204, 80, 24};
  Rectangle p2R{cx + 106, cy + 204, 80, 24};
  Color p1Col = game.createHostIsP1 ? Color{57, 79, 72, 255} : Color{25, 27, 28, 220};
  Color p2Col = game.createHostIsP1 ? Color{25, 27, 28, 220} : Color{57, 79, 72, 255};
  if (CheckCollisionPointRec(mouse, p1R)) { p1Col.r = std::min(255, p1Col.r + 15); p1Col.g = std::min(255, p1Col.g + 15); p1Col.b = std::min(255, p1Col.b + 10); }
  if (CheckCollisionPointRec(mouse, p2R)) { p2Col.r = std::min(255, p2Col.r + 15); p2Col.g = std::min(255, p2Col.g + 15); p2Col.b = std::min(255, p2Col.b + 10); }
  DrawRectangleRounded(p1R, 0.12f, 6, p1Col);
  DrawTextF(game.font, "我", {cx + 34, cy + 207}, 16 * us, {236, 240, 230, 255});
  DrawRectangleRounded(p2R, 0.12f, 6, p2Col);
  DrawTextF(game.font, "客人", {cx + 112, cy + 207}, 16 * us, {236, 240, 230, 255});
  Rectangle okR{cx + cw * 0.5f - 70, cy + ch - 44, 64, 28};
  Rectangle cancelR{cx + cw * 0.5f + 10, cy + ch - 44, 64, 28};
  bool okH = CheckCollisionPointRec(mouse, okR), cancelH = CheckCollisionPointRec(mouse, cancelR);
  DrawRectangleRounded(okR, 0.12f, 6, okH ? Color{77, 99, 88, 255} : Color{57, 79, 72, 255});
  DrawTextF(game.font, "确定", {okR.x + 10, okR.y + 5}, 16 * us, {236, 240, 230, 255});
  DrawRectangleRounded(cancelR, 0.12f, 6, cancelH ? Color{35, 39, 38, 240} : Color{25, 27, 28, 220});
  DrawTextF(game.font, "取消", {cancelR.x + 10, cancelR.y + 5}, 16 * us, {200, 205, 198, 255});
}
void DrawLobby(Game &game) {
  const float sw = static_cast<float>(GetScreenWidth());
  const float us = OnlineUiScale();
  const Vector2 mouse = GetMousePosition();
  ClearBackground(BLACK);
  DrawTitleBar(game);
  DrawTextF(game.font, "局域网联机", {40, 50}, 32 * us, {236, 238, 228, 255});
  DrawTextF(game.font, "LAN 房间列表", {40, 90}, 18 * us, {150, 160, 148, 220});
  float y = 130;
  game.roomList = game.client.GetRooms();
  for (size_t i = 0; i < game.roomList.size(); ++i) {
    auto &r = game.roomList[i];
    Rectangle row{40, y, sw - 80, 44};
    bool hover = CheckCollisionPointRec(mouse, row);
    DrawRectangleRounded(row, 0.05f, 8, hover ? Color{28, 32, 30, 230} : Color{18, 20, 20, 200});
    char info[256];
    std::snprintf(info, sizeof(info), "%s  %s  %d/2  %s",
                  r.name.c_str(), r.hasPassword ? "[密码]" : "",
                  r.playerCount, r.hostIP.c_str());
    DrawTextF(game.font, info, {row.x + 14, row.y + 12}, 16 * us, {210, 218, 206, 255});
    Rectangle joinBtn{row.x + row.width - 70, row.y + 8, 56, 28};
    bool jHover = CheckCollisionPointRec(mouse, joinBtn);
    DrawRectangleRounded(joinBtn, 0.12f, 6, jHover ? Color{77, 99, 88, 255} : Color{57, 79, 72, 255});
    DrawTextF(game.font, "加入", {joinBtn.x + 10, joinBtn.y + 5}, 15 * us, {236, 240, 230, 255});
    y += 52;
  }
  if (game.roomList.empty()) {
    DrawTextF(game.font, "(没有房间列表, 点击创建)", {50, y}, 16 * us, {120, 128, 122, 180});
  }
  Rectangle createBtn = LobbyCreateButtonRect();
  bool cHover = CheckCollisionPointRec(mouse, createBtn);
  DrawRectangleRounded(createBtn, 0.12f, 8, cHover ? Color{77, 99, 88, 255} : Color{57, 79, 72, 255});
  DrawTextF(game.font, "创建房间", {createBtn.x + 21.0f * us, createBtn.y + 8.0f * us}, 18 * us, {236, 240, 230, 255});
  Rectangle refreshBtn = LobbyRefreshButtonRect();
  bool rHover = CheckCollisionPointRec(mouse, refreshBtn);
  DrawRectangleRounded(refreshBtn, 0.12f, 8, rHover ? Color{35, 39, 38, 240} : Color{25, 27, 28, 220});
  DrawTextF(game.font, "刷新", {refreshBtn.x + 42.0f * us, refreshBtn.y + 8.0f * us}, 18 * us, {200, 205, 198, 255});
}
void DrawRoomWait(Game &game) {
  const float sw = static_cast<float>(GetScreenWidth());
  const float sh = static_cast<float>(GetScreenHeight());
  const float us = OnlineUiScale();
  const Vector2 mouse = GetMousePosition();
  ClearBackground(BLACK);
  DrawTitleBar(game);
  DrawTextF(game.font, game.roomName.c_str(), {40, 50}, 28 * us, {236, 238, 228, 255});
  std::string status = game.amReady ? "已就绪" : "未就绪";
  DrawTextF(game.font, ("我的状态: " + status).c_str(), {40, 100}, 18 * us, {180, 190, 180, 255});
  std::string opStatus;
  if (game.appMode == AppMode::RoomHost) {
    opStatus = game.host.HasClient() ? (game.opReady ? "对手已就绪" : "对手未就绪") : "等待对手加入...";
  } else {
    if (!game.client.IsAccepted() && game.client.IsConnecting())
      opStatus = "连接中...";
    else if (!game.client.IsAccepted())
      opStatus = "连接失败";
    else
      opStatus = game.opReady ? "对手已就绪" : "对手未就绪";
  }
  DrawTextF(game.font, opStatus.c_str(), {40, 130}, 18 * us, {180, 190, 180, 255});
  // Pulse animation for ready status
  float pulse = 0.5f + 0.5f * sinf(static_cast<float>(GetTime()) * 3.0f);
  if (!game.opReady && game.appMode == AppMode::RoomHost) {
    DrawTextF(game.font, "等待玩家...", {sw * 0.5f - 60, sh * 0.5f}, 16 * us,
              {static_cast<unsigned char>(160 + 60 * pulse),
               static_cast<unsigned char>(170 + 60 * pulse),
               static_cast<unsigned char>(160 + 60 * pulse), 255});
  }
  // Only show ready button when connected (client) or always (host)
  bool showReadyBtn = (game.appMode == AppMode::RoomHost) || game.client.IsAccepted();
  if (showReadyBtn) {
    Rectangle readyBtn{sw * 0.5f - 60, sh - 120, 120, 34};
    bool rHover = CheckCollisionPointRec(mouse, readyBtn);
    Color rCol = game.amReady ? Color{80, 100, 90, 255} : Color{57, 79, 72, 255};
    if (rHover) { rCol.r = static_cast<unsigned char>(std::min(255, rCol.r + 20)); rCol.g = static_cast<unsigned char>(std::min(255, rCol.g + 20)); rCol.b = static_cast<unsigned char>(std::min(255, rCol.b + 15)); }
    DrawRectangleRounded(readyBtn, 0.12f, 8, rCol);
    DrawTextF(game.font, game.amReady ? "取消就绪" : "就绪",
              {readyBtn.x + 22, readyBtn.y + 6}, 18 * us, {236, 240, 230, 255});
  }
  DrawChatWindow(game);
}
void DrawJoinPasswordDialog(Game &game) {
  {
    const float sw = static_cast<float>(GetScreenWidth());
    const float sh = static_cast<float>(GetScreenHeight());
    const float us = OnlineUiScale();
    const float cw = 360.0f * us;
    const float ch = 170.0f * us;
    const float cx = sw * 0.5f - cw * 0.5f;
    const float cy = sh * 0.5f - ch * 0.5f;
    DrawRectangleRounded({cx, cy, cw, ch}, 0.04f, 12, {12, 14, 14, 245});
    DrawRectangleRoundedLines({cx, cy, cw, ch}, 0.04f, 12, {70, 82, 76, 210});
    DrawTextF(game.font, "输入房间密码", {cx + 20.0f * us, cy + 18.0f * us},
              24.0f * us, {236, 238, 228, 255});
    std::string joinPwdMasked(game.joinPwdInput.size(), '*');
    bool joinPwdCursorVisible = (static_cast<int>(GetTime() * 2.5) % 2) == 0;
    DrawRectangleRounded({cx + 20.0f * us, cy + 62.0f * us, cw - 40.0f * us, 34.0f * us},
                         0.08f, 6, {25, 27, 28, 235});
    DrawTextInputWithCursor(game.font, joinPwdMasked, game.joinPwdCursor,
                            {cx + 30.0f * us, cy + 68.0f * us},
                            20.0f * us, {236, 240, 230, 255}, joinPwdCursorVisible,
                            game.textAllSelected);
    Rectangle okR{cx + cw * 0.5f - 82.0f * us, cy + ch - 48.0f * us,
                  74.0f * us, 32.0f * us};
    Rectangle cancelR{cx + cw * 0.5f + 10.0f * us, okR.y, 74.0f * us, 32.0f * us};
    DrawRectangleRounded(okR, 0.1f, 6, {57, 79, 72, 255});
    DrawTextF(game.font, "确定", {okR.x + 19.0f * us, okR.y + 6.0f * us},
              17.0f * us, {236, 240, 230, 255});
    DrawRectangleRounded(cancelR, 0.1f, 6, {25, 27, 28, 220});
    DrawTextF(game.font, "取消", {cancelR.x + 19.0f * us, cancelR.y + 6.0f * us},
              17.0f * us, {200, 205, 198, 255});
    return;
  }
  const float sw = static_cast<float>(GetScreenWidth());
  const float sh = static_cast<float>(GetScreenHeight());
  float cw = 280.0f, ch = 140.0f, cx = sw * 0.5f - cw * 0.5f, cy = sh * 0.5f - ch * 0.5f;
  DrawRectangleRounded({cx, cy, cw, ch}, 0.05f, 12, {12, 14, 14, 245});
  DrawTextF(game.font, "输入房间密码", {cx + 16, cy + 14}, 20, {236, 238, 228, 255});
  DrawRectangleRounded({cx + 16, cy + 44, cw - 32, 28}, 0.08f, 6, {25, 27, 28, 220});
  std::string joinPwdMasked(game.joinPwdInput.size(), '*');
  bool joinPwdCursorVisible = (static_cast<int>(GetTime() * 2.5) % 2) == 0;
  DrawTextInputWithCursor(game.font, joinPwdMasked, game.joinPwdCursor,
                          {cx + 22, cy + 48}, 18, {236, 240, 230, 255}, joinPwdCursorVisible);
  Rectangle okR{cx + cw * 0.5f - 70, cy + ch - 34, 64, 24};
  Rectangle cancelR{cx + cw * 0.5f + 6, cy + ch - 34, 64, 24};
  DrawRectangleRounded(okR, 0.12f, 6, {57, 79, 72, 255});
  DrawTextF(game.font, "确定", {okR.x + 14, okR.y + 3}, 15, {236, 240, 230, 255});
  DrawRectangleRounded(cancelR, 0.12f, 6, {25, 27, 28, 220});
  DrawTextF(game.font, "取消", {cancelR.x + 14, cancelR.y + 3}, 15, {200, 205, 198, 255});
}
void DrawTitleBar(Game &game) {
  const float w = static_cast<float>(GetScreenWidth());
  const Vector2 mouse = GetMousePosition();
  const Rectangle helpR = HelpButtonRect();
  const Rectangle netR{286.0f, 0.0f, 74.0f, kTitleBarHeight};
  const Rectangle titleR{12.0f, 0.0f, 92.0f, kTitleBarHeight};
  const Rectangle modeR{106.0f, 0.0f, 96.0f, kTitleBarHeight};
  const Rectangle closeR{w - kTitleWindowButtonWidth - 1.0f, 0.0f,
                         kTitleWindowButtonWidth, kTitleBarHeight};
  const Rectangle maxR{closeR.x - kTitleWindowButtonWidth - 1.0f, 0.0f,
                       kTitleWindowButtonWidth, kTitleBarHeight};
  const Rectangle minR{maxR.x - kTitleWindowButtonWidth - 1.0f, 0.0f,
                       kTitleWindowButtonWidth, kTitleBarHeight};
  Rectangle bar{0.0f, 0.0f, w, kTitleBarHeight};
  DrawRectangleRec(bar, BLACK);
  DrawRectangle(0, static_cast<int>(kTitleBarHeight - 1.0f), GetScreenWidth(), 1,
                {50, 57, 55, 120});
  DrawTitleTextItem(game.font, titleR, "中式台球", 21.0f,
                    {218, 224, 216, 255}, false);
  const char *modeLabel = "双人对战";
  if (game.appMode == AppMode::Lobby) modeLabel = "局域网联机";
  else if (game.appMode == AppMode::RoomHost || game.appMode == AppMode::RoomClient)
    modeLabel = "房间对战";
  else if (game.appMode == AppMode::PlayingOnline) modeLabel = "在线对战中";
  DrawTitleTextItem(game.font, modeR, modeLabel, 18.5f,
                    {142, 154, 147, 230}, false);
  DrawTitleTextItem(game.font, helpR, "帮助", 19.5f,
                    {198, 204, 197, 235},
                    game.helpOpen || PointInRect(mouse, helpR));
  const char *netLabel = (game.appMode == AppMode::Single) ? "单机" : "联机";
  DrawTitleTextItem(game.font, netR, netLabel, 18.5f,
                    {155, 190, 170, 235}, PointInRect(mouse, netR));
  DrawWindowButton(minR, WindowButtonKind::Minimize, PointInRect(mouse, minR));
  DrawWindowButton(maxR,
                   IsWindowMaximized() ? WindowButtonKind::Restore
                                       : WindowButtonKind::Maximize,
                   PointInRect(mouse, maxR));
  DrawWindowButton(closeR, WindowButtonKind::Close, PointInRect(mouse, closeR));
}
void DrawFpsCounter(Font font) {
  char text[32]{};
  std::snprintf(text, sizeof(text), "FPS %d", GetFPS());
  DrawTextF(font, text, {18.0f, 38.0f}, 18.0f, {180, 192, 184, 235});
}
float SmoothStep(float t) {
  t = hb::Clamp(t, 0.0f, 1.0f);
  return t * t * (3.0f - 2.0f * t);
}
bool SplashActive(const Game &game) {
  constexpr double kSplashTotalSeconds = 4.1;
  return !game.splashDone && GetTime() - game.splashStartTime < kSplashTotalSeconds;
}
bool SplashSkipPressed() {
  return IsKeyPressed(KEY_ESCAPE) || IsKeyPressed(KEY_ENTER) ||
         IsKeyPressed(KEY_SPACE);
}
void DrawCenteredSplashText(Font font, const char *text, float centerX, float y,
                            float fontSize, Color color) {
  const Vector2 sz = MeasureTextEx(font, text, fontSize, 0.0f);
  DrawTextF(font, text, {centerX - sz.x * 0.5f, y}, fontSize, color);
}
void EnsureSplashTexture(Game &game) {
  if (game.splashTexture.texture.id != 0) return;
  constexpr int kTextureW = 900;
  constexpr int kTextureH = 300;
  game.splashTexture = LoadRenderTexture(kTextureW, kTextureH);
  if (game.splashTexture.texture.id == 0) return;
  SetTextureFilter(game.splashTexture.texture, TEXTURE_FILTER_BILINEAR);
  BeginTextureMode(game.splashTexture);
  ClearBackground({0, 0, 0, 0});
  const float cx = kTextureW * 0.5f;
  float y = 42.0f;
  DrawCenteredSplashText(game.font, "《健康游戏忠告》", cx, y, 42.0f,
                         {236, 240, 230, 255});
  y += 82.0f;
  const char *lines[] = {
      "抵制不良游戏，拒绝盗版游戏。",
      "注意自我保护，谨防受骗上当。",
      "适度游戏益脑，沉迷游戏伤身。",
      "合理安排时间，享受健康生活。"};
  for (const char *line : lines) {
    DrawCenteredSplashText(game.font, line, cx, y, 28.0f,
                           {198, 207, 198, 255});
    y += 43.0f;
  }
  EndTextureMode();
}
void DrawHealthSplash(Game &game) {
  constexpr double kFadeInSeconds = 0.70;
  constexpr double kFadeStartSeconds = 2.45;
  constexpr double kSplashTotalSeconds = 4.1;
  EnsureSplashTexture(game);
  const double elapsed = GetTime() - game.splashStartTime;
  const float fadeInT = SmoothStep(static_cast<float>(elapsed / kFadeInSeconds));
  const float fadeT = SmoothStep(static_cast<float>(
      (elapsed - kFadeStartSeconds) / (kSplashTotalSeconds - kFadeStartSeconds)));
  const unsigned char alpha = static_cast<unsigned char>(hb::Clamp(
      static_cast<int>(std::round(255.0f * fadeInT * (1.0f - fadeT))), 0, 255));
  ClearBackground(BLACK);
  if (game.splashTexture.texture.id == 0) {
    return;
  }
  const float s = UiScale() * (1.0f + 0.12f * fadeT);
  const float dstW = game.splashTexture.texture.width * s;
  const float dstH = game.splashTexture.texture.height * s;
  const Rectangle src{0.0f, 0.0f,
                      static_cast<float>(game.splashTexture.texture.width),
                      -static_cast<float>(game.splashTexture.texture.height)};
  const Rectangle dst{GetScreenWidth() * 0.5f - dstW * 0.5f,
                      GetScreenHeight() * 0.5f - dstH * 0.5f,
                      dstW, dstH};
  DrawTexturePro(game.splashTexture.texture, src, dst, {0.0f, 0.0f}, 0.0f,
                 {255, 255, 255, alpha});
}
void DrawHelpWindow(Game &game) {
  if (!game.helpOpen) {
    return;
  }
  const float s = UiScale();
  const Rectangle panel = HelpWindowRect(s);
  DrawRoundedFillWithBorder(panel, 0.035f, 12, {11, 13, 13, 246},
                            {72, 84, 78, 230});
  DrawTextF(game.font, "帮助", {panel.x + 18.0f * s, panel.y + 14.0f * s},
            26.0f * s, {238, 241, 231, 255});
  const Rectangle closeR{panel.x + panel.width - 42.0f * s,
                         panel.y + 10.0f * s, 26.0f * s, 26.0f * s};
  TitleButton(closeR, "关", game.font, PointInRect(GetMousePosition(), closeR));
  const char *lines[] = {
      "本游戏为中式台球（黑八）规则。",
      "先合法击入己方全色球或半色球再击入8号球者获胜。",
      "白球必须先击中己方球；否则犯规由对手自动判断。",
      "瞄准移动鼠标控制方向，右键蓄力、左键击球或滚轮击球。",
      "右侧面板可调整击点高低杆、左右塞，还原按钮回到中杆。",
      "自由球时移动鼠标选择放置位置，然后确认击打。",
      "Esc 暂停菜单，C 还原，空格开始下一局。"};
  float y = panel.y + 56.0f * s;
  for (const char *line : lines) {
    DrawTextF(game.font, line, {panel.x + 20.0f * s, y}, 18.0f * s,
              {207, 216, 207, 245});
    y += 32.0f * s;
  }
}
bool HandleTitleBarInput(Game &game) {
  const Vector2 mouse = GetMousePosition();
  const bool windowActionActive = game.draggingTitle || game.resizeMode != ResizeNone;
  const bool leftDown = IsPrimaryMouseDown();
  const float w = static_cast<float>(GetScreenWidth());
  const float h = static_cast<float>(GetScreenHeight());
  constexpr float edge = 8.0f;
  constexpr int minW = 960;
  constexpr int minH = 600;
  int hit = ResizeNone;
  if (mouse.x <= edge) hit |= ResizeLeft;
  if (mouse.x >= w - edge) hit |= ResizeRight;
  if (mouse.y <= edge) hit |= ResizeTop;
  if (mouse.y >= h - edge) hit |= ResizeBottom;
  if (game.resizeMode == ResizeNone && !game.draggingTitle) {
    if ((hit & (ResizeLeft | ResizeRight)) && (hit & (ResizeTop | ResizeBottom))) {
      SetMouseCursor((hit == (ResizeLeft | ResizeTop)) || (hit == (ResizeRight | ResizeBottom))
                         ? MOUSE_CURSOR_RESIZE_NWSE
                         : MOUSE_CURSOR_RESIZE_NESW);
    } else if (hit & (ResizeLeft | ResizeRight)) {
      SetMouseCursor(MOUSE_CURSOR_RESIZE_EW);
    } else if (hit & (ResizeTop | ResizeBottom)) {
      SetMouseCursor(MOUSE_CURSOR_RESIZE_NS);
    } else {
      SetMouseCursor(MOUSE_CURSOR_DEFAULT);
    }
  }
  const Rectangle helpR = HelpButtonRect();
  const Rectangle netR{286.0f, 0.0f, 74.0f, kTitleBarHeight};
  const Rectangle closeR{w - kTitleWindowButtonWidth - 1.0f, 0.0f,
                         kTitleWindowButtonWidth, kTitleBarHeight};
  const Rectangle maxR{closeR.x - kTitleWindowButtonWidth - 1.0f, 0.0f,
                       kTitleWindowButtonWidth, kTitleBarHeight};
  const Rectangle minR{maxR.x - kTitleWindowButtonWidth - 1.0f, 0.0f,
                       kTitleWindowButtonWidth, kTitleBarHeight};
  const Rectangle dragR{0.0f, 0.0f, minR.x, kTitleBarHeight};
  if (windowActionActive && !leftDown) {
    game.draggingTitle = false;
    game.resizeMode = ResizeNone;
    return true;
  }
  if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
    if (hit != ResizeNone && !PointInRect(mouse, minR) &&
        !PointInRect(mouse, maxR) && !PointInRect(mouse, closeR) &&
        !PointInRect(mouse, helpR) && !PointInRect(mouse, netR)) {
      BeginWindowAction(game, hit);
      return true;
    }
    if (PointInRect(mouse, closeR)) {
      game.requestClose = true;
      return true;
    }
    if (PointInRect(mouse, minR)) {
      MinimizeWindow();
      return true;
    }
    if (PointInRect(mouse, maxR)) {
      if (IsWindowMaximized()) {
        RestoreWindow();
      } else {
        MaximizeWindow();
      }
      return true;
    }
    if (PointInRect(mouse, helpR)) {
      game.helpOpen = !game.helpOpen;
      return true;
    }
    if (PointInRect(mouse, netR)) {
      if (game.appMode == AppMode::Single) {
        game.savedWorld = game.world;
        game.savedRules = game.rules;
        game.savedPhase = game.phase;
        game.savedAim = game.aim;
        game.savedPower = game.power;
        game.appMode = AppMode::Lobby;
        game.roomList.clear();
        game.showCreateDlg = false;
        game.showJoinPwdDlg = false;
        game.client.Stop();
        game.host.Stop();
        game.client.Start();
      } else {
        game.client.Stop();
        game.host.Stop();
        game.appMode = AppMode::Single;
        game.world = game.savedWorld;
        game.rules = game.savedRules;
        game.phase = game.savedPhase;
        game.aim = game.savedAim;
        game.power = game.savedPower;
      }
      return true;
    }
    if (PointInRect(mouse, dragR)) {
      BeginWindowAction(game, ResizeNone);
      return true;
    }
  }
  if (IsMouseButtonReleased(MOUSE_LEFT_BUTTON)) {
    game.draggingTitle = false;
    game.resizeMode = ResizeNone;
  }
  if (game.resizeMode != ResizeNone && leftDown) {
    const Vector2 pointer = DesktopMousePosition();
    const int dx = static_cast<int>(std::round(pointer.x - game.windowActionMouseStart.x));
    const int dy = static_cast<int>(std::round(pointer.y - game.windowActionMouseStart.y));
    int newX = static_cast<int>(std::round(game.windowActionPosStart.x));
    int newY = static_cast<int>(std::round(game.windowActionPosStart.y));
    int newW = game.windowActionWidthStart;
    int newH = game.windowActionHeightStart;
    if (game.resizeMode & ResizeLeft) {
      newW = std::max(minW, game.windowActionWidthStart - dx);
      newX += game.windowActionWidthStart - newW;
    }
    if (game.resizeMode & ResizeRight) {
      newW = std::max(minW, game.windowActionWidthStart + dx);
    }
    if (game.resizeMode & ResizeTop) {
      newH = std::max(minH, game.windowActionHeightStart - dy);
      newY += game.windowActionHeightStart - newH;
    }
    if (game.resizeMode & ResizeBottom) {
      newH = std::max(minH, game.windowActionHeightStart + dy);
    }
    ApplyWindowBounds(newX, newY, newW, newH);
    return true;
  }
  if (game.draggingTitle && leftDown) {
    const Vector2 pointer = DesktopMousePosition();
    const int newX = static_cast<int>(std::round(
        game.windowActionPosStart.x + pointer.x - game.windowActionMouseStart.x));
    const int newY = static_cast<int>(std::round(
        game.windowActionPosStart.y + pointer.y - game.windowActionMouseStart.y));
    ApplyWindowBounds(newX, newY, game.windowActionWidthStart, game.windowActionHeightStart);
    return true;
  }
  return false;
}
bool HandleUiInput(Game &game) {
  const float s = UiScale();
  const Vector2 mouse = GetMousePosition();
  Rectangle panel = ControlPanelRect(s);
  const Vector2 tipCenter = TipControlCenter(panel, s);
  const float tipR = 50.0f * s;
  if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
    if (game.helpOpen) {
      const Rectangle help = HelpWindowRect(s);
      const Rectangle closeR{help.x + help.width - 42.0f * s,
                             help.y + 10.0f * s, 26.0f * s, 26.0f * s};
      if (PointInRect(mouse, closeR)) {
        game.helpOpen = false;
        return true;
      }
      if (PointInRect(mouse, help)) {
        return true;
      }
    }
    if (Vector2Distance(mouse, tipCenter) <= tipR) {
      game.tipX = hb::Clamp((mouse.x - tipCenter.x) / tipR, -1.0, 1.0);
      game.tipY = hb::Clamp(-(mouse.y - tipCenter.y) / tipR, -1.0, 1.0);
      return true;
    }
    if (PointInRect(mouse, {panel.x + 18 * s, panel.y + 238 * s, 188 * s, 30 * s})) {
      game.tipX = 0.0;
      game.tipY = 0.0;
      return true;
    }
    if (PointInRect(mouse, {panel.x + 18 * s, panel.y + 354 * s, 188 * s, 30 * s})) {
      game.world.ResetRack();
      game.rules.ResetRack(1 - game.rules.State().currentPlayer);
      game.phase = Phase::Aiming;
      game.power = 0.0;
      return true;
    }
    if (game.phase == Phase::GroupChoice) {
      Rectangle modal{static_cast<float>(GetScreenWidth()) * 0.5f - 170.0f,
                      static_cast<float>(GetScreenHeight()) * 0.5f - 62.0f, 340.0f, 124.0f};
      if (PointInRect(mouse, {modal.x + 34, modal.y + 69, 126, 34})) {
        game.phase = game.rules.ChooseGroup(BallGroup::Solids).nextPhase;
        return true;
      } else if (PointInRect(mouse, {modal.x + 180, modal.y + 69, 126, 34})) {
        game.phase = game.rules.ChooseGroup(BallGroup::Stripes).nextPhase;
        return true;
      }
    }
  }
  return false;
}
void UpdateGame(Game &game, const View &view) {
  // Title bar always works
  if (HandleTitleBarInput(game)) return;
  // Chat input handling
  if (IsKeyPressed(KEY_ENTER)) {
    if (!game.chatInputActive) {
      game.chatInputActive = true;
      game.chatInputBuf.clear();
      game.chatInputCursor = 0;
      game.textAllSelected = false;
    } else if (!game.chatInputBuf.empty()) {
      if (game.appMode == AppMode::PlayingOnline) {
        if (game.host.HasClient()) game.host.SendChat(game.chatInputBuf);
        else game.client.SendChat(game.chatInputBuf);
        game.chatHistory.push_back("我: " + game.chatInputBuf);
      } else if (game.appMode == AppMode::RoomHost) {
        game.host.SendChat(game.chatInputBuf);
        game.chatHistory.push_back("我: " + game.chatInputBuf);
      } else if (game.appMode == AppMode::RoomClient) {
        game.client.SendChat(game.chatInputBuf);
        game.chatHistory.push_back("我: " + game.chatInputBuf);
      }
      game.chatInputBuf.clear();
      game.chatInputActive = false;
      game.chatInputCursor = 0;
      game.textAllSelected = false;
    }
  }
  if (game.chatInputActive) {
    if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
      float sh = static_cast<float>(GetScreenHeight());
      float us = UiScale();
      float cw = 280.0f * us;
      float ch = 150.0f * us;
      float cx = 12.0f * us;
      float cy = sh - ch - 12.0f * us;
      Rectangle inputBox{cx + 4, cy + ch - 28.0f * us, cw - 8, 24.0f * us};
      Vector2 m = GetMousePosition();
      if (CheckCollisionPointRec(m, inputBox)) {
        game.chatInputCursor = TextCursorFromMouseX(game.font, game.chatInputBuf, m.x, cx + 10.0f * us, 15.0f * us);
        game.textAllSelected = false;
      }
    }
    int key = GetCharPressed();
    while (key > 0) {
      if (game.textAllSelected) {
        game.chatInputBuf.clear();
        game.chatInputCursor = 0;
        game.textAllSelected = false;
      }
      if (key >= 32 && key <= 0x9FFF && static_cast<int>(game.chatInputBuf.size()) < 120)
        TextInsertAt(game.chatInputBuf, game.chatInputCursor, key);
      key = GetCharPressed();
    }
    const double now = GetTime();
    if (game.leftRepeat.tick(IsKeyDown(KEY_LEFT), now)) {
      if (game.textAllSelected) {
        game.chatInputCursor = 0;
        game.textAllSelected = false;
      } else {
        game.chatInputCursor = Utf8PrevPos(game.chatInputBuf, game.chatInputCursor);
      }
    }
    if (game.rightRepeat.tick(IsKeyDown(KEY_RIGHT), now)) {
      if (game.textAllSelected) {
        game.chatInputCursor = static_cast<int>(game.chatInputBuf.size());
        game.textAllSelected = false;
      } else {
        game.chatInputCursor = Utf8NextPos(game.chatInputBuf, game.chatInputCursor);
      }
    }
    if (IsKeyPressed(KEY_HOME))
      { game.chatInputCursor = 0; game.textAllSelected = false; }
    if (IsKeyPressed(KEY_END))
      { game.chatInputCursor = static_cast<int>(game.chatInputBuf.size()); game.textAllSelected = false; }
    if (game.delRepeat.tick(IsKeyDown(KEY_DELETE), now)) {
      if (game.textAllSelected) {
        game.chatInputBuf.clear();
        game.chatInputCursor = 0;
        game.textAllSelected = false;
      } else if (game.chatInputCursor < static_cast<int>(game.chatInputBuf.size())) {
        TextEraseAfter(game.chatInputBuf, game.chatInputCursor);
      }
    }
    bool ctrl = IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL);
    if (ctrl && IsKeyPressed(KEY_A) && !game.chatInputBuf.empty()) {
      game.chatInputCursor = static_cast<int>(game.chatInputBuf.size());
      game.textAllSelected = true;
    }
    if (game.bsRepeat.tick(IsKeyDown(KEY_BACKSPACE), now)) {
      if (game.textAllSelected) {
        game.chatInputBuf.clear();
        game.chatInputCursor = 0;
        game.textAllSelected = false;
      } else if (game.chatInputCursor > 0) {
        TextEraseBefore(game.chatInputBuf, game.chatInputCursor);
      }
    }
    if (IsKeyPressed(KEY_ESCAPE)) {
      game.chatInputActive = false;
      game.chatInputBuf.clear();
      game.chatInputCursor = 0;
      game.textAllSelected = false;
    }
  }
  // Mode-specific updates
  if (game.appMode == AppMode::Lobby) {
    if (HandleTitleBarInput(game)) return;
    game.client.Poll();
    if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
      Vector2 mouse = GetMousePosition();
      float sw = static_cast<float>(GetScreenWidth()), sh = static_cast<float>(GetScreenHeight());
      Rectangle createBtn = LobbyCreateButtonRect();
      Rectangle refreshBtn = LobbyRefreshButtonRect();
      if (game.showCreateDlg) {
        const CreateRoomDialogLayout l = CreateRoomLayout();
        if (CheckCollisionPointRec(mouse, l.ok)) {
          if (game.createRoomName.empty()) {
            game.createRoomNotice = "房间名不能为空";
          } else {
            game.roomName = game.createRoomName;
            game.roomPassword = game.createRoomPwd;
            game.hostIsPlayer1 = game.createHostIsP1;
            game.client.Stop();
            if (game.host.Start(game.roomName, game.roomPassword, game.hostIsPlayer1)) {
              game.appMode = AppMode::RoomHost;
              game.amReady = false;
              game.opReady = false;
              game.chatHistory.clear();
            }
            CloseCreateRoomDialog(game);
          }
          return;
        }
        if (CheckCollisionPointRec(mouse, l.cancel)) {
          game.host.Stop();
          CloseCreateRoomDialog(game);
          return;
        }
        if (CheckCollisionPointRec(mouse, l.p1)) {
          game.createHostIsP1 = true;
          return;
        }
        if (CheckCollisionPointRec(mouse, l.p2)) {
          game.createHostIsP1 = false;
          return;
        }
        if (CheckCollisionPointRec(mouse, l.showPwd)) {
          game.showPwd = !game.showPwd;
          return;
        }
        if (CheckCollisionPointRec(mouse, l.nameBox)) {
          game.createDlgFocus = 0;
          game.createNameCursor = TextCursorFromMouseX(
              game.font, game.createRoomName, mouse.x, l.nameBox.x + 10.0f * l.scale,
              21.0f * l.scale);
          return;
        }
        if (CheckCollisionPointRec(mouse, l.pwdBox)) {
          game.createDlgFocus = 1;
          std::string display =
              game.showPwd ? game.createRoomPwd : std::string(game.createRoomPwd.size(), '*');
          game.createPwdCursor = TextCursorFromMouseX(
              game.font, display, mouse.x, l.pwdBox.x + 10.0f * l.scale,
              21.0f * l.scale);
          return;
        }
        return;
      }
      if (!game.showJoinPwdDlg && CheckCollisionPointRec(mouse, createBtn)) {
        ResetCreateRoomDialog(game);
        return;
      }
      if (!game.showJoinPwdDlg && CheckCollisionPointRec(mouse, refreshBtn)) {
        game.client.Stop();
        game.client.Start();
        game.roomList.clear();
        return;
      }
      if (game.showJoinPwdDlg) {
        const float us2 = OnlineUiScale();
        const float cw2 = 360.0f * us2, ch2 = 170.0f * us2;
        const float cx2 = sw * 0.5f - cw2 * 0.5f;
        const float cy2 = sh * 0.5f - ch2 * 0.5f;
        Rectangle okR2{cx2 + cw2 * 0.5f - 82.0f * us2, cy2 + ch2 - 48.0f * us2,
                       74.0f * us2, 32.0f * us2};
        Rectangle cancelR2{cx2 + cw2 * 0.5f + 10.0f * us2, okR2.y,
                           74.0f * us2, 32.0f * us2};
        if (CheckCollisionPointRec(mouse, okR2)) {
          game.client.Stop(); game.client.Start();
          if (game.client.Connect(game.pendingJoinHostIP, game.joinPwdInput)) {
            game.appMode = AppMode::RoomClient;
            game.roomName = game.pendingJoinRoomName;
            game.amReady = false;
            game.opReady = false;
            game.chatHistory.clear();
          }
          CloseJoinPasswordDialog(game);
          return;
        }
        if (CheckCollisionPointRec(mouse, cancelR2)) {
          CloseJoinPasswordDialog(game);
          return;
        }
        Rectangle pwdBox3{cx2 + 20.0f * us2, cy2 + 62.0f * us2,
                          cw2 - 40.0f * us2, 34.0f * us2};
        if (CheckCollisionPointRec(mouse, pwdBox3)) {
          std::string display = std::string(game.joinPwdInput.size(), '*');
          game.joinPwdCursor =
              TextCursorFromMouseX(game.font, display, mouse.x, cx2 + 30.0f * us2,
                                   20.0f * us2);
          game.textAllSelected = false;
        }
        return;
      }
      if (CheckCollisionPointRec(mouse, createBtn)) {
        ResetCreateRoomDialog(game);
      } else if (CheckCollisionPointRec(mouse, refreshBtn)) {
        game.client.Stop();
        game.client.Start();
        game.roomList.clear();
      } else if (game.showCreateDlg) {
        float cw = 360.0f, ch = 280.0f, cx = sw * 0.5f - cw * 0.5f, cy = sh * 0.5f - ch * 0.5f;
        Rectangle okR{cx + cw * 0.5f - 70, cy + ch - 44, 64, 28};
        Rectangle cancelR{cx + cw * 0.5f + 10, cy + ch - 44, 64, 28};
        Rectangle p1R{cx + 16, cy + 204, 80, 24};
        Rectangle p2R{cx + 106, cy + 204, 80, 24};
        if (CheckCollisionPointRec(mouse, okR)) {
          game.roomName = game.createRoomName;
          game.roomPassword = game.createRoomPwd;
          game.hostIsPlayer1 = game.createHostIsP1;
          game.client.Stop();
          if (game.host.Start(game.roomName, game.roomPassword, game.hostIsPlayer1)) {
            game.appMode = AppMode::RoomHost;
            game.amReady = false;
            game.opReady = false;
            game.chatHistory.clear();
          }
          game.showCreateDlg = false;
        } else if (CheckCollisionPointRec(mouse, cancelR)) {
          game.host.Stop();
          game.showCreateDlg = false;
        } else if (CheckCollisionPointRec(mouse, p1R)) {
          game.createHostIsP1 = true;
        } else if (CheckCollisionPointRec(mouse, p2R)) {
          game.createHostIsP1 = false;
        }
        float pwdBoxW2 = cw - 32 - 46;
        Rectangle showPwdR2{cx + 16 + pwdBoxW2 + 6, cy + 142, 40, 28};
        if (CheckCollisionPointRec(mouse, showPwdR2)) {
          game.showPwd = !game.showPwd;
        }
      } else {
        float y = 130;
        game.roomList = game.client.GetRooms();
        for (size_t i = 0; i < game.roomList.size(); ++i) {
          Rectangle row{40, y, sw - 80, 44};
          Rectangle joinBtn{row.x + row.width - 70, row.y + 8, 56, 28};
          if (CheckCollisionPointRec(mouse, joinBtn)) {
            if (game.roomList[i].hasPassword) {
              OpenJoinPasswordDialog(game, game.roomList[i]);
              game.roomList.clear(); // will re-poll
            } else {
              game.client.Stop(); game.client.Start();
              if (game.client.Connect(game.roomList[i].hostIP, "")) {
                game.appMode = AppMode::RoomClient;
                game.roomName = game.roomList[i].name;
                game.amReady = false;
                game.opReady = false;
                game.chatHistory.clear();
              }
            }
            break;
          }
          y += 52;
        }
      }
    }
    // Handle create dialog text input
    if (game.showCreateDlg) {
      // Mouse click to switch focus between room name and password fields
      if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
        Vector2 m = GetMousePosition();
        float scw = static_cast<float>(GetScreenWidth());
        float sch = static_cast<float>(GetScreenHeight());
        float cw3 = 360.0f, ch3 = 280.0f, cx3 = scw * 0.5f - cw3 * 0.5f,
              cy3 = sch * 0.5f - ch3 * 0.5f;
        Rectangle nameBox2{cx3 + 16, cy3 + 80, cw3 - 32, 28};
        Rectangle pwdBox2{cx3 + 16, cy3 + 142, cw3 - 32, 28};
        float us3 = UiScale();
        if (CheckCollisionPointRec(m, nameBox2)) {
          game.createDlgFocus = 0;
          game.createNameCursor = TextCursorFromMouseX(game.font, game.createRoomName, m.x, cx3 + 22, 18.0f * us3);
        } else if (CheckCollisionPointRec(m, pwdBox2)) {
          game.createDlgFocus = 1;
          std::string display = std::string(game.createRoomPwd.size(), '*');
          game.createPwdCursor = TextCursorFromMouseX(game.font, display, m.x, cx3 + 22, 18.0f * us3);
        }
      }
      int key = GetCharPressed();
      while (key > 0) {
        if (game.textAllSelected) {
          std::string &selectedText =
              game.createDlgFocus == 0 ? game.createRoomName : game.createRoomPwd;
          int &selectedCursor =
              game.createDlgFocus == 0 ? game.createNameCursor : game.createPwdCursor;
          selectedText.clear();
          selectedCursor = 0;
          game.textAllSelected = false;
        }
        if (game.createDlgFocus == 0 && key >= 32) {
          if (Utf8CodepointCount(game.createRoomName) < kMaxRoomNameChars) {
            TextInsertAt(game.createRoomName, game.createNameCursor, key);
            game.createRoomNotice.clear();
          } else {
            game.createRoomNotice = "房间名最多 24 个字符";
          }
        } else if (game.createDlgFocus == 1 && key > 0 &&
                   Utf8CodepointCount(game.createRoomPwd) < kMaxRoomPasswordChars) {
          TextInsertAt(game.createRoomPwd, game.createPwdCursor, key);
        }
        key = GetCharPressed();
      }
      int &cursor = game.createDlgFocus == 0 ? game.createNameCursor : game.createPwdCursor;
      std::string &text = game.createDlgFocus == 0 ? game.createRoomName : game.createRoomPwd;
      const double now = GetTime();
      if (game.leftRepeat.tick(IsKeyDown(KEY_LEFT), now)) {
        if (game.textAllSelected) {
          cursor = 0;
          game.textAllSelected = false;
        } else {
          cursor = Utf8PrevPos(text, cursor);
        }
      }
      if (game.rightRepeat.tick(IsKeyDown(KEY_RIGHT), now)) {
        if (game.textAllSelected) {
          cursor = static_cast<int>(text.size());
          game.textAllSelected = false;
        } else {
          cursor = Utf8NextPos(text, cursor);
        }
      }
      if (IsKeyPressed(KEY_HOME)) { cursor = 0; game.textAllSelected = false; }
      if (IsKeyPressed(KEY_END)) { cursor = static_cast<int>(text.size()); game.textAllSelected = false; }
      if (game.delRepeat.tick(IsKeyDown(KEY_DELETE), now)) {
        if (game.textAllSelected) {
          text.clear();
          cursor = 0;
          game.textAllSelected = false;
        } else if (cursor < static_cast<int>(text.size())) {
          TextEraseAfter(text, cursor);
        }
      }
      bool ctrl = IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL);
      if (ctrl && IsKeyPressed(KEY_A) && !text.empty()) {
        cursor = static_cast<int>(text.size());
        game.textAllSelected = true;
      }
      if (game.bsRepeat.tick(IsKeyDown(KEY_BACKSPACE), now)) {
        if (game.textAllSelected) {
          text.clear();
          cursor = 0;
          game.textAllSelected = false;
        } else if (cursor > 0) {
          TextEraseBefore(text, cursor);
        }
      }
      // Tab to switch focus
      if (IsKeyPressed(KEY_TAB)) {
        game.createDlgFocus = 1 - game.createDlgFocus;
        game.textAllSelected = false;
      }
    }
    // Handle join password dialog (also shown in lobby)
    if (game.showJoinPwdDlg && game.appMode == AppMode::Lobby) {
      // This is actually the join password dialog
      float sw2 = static_cast<float>(GetScreenWidth()), sh2 = static_cast<float>(GetScreenHeight());
      float us2 = OnlineUiScale();
      float cw2 = 360.0f * us2, ch2 = 170.0f * us2, cx2 = sw2 * 0.5f - cw2 * 0.5f, cy2 = sh2 * 0.5f - ch2 * 0.5f;
      Rectangle okR2{cx2 + cw2 * 0.5f - 82.0f * us2, cy2 + ch2 - 48.0f * us2,
                     74.0f * us2, 32.0f * us2};
      Rectangle cancelR2{cx2 + cw2 * 0.5f + 10.0f * us2, okR2.y,
                         74.0f * us2, 32.0f * us2};
      Vector2 mouse = GetMousePosition();
      if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
        if (CheckCollisionPointRec(mouse, okR2)) {
          game.client.Stop(); game.client.Start();
          if (game.client.Connect(game.pendingJoinHostIP, game.joinPwdInput)) {
            game.appMode = AppMode::RoomClient;
            game.roomName = game.pendingJoinRoomName;
            game.amReady = false;
            game.opReady = false;
            game.chatHistory.clear();
          }
          CloseJoinPasswordDialog(game);
        } else if (CheckCollisionPointRec(mouse, cancelR2)) {
          CloseJoinPasswordDialog(game);
        } else {
          Rectangle pwdBox3{cx2 + 20.0f * us2, cy2 + 62.0f * us2,
                            cw2 - 40.0f * us2, 34.0f * us2};
          if (CheckCollisionPointRec(mouse, pwdBox3)) {
            std::string display = std::string(game.joinPwdInput.size(), '*');
            game.joinPwdCursor = TextCursorFromMouseX(game.font, display, mouse.x,
                                                      cx2 + 30.0f * us2, 20.0f * us2);
            game.textAllSelected = false;
          }
        }
      }
      int key2 = GetCharPressed();
      while (key2 > 0) {
        if (game.textAllSelected) {
          game.joinPwdInput.clear();
          game.joinPwdCursor = 0;
          game.textAllSelected = false;
        }
        if (key2 >= 32 && key2 <= 126 && static_cast<int>(game.joinPwdInput.size()) < 20)
          TextInsertAt(game.joinPwdInput, game.joinPwdCursor, key2);
        key2 = GetCharPressed();
      }
      double now2 = GetTime();
      if (game.leftRepeat.tick(IsKeyDown(KEY_LEFT), now2)) {
        if (game.textAllSelected) {
          game.joinPwdCursor = 0;
          game.textAllSelected = false;
        } else {
          game.joinPwdCursor = Utf8PrevPos(game.joinPwdInput, game.joinPwdCursor);
        }
      }
      if (game.rightRepeat.tick(IsKeyDown(KEY_RIGHT), now2)) {
        if (game.textAllSelected) {
          game.joinPwdCursor = static_cast<int>(game.joinPwdInput.size());
          game.textAllSelected = false;
        } else {
          game.joinPwdCursor = Utf8NextPos(game.joinPwdInput, game.joinPwdCursor);
        }
      }
      if (IsKeyPressed(KEY_HOME)) { game.joinPwdCursor = 0; game.textAllSelected = false; }
      if (IsKeyPressed(KEY_END)) { game.joinPwdCursor = static_cast<int>(game.joinPwdInput.size()); game.textAllSelected = false; }
      if (game.delRepeat.tick(IsKeyDown(KEY_DELETE), now2)) {
        if (game.textAllSelected) {
          game.joinPwdInput.clear();
          game.joinPwdCursor = 0;
          game.textAllSelected = false;
        } else if (game.joinPwdCursor < static_cast<int>(game.joinPwdInput.size())) {
          TextEraseAfter(game.joinPwdInput, game.joinPwdCursor);
        }
      }
      bool ctrl2 = IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL);
      if (ctrl2 && IsKeyPressed(KEY_A) && !game.joinPwdInput.empty()) {
        game.joinPwdCursor = static_cast<int>(game.joinPwdInput.size());
        game.textAllSelected = true;
      }
      if (game.bsRepeat.tick(IsKeyDown(KEY_BACKSPACE), now2)) {
        if (game.textAllSelected) {
          game.joinPwdInput.clear();
          game.joinPwdCursor = 0;
          game.textAllSelected = false;
        } else if (game.joinPwdCursor > 0) {
          TextEraseBefore(game.joinPwdInput, game.joinPwdCursor);
        }
      }
    }
    return;
  }
  if (game.appMode == AppMode::RoomHost) {
    game.host.Poll();
    auto syncHostReadyToClient = [&]() {
      if (game.amReady) game.host.SendReady();
      else game.host.SendUnready();
    };
    if (game.host.HasJoinRequest(nullptr)) {
      game.opReady = false;
      syncHostReadyToClient();
    }
    if (game.host.HasReady()) {
      game.opReady = true;
      syncHostReadyToClient();
    }
    if (game.host.HasUnready()) {
      game.opReady = false;
      syncHostReadyToClient();
    }
    if (game.host.HasChat(nullptr)) {
      std::string msg; game.host.HasChat(&msg);
      game.chatHistory.push_back("对手: " + msg);
    }
    if (game.host.HasBye() || game.host.ClientDisconnected()) {
      game.opReady = false;
      game.amReady = false;
      game.chatHistory.clear();
    }
    if (game.host.HasAim(nullptr, nullptr, nullptr, nullptr, nullptr)) {
      game.host.HasAim(&game.opAimTipX, &game.opAimTipY, &game.opPower, &game.opAimX, &game.opAimY);
    }
    if (game.amReady && game.opReady) {
      int clientPlayer = game.hostIsPlayer1 ? 1 : 0;
      game.host.SendAssign(clientPlayer);
      game.world.ResetRack();
      game.rules.ResetRack(game.hostIsPlayer1 ? 0 : 1);
      game.phase = Phase::Aiming;
      game.onlineTurn = 0;
      if (!game.hostIsPlayer1) game.onlineTurn = 1;
      game.appMode = AppMode::PlayingOnline;
      game.host.SendTurn(game.rules.State().currentPlayer);
      game.host.SendPositions(game.world.Balls());
      game.syncedBalls = game.world.Balls();
    }
    // Ready button
    if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
      Vector2 mouse = GetMousePosition();
      float sw = static_cast<float>(GetScreenWidth()), sh = static_cast<float>(GetScreenHeight());
      Rectangle readyBtn{sw * 0.5f - 60, sh - 120, 120, 34};
      if (CheckCollisionPointRec(mouse, readyBtn)) {
        game.amReady = !game.amReady;
        if (game.amReady) game.host.SendReady();
        else game.host.SendUnready();
      }
    }
    return;
  }
  if (game.appMode == AppMode::RoomClient) {
    game.client.Poll();
    if (game.client.HasAssign(nullptr)) {
      game.client.HasAssign(&game.assignedPlayer);
    }
    if (game.client.HasTurn(nullptr)) {
      int turn;
      game.client.HasTurn(&turn);
      game.rules.State().currentPlayer = turn;
    }
    if (game.client.HasPositions(nullptr)) {
      game.client.HasPositions(&game.syncedBalls);
    }
    if (game.client.HasReadyAck()) { game.opReady = true; }
    if (game.client.HasUnreadyAck()) { game.opReady = false; }
    if (game.client.HasChat(nullptr)) {
      std::string msg; game.client.HasChat(&msg);
      game.chatHistory.push_back("对手: " + msg);
    }
    if (game.client.HasBye() || game.client.Disconnected()) {
      game.opReady = false;
      game.amReady = false;
      game.assignedPlayer = -1;
      game.chatHistory.clear();
    }
    // Auto-return to lobby if connection was rejected or failed
    if (!game.client.IsAccepted() && !game.client.IsConnecting()) {
      game.client.Stop(); game.client.Start();
      game.appMode = AppMode::Lobby;
      game.showCreateDlg = false;
      game.showJoinPwdDlg = false;
      game.roomPassword.clear();
      game.assignedPlayer = -1;
      return;
    }
    if (game.client.HasAim(nullptr, nullptr, nullptr, nullptr, nullptr)) {
      game.client.HasAim(&game.opAimTipX, &game.opAimTipY, &game.opPower, &game.opAimX, &game.opAimY);
    }
    if (game.amReady && game.opReady && game.client.IsAccepted() && game.assignedPlayer >= 0) {
      game.onlineTurn = game.assignedPlayer;
      game.world.ResetRack();
      // Apply synced positions to world
      if (game.syncedBalls[0].pos.x != 0.0 || game.syncedBalls[0].pos.y != 0.0) {
        ApplySyncedBalls(game);
      }
      game.phase = Phase::Aiming;
      game.appMode = AppMode::PlayingOnline;
    }
    if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON) && game.client.IsAccepted()) {
      Vector2 mouse = GetMousePosition();
      float sw = static_cast<float>(GetScreenWidth()), sh = static_cast<float>(GetScreenHeight());
      Rectangle readyBtn{sw * 0.5f - 60, sh - 120, 120, 34};
      if (CheckCollisionPointRec(mouse, readyBtn)) {
        game.amReady = !game.amReady;
        if (game.amReady) game.client.SendReady();
        else game.client.SendUnready();
      }
    }
    return;
  }
  // Playing Online
  if (game.appMode == AppMode::PlayingOnline) {
    if (HandleTitleBarInput(game)) return;
    bool amHost = game.host.HasClient();
    if (amHost) game.host.Poll(); else game.client.Poll();
    // Handle opponent bye/disconnect
    if (amHost) {
      if (game.host.HasBye() || game.host.ClientDisconnected()) {
        game.appMode = AppMode::RoomHost; game.amReady = false; game.opReady = false; game.chatHistory.clear(); return;
      }
      if (game.host.HasChat(nullptr)) { std::string m; game.host.HasChat(&m); game.chatHistory.push_back("对手: " + m); }
      if (game.host.HasAim(nullptr, nullptr, nullptr, nullptr, nullptr)) { game.host.HasAim(&game.opAimTipX, &game.opAimTipY, &game.opPower, &game.opAimX, &game.opAimY); }
    } else {
      if (game.client.HasBye() || game.client.Disconnected()) {
        game.appMode = AppMode::RoomClient; game.amReady = false; game.opReady = false; game.assignedPlayer = -1; game.chatHistory.clear(); return;
      }
      if (game.client.HasChat(nullptr)) { std::string m; game.client.HasChat(&m); game.chatHistory.push_back("对手: " + m); }
      if (game.client.HasAim(nullptr, nullptr, nullptr, nullptr, nullptr)) { game.client.HasAim(&game.opAimTipX, &game.opAimTipY, &game.opPower, &game.opAimX, &game.opAimY); }
    }
    if (!amHost && game.client.HasTurn(nullptr)) {
      int turn;
      game.client.HasTurn(&turn);
      game.rules.State().currentPlayer = turn;
      if (game.phase == Phase::Moving) {
        game.phase = Phase::Aiming;
      }
    }
    bool isMyTurn = (game.rules.State().currentPlayer == game.onlineTurn);
    bool isOpponentTurn = !isMyTurn;
    const float dt = GetFrameTime();
    double now = GetTime();
    if (isMyTurn && game.phase == Phase::Aiming && !game.world.CueBall().pocketed &&
        !game.world.CueBall().sinking) {
      const Vec2 mouseWorld = ScreenToWorld(view, GetMousePosition());
      game.aim = hb::Normalize(mouseWorld - game.world.CueBall().pos);
      const float wheel = GetMouseWheelMove();
      if (wheel < 0.0f) {
        game.power = hb::Clamp(game.power + (-wheel) * 0.055, 0.0, 1.0);
      } else if (wheel > 0.0f && game.power > 0.015) {
        ShotParams shot{game.aim, game.power, game.tipX, game.tipY};
        game.shotEvents.Clear();
        game.world.StrikeCue(shot);
        game.phase = Phase::Moving;
        game.power = 0.0;
        if (amHost) {
          game.host.SendShot(game.tipX, game.tipY, shot.power, game.aim.x, game.aim.y);
          game.host.SendPositions(game.world.Balls());
          game.syncedBalls = game.world.Balls();
        } else {
          game.client.SendShot(game.tipX, game.tipY, shot.power, game.aim.x, game.aim.y);
        }
      }
      // Send aim updates
      if (now - game.lastAimSend > 0.067) {
        game.lastAimSend = now;
        if (amHost) game.host.SendAim(game.tipX, game.tipY, game.power, game.aim.x, game.aim.y);
        else game.client.SendAim(game.tipX, game.tipY, game.power, game.aim.x, game.aim.y);
        // also send aim direction? For simplicity, just tipX/tipY/power
      }
    } else if (isMyTurn && game.phase == Phase::BallInHand) {
      Vec2 pos = ClampCuePlacement(ScreenToWorld(view, GetMousePosition()));
      if (game.world.CanPlaceCue(pos)) {
        game.world.PlaceCue(pos);
        if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
          game.rules.State().ballInHand = false;
          game.phase = Phase::Aiming;
        }
      }
    } else if (isMyTurn && game.phase == Phase::Moving) {
      game.world.Step(dt, &game.shotEvents);
      if (!game.world.IsMoving()) {
        auto decision = game.rules.ApplyShot(game.shotEvents, game.world);
        game.phase = decision.nextPhase;
        int cp = game.rules.State().currentPlayer;
        if (amHost) {
          game.host.SendTurn(cp);
          game.host.SendPositions(game.world.Balls());
          game.syncedBalls = game.world.Balls();
        }
        if (game.phase == Phase::BallInHand) {
          game.world.PlaceCue({hb::kHeadSpotX, 0.0});
        }
      }
    } else if (isMyTurn && game.phase == Phase::RackOver && !game.chatInputActive && IsKeyPressed(KEY_SPACE)) {
      int nb = game.rules.State().winner >= 0 ? game.rules.State().winner : 0;
      game.world.ResetRack();
      game.rules.ResetRack(nb);
      game.phase = Phase::Aiming;
    }
    // Host: compute physics for opponent's shot if received
    if (amHost && isOpponentTurn && game.phase == Phase::Aiming) {
      if (game.host.HasShot(nullptr)) {
        ShotParams opShot;
        game.host.HasShot(&opShot);
        game.shotEvents.Clear();
        game.world.StrikeCue(opShot);
        game.phase = Phase::Moving;
      }
    }
    // Also handle opponent's Moving phase on host
    if (amHost && isOpponentTurn && game.phase == Phase::Moving) {
      game.world.Step(dt, &game.shotEvents);
      if (!game.world.IsMoving()) {
        auto decision = game.rules.ApplyShot(game.shotEvents, game.world);
        game.phase = decision.nextPhase;
        int cp = game.rules.State().currentPlayer;
        game.host.SendTurn(cp);
        game.host.SendPositions(game.world.Balls());
        game.syncedBalls = game.world.Balls();
        if (game.phase == Phase::BallInHand) {
          game.world.PlaceCue({hb::kHeadSpotX, 0.0});
        }
      }
    }
    // Client: apply received shot
    if (!amHost && isOpponentTurn && game.phase == Phase::Aiming) {
      if (game.client.HasShot(nullptr)) {
        ShotParams opShot;
        game.client.HasShot(&opShot);
        game.shotEvents.Clear();
        game.world.StrikeCue(opShot);
        game.phase = Phase::Moving;
      }
    }
    // Host: send positions during moving phase
    if (amHost && game.phase == Phase::Moving) {
      if (now - game.lastPosSend > 1.0 / 30.0) {
        game.lastPosSend = now;
        game.host.SendPositions(game.world.Balls());
        game.syncedBalls = game.world.Balls();
      }
    }
    // Client: receive and render positions
    if (!amHost) {
      if (game.client.HasPositions(nullptr)) {
        game.client.HasPositions(&game.syncedBalls);
      }
      // Override world ball positions with synced data when not actively computing physics
      if (!isMyTurn || game.phase != Phase::Moving) {
        ApplySyncedBalls(game);
      }
    }
    // Handle turn messages for client
    if (!amHost && game.client.HasTurn(nullptr)) {
      int turn;
      game.client.HasTurn(&turn);
      game.rules.State().currentPlayer = turn;
    }
    if (!game.chatInputActive && IsKeyPressed(KEY_C)) { game.tipX = 0.0; game.tipY = 0.0; }
    return;
  }
  // Single-player (original)
  if (!game.chatInputActive && IsKeyPressed(KEY_ESCAPE)) {
    if (game.power > 0.0) {
      game.power = 0.0;
      return;
    }
  }
  if (HandleTitleBarInput(game)) {
    return;
  }
  const bool uiHandled = HandleUiInput(game);
  const float dt = GetFrameTime();
  if (!uiHandled && game.phase == Phase::Aiming && !game.world.CueBall().pocketed &&
      !game.world.CueBall().sinking) {
    const Vec2 mouseWorld = ScreenToWorld(view, GetMousePosition());
    game.aim = hb::Normalize(mouseWorld - game.world.CueBall().pos);
    const float wheel = GetMouseWheelMove();
    if (wheel < 0.0f) {
      game.power = hb::Clamp(game.power + (-wheel) * 0.055, 0.0, 1.0);
    } else if (wheel > 0.0f && game.power > 0.015) {
      ShotParams shot{game.aim, game.power, game.tipX, game.tipY};
      game.shotEvents.Clear();
      game.world.StrikeCue(shot);
      game.phase = Phase::Moving;
      game.power = 0.0;
    }
  } else if (game.phase == Phase::BallInHand) {
    Vec2 pos = ClampCuePlacement(ScreenToWorld(view, GetMousePosition()));
    if (!uiHandled && game.world.CanPlaceCue(pos)) {
      game.world.PlaceCue(pos);
      if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
        game.rules.State().ballInHand = false;
        game.phase = Phase::Aiming;
      }
    }
  } else if (game.phase == Phase::Moving) {
    game.world.Step(dt, &game.shotEvents);
    if (!game.world.IsMoving()) {
      auto decision = game.rules.ApplyShot(game.shotEvents, game.world);
      game.phase = decision.nextPhase;
      if (game.phase == Phase::BallInHand) {
        game.world.PlaceCue({hb::kHeadSpotX, 0.0});
      }
    }
  } else if (game.phase == Phase::RackOver && !game.chatInputActive && IsKeyPressed(KEY_SPACE)) {
    const int nextBreaker = game.rules.State().winner >= 0 ? game.rules.State().winner : 0;
    game.world.ResetRack();
    game.rules.ResetRack(nextBreaker);
    game.phase = Phase::Aiming;
  }
  if (!game.chatInputActive && IsKeyPressed(KEY_C)) {
    game.tipX = 0.0;
    game.tipY = 0.0;
  }
}
void DrawGame(Game &game, const View &view) {
  if (game.appMode == AppMode::Lobby) {
    DrawLobby(game);
    if (game.showCreateDlg || game.showJoinPwdDlg) {
      DrawBackdrop();
      if (game.showJoinPwdDlg) {
        DrawJoinPasswordDialog(game);
      } else if (game.showCreateDlg) {
        DrawCreateRoomDialog(game);
      }
    }
    return;
  }
  if (game.appMode == AppMode::RoomHost || game.appMode == AppMode::RoomClient) {
    DrawRoomWait(game);
    return;
  }
  ClearBackground(BLACK);
  DrawTable(view);
  DrawPockets(view);
  for (const Ball &ball : game.world.Balls()) {
    if (!ball.sinking) {
      DrawBall(game.font, view, ball);
    }
  }
  for (const Ball &ball : game.world.Balls()) {
    if (ball.sinking) {
      DrawBall(game.font, view, ball);
    }
  }
  if (game.appMode == AppMode::PlayingOnline) {
    bool isMyTurn = (game.rules.State().currentPlayer == game.onlineTurn);
    if (isMyTurn && game.phase == Phase::Aiming) {
      DrawCueAndAim(game, view);
      DrawCueShadow(game, view);
    } else if (!isMyTurn && game.phase == Phase::Aiming) {
      const Vector2 oc = WorldToScreen(view, game.world.CueBall().pos);
      Vec2 opAim = hb::Normalize({game.opAimX, game.opAimY});
      Vector2 aimDir{static_cast<float>(opAim.x), static_cast<float>(opAim.y)};
      const float ballR = static_cast<float>(hb::kBallRadius * view.scale);
      DrawDetailedCue(oc, aimDir, ballR, static_cast<float>(game.opPower), view);
      // Draw opponent aim line
      Vec2 ghostPos{};
      const bool hasGhost = FirstAimTarget(game.world, game.world.CueBall().pos, opAim, &ghostPos) > 0;
      const Vec2 aimStop = hasGhost ? ghostPos : RayToTableEdge(game.world.CueBall().pos, opAim);
      const Vector2 aimEnd = WorldToScreen(view, aimStop);
      DrawLineEx(oc, aimEnd, 1.0f, {214, 225, 210, 70});
      if (hasGhost) {
        const Vector2 ghost = WorldToScreen(view, ghostPos);
        const float ghostR = static_cast<float>(hb::kBallRadius * view.scale);
        DrawCircleLines(static_cast<int>(ghost.x), static_cast<int>(ghost.y), ghostR,
                        {236, 241, 228, 205});
      }
    }
    DrawChatWindow(game);
  } else {
    DrawCueAndAim(game, view);
    DrawCueShadow(game, view);
  }
  DrawTopStatus(game);
  DrawFpsCounter(game.font);
  if (game.phase == Phase::BallInHand) {
    const Vector2 p = WorldToScreen(view, game.world.CueBall().pos);
    DrawCircleLines(static_cast<int>(p.x), static_cast<int>(p.y),
                    static_cast<float>(hb::kBallRadius * view.scale * 1.45),
                    {236, 238, 228, 150});
  }
  DrawUI(game);
  if (game.helpOpen) DrawBackdrop();
  DrawHelpWindow(game);
  if (game.phase == Phase::RackOver) {
    DrawBackdrop();
    Rectangle r{static_cast<float>(GetScreenWidth()) * 0.5f - 190.0f,
                static_cast<float>(GetScreenHeight()) * 0.5f - 45.0f, 380.0f, 90.0f};
    DrawRoundedFillWithBorder(r, 0.05f, 12, {11, 12, 12, 240},
                              {70, 82, 76, 210});
    const std::string rackMessage = LocalStatusMessage(game);
    DrawTextF(game.font, rackMessage.c_str(), {r.x + 35, r.y + 15}, 28,
              {236, 238, 228, 255});
    DrawTextF(game.font, "按空格开始新一局", {r.x + 63, r.y + 52}, 22,
              {180, 190, 182, 255});
  }
  DrawTitleBar(game);
}
} // namespace
void EnableDebugConsoleIfRequested(int argc, char **argv) {
#ifdef _WIN32
  for (int i = 1; i < argc; ++i) {
    if (std::strcmp(argv[i], "--console") == 0 ||
        std::strcmp(argv[i], "--debug-console") == 0) {
      AllocConsole();
      FILE *ignored = nullptr;
      freopen_s(&ignored, "CONOUT$", "w", stdout);
      freopen_s(&ignored, "CONOUT$", "w", stderr);
      freopen_s(&ignored, "CONIN$", "r", stdin);
      break;
    }
  }
#else
  (void)argc;
  (void)argv;
#endif
}
int main(int argc, char **argv) {
  EnableDebugConsoleIfRequested(argc, argv);
  SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_WINDOW_RESIZABLE | FLAG_WINDOW_UNDECORATED);
  InitWindow(1360, 820, "\xe4\xb8\xad\xe5\xbc\x8f\xe5\x8f\xb0\xe7\x90\x83");
  SetExitKey(KEY_NULL);
  SetWindowMinSize(960, 600);
  SetTargetFPS(240);
  Game game;
  std::vector<int> codepoints = BuildFontCodepoints();
  game.font = LoadGameFont(codepoints);
  game.customFont = game.font.texture.id != 0;
  if (game.customFont) {
    ConfigureFontTexture(game.font);
  }
  if (!game.customFont) {
    game.font = GetFontDefault();
  }
  game.splashStartTime = GetTime();
  while (!WindowShouldClose() && !game.requestClose) {
    const View view = MakeView(GetScreenWidth(), GetScreenHeight());
    if (SplashActive(game)) {
      if (SplashSkipPressed()) {
        game.splashDone = true;
      }
    } else {
      game.splashDone = true;
      UpdateGame(game, view);
    }
    BeginDrawing();
    if (!game.splashDone) {
      DrawHealthSplash(game);
    } else {
      DrawGame(game, view);
    }
    EndDrawing();
  }
  if (game.splashTexture.texture.id != 0) {
    UnloadRenderTexture(game.splashTexture);
  }
  if (game.customFont) {
    UnloadFont(game.font);
  }
  CloseWindow();
  return 0;
}
