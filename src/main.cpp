#include "core.hpp"

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
extern "C" __declspec(dllimport) void *__stdcall GetModuleHandleA(const char *);
extern "C" __declspec(dllimport) void *__stdcall FindResourceA(void *, const char *, const char *);
extern "C" __declspec(dllimport) void *__stdcall LoadResource(void *, void *);
extern "C" __declspec(dllimport) unsigned long __stdcall SizeofResource(void *, void *);
extern "C" __declspec(dllimport) void *__stdcall LockResource(void *);
#endif

namespace {

using hb::Ball;
using hb::BallGroup;
using hb::Phase;
using hb::ShotEvents;
using hb::ShotParams;
using hb::Vec2;

constexpr int kPingFangResourceId = 101;
constexpr int kWinResourceTypeRCDATA = 10;
constexpr int kUiFontSize = 40;

struct View {
  Rectangle play{};
  double scale = 1.0;
  float uiScale = 1.0f;
};

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
  bool requestClose = false;
  bool helpOpen = false;
};

enum ResizeEdge {
  ResizeNone = 0,
  ResizeLeft = 1,
  ResizeRight = 2,
  ResizeTop = 4,
  ResizeBottom = 8
};

View MakeView(int sw, int sh) {
  const float uiScale = static_cast<float>(
      hb::Clamp(std::min(sw / 1360.0, sh / 820.0), 0.72, 1.55));
  const float margin = 78.0f * uiScale;
  const float uiReserve = 285.0f * uiScale;
  const float availableW = static_cast<float>(sw) - margin * 2.0f - uiReserve;
  const float availableH = static_cast<float>(sh) - margin * 2.0f;
  const float tableAspect = static_cast<float>(hb::kTableWidth / hb::kTableHeight);
  float w = availableW;
  float h = w / tableAspect;
  if (h > availableH) {
    h = availableH;
    w = h * tableAspect;
  }
  Rectangle r{margin, (static_cast<float>(sh) - h) * 0.5f, w, h};
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

void AddCodepoint(std::vector<int> &codepoints, int codepoint) {
  if (std::find(codepoints.begin(), codepoints.end(), codepoint) ==
      codepoints.end()) {
    codepoints.push_back(codepoint);
  }
}

void AddUtf8Codepoints(std::vector<int> &codepoints, const char *text) {
  const auto *p = reinterpret_cast<const unsigned char *>(text);
  while (*p != 0) {
    int cp = 0;
    int bytes = 0;
    if ((*p & 0x80) == 0) {
      cp = *p;
      bytes = 1;
    } else if ((*p & 0xE0) == 0xC0) {
      cp = *p & 0x1F;
      bytes = 2;
    } else if ((*p & 0xF0) == 0xE0) {
      cp = *p & 0x0F;
      bytes = 3;
    } else if ((*p & 0xF8) == 0xF0) {
      cp = *p & 0x07;
      bytes = 4;
    } else {
      ++p;
      continue;
    }
    for (int i = 1; i < bytes && p[i] != 0; ++i) {
      cp = (cp << 6) | (p[i] & 0x3F);
    }
    AddCodepoint(codepoints, cp);
    p += bytes;
  }
}

std::vector<int> BuildFontCodepoints() {
  std::vector<int> codepoints;
  for (int c = 32; c <= 126; ++c) {
    codepoints.push_back(c);
  }
  AddUtf8Codepoints(
      codepoints,
      "中式八球双人对战当前目前击球号玩家组未定全色半控制点左右高低还原力度"
      "重新摆选择或按空格键开始下一局白落袋获得自由首次触错误碰后无且库"
      "开不合法打进复位方继续赢得本局轮到目标请选择");
  AddUtf8Codepoints(codepoints, "：，×");
  AddUtf8Codepoints(
      codepoints,
      "帮助规则两名轮流先合法打完自己的再打进获胜犯规对手自动判定操作移动鼠标瞄准滚轮"
      "下滑蓄力上滑出杆右侧击点盘调整高低杆和左右塞按钮回到中杆确认放置清空关闭：，。；、×");
  return codepoints;
}

const char *ResourceId(int id) {
  return reinterpret_cast<const char *>(
      static_cast<std::uintptr_t>(static_cast<unsigned short>(id)));
}

Font LoadPingFangFont(std::vector<int> &codepoints) {
#ifdef _WIN32
  void *module = GetModuleHandleA(nullptr);
  if (module) {
    void *resource =
        FindResourceA(module, ResourceId(kPingFangResourceId),
                      ResourceId(kWinResourceTypeRCDATA));
    if (resource) {
      const unsigned long size = SizeofResource(module, resource);
      void *handle = LoadResource(module, resource);
      const auto *data = static_cast<const unsigned char *>(LockResource(handle));
      if (data && size > 0 && size <= 0x7fffffffUL) {
        Font font = LoadFontFromMemory(".otf", data, static_cast<int>(size),
                                       kUiFontSize, codepoints.data(),
                                       static_cast<int>(codepoints.size()));
        if (font.texture.id != 0) {
          return font;
        }
      }
    }
  }
#endif
  return LoadFontEx("C:/Windows/Fonts/PingFangSC.otf", kUiFontSize, codepoints.data(),
                    static_cast<int>(codepoints.size()));
}

void ConfigureFontTexture(Font &font) {
  if (font.texture.id == 0) {
    return;
  }
  SetTextureFilter(font.texture, TEXTURE_FILTER_BILINEAR);
}

void DrawTextF(Font font, const char *text, Vector2 pos, float size, Color color) {
  const Vector2 p{std::round(pos.x), std::round(pos.y)};
  const float fs = std::max(1.0f, std::round(size));
  DrawTextEx(font, text, {p.x + 1.0f, p.y + 1.0f}, fs, 0.0f,
             {0, 0, 0,
              static_cast<unsigned char>(std::min(180, static_cast<int>(color.a) * 3 / 5))});
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
  DrawLineEx(cue, aimEnd, 1.0f,
             {214, 225, 210, 70});
  if (hasGhost) {
    const Vector2 ghost = WorldToScreen(view, ghostPos);
    const float ghostR = static_cast<float>(hb::kBallRadius * view.scale);
    DrawCircleLines(static_cast<int>(ghost.x), static_cast<int>(ghost.y), ghostR,
                    {236, 241, 228, 205});
  }
  DrawDetailedCue(cue, dir, ballR, static_cast<float>(game.power), view);
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
  std::snprintf(line, sizeof(line), "目前击球：%d号玩家    击打：%s",
                rs.currentPlayer + 1,
                hb::GroupName(rs.players[rs.currentPlayer].group));
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

Rectangle ControlPanelRect(float s) {
  return {static_cast<float>(GetScreenWidth()) - 258.0f * s, 46.0f * s,
          224.0f * s, 390.0f * s};
}

Vector2 TipControlCenter(Rectangle panel, float s) {
  return {panel.x + 112.0f * s, panel.y + 146.0f * s};
}

Rectangle HelpButtonRect() {
  return {188.0f, 0.0f, 72.0f, 34.0f};
}

Rectangle HelpWindowRect(float s) {
  return {108.0f * s, 54.0f * s, 520.0f * s, 314.0f * s};
}

Vector2 PointAlong(Vector2 a, Vector2 b, float t) {
  return {a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t};
}

float LerpF(float a, float b, float t) {
  return a + (b - a) * t;
}

void DrawTaperedSection(Vector2 a, Vector2 b, Vector2 normal, float widthA,
                        float widthB, Color color) {
  const Vector2 aL{a.x + normal.x * widthA * 0.5f,
                   a.y + normal.y * widthA * 0.5f};
  const Vector2 aR{a.x - normal.x * widthA * 0.5f,
                   a.y - normal.y * widthA * 0.5f};
  const Vector2 bL{b.x + normal.x * widthB * 0.5f,
                   b.y + normal.y * widthB * 0.5f};
  const Vector2 bR{b.x - normal.x * widthB * 0.5f,
                   b.y - normal.y * widthB * 0.5f};
  DrawTriangle(aL, aR, bR, color);
  DrawTriangle(aL, bR, bL, color);
}

void DrawCueRing(Vector2 butt, Vector2 tip, Vector2 normal, float t1, float t2,
                 float buttW, float tipW, Color color) {
  const Vector2 a = PointAlong(butt, tip, t1);
  const Vector2 b = PointAlong(butt, tip, t2);
  DrawTaperedSection(a, b, normal, LerpF(buttW, tipW, t1),
                     LerpF(buttW, tipW, t2), color);
}

void DrawCueInlay(Vector2 butt, Vector2 tip, Vector2 normal, float baseT,
                  float pointT, float baseWidth, float offset, Color color) {
  const Vector2 base = PointAlong(butt, tip, baseT);
  const Vector2 point = PointAlong(butt, tip, pointT);
  const Vector2 c{base.x + normal.x * offset, base.y + normal.y * offset};
  const Vector2 a{c.x + normal.x * baseWidth * 0.5f,
                  c.y + normal.y * baseWidth * 0.5f};
  const Vector2 b{c.x - normal.x * baseWidth * 0.5f,
                  c.y - normal.y * baseWidth * 0.5f};
  DrawTriangle(a, b, point, color);
}

void DrawDetailedCue(Vector2 cueBall, Vector2 aimDir, float ballR,
                     float power, const View &view) {
  const Vector2 forward = Vector2Normalize(aimDir);
  const Vector2 normal{-forward.y, forward.x};
  const float cueLength = static_cast<float>(1.18 * view.scale);
  const float pullback = static_cast<float>((10.0 + power * 118.0) * view.uiScale);
  const float tipGap = (4.0f + pullback);
  const Vector2 tip{cueBall.x - forward.x * (ballR + tipGap),
                    cueBall.y - forward.y * (ballR + tipGap)};
  const Vector2 butt{tip.x - forward.x * cueLength,
                     tip.y - forward.y * cueLength};

  const float tipW =
      std::max(8.2f * view.uiScale, static_cast<float>(0.020 * view.scale));
  const float buttW = std::max(
      std::max(tipW * 2.45f, static_cast<float>(0.056 * view.scale)),
      20.0f * view.uiScale);
  const float collarW = LerpF(buttW, tipW, 0.40f);
  const float shaftTipW = tipW * 1.02f;

  const Vector2 castShadow{2.6f * view.uiScale, 3.1f * view.uiScale};
  DrawTaperedSection({butt.x + castShadow.x, butt.y + castShadow.y},
                     {tip.x + castShadow.x, tip.y + castShadow.y},
                     normal, buttW * 1.16f, tipW * 1.16f, {0, 0, 0, 86});
  DrawTaperedSection({butt.x + castShadow.x * 0.45f, butt.y + castShadow.y * 0.45f},
                     {tip.x + castShadow.x * 0.45f, tip.y + castShadow.y * 0.45f},
                     normal, buttW * 1.06f, tipW * 1.06f, {0, 0, 0, 112});
  DrawTaperedSection(butt, tip, normal, buttW * 1.04f, tipW * 1.04f,
                     {7, 8, 7, 255});

  DrawTaperedSection(PointAlong(butt, tip, 0.00f), PointAlong(butt, tip, 0.035f),
                     normal, buttW * 0.96f, buttW * 0.92f, {12, 12, 11, 255});
  DrawCueRing(butt, tip, normal, 0.035f, 0.048f, buttW, tipW,
              {176, 166, 136, 255});
  DrawTaperedSection(PointAlong(butt, tip, 0.048f), PointAlong(butt, tip, 0.315f),
                     normal, LerpF(buttW, tipW, 0.048f),
                     LerpF(buttW, tipW, 0.315f), {33, 22, 15, 255});
  DrawCueRing(butt, tip, normal, 0.315f, 0.328f, buttW, tipW,
              {201, 184, 139, 255});
  DrawCueRing(butt, tip, normal, 0.338f, 0.346f, buttW, tipW,
              {96, 63, 37, 255});
  DrawCueRing(butt, tip, normal, 0.356f, 0.371f, buttW, tipW,
              {192, 179, 148, 255});

  DrawTaperedSection(PointAlong(butt, tip, 0.371f), PointAlong(butt, tip, 0.955f),
                     normal, collarW, shaftTipW, {214, 171, 104, 255});

  DrawLineEx({PointAlong(butt, tip, 0.060f).x - normal.x * buttW * 0.22f,
              PointAlong(butt, tip, 0.060f).y - normal.y * buttW * 0.22f},
             {PointAlong(butt, tip, 0.305f).x - normal.x * buttW * 0.16f,
              PointAlong(butt, tip, 0.305f).y - normal.y * buttW * 0.16f},
             std::max(1.0f, buttW * 0.13f), {10, 7, 5, 255});
  DrawLineEx({PointAlong(butt, tip, 0.385f).x - normal.x * collarW * 0.20f,
              PointAlong(butt, tip, 0.385f).y - normal.y * collarW * 0.20f},
             {PointAlong(butt, tip, 0.940f).x - normal.x * shaftTipW * 0.24f,
              PointAlong(butt, tip, 0.940f).y - normal.y * shaftTipW * 0.24f},
             std::max(1.0f, shaftTipW * 0.20f), {138, 91, 44, 255});
  DrawLineEx({PointAlong(butt, tip, 0.390f).x + normal.x * collarW * 0.18f,
              PointAlong(butt, tip, 0.390f).y + normal.y * collarW * 0.18f},
             {PointAlong(butt, tip, 0.935f).x + normal.x * shaftTipW * 0.22f,
              PointAlong(butt, tip, 0.935f).y + normal.y * shaftTipW * 0.22f},
             std::max(1.0f, shaftTipW * 0.14f), {246, 219, 162, 255});

  const float inlayW = buttW * 0.28f;
  DrawCueInlay(butt, tip, normal, 0.105f, 0.365f, inlayW, -buttW * 0.28f,
               {190, 133, 63, 255});
  DrawCueInlay(butt, tip, normal, 0.145f, 0.390f, inlayW * 0.9f, 0.0f,
               {226, 176, 92, 255});
  DrawCueInlay(butt, tip, normal, 0.185f, 0.382f, inlayW, buttW * 0.28f,
               {190, 133, 63, 255});
  DrawCueInlay(butt, tip, normal, 0.225f, 0.405f, inlayW * 0.72f, 0.0f,
               {62, 38, 23, 255});

  for (int i = 0; i < 3; ++i) {
    const float offset = (-0.18f + i * 0.18f) * collarW;
    const Vector2 a = PointAlong(butt, tip, 0.40f);
    const Vector2 b = PointAlong(butt, tip, 0.93f);
    DrawLineEx({a.x + normal.x * offset, a.y + normal.y * offset},
               {b.x + normal.x * offset * 0.48f, b.y + normal.y * offset * 0.48f},
               0.85f * view.uiScale, {111, 71, 31, 115});
  }

  DrawCueRing(butt, tip, normal, 0.955f, 0.979f, buttW, tipW,
              {215, 211, 196, 255});
  DrawCueRing(butt, tip, normal, 0.979f, 1.0f, buttW, tipW,
              {80, 50, 32, 255});
  DrawLineEx(PointAlong(butt, tip, 0.41f), PointAlong(butt, tip, 0.945f),
             std::max(1.0f, shaftTipW * 0.18f), {246, 220, 164, 230});
  DrawLineEx(PointAlong(butt, tip, 0.065f), PointAlong(butt, tip, 0.30f),
             std::max(1.0f, buttW * 0.14f), {91, 57, 30, 235});
}

void DrawUI(Game &game) {
  const float s = UiScale();
  Rectangle panel = ControlPanelRect(s);
  DrawRoundedFillWithBorder(panel, 0.045f, 12, {12, 13, 14, 232},
                            {50, 58, 55, 190});

  const auto &rs = game.rules.State();
  DrawTextF(game.font, "击球控制", {panel.x + 18 * s, panel.y + 17 * s}, 27 * s,
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
  DrawTextF(game.font, "高", {center.x - 9 * s, center.y - padR - 29 * s}, 18 * s,
            {177, 186, 178, 255});
  DrawTextF(game.font, "低", {center.x - 9 * s, center.y + padR + 13 * s}, 18 * s,
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

  Rectangle msg{48.0f * s, static_cast<float>(GetScreenHeight()) - 58.0f * s,
                static_cast<float>(GetScreenWidth()) - 96.0f * s, 34.0f * s};
  DrawRectangleRounded(msg, 0.08f, 8, {9, 10, 10, 210});
  DrawTextF(game.font, rs.message.c_str(), {msg.x + 14 * s, msg.y + 6 * s}, 22 * s, {221, 224, 216, 255});

  if (game.phase == Phase::GroupChoice) {
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
  const float s = 8.0f;
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

void DrawTitleBar(Game &game) {
  const float w = static_cast<float>(GetScreenWidth());
  const Vector2 mouse = GetMousePosition();
  const Rectangle helpR = HelpButtonRect();
  const Rectangle titleR{12.0f, 0.0f, 84.0f, 34.0f};
  const Rectangle modeR{98.0f, 0.0f, 86.0f, 34.0f};
  const Rectangle minR{w - 129.0f, 0.0f, 42.0f, 34.0f};
  const Rectangle maxR{w - 86.0f, 0.0f, 42.0f, 34.0f};
  const Rectangle closeR{w - 43.0f, 0.0f, 42.0f, 34.0f};
  Rectangle bar{0.0f, 0.0f, w, 34.0f};
  DrawRectangleRec(bar, BLACK);
  DrawRectangle(0, 33, GetScreenWidth(), 1, {50, 57, 55, 120});
  DrawTitleTextItem(game.font, titleR, "中式八球", 19.0f,
                    {218, 224, 216, 255}, false);
  DrawTitleTextItem(game.font, modeR, "双人对战", 17.0f,
                    {142, 154, 147, 230}, false);
  DrawTitleTextItem(game.font, helpR, "帮助", 18.0f,
                    {198, 204, 197, 235},
                    game.helpOpen || PointInRect(mouse, helpR));
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
  TitleButton(closeR, "×", game.font, PointInRect(GetMousePosition(), closeR));

  const char *lines[] = {
      "规则：中式八球，两名玩家轮流击球。",
      "先合法打完自己的全色球或半色球，再打进8号球获胜。",
      "犯规后对手获得自由球；开局或未定组时按规则自动判定。",
      "操作：移动鼠标瞄准，滚轮下滑蓄力，滚轮上滑出杆。",
      "右侧击点盘调整高低杆和左右塞；还原按钮回到中杆。",
      "自由球时移动鼠标选择白球位置，左键确认放置。",
      "Esc 清空力度，C 还原击点，空格开始下一局。"};
  float y = panel.y + 56.0f * s;
  for (const char *line : lines) {
    DrawTextF(game.font, line, {panel.x + 20.0f * s, y}, 18.0f * s,
              {207, 216, 207, 245});
    y += 32.0f * s;
  }
}

bool HandleTitleBarInput(Game &game) {
  const Vector2 mouse = GetMousePosition();
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
  const Rectangle minR{w - 129.0f, 0.0f, 42.0f, 34.0f};
  const Rectangle maxR{w - 86.0f, 0.0f, 42.0f, 34.0f};
  const Rectangle closeR{w - 43.0f, 0.0f, 42.0f, 34.0f};
  const Rectangle dragR{0.0f, 0.0f, w - 131.0f, 34.0f};

  if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
    if (hit != ResizeNone && !PointInRect(mouse, minR) &&
        !PointInRect(mouse, maxR) && !PointInRect(mouse, closeR) &&
        !PointInRect(mouse, helpR)) {
      game.resizeMode = hit;
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
    if (PointInRect(mouse, dragR)) {
      game.draggingTitle = true;
      return true;
    }
  }
  if (IsMouseButtonReleased(MOUSE_LEFT_BUTTON)) {
    game.draggingTitle = false;
    game.resizeMode = ResizeNone;
  }
  if (game.resizeMode != ResizeNone && IsMouseButtonDown(MOUSE_LEFT_BUTTON)) {
    const Vector2 delta = GetMouseDelta();
    Vector2 pos = GetWindowPosition();
    int newW = GetScreenWidth();
    int newH = GetScreenHeight();
    if (game.resizeMode & ResizeLeft) {
      const int proposed = newW - static_cast<int>(std::round(delta.x));
      if (proposed >= minW) {
        newW = proposed;
        pos.x += delta.x;
      }
    }
    if (game.resizeMode & ResizeRight) {
      newW = std::max(minW, newW + static_cast<int>(std::round(delta.x)));
    }
    if (game.resizeMode & ResizeTop) {
      const int proposed = newH - static_cast<int>(std::round(delta.y));
      if (proposed >= minH) {
        newH = proposed;
        pos.y += delta.y;
      }
    }
    if (game.resizeMode & ResizeBottom) {
      newH = std::max(minH, newH + static_cast<int>(std::round(delta.y)));
    }
    SetWindowPosition(static_cast<int>(pos.x), static_cast<int>(pos.y));
    SetWindowSize(newW, newH);
    return true;
  }
  if (game.draggingTitle && IsMouseButtonDown(MOUSE_LEFT_BUTTON)) {
    const Vector2 delta = GetMouseDelta();
    const Vector2 pos = GetWindowPosition();
    SetWindowPosition(static_cast<int>(pos.x + delta.x),
                      static_cast<int>(pos.y + delta.y));
    return true;
  }
  return PointInRect(mouse, {0.0f, 0.0f, w, 34.0f});
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
  if (IsKeyPressed(KEY_ESCAPE)) {
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
  } else if (game.phase == Phase::RackOver && IsKeyPressed(KEY_SPACE)) {
    const int nextBreaker = game.rules.State().winner >= 0 ? game.rules.State().winner : 0;
    game.world.ResetRack();
    game.rules.ResetRack(nextBreaker);
    game.phase = Phase::Aiming;
  }

  if (IsKeyPressed(KEY_C)) {
    game.tipX = 0.0;
    game.tipY = 0.0;
  }
}

void DrawGame(Game &game, const View &view) {
  ClearBackground(BLACK);
  DrawTitleBar(game);
  DrawTopStatus(game);
  DrawFpsCounter(game.font);
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
  DrawCueAndAim(game, view);
  if (game.phase == Phase::BallInHand) {
    const Vector2 p = WorldToScreen(view, game.world.CueBall().pos);
    DrawCircleLines(static_cast<int>(p.x), static_cast<int>(p.y),
                    static_cast<float>(hb::kBallRadius * view.scale * 1.45),
                    {236, 238, 228, 150});
  }
  DrawUI(game);
  DrawHelpWindow(game);

  if (game.phase == Phase::RackOver) {
    Rectangle r{static_cast<float>(GetScreenWidth()) * 0.5f - 190.0f,
                static_cast<float>(GetScreenHeight()) * 0.5f - 45.0f, 380.0f, 90.0f};
    DrawRoundedFillWithBorder(r, 0.05f, 12, {11, 12, 12, 240},
                              {70, 82, 76, 210});
    DrawTextF(game.font, game.rules.State().message.c_str(), {r.x + 35, r.y + 15}, 28,
              {236, 238, 228, 255});
    DrawTextF(game.font, "按空格键开始下一局", {r.x + 63, r.y + 52}, 22,
              {180, 190, 182, 255});
  }
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
  InitWindow(1360, 820, "中式八球");
  SetExitKey(KEY_NULL);
  SetWindowMinSize(960, 600);
  SetTargetFPS(240);

  Game game;
  std::vector<int> codepoints = BuildFontCodepoints();
  game.font = LoadPingFangFont(codepoints);
  game.customFont = game.font.texture.id != 0;
  if (game.customFont) {
    ConfigureFontTexture(game.font);
  }
  if (!game.customFont) {
    game.font = GetFontDefault();
  }

  while (!WindowShouldClose() && !game.requestClose) {
    const View view = MakeView(GetScreenWidth(), GetScreenHeight());
    UpdateGame(game, view);
    BeginDrawing();
    DrawGame(game, view);
    EndDrawing();
  }

  if (game.customFont) {
    UnloadFont(game.font);
  }
  CloseWindow();
  return 0;
}
