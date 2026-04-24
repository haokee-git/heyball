#include "core.hpp"

#include <raylib.h>
#include <raymath.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
#ifdef _WIN32
extern "C" __declspec(dllimport) int __stdcall AllocConsole(void);
#endif

namespace {

using hb::Ball;
using hb::BallGroup;
using hb::Phase;
using hb::ShotEvents;
using hb::ShotParams;
using hb::Vec2;

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
  return codepoints;
}

void DrawTextF(Font font, const char *text, Vector2 pos, float size, Color color) {
  Color soft = color;
  soft.a = static_cast<unsigned char>(std::min(255, static_cast<int>(soft.a) * 3 / 4));
  DrawTextEx(font, text, {pos.x + 0.55f, pos.y}, size, 0.25f, soft);
  DrawTextEx(font, text, pos, size, 0.25f, color);
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

Vector3 RotateX(Vector3 v, float a) {
  const float c = std::cos(a);
  const float s = std::sin(a);
  return {v.x, v.y * c - v.z * s, v.y * s + v.z * c};
}

Vector3 RotateY(Vector3 v, float a) {
  const float c = std::cos(a);
  const float s = std::sin(a);
  return {v.x * c + v.z * s, v.y, -v.x * s + v.z * c};
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
      Vector3 local = RotateY(RotateX(normal, static_cast<float>(ball.decal.y) * 0.65f),
                              -static_cast<float>(ball.decal.x) * 0.65f);
      Color material = ball.number == 0 ? ivory : base;
      if (stripe) {
        const float stripeWidth = 0.40f;
        const float feather = 0.045f;
        const float band = std::abs(local.y);
        if (band > stripeWidth + feather) {
          material = ivory;
        } else if (band > stripeWidth) {
          const float t = (band - stripeWidth) / feather;
          material = {
              static_cast<unsigned char>(base.r * (1.0f - t) + ivory.r * t),
              static_cast<unsigned char>(base.g * (1.0f - t) + ivory.g * t),
              static_cast<unsigned char>(base.b * (1.0f - t) + ivory.b * t), 255};
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
    const float sx = static_cast<float>(std::sin(ball.decal.x));
    const float sy = static_cast<float>(std::sin(ball.decal.y));
    const float cx = static_cast<float>(std::cos(ball.decal.x));
    const float cy = static_cast<float>(std::cos(ball.decal.y));
    const float normalZ = cx * cy;
    const float visibility = std::max(0.0f, std::min(1.0f, normalZ * 0.85f + 0.25f));
    const float decalR = r * (0.30f + 0.14f * visibility);
    const Vector2 d{c.x + sx * r * 0.47f, c.y - sy * r * 0.47f};
    DrawCircleV(d, decalR,
                {244, 241, 224,
                 static_cast<unsigned char>(245 * visibility * visualAlpha)});
    char label[4]{};
    std::snprintf(label, sizeof(label), "%d", ball.number);
    const float fs = (ball.number >= 10 ? r * 0.54f : r * 0.66f) * (0.76f + 0.24f * visibility);
    const Vector2 sz = MeasureTextEx(font, label, fs, 0.0f);
    const float rotation =
        static_cast<float>(std::fmod((ball.decal.x - ball.decal.y) * 42.0, 360.0));
    DrawTextPro(font, label, {d.x, d.y}, {sz.x * 0.5f, sz.y * 0.5f}, rotation, fs,
                0.0f,
                {15, 15, 15,
                 static_cast<unsigned char>(255 * visibility * visualAlpha)});
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
  DrawCircleV(WorldToScreen(view, {-hb::kTableWidth * 0.25, 0.0}), spotR,
              {210, 220, 205, 85});
  DrawCircleV(WorldToScreen(view, {hb::kTableWidth * 0.25, 0.0}), spotR,
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
  DrawRectangleRounded(r, 0.12f, 8, active ? Color{59, 79, 73, 255} : Color{27, 30, 31, 255});
  DrawRectangleRoundedLines(r, 0.12f, 8, active ? Color{141, 166, 151, 180} : Color{91, 98, 96, 140});
  const float fs = std::max(12.0f, std::min(18.0f, r.height * 0.64f));
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
  const float fs = 22.0f * s;
  const Vector2 sz = MeasureTextEx(game.font, line, fs, 0.0f);
  const Vector2 pos{(GetScreenWidth() - sz.x) * 0.5f, 47.0f * s};
  DrawTextF(game.font, line, pos, fs, {230, 234, 224, 235});
  DrawLineEx({pos.x, pos.y + sz.y + 3.0f * s},
             {pos.x + sz.x, pos.y + sz.y + 3.0f * s},
             1.0f, {93, 112, 101, 130});
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
  const float cueLength = static_cast<float>(1.47 * view.scale);
  const float pullback = static_cast<float>((10.0 + power * 118.0) * view.uiScale);
  const float tipGap = (4.0f + pullback);
  const Vector2 tip{cueBall.x - forward.x * (ballR + tipGap),
                    cueBall.y - forward.y * (ballR + tipGap)};
  const Vector2 butt{tip.x - forward.x * cueLength,
                     tip.y - forward.y * cueLength};

  const float tipW = std::max(3.0f, static_cast<float>(0.0108 * view.scale));
  const float buttW = std::max(tipW * 2.25f, static_cast<float>(0.029 * view.scale));
  const float collarW = LerpF(buttW, tipW, 0.40f);
  const float shaftTipW = tipW * 0.92f;

  DrawLineEx({butt.x + 2.0f * view.uiScale, butt.y + 2.0f * view.uiScale},
             {tip.x + 2.0f * view.uiScale, tip.y + 2.0f * view.uiScale},
             buttW * 1.05f, {0, 0, 0, 95});

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

  const float inlayW = buttW * 0.28f;
  DrawCueInlay(butt, tip, normal, 0.105f, 0.365f, inlayW, -buttW * 0.28f,
               {190, 133, 63, 230});
  DrawCueInlay(butt, tip, normal, 0.145f, 0.390f, inlayW * 0.9f, 0.0f,
               {226, 176, 92, 235});
  DrawCueInlay(butt, tip, normal, 0.185f, 0.382f, inlayW, buttW * 0.28f,
               {190, 133, 63, 230});
  DrawCueInlay(butt, tip, normal, 0.225f, 0.405f, inlayW * 0.72f, 0.0f,
               {62, 38, 23, 240});

  for (int i = 0; i < 5; ++i) {
    const float offset = (-0.30f + i * 0.15f) * collarW;
    const Vector2 a = PointAlong(butt, tip, 0.40f);
    const Vector2 b = PointAlong(butt, tip, 0.93f);
    DrawLineEx({a.x + normal.x * offset, a.y + normal.y * offset},
               {b.x + normal.x * offset * 0.48f, b.y + normal.y * offset * 0.48f},
               0.8f * view.uiScale, {111, 71, 31, 70});
  }

  DrawCueRing(butt, tip, normal, 0.955f, 0.979f, buttW, tipW,
              {215, 211, 196, 255});
  DrawCueRing(butt, tip, normal, 0.979f, 1.0f, buttW, tipW,
              {80, 50, 32, 255});
  DrawLineEx(PointAlong(butt, tip, 0.41f), PointAlong(butt, tip, 0.945f),
             std::max(1.0f, shaftTipW * 0.22f), {246, 220, 164, 120});
  DrawLineEx(PointAlong(butt, tip, 0.065f), PointAlong(butt, tip, 0.30f),
             std::max(1.0f, buttW * 0.16f), {91, 57, 30, 150});
}

void DrawUI(Game &game) {
  const float s = static_cast<float>(
      hb::Clamp(std::min(GetScreenWidth() / 1360.0, GetScreenHeight() / 820.0),
                0.72, 1.55));
  const int sw = GetScreenWidth();
  Rectangle panel{static_cast<float>(sw) - 248.0f * s, 46.0f * s,
                  214.0f * s, 332.0f * s};
  DrawRectangleRounded(panel, 0.045f, 12, {12, 13, 14, 232});
  DrawRectangleRoundedLines(panel, 0.045f, 12, {91, 101, 98, 140});

  const auto &rs = game.rules.State();
  DrawTextF(game.font, "击球控制", {panel.x + 18 * s, panel.y + 18 * s}, 23 * s,
            {236, 238, 228, 255});

  DrawTextF(game.font, "击点", {panel.x + 18 * s, panel.y + 58 * s}, 18 * s,
            {188, 196, 188, 255});
  const Vector2 center{panel.x + 107 * s, panel.y + 135 * s};
  const float padR = 54.0f * s;
  DrawCircleV(center, padR, {25, 29, 29, 255});
  DrawCircleLines(static_cast<int>(center.x), static_cast<int>(center.y), padR, {113, 128, 121, 180});
  DrawLineEx({center.x - padR, center.y}, {center.x + padR, center.y}, 1.0f, {110, 119, 115, 125});
  DrawLineEx({center.x, center.y - padR}, {center.x, center.y + padR}, 1.0f, {110, 119, 115, 125});
  DrawCircleV({center.x + static_cast<float>(game.tipX * padR),
               center.y - static_cast<float>(game.tipY * padR)},
              7.0f, {228, 232, 218, 255});
  DrawTextF(game.font, "左", {center.x - padR - 20 * s, center.y - 9 * s}, 15 * s,
            {150, 159, 152, 255});
  DrawTextF(game.font, "右", {center.x + padR + 8 * s, center.y - 9 * s}, 15 * s,
            {150, 159, 152, 255});
  DrawTextF(game.font, "高", {center.x - 8 * s, center.y - padR - 23 * s}, 15 * s,
            {150, 159, 152, 255});
  DrawTextF(game.font, "低", {center.x - 8 * s, center.y + padR + 7 * s}, 15 * s,
            {150, 159, 152, 255});

  Button(game.font, {panel.x + 18 * s, panel.y + 205 * s, 178 * s, 28 * s},
         "还原", true);

  char powerText[64]{};
  std::snprintf(powerText, sizeof(powerText), "力度 %3d%%", static_cast<int>(game.power * 100.0));
  DrawTextF(game.font, powerText, {panel.x + 18 * s, panel.y + 250 * s}, 18 * s,
            {213, 218, 209, 255});
  DrawRectangleRounded({panel.x + 18 * s, panel.y + 278 * s, 178 * s, 12 * s}, 0.4f, 8, {28, 31, 31, 255});
  DrawRectangleRounded({panel.x + 18 * s, panel.y + 278 * s, static_cast<float>(178.0 * game.power * s), 12 * s},
                       0.4f, 8, {143, 170, 151, 255});

  Button(game.font, {panel.x + 18 * s, panel.y + 304 * s, 178 * s, 28 * s},
         "重新摆球", true);

  Rectangle msg{48.0f * s, static_cast<float>(GetScreenHeight()) - 58.0f * s,
                static_cast<float>(GetScreenWidth()) - 96.0f * s, 34.0f * s};
  DrawRectangleRounded(msg, 0.08f, 8, {9, 10, 10, 210});
  DrawTextF(game.font, rs.message.c_str(), {msg.x + 14 * s, msg.y + 8 * s}, 18 * s, {221, 224, 216, 255});

  if (game.phase == Phase::GroupChoice) {
    Rectangle modal{static_cast<float>(GetScreenWidth()) * 0.5f - 170.0f,
                    static_cast<float>(GetScreenHeight()) * 0.5f - 62.0f, 340.0f, 124.0f};
    DrawRectangleRounded(modal, 0.04f, 12, {13, 15, 15, 245});
    DrawRectangleRoundedLines(modal, 0.04f, 12, {130, 142, 136, 190});
    DrawTextF(game.font, "选择球组", {modal.x + 114, modal.y + 20}, 22, {236, 238, 228, 255});
    Button(game.font, {modal.x + 34, modal.y + 70, 126, 32}, "全色球", true);
    Button(game.font, {modal.x + 180, modal.y + 70, 126, 32}, "半色球", true);
  }
}

Rectangle TitleButton(float x, const char *label, Font font, bool hot) {
  Rectangle r{x, 0.0f, 42.0f, 34.0f};
  DrawRectangleRec(r, hot ? Color{30, 33, 33, 255} : Color{0, 0, 0, 0});
  const float fs = 20.0f;
  const Vector2 sz = MeasureTextEx(font, label, fs, 0.0f);
  DrawTextF(font, label, {r.x + (r.width - sz.x) * 0.5f, r.y + 6.0f}, fs,
            {198, 204, 197, 235});
  return r;
}

void DrawTitleBar(Game &game) {
  const float w = static_cast<float>(GetScreenWidth());
  const Vector2 mouse = GetMousePosition();
  Rectangle bar{0.0f, 0.0f, w, 34.0f};
  DrawRectangleRec(bar, BLACK);
  DrawRectangle(0, 33, GetScreenWidth(), 1, {50, 57, 55, 120});
  DrawTextF(game.font, "中式八球", {18.0f, 7.0f}, 18.0f, {207, 214, 207, 220});
  DrawTextF(game.font, "双人对战", {100.0f, 8.0f}, 16.0f,
            {117, 128, 122, 210});
  const Rectangle minR = TitleButton(w - 86.0f, "-", game.font,
                                     PointInRect(mouse, {w - 86.0f, 0.0f, 42.0f, 34.0f}));
  const Rectangle closeR = TitleButton(w - 43.0f, "×", game.font,
                                       PointInRect(mouse, {w - 43.0f, 0.0f, 42.0f, 34.0f}));
  (void)minR;
  (void)closeR;
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

  const Rectangle minR{w - 86.0f, 0.0f, 42.0f, 34.0f};
  const Rectangle closeR{w - 43.0f, 0.0f, 42.0f, 34.0f};
  const Rectangle dragR{0.0f, 0.0f, w - 88.0f, 34.0f};

  if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
    if (hit != ResizeNone && !PointInRect(mouse, minR) && !PointInRect(mouse, closeR)) {
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

void HandleUiInput(Game &game) {
  const float s = static_cast<float>(
      hb::Clamp(std::min(GetScreenWidth() / 1360.0, GetScreenHeight() / 820.0),
                0.72, 1.55));
  const Vector2 mouse = GetMousePosition();
  const int sw = GetScreenWidth();
  Rectangle panel{static_cast<float>(sw) - 248.0f * s, 46.0f * s,
                  214.0f * s, 332.0f * s};
  const Vector2 tipCenter{panel.x + 107 * s, panel.y + 135 * s};
  const float tipR = 54.0f * s;

  if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
    if (Vector2Distance(mouse, tipCenter) <= tipR) {
      game.tipX = hb::Clamp((mouse.x - tipCenter.x) / tipR, -1.0, 1.0);
      game.tipY = hb::Clamp(-(mouse.y - tipCenter.y) / tipR, -1.0, 1.0);
    }
    if (PointInRect(mouse, {panel.x + 18 * s, panel.y + 205 * s, 178 * s, 28 * s})) {
      game.tipX = 0.0;
      game.tipY = 0.0;
    }
    if (PointInRect(mouse, {panel.x + 18 * s, panel.y + 304 * s, 178 * s, 28 * s})) {
      game.world.ResetRack();
      game.rules.ResetRack(1 - game.rules.State().currentPlayer);
      game.phase = Phase::Aiming;
      game.power = 0.0;
    }
    if (game.phase == Phase::GroupChoice) {
      Rectangle modal{static_cast<float>(GetScreenWidth()) * 0.5f - 170.0f,
                      static_cast<float>(GetScreenHeight()) * 0.5f - 62.0f, 340.0f, 124.0f};
      if (PointInRect(mouse, {modal.x + 34, modal.y + 70, 126, 32})) {
        game.phase = game.rules.ChooseGroup(BallGroup::Solids).nextPhase;
      } else if (PointInRect(mouse, {modal.x + 180, modal.y + 70, 126, 32})) {
        game.phase = game.rules.ChooseGroup(BallGroup::Stripes).nextPhase;
      }
    }
  }
}

void UpdateGame(Game &game, const View &view) {
  if (HandleTitleBarInput(game)) {
    return;
  }
  HandleUiInput(game);
  const float dt = GetFrameTime();

  if (game.phase == Phase::Aiming && !game.world.CueBall().pocketed &&
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
    Vec2 pos = ScreenToWorld(view, GetMousePosition());
    if (game.world.CanPlaceCue(pos)) {
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
        game.world.PlaceCue({-hb::kTableWidth * 0.30, 0.0});
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

  if (game.phase == Phase::RackOver) {
    Rectangle r{static_cast<float>(GetScreenWidth()) * 0.5f - 190.0f,
                static_cast<float>(GetScreenHeight()) * 0.5f - 45.0f, 380.0f, 90.0f};
    DrawRectangleRounded(r, 0.05f, 12, {11, 12, 12, 240});
    DrawRectangleRoundedLines(r, 0.05f, 12, {128, 142, 134, 170});
    DrawTextF(game.font, game.rules.State().message.c_str(), {r.x + 42, r.y + 20}, 24,
              {236, 238, 228, 255});
    DrawTextF(game.font, "按空格键开始下一局", {r.x + 74, r.y + 54}, 18,
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
  SetWindowMinSize(960, 600);
  SetTargetFPS(240);

  Game game;
  std::vector<int> codepoints = BuildFontCodepoints();
  game.font = LoadFontEx("C:/Windows/Fonts/Dengb.ttf", 96, codepoints.data(),
                         static_cast<int>(codepoints.size()));
  game.customFont = game.font.texture.id != 0;
  if (game.customFont) {
    SetTextureFilter(game.font.texture, TEXTURE_FILTER_BILINEAR);
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
