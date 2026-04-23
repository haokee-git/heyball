#include "core.hpp"

#include <raylib.h>
#include <raymath.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
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
  double shotClock = 45.0;
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
  const float uiReserve = 260.0f * uiScale;
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
  const double left = -hb::kTableWidth * 0.5;
  const double right = hb::kTableWidth * 0.5;
  const double top = -hb::kTableHeight * 0.5;
  const double bottom = hb::kTableHeight * 0.5;
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

void DrawBezierQuad(Vector2 start, Vector2 end, Vector2 control, float thick, Color color) {
  constexpr int segments = 18;
  Vector2 prev = start;
  for (int i = 1; i <= segments; ++i) {
    const float t = static_cast<float>(i) / segments;
    const float u = 1.0f - t;
    Vector2 p{u * u * start.x + 2.0f * u * t * control.x + t * t * end.x,
              u * u * start.y + 2.0f * u * t * control.y + t * t * end.y};
    DrawLineEx(prev, p, thick, color);
    prev = p;
  }
}

void DrawBall(Font font, const View &view, const Ball &ball) {
  if (ball.pocketed && ball.pocketFade >= 1.0) {
    return;
  }
  const Vector2 c = WorldToScreen(view, ball.pos);
  float r = static_cast<float>(hb::kBallRadius * view.scale);
  if (ball.pocketed) {
    r *= static_cast<float>(1.0 - ball.pocketFade * 0.7);
  }
  DrawCircleV({c.x + r * 0.18f, c.y + r * 0.22f}, r * 1.02f, {0, 0, 0, 115});

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
      DrawPixel(px, py, ShadeBallPixel(material, normal, edgeAlpha));
    }
  }
  DrawCircleV({c.x - r * 0.32f, c.y - r * 0.34f}, r * 0.18f, {255, 255, 255, 110});
  DrawCircleLines(static_cast<int>(c.x), static_cast<int>(c.y), r, {25, 25, 25, 150});

  if (ball.number > 0) {
    const float sx = static_cast<float>(std::sin(ball.decal.x));
    const float sy = static_cast<float>(std::sin(ball.decal.y));
    const float cx = static_cast<float>(std::cos(ball.decal.x));
    const float cy = static_cast<float>(std::cos(ball.decal.y));
    const float normalZ = cx * cy;
    const float visibility = std::max(0.0f, std::min(1.0f, normalZ * 0.85f + 0.25f));
    const float decalR = r * (0.30f + 0.14f * visibility);
    const Vector2 d{c.x + sx * r * 0.47f, c.y - sy * r * 0.47f};
    DrawCircleV(d, decalR, {244, 241, 224, static_cast<unsigned char>(245 * visibility)});
    char label[4]{};
    std::snprintf(label, sizeof(label), "%d", ball.number);
    const float fs = (ball.number >= 10 ? r * 0.54f : r * 0.66f) * (0.76f + 0.24f * visibility);
    const Vector2 sz = MeasureTextEx(font, label, fs, 0.0f);
    const float rotation =
        static_cast<float>(std::fmod((ball.decal.x - ball.decal.y) * 42.0, 360.0));
    DrawTextPro(font, label, {d.x, d.y}, {sz.x * 0.5f, sz.y * 0.5f}, rotation, fs,
                0.0f, {15, 15, 15, static_cast<unsigned char>(255 * visibility)});
  }
}

void DrawRailSegment(Rectangle r, bool horizontal) {
  DrawRectangleRec(r, {43, 38, 31, 255});
  if (horizontal) {
    DrawLineEx({r.x, r.y + r.height * 0.20f}, {r.x + r.width, r.y + r.height * 0.20f},
               2.0f, {76, 63, 48, 180});
    DrawLineEx({r.x, r.y + r.height - 2.0f}, {r.x + r.width, r.y + r.height - 2.0f},
               2.0f, {20, 22, 20, 180});
  } else {
    DrawLineEx({r.x + r.width * 0.20f, r.y}, {r.x + r.width * 0.20f, r.y + r.height},
               2.0f, {76, 63, 48, 180});
    DrawLineEx({r.x + r.width - 2.0f, r.y}, {r.x + r.width - 2.0f, r.y + r.height},
               2.0f, {20, 22, 20, 180});
  }
}

void DrawPocketShape(const View &view, int index, Vector2 p) {
  const bool side = index == 1 || index == 4;
  const double mouth = side ? hb::kSidePocketMouth : hb::kCornerPocketMouth;
  const float mouthPx = static_cast<float>(mouth * view.scale);
  const float pocketR = mouthPx * 0.50f;
  const float jaw = std::max(5.0f, mouthPx * 0.18f);
  const float curve = std::max(12.0f, mouthPx * 0.42f);
  Color hole{0, 0, 0, 255};
  Color facing{13, 18, 16, 255};
  Color rubber{24, 31, 27, 255};

  if (side) {
    const float sy = (index == 1) ? -1.0f : 1.0f;
    const Vector2 leftJaw{p.x - mouthPx * 0.64f, p.y};
    const Vector2 rightJaw{p.x + mouthPx * 0.64f, p.y};
    const Vector2 leftBack{p.x - mouthPx * 0.34f, p.y + sy * curve};
    const Vector2 rightBack{p.x + mouthPx * 0.34f, p.y + sy * curve};
    DrawCircleV({p.x, p.y + sy * curve * 0.25f}, pocketR, hole);
    DrawBezierQuad(leftJaw, leftBack, {p.x - mouthPx * 0.54f, p.y + sy * curve * 0.28f},
                   jaw, rubber);
    DrawBezierQuad(rightJaw, rightBack, {p.x + mouthPx * 0.54f, p.y + sy * curve * 0.28f},
                   jaw, rubber);
    DrawLineEx(leftJaw, {leftJaw.x - mouthPx * 0.14f, leftJaw.y - sy * mouthPx * 0.06f},
               jaw, facing);
    DrawLineEx(rightJaw, {rightJaw.x + mouthPx * 0.14f, rightJaw.y - sy * mouthPx * 0.06f},
               jaw, facing);
  } else {
    const float sx = (index == 0 || index == 3) ? -1.0f : 1.0f;
    const float sy = (index <= 2) ? -1.0f : 1.0f;
    const Vector2 jawX{p.x - sx * mouthPx * 0.68f, p.y};
    const Vector2 jawY{p.x, p.y - sy * mouthPx * 0.68f};
    const Vector2 back{p.x + sx * curve * 0.36f, p.y + sy * curve * 0.36f};
    DrawCircleV({p.x + sx * curve * 0.13f, p.y + sy * curve * 0.13f}, pocketR, hole);
    DrawBezierQuad(jawX, back, {p.x + sx * curve * 0.05f, p.y - sy * mouthPx * 0.38f},
                   jaw, rubber);
    DrawBezierQuad(jawY, back, {p.x - sx * mouthPx * 0.38f, p.y + sy * curve * 0.05f},
                   jaw, rubber);
    DrawLineEx(jawX, {jawX.x - sx * mouthPx * 0.10f, jawX.y - sy * mouthPx * 0.04f},
               jaw, facing);
    DrawLineEx(jawY, {jawY.x - sx * mouthPx * 0.04f, jawY.y - sy * mouthPx * 0.10f},
               jaw, facing);
  }
}

void DrawTable(const View &view) {
  const float railW = 30.0f * view.uiScale;
  const float sideGap = static_cast<float>(hb::kSidePocketMouth * view.scale * 1.55f);
  const float cornerGap = static_cast<float>(hb::kCornerPocketMouth * view.scale * 1.28f);
  Rectangle rail{view.play.x - railW, view.play.y - railW, view.play.width + railW * 2,
                 view.play.height + railW * 2};
  DrawRectangleRounded(rail, 0.035f, 16, {21, 23, 22, 255});
  DrawRectangleRounded({rail.x + 10.0f * view.uiScale, rail.y + 10.0f * view.uiScale,
                        rail.width - 20.0f * view.uiScale, rail.height - 20.0f * view.uiScale},
                       0.028f, 16, {48, 42, 34, 255});
  DrawRectangleRec(view.play, {35, 87, 74, 255});

  const float topInnerY = view.play.y - railW * 0.60f;
  const float bottomInnerY = view.play.y + view.play.height;
  const float sideInnerW = railW * 0.60f;
  DrawRailSegment({view.play.x + cornerGap, topInnerY, view.play.width * 0.5f - sideGap * 0.5f - cornerGap, railW * 0.60f}, true);
  DrawRailSegment({view.play.x + view.play.width * 0.5f + sideGap * 0.5f, topInnerY,
                   view.play.width * 0.5f - sideGap * 0.5f - cornerGap, railW * 0.60f}, true);
  DrawRailSegment({view.play.x + cornerGap, bottomInnerY, view.play.width * 0.5f - sideGap * 0.5f - cornerGap, railW * 0.60f}, true);
  DrawRailSegment({view.play.x + view.play.width * 0.5f + sideGap * 0.5f, bottomInnerY,
                   view.play.width * 0.5f - sideGap * 0.5f - cornerGap, railW * 0.60f}, true);
  DrawRailSegment({view.play.x - sideInnerW, view.play.y + cornerGap, sideInnerW, view.play.height - cornerGap * 2}, false);
  DrawRailSegment({view.play.x + view.play.width, view.play.y + cornerGap, sideInnerW, view.play.height - cornerGap * 2}, false);
  DrawRectangleLinesEx(view.play, 1.5f, {123, 145, 129, 120});

  const Vector2 pockets[6] = {
      WorldToScreen(view, {-hb::kTableWidth * 0.5, -hb::kTableHeight * 0.5}),
      WorldToScreen(view, {0.0, -hb::kTableHeight * 0.5}),
      WorldToScreen(view, {hb::kTableWidth * 0.5, -hb::kTableHeight * 0.5}),
      WorldToScreen(view, {-hb::kTableWidth * 0.5, hb::kTableHeight * 0.5}),
      WorldToScreen(view, {0.0, hb::kTableHeight * 0.5}),
      WorldToScreen(view, {hb::kTableWidth * 0.5, hb::kTableHeight * 0.5}),
  };
  for (int i = 0; i < 6; ++i) {
    const Vector2 p = pockets[i];
    DrawPocketShape(view, i, p);
  }

  const float spotR = 3.0f;
  DrawCircleV(WorldToScreen(view, {-hb::kTableWidth * 0.25, 0.0}), spotR, {210, 220, 205, 85});
  DrawCircleV(WorldToScreen(view, {hb::kTableWidth * 0.25, 0.0}), spotR, {210, 220, 205, 85});
}

void DrawCueAndAim(const Game &game, const View &view) {
  if (game.phase != Phase::Aiming || game.world.CueBall().pocketed ||
      game.world.CueBall().sinking) {
    return;
  }
  const Vector2 cue = WorldToScreen(view, game.world.CueBall().pos);
  const Vec2 aim = hb::Normalize(game.aim);
  const Vector2 aimEnd = WorldToScreen(view, RayToTableEdge(game.world.CueBall().pos, aim));
  Vec2 ghostPos{};
  const bool hasGhost = FirstAimTarget(game.world, game.world.CueBall().pos, aim, &ghostPos) > 0;
  const Vector2 dir{static_cast<float>(aim.x), static_cast<float>(aim.y)};
  const float ballR = static_cast<float>(hb::kBallRadius * view.scale);
  const float retreat = static_cast<float>(22.0 + game.power * 118.0);
  const Vector2 tip{cue.x - dir.x * (ballR + 4.0f),
                    cue.y - dir.y * (ballR + 4.0f)};
  const Vector2 butt{cue.x - dir.x * (ballR + retreat + 260.0f),
                     cue.y - dir.y * (ballR + retreat + 260.0f)};
  DrawLineEx(cue, aimEnd, 1.0f,
             {214, 225, 210, 70});
  if (hasGhost) {
    const Vector2 ghost = WorldToScreen(view, ghostPos);
    const float ghostR = static_cast<float>(hb::kBallRadius * view.scale);
    DrawCircleLines(static_cast<int>(ghost.x), static_cast<int>(ghost.y), ghostR,
                    {235, 239, 226, 180});
    DrawCircleLines(static_cast<int>(ghost.x), static_cast<int>(ghost.y), ghostR + 1.8f,
                    {15, 17, 16, 180});
  }
  DrawLineEx(butt, tip, 5.0f, {178, 132, 78, 255});
  DrawLineEx(butt, tip, 1.5f, {236, 219, 174, 255});
  DrawCircleV(tip, 3.5f, {42, 70, 105, 255});
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

void DrawUI(Game &game) {
  const float s = static_cast<float>(
      hb::Clamp(std::min(GetScreenWidth() / 1360.0, GetScreenHeight() / 820.0),
                0.72, 1.55));
  const int sw = GetScreenWidth();
  Rectangle panel{static_cast<float>(sw) - 240.0f * s, 34.0f, 205.0f * s, 372.0f * s};
  DrawRectangleRounded(panel, 0.045f, 12, {12, 13, 14, 232});
  DrawRectangleRoundedLines(panel, 0.045f, 12, {91, 101, 98, 140});

  const auto &rs = game.rules.State();
  char line[128]{};
  std::snprintf(line, sizeof(line), "Player %d", rs.currentPlayer + 1);
  DrawTextF(game.font, line, {panel.x + 18 * s, panel.y + 18 * s}, 24 * s, {236, 238, 228, 255});
  std::snprintf(line, sizeof(line), "Group: %s", hb::GroupName(rs.players[rs.currentPlayer].group));
  DrawTextF(game.font, line, {panel.x + 18 * s, panel.y + 50 * s}, 17 * s, {183, 192, 184, 255});
  std::snprintf(line, sizeof(line), "Clock: %02d", static_cast<int>(std::ceil(game.shotClock)));
  DrawTextF(game.font, line, {panel.x + 18 * s, panel.y + 78 * s}, 20 * s,
            game.shotClock < 8.0 ? Color{226, 112, 96, 255} : Color{210, 216, 206, 255});

  DrawTextF(game.font, "Tip", {panel.x + 18 * s, panel.y + 116 * s}, 18 * s, {188, 196, 188, 255});
  const Vector2 center{panel.x + 102 * s, panel.y + 194 * s};
  const float padR = 58.0f * s;
  DrawCircleV(center, padR, {25, 29, 29, 255});
  DrawCircleLines(static_cast<int>(center.x), static_cast<int>(center.y), padR, {113, 128, 121, 180});
  DrawLineEx({center.x - padR, center.y}, {center.x + padR, center.y}, 1.0f, {110, 119, 115, 125});
  DrawLineEx({center.x, center.y - padR}, {center.x, center.y + padR}, 1.0f, {110, 119, 115, 125});
  DrawCircleV({center.x + static_cast<float>(game.tipX * padR),
               center.y - static_cast<float>(game.tipY * padR)},
              7.0f, {228, 232, 218, 255});
  DrawTextF(game.font, "L", {center.x - padR - 16 * s, center.y - 9 * s}, 15 * s, {150, 159, 152, 255});
  DrawTextF(game.font, "R", {center.x + padR + 8 * s, center.y - 9 * s}, 15 * s, {150, 159, 152, 255});
  DrawTextF(game.font, "H", {center.x - 5 * s, center.y - padR - 22 * s}, 15 * s, {150, 159, 152, 255});
  DrawTextF(game.font, "L", {center.x - 5 * s, center.y + padR + 6 * s}, 15 * s, {150, 159, 152, 255});

  char powerText[64]{};
  std::snprintf(powerText, sizeof(powerText), "Power %3d%%", static_cast<int>(game.power * 100.0));
  DrawTextF(game.font, powerText, {panel.x + 18 * s, panel.y + 275 * s}, 18 * s, {213, 218, 209, 255});
  DrawRectangleRounded({panel.x + 18 * s, panel.y + 304 * s, 169 * s, 12 * s}, 0.4f, 8, {28, 31, 31, 255});
  DrawRectangleRounded({panel.x + 18 * s, panel.y + 304 * s, static_cast<float>(169.0 * game.power * s), 12 * s},
                       0.4f, 8, {143, 170, 151, 255});

  Button(game.font, {panel.x + 18 * s, panel.y + 334 * s, 80 * s, 28 * s}, "Ext", !rs.players[rs.currentPlayer].extensionUsed);
  Button(game.font, {panel.x + 107 * s, panel.y + 334 * s, 80 * s, 28 * s}, "Rack", true);

  Rectangle msg{48.0f * s, static_cast<float>(GetScreenHeight()) - 58.0f * s,
                static_cast<float>(GetScreenWidth()) - 96.0f * s, 34.0f * s};
  DrawRectangleRounded(msg, 0.08f, 8, {9, 10, 10, 210});
  DrawTextF(game.font, rs.message.c_str(), {msg.x + 14 * s, msg.y + 8 * s}, 18 * s, {221, 224, 216, 255});

  if (game.phase == Phase::GroupChoice) {
    Rectangle modal{static_cast<float>(GetScreenWidth()) * 0.5f - 170.0f,
                    static_cast<float>(GetScreenHeight()) * 0.5f - 62.0f, 340.0f, 124.0f};
    DrawRectangleRounded(modal, 0.04f, 12, {13, 15, 15, 245});
    DrawRectangleRoundedLines(modal, 0.04f, 12, {130, 142, 136, 190});
    DrawTextF(game.font, "Choose group", {modal.x + 92, modal.y + 20}, 22, {236, 238, 228, 255});
    Button(game.font, {modal.x + 34, modal.y + 70, 126, 32}, "Solids", true);
    Button(game.font, {modal.x + 180, modal.y + 70, 126, 32}, "Stripes", true);
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
  DrawTextF(game.font, "HEYBALL", {18.0f, 7.0f}, 18.0f, {207, 214, 207, 220});
  DrawTextF(game.font, "Chinese Eight Ball", {112.0f, 8.0f}, 16.0f,
            {117, 128, 122, 210});
  const Rectangle minR = TitleButton(w - 86.0f, "-", game.font,
                                     PointInRect(mouse, {w - 86.0f, 0.0f, 42.0f, 34.0f}));
  const Rectangle closeR = TitleButton(w - 43.0f, "x", game.font,
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
  Rectangle panel{static_cast<float>(sw) - 240.0f * s, 34.0f, 205.0f * s, 372.0f * s};
  const Vector2 tipCenter{panel.x + 102 * s, panel.y + 194 * s};
  const float tipR = 58.0f * s;

  if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
    if (Vector2Distance(mouse, tipCenter) <= tipR) {
      game.tipX = hb::Clamp((mouse.x - tipCenter.x) / tipR, -1.0, 1.0);
      game.tipY = hb::Clamp(-(mouse.y - tipCenter.y) / tipR, -1.0, 1.0);
    }
    if (PointInRect(mouse, {panel.x + 18 * s, panel.y + 334 * s, 80 * s, 28 * s}) &&
        !game.rules.State().players[game.rules.State().currentPlayer].extensionUsed) {
      game.rules.State().players[game.rules.State().currentPlayer].extensionUsed = true;
      game.shotClock += 30.0;
    }
    if (PointInRect(mouse, {panel.x + 107 * s, panel.y + 334 * s, 80 * s, 28 * s})) {
      game.world.ResetRack();
      game.rules.ResetRack(1 - game.rules.State().currentPlayer);
      game.phase = Phase::Aiming;
      game.power = 0.0;
      game.shotClock = 45.0;
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

  if (game.phase == Phase::Aiming || game.phase == Phase::BallInHand) {
    game.shotClock -= dt;
    if (game.shotClock <= 0.0) {
      game.phase = game.rules.ForceFoul("Shot clock foul").nextPhase;
      game.shotClock = 45.0;
    }
  }

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
      game.shotClock = 45.0;
    }
  } else if (game.phase == Phase::BallInHand) {
    Vec2 pos = ScreenToWorld(view, GetMousePosition());
    if (game.world.CanPlaceCue(pos)) {
      game.world.PlaceCue(pos);
      if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
        game.rules.State().ballInHand = false;
        game.phase = Phase::Aiming;
        game.shotClock = 45.0;
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
      game.shotClock = 45.0;
    }
  } else if (game.phase == Phase::RackOver && IsKeyPressed(KEY_SPACE)) {
    const int nextBreaker = game.rules.State().winner >= 0 ? game.rules.State().winner : 0;
    game.world.ResetRack();
    game.rules.ResetRack(nextBreaker);
    game.phase = Phase::Aiming;
    game.shotClock = 45.0;
  }

  if (IsKeyPressed(KEY_C)) {
    game.tipX = 0.0;
    game.tipY = 0.0;
  }
}

void DrawGame(Game &game, const View &view) {
  ClearBackground(BLACK);
  DrawTitleBar(game);
  DrawTable(view);
  for (const Ball &ball : game.world.Balls()) {
    DrawBall(game.font, view, ball);
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
    DrawTextF(game.font, "Press Space for next rack", {r.x + 70, r.y + 54}, 18,
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
  InitWindow(1360, 820, "Heyball - Chinese Eight Ball");
  SetWindowMinSize(960, 600);
  SetTargetFPS(240);

  Game game;
  game.font = LoadFontEx("C:/Windows/Fonts/Dengb.ttf", 96, nullptr, 0);
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
