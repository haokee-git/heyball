#include "core.hpp"

#include <algorithm>
#include <cmath>

namespace hb {
namespace {

constexpr double kRestitutionBall = 0.965;
constexpr double kRestitutionRail = 0.780;
constexpr double kGravity = 9.80665;
constexpr double kSlideMu = 0.145;
constexpr double kRollDrag = 0.055;
constexpr double kSideDecay = 0.78;
constexpr double kCueMaxSpeed = 4.85;
std::array<Vec2, 6> PocketCenters() {
  return {{{-kTableWidth * 0.5, -kTableHeight * 0.5},
           {0.0, -kTableHeight * 0.5},
           {kTableWidth * 0.5, -kTableHeight * 0.5},
           {-kTableWidth * 0.5, kTableHeight * 0.5},
           {0.0, kTableHeight * 0.5},
           {kTableWidth * 0.5, kTableHeight * 0.5}}};
}

double PocketRadiusForIndex(int i) {
  const double mouth = (i == 1 || i == 4) ? kSidePocketMouth : kCornerPocketMouth;
  return mouth * 0.78;
}

bool IsMovingIntoPocket(Vec2 pos, Vec2 vel, Vec2 center) {
  const double speed = Length(vel);
  if (speed < 0.035) {
    return false;
  }
  const Vec2 toPocket = center - pos;
  const double distance = Length(toPocket);
  if (distance < 1e-6) {
    return true;
  }
  return Dot(vel, toPocket / distance) > 0.012;
}

bool IsPocketCapture(Vec2 pos, Vec2 vel, int index, Vec2 center) {
  const bool committedByMotion = IsMovingIntoPocket(pos, vel, center);

  const bool sidePocket = index == 1 || index == 4;
  if (sidePocket) {
    const double halfMouth = kSidePocketMouth * 0.74 + kBallRadius * 0.35;
    const double throatDepth = kCushionNoseInset + kBallRadius * 0.85;
    const double fallDepth = kBallRadius * 0.44;
    if (std::abs(pos.x - center.x) > halfMouth) {
      return false;
    }
    const bool inThroat = index == 1 ? pos.y <= center.y + throatDepth
                                     : pos.y >= center.y - throatDepth;
    const bool overDrop = index == 1 ? pos.y <= center.y + fallDepth
                                     : pos.y >= center.y - fallDepth;
    return overDrop || (inThroat && committedByMotion);
  }

  const bool left = index == 0 || index == 3;
  const bool top = index == 0 || index == 2;
  const double inwardX = left ? pos.x - center.x : center.x - pos.x;
  const double inwardY = top ? pos.y - center.y : center.y - pos.y;
  const double mouth = kCornerPocketMouth * 0.74 + kBallRadius * 0.35;
  const double throatDepth = kCushionNoseInset + kBallRadius * 0.85;
  const double fallDepth = kBallRadius * 0.44;
  const double outsideAllowance = kBallRadius * 0.25;
  if (inwardX < -outsideAllowance || inwardY < -outsideAllowance ||
      inwardX > mouth || inwardY > mouth) {
    return false;
  }
  const bool overDrop = inwardX <= fallDepth && inwardY <= fallDepth;
  const bool inThroat = inwardX <= throatDepth || inwardY <= throatDepth ||
                        Distance(pos, center) < PocketRadiusForIndex(index);
  return overDrop || (inThroat && committedByMotion);
}

bool ContainsNumber(const std::vector<int> &values, int number) {
  return std::find(values.begin(), values.end(), number) != values.end();
}

struct CushionSegment {
  Vec2 a{};
  Vec2 b{};
  Vec2 normal{};
};

Vec2 BezierCubic(Vec2 a, Vec2 b, Vec2 c, Vec2 d, double t) {
  const double u = 1.0 - t;
  return a * (u * u * u) + b * (3.0 * u * u * t) +
         c * (3.0 * u * t * t) + d * (t * t * t);
}

void AddCushionSegment(std::vector<CushionSegment> &segments, Vec2 a, Vec2 b) {
  const Vec2 tangent = b - a;
  if (LengthSq(tangent) < 1e-12) {
    return;
  }
  const Vec2 t = Normalize(tangent);
  const Vec2 mid = (a + b) * 0.5;
  const Vec2 towardCenter = Normalize(Vec2{} - mid);
  Vec2 normal = Perp(t);
  if (Dot(normal, towardCenter) < 0.0) {
    normal *= -1.0;
  }
  segments.push_back({a, b, normal});
}

void AppendCushionBezier(std::vector<CushionSegment> &segments, Vec2 a, Vec2 b,
                         Vec2 c, Vec2 d) {
  constexpr int kSegments = 10;
  Vec2 previous = a;
  for (int i = 1; i <= kSegments; ++i) {
    const Vec2 next =
        BezierCubic(a, b, c, d, static_cast<double>(i) / kSegments);
    AddCushionSegment(segments, previous, next);
    previous = next;
  }
}

Vec2 ClosestPointOnSegment(Vec2 p, Vec2 a, Vec2 b) {
  const Vec2 ab = b - a;
  const double denom = LengthSq(ab);
  if (denom < 1e-12) {
    return a;
  }
  const double t = Clamp(Dot(p - a, ab) / denom, 0.0, 1.0);
  return a + ab * t;
}

void AddHorizontalCushion(std::vector<CushionSegment> &segments, double x1,
                          double x2, double outerY, bool top) {
  if (x2 <= x1 + kCushionNoseInset * 2.0) {
    return;
  }
  const double sy = top ? 1.0 : -1.0;
  const double innerY = outerY + sy * kCushionNoseInset;
  const double jaw =
      std::min(kCushionNoseInset * 2.45, (x2 - x1) * 0.42);
  AppendCushionBezier(segments, {x1, outerY},
                      {x1 + jaw * 0.08, outerY + sy * kCushionNoseInset * 0.16},
                      {x1 + jaw * 0.52, innerY}, {x1 + jaw, innerY});
  AddCushionSegment(segments, {x1 + jaw, innerY}, {x2 - jaw, innerY});
  AppendCushionBezier(segments, {x2 - jaw, innerY},
                      {x2 - jaw * 0.52, innerY},
                      {x2 - jaw * 0.08, outerY + sy * kCushionNoseInset * 0.16},
                      {x2, outerY});
}

void AddVerticalCushion(std::vector<CushionSegment> &segments, double outerX,
                        double y1, double y2, bool left) {
  if (y2 <= y1 + kCushionNoseInset * 2.0) {
    return;
  }
  const double sx = left ? 1.0 : -1.0;
  const double innerX = outerX + sx * kCushionNoseInset;
  const double jaw =
      std::min(kCushionNoseInset * 2.45, (y2 - y1) * 0.42);
  AppendCushionBezier(segments, {outerX, y1},
                      {outerX + sx * kCushionNoseInset * 0.16, y1 + jaw * 0.08},
                      {innerX, y1 + jaw * 0.52}, {innerX, y1 + jaw});
  AddCushionSegment(segments, {innerX, y1 + jaw}, {innerX, y2 - jaw});
  AppendCushionBezier(segments, {innerX, y2 - jaw},
                      {innerX, y2 - jaw * 0.52},
                      {outerX + sx * kCushionNoseInset * 0.16, y2 - jaw * 0.08},
                      {outerX, y2});
}

std::vector<CushionSegment> CushionSegments() {
  const double left = -kTableWidth * 0.5;
  const double right = kTableWidth * 0.5;
  const double top = -kTableHeight * 0.5;
  const double bottom = kTableHeight * 0.5;
  const double cornerGap = kCornerPocketMouth * 0.74;
  const double sideGap = kSidePocketMouth * 0.74;

  std::vector<CushionSegment> segments;
  segments.reserve(72);
  AddHorizontalCushion(segments, left + cornerGap, -sideGap, top, true);
  AddHorizontalCushion(segments, sideGap, right - cornerGap, top, true);
  AddHorizontalCushion(segments, left + cornerGap, -sideGap, bottom, false);
  AddHorizontalCushion(segments, sideGap, right - cornerGap, bottom, false);
  AddVerticalCushion(segments, left, top + cornerGap, bottom - cornerGap, true);
  AddVerticalCushion(segments, right, top + cornerGap, bottom - cornerGap, false);
  return segments;
}

} // namespace

Vec2 operator+(Vec2 a, Vec2 b) { return {a.x + b.x, a.y + b.y}; }
Vec2 operator-(Vec2 a, Vec2 b) { return {a.x - b.x, a.y - b.y}; }
Vec2 operator*(Vec2 a, double s) { return {a.x * s, a.y * s}; }
Vec2 operator*(double s, Vec2 a) { return a * s; }
Vec2 operator/(Vec2 a, double s) { return {a.x / s, a.y / s}; }
Vec2 &operator+=(Vec2 &a, Vec2 b) {
  a.x += b.x;
  a.y += b.y;
  return a;
}
Vec2 &operator-=(Vec2 &a, Vec2 b) {
  a.x -= b.x;
  a.y -= b.y;
  return a;
}
Vec2 &operator*=(Vec2 &a, double s) {
  a.x *= s;
  a.y *= s;
  return a;
}
double Dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }
double LengthSq(Vec2 a) { return Dot(a, a); }
double Length(Vec2 a) { return std::sqrt(LengthSq(a)); }
double Distance(Vec2 a, Vec2 b) { return Length(a - b); }
Vec2 Normalize(Vec2 a) {
  const double len = Length(a);
  if (len < 1e-9) {
    return {1.0, 0.0};
  }
  return a / len;
}
Vec2 Perp(Vec2 a) { return {-a.y, a.x}; }
double Clamp(double v, double lo, double hi) {
  return std::max(lo, std::min(v, hi));
}

bool IsSolid(int number) { return number >= 1 && number <= 7; }
bool IsStripe(int number) { return number >= 9 && number <= 15; }
BallGroup GroupForBall(int number) {
  if (IsSolid(number)) {
    return BallGroup::Solids;
  }
  if (IsStripe(number)) {
    return BallGroup::Stripes;
  }
  return BallGroup::Open;
}

const char *GroupName(BallGroup group) {
  switch (group) {
  case BallGroup::Solids:
    return "全色球";
  case BallGroup::Stripes:
    return "半色球";
  case BallGroup::Open:
  default:
    return "未定";
  }
}

void ShotEvents::Clear() {
  firstContact = -1;
  potted.clear();
  railContacts.clear();
  cuePocketed = false;
  eightPocketed = false;
}

bool ShotEvents::PottedGroup(BallGroup group) const {
  for (int n : potted) {
    if (GroupForBall(n) == group) {
      return true;
    }
  }
  return false;
}

bool ShotEvents::PottedObject() const {
  for (int n : potted) {
    if (n != 0) {
      return true;
    }
  }
  return false;
}

PhysicsWorld::PhysicsWorld() { ResetRack(); }

void PhysicsWorld::ResetRack() {
  accumulator_ = 0.0;
  for (int i = 0; i < 16; ++i) {
    balls_[i] = Ball{};
    balls_[i].number = i;
    balls_[i].decal = {0.0, 0.0};
  }

  balls_[0].pos = {-kTableWidth * 0.30, 0.0};

  const double gap = 0.0015;
  const double spacing = kBallDiameter + gap;
  const double rowDx = std::sqrt(3.0) * spacing * 0.5;
  const Vec2 apex{kTableWidth * 0.235, 0.0};
  const std::array<std::array<int, 5>, 5> rack = {{
      {{1, 0, 0, 0, 0}},
      {{9, 2, 0, 0, 0}},
      {{10, 8, 3, 0, 0}},
      {{11, 4, 12, 5, 0}},
      {{6, 13, 7, 14, 15}},
  }};

  for (int row = 0; row < 5; ++row) {
    for (int col = 0; col <= row; ++col) {
      const int n = rack[row][col];
      if (n <= 0) {
        continue;
      }
      balls_[n].pos = {apex.x + row * rowDx,
                       apex.y + (col - row * 0.5) * spacing};
    }
  }
}

void PhysicsWorld::StrikeCue(const ShotParams &shot) {
  Ball &cue = balls_[0];
  if (cue.pocketed || cue.sinking) {
    return;
  }
  const Vec2 aim = Normalize(shot.aim);
  const double speed = kCueMaxSpeed * Clamp(shot.power, 0.0, 1.0);
  cue.vel += aim * speed;

  const Vec2 rollAxis = Perp(aim);
  cue.rollOmega += rollAxis * (shot.tipY * speed * 1.6 / kBallRadius);
  cue.rollOmega += rollAxis * (0.35 * speed / kBallRadius);
  cue.sideOmega += shot.tipX * speed * 9.0;
}

bool PhysicsWorld::IsMoving(double threshold) const {
  for (const Ball &ball : balls_) {
    if (ball.sinking) {
      return true;
    }
    if (!ball.pocketed && !ball.sinking && Length(ball.vel) > threshold) {
      return true;
    }
    if (!ball.pocketed && !ball.sinking && std::abs(ball.sideOmega) > 1.2) {
      return true;
    }
  }
  return false;
}

bool PhysicsWorld::CanPlaceCue(Vec2 pos) const {
  const double left = -kTableWidth * 0.5 + kCushionNoseInset + kBallRadius;
  const double right = kTableWidth * 0.5 - kCushionNoseInset - kBallRadius;
  const double top = -kTableHeight * 0.5 + kCushionNoseInset + kBallRadius;
  const double bottom = kTableHeight * 0.5 - kCushionNoseInset - kBallRadius;
  if (pos.x < left || pos.x > right || pos.y < top || pos.y > bottom) {
    return false;
  }
  for (const Ball &ball : balls_) {
    if (ball.number == 0 || ball.pocketed || ball.sinking) {
      continue;
    }
    if (Distance(pos, ball.pos) < kBallDiameter + 0.001) {
      return false;
    }
  }
  return true;
}

void PhysicsWorld::PlaceCue(Vec2 pos) {
  Ball &cue = balls_[0];
  cue.pos = pos;
  cue.vel = {};
  cue.rollOmega = {};
  cue.sideOmega = 0.0;
  cue.sinking = false;
  cue.pocketed = false;
  cue.pocketFade = 0.0;
}

void PhysicsWorld::SpotBall(int number) {
  if (number < 1 || number > 15) {
    return;
  }
  const Vec2 spot{kTableWidth * 0.25, 0.0};
  Vec2 pos = spot;
  for (int attempt = 0; attempt < 80 && !CanPlaceCue(pos); ++attempt) {
    pos.x += kBallDiameter * 0.55;
    if (pos.x > kTableWidth * 0.5 - kBallRadius) {
      pos.x = spot.x;
      pos.y += kBallDiameter * ((attempt % 2 == 0) ? 0.5 : -0.5);
    }
  }
  Ball &ball = balls_[number];
  ball.pos = pos;
  ball.vel = {};
  ball.rollOmega = {};
  ball.sideOmega = 0.0;
  ball.sinking = false;
  ball.pocketed = false;
  ball.pocketFade = 0.0;
}

void PhysicsWorld::Step(double dt, ShotEvents *events) {
  accumulator_ += dt;
  int guard = 0;
  while (accumulator_ >= kFixedStep && guard++ < 24) {
    CheckPockets(events);
    Integrate(kFixedStep);
    CheckPockets(events);
    ResolveBallContacts(events);
    ResolveRailContacts(events);
    CheckPockets(events);
    accumulator_ -= kFixedStep;
  }
  if (guard >= 24) {
    accumulator_ = 0.0;
  }
}

void PhysicsWorld::Integrate(double dt) {
  for (Ball &ball : balls_) {
    if (ball.pocketed) {
      continue;
    }
    if (ball.sinking) {
      ball.pocketFade = Clamp(ball.pocketFade + dt * 2.9, 0.0, 1.0);
      ball.pos += (ball.sinkTarget - ball.pos) * std::min(1.0, dt * 9.5);
      if (ball.pocketFade >= 1.0) {
        ball.sinking = false;
        ball.pocketed = true;
      }
      continue;
    }
    ApplyFriction(ball, dt);
    const Vec2 delta = ball.vel * dt;
    ball.pos += delta;
    ball.rollAngle += Length(ball.vel) * dt / kBallRadius;
    ball.decal.x += delta.x / kBallRadius;
    ball.decal.y += delta.y / kBallRadius;
  }
}

void PhysicsWorld::ApplyFriction(Ball &ball, double dt) {
  const double speed = Length(ball.vel);
  if (speed < 0.003) {
    ball.vel = {};
    ball.rollOmega *= std::max(0.0, 1.0 - dt * 5.0);
  } else {
    Vec2 surface{kBallRadius * ball.rollOmega.y,
                 -kBallRadius * ball.rollOmega.x};
    Vec2 slip = ball.vel - surface;
    const double slipSpeed = Length(slip);
    if (slipSpeed > 0.006) {
      const Vec2 dir = Normalize(slip);
      const double amount = std::min(slipSpeed, kSlideMu * kGravity * dt);
      const Vec2 adjust = dir * amount;
      ball.vel -= adjust * 0.46;
      surface += adjust * 1.18;
      ball.rollOmega = {-surface.y / kBallRadius, surface.x / kBallRadius};
    } else {
      ball.rollOmega = {-ball.vel.y / kBallRadius, ball.vel.x / kBallRadius};
      const double drop = std::min(speed, kRollDrag * dt);
      ball.vel -= Normalize(ball.vel) * drop;
    }
  }

  ball.sideOmega *= std::exp(-kSideDecay * dt);
  if (std::abs(ball.sideOmega) < 0.05) {
    ball.sideOmega = 0.0;
  }
}

void PhysicsWorld::ResolveBallContacts(ShotEvents *events) {
  for (int i = 0; i < 16; ++i) {
    Ball &a = balls_[i];
    if (a.pocketed || a.sinking) {
      continue;
    }
    for (int j = i + 1; j < 16; ++j) {
      Ball &b = balls_[j];
      if (b.pocketed || b.sinking) {
        continue;
      }
      Vec2 delta = b.pos - a.pos;
      double dist = Length(delta);
      if (dist < 1e-8) {
        delta = {1.0, 0.0};
        dist = 1.0;
      }
      const double overlap = kBallDiameter - dist;
      if (overlap <= 0.0) {
        continue;
      }
      const Vec2 n = delta / dist;
      const Vec2 correction = n * (overlap * 0.52);
      a.pos -= correction;
      b.pos += correction;

      const Vec2 rel = b.vel - a.vel;
      const double relN = Dot(rel, n);
      if (relN < 0.0) {
        const double jn = -(1.0 + kRestitutionBall) * relN * 0.5;
        a.vel -= n * jn;
        b.vel += n * jn;

        const Vec2 t = Perp(n);
        const double tangentSpeed =
            Dot(rel, t) - (a.sideOmega + b.sideOmega) * kBallRadius;
        double jt = -tangentSpeed / 7.0;
        const double maxTangent = std::abs(jn) * 0.22;
        jt = Clamp(jt, -maxTangent, maxTangent);
        a.vel -= t * jt;
        b.vel += t * jt;
        a.sideOmega -= jt * 2.5 / kBallRadius;
        b.sideOmega -= jt * 2.5 / kBallRadius;
      }

      if (events && events->firstContact < 0 && (i == 0 || j == 0)) {
        events->firstContact = (i == 0) ? j : i;
      }
    }
  }
}

void PhysicsWorld::ResolveRailContacts(ShotEvents *events) {
  const auto segments = CushionSegments();

  for (Ball &ball : balls_) {
    if (ball.pocketed || ball.sinking) {
      continue;
    }
    bool hitAny = false;
    for (const CushionSegment &segment : segments) {
      const Vec2 closest = ClosestPointOnSegment(ball.pos, segment.a, segment.b);
      const Vec2 delta = ball.pos - closest;
      const double dist = Length(delta);
      const double signedDistance = Dot(delta, segment.normal);
      if (dist >= kBallRadius || signedDistance <= -kBallRadius * 0.35) {
        continue;
      }
      Vec2 normal = segment.normal;
      if (dist > 1e-8) {
        const Vec2 candidate = delta / dist;
        if (Dot(candidate, segment.normal) > 0.12) {
          normal = candidate;
        }
      }
      ball.pos += normal * (kBallRadius - dist + 0.0002);
      const double vN = Dot(ball.vel, normal);
      if (vN < 0.0) {
        ball.vel -= normal * ((1.0 + kRestitutionRail) * vN);
      }
      const Vec2 t = Perp(normal);
      const double spinKick = ball.sideOmega * kBallRadius * 0.095;
      ball.vel += t * spinKick;
      ball.vel -= t * (Dot(ball.vel, t) * 0.035);
      ball.sideOmega *= 0.72;
      hitAny = true;
    }
    if (hitAny && events) {
      events->railContacts.insert(ball.number);
    }
  }
}

void PhysicsWorld::CheckPockets(ShotEvents *events) {
  const auto pockets = PocketCenters();
  for (Ball &ball : balls_) {
    if (ball.pocketed || ball.sinking) {
      continue;
    }
    for (int i = 0; i < static_cast<int>(pockets.size()); ++i) {
      if (IsPocketCapture(ball.pos, ball.vel, i, pockets[i])) {
        ball.sinking = true;
        ball.sinkTarget = pockets[i];
        ball.vel = {};
        ball.rollOmega = {};
        ball.sideOmega = 0.0;
        ball.pocketFade = 0.0;
        if (events && !ContainsNumber(events->potted, ball.number)) {
          events->potted.push_back(ball.number);
          if (ball.number == 0) {
            events->cuePocketed = true;
          }
          if (ball.number == 8) {
            events->eightPocketed = true;
          }
        }
        break;
      }
    }
  }
}

void RulesEngine::ResetRack(int breaker) {
  state_ = RulesState{};
  state_.currentPlayer = breaker;
  state_.message =
      std::to_string(breaker + 1) + "号玩家开球";
}

TurnDecision RulesEngine::ForceFoul(const std::string &reason) {
  SwitchTurn();
  state_.ballInHand = true;
  state_.message = reason + "，" + std::to_string(state_.currentPlayer + 1) +
                   "号玩家获得自由球";
  return {Phase::BallInHand, state_.message};
}

TurnDecision RulesEngine::ChooseGroup(BallGroup group) {
  if (!state_.pendingGroupChoice ||
      (group != BallGroup::Solids && group != BallGroup::Stripes)) {
    return {Phase::GroupChoice, "请选择全色球或半色球"};
  }
  AssignGroups(state_.currentPlayer, group);
  state_.pendingGroupChoice = false;
  state_.message = std::to_string(state_.currentPlayer + 1) + "号玩家选择" +
                   GroupName(group);
  return {Phase::Aiming, state_.message};
}

TurnDecision RulesEngine::ApplyShot(const ShotEvents &events,
                                    PhysicsWorld &world) {
  state_.ballInHand = false;
  state_.pendingGroupChoice = false;

  if (events.eightPocketed) {
    const bool legalEight =
        !events.cuePocketed && !state_.breakShot &&
        GroupCleared(state_.players[state_.currentPlayer].group, world);
    if (state_.breakShot) {
      state_.breakShot = false;
      world.SpotBall(8);
      if (events.cuePocketed) {
        return ForceFoul("开球白球落袋");
      }
      state_.message = "开球打进8号球，8号球复位，击球方继续";
      return {Phase::Aiming, state_.message};
    }
    state_.winner = legalEight ? state_.currentPlayer : 1 - state_.currentPlayer;
    state_.message = std::to_string(state_.winner + 1) + "号玩家赢得本局";
    return {Phase::RackOver, state_.message};
  }

  bool foul = false;
  std::string reason;

  if (events.cuePocketed) {
    foul = true;
    reason = "白球落袋";
  } else if (events.firstContact < 0) {
    foul = true;
    reason = "未碰到目标球";
  } else if (!IsLegalFirstContact(events.firstContact, world)) {
    foul = true;
    reason = "首次触球错误";
  }

  if (!foul && !events.PottedObject() && events.railContacts.empty()) {
    foul = true;
    reason = "碰球后无球落袋且无球触库";
  }

  if (state_.breakShot) {
    const bool legalBreak = events.PottedObject() || events.railContacts.size() >= 4;
    state_.breakShot = false;
    if (!legalBreak && !foul) {
      foul = true;
      reason = "开球不合法";
    }
  }

  if (foul) {
    return ForceFoul(reason);
  }

  if (state_.players[state_.currentPlayer].group == BallGroup::Open) {
    const bool solids = events.PottedGroup(BallGroup::Solids);
    const bool stripes = events.PottedGroup(BallGroup::Stripes);
    if (solids && stripes) {
      state_.pendingGroupChoice = true;
      state_.message = "请选择球组：全色球或半色球";
      return {Phase::GroupChoice, state_.message};
    }
    if (solids || stripes) {
      AssignGroups(state_.currentPlayer,
                   solids ? BallGroup::Solids : BallGroup::Stripes);
    }
  }

  const BallGroup own = state_.players[state_.currentPlayer].group;
  const bool ownPotted = (own != BallGroup::Open) && events.PottedGroup(own);
  const bool openPotted = own == BallGroup::Open && events.PottedObject();
  if (ownPotted || openPotted) {
    state_.message =
        std::to_string(state_.currentPlayer + 1) + "号玩家继续击球";
    return {Phase::Aiming, state_.message};
  }

  SwitchTurn();
  state_.message =
      "轮到" + std::to_string(state_.currentPlayer + 1) + "号玩家击球";
  return {Phase::Aiming, state_.message};
}

bool RulesEngine::GroupCleared(BallGroup group,
                               const PhysicsWorld &world) const {
  if (group == BallGroup::Open) {
    return false;
  }
  for (const Ball &ball : world.Balls()) {
    if (!ball.pocketed && !ball.sinking && GroupForBall(ball.number) == group) {
      return false;
    }
  }
  return true;
}

BallGroup RulesEngine::NeededGroupForCurrent(const PhysicsWorld &world) const {
  const BallGroup group = state_.players[state_.currentPlayer].group;
  if (group == BallGroup::Open) {
    return BallGroup::Open;
  }
  return GroupCleared(group, world) ? BallGroup::Open : group;
}

bool RulesEngine::IsLegalFirstContact(int ballNumber,
                                      const PhysicsWorld &world) const {
  if (ballNumber <= 0) {
    return false;
  }
  if (state_.breakShot) {
    return true;
  }
  const BallGroup needed = NeededGroupForCurrent(world);
  if (ballNumber == 8) {
    return needed == BallGroup::Open &&
           state_.players[state_.currentPlayer].group != BallGroup::Open;
  }
  if (needed == BallGroup::Open) {
    return ballNumber != 8;
  }
  return GroupForBall(ballNumber) == needed;
}

bool RulesEngine::IsOwnGroupBall(int ballNumber) const {
  return GroupForBall(ballNumber) ==
         state_.players[state_.currentPlayer].group;
}

void RulesEngine::SwitchTurn() { state_.currentPlayer = 1 - state_.currentPlayer; }

void RulesEngine::AssignGroups(int player, BallGroup group) {
  state_.players[player].group = group;
  state_.players[1 - player].group =
      group == BallGroup::Solids ? BallGroup::Stripes : BallGroup::Solids;
}

} // namespace hb
