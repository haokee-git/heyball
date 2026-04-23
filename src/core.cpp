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
  return mouth * 0.72;
}

bool ContainsNumber(const std::vector<int> &values, int number) {
  return std::find(values.begin(), values.end(), number) != values.end();
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
    return "solids";
  case BallGroup::Stripes:
    return "stripes";
  case BallGroup::Open:
  default:
    return "open";
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
  const double left = -kTableWidth * 0.5 + kBallRadius;
  const double right = kTableWidth * 0.5 - kBallRadius;
  const double top = -kTableHeight * 0.5 + kBallRadius;
  const double bottom = kTableHeight * 0.5 - kBallRadius;
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
  const double left = -kTableWidth * 0.5 + kBallRadius;
  const double right = kTableWidth * 0.5 - kBallRadius;
  const double top = -kTableHeight * 0.5 + kBallRadius;
  const double bottom = kTableHeight * 0.5 - kBallRadius;

  for (Ball &ball : balls_) {
    if (ball.pocketed || ball.sinking) {
      continue;
    }
    Vec2 normal{};
    bool hit = false;
    if (ball.pos.x < left) {
      ball.pos.x = left;
      normal = {1.0, 0.0};
      hit = true;
    } else if (ball.pos.x > right) {
      ball.pos.x = right;
      normal = {-1.0, 0.0};
      hit = true;
    }
    if (ball.pos.y < top) {
      ball.pos.y = top;
      normal = {0.0, 1.0};
      hit = true;
    } else if (ball.pos.y > bottom) {
      ball.pos.y = bottom;
      normal = {0.0, -1.0};
      hit = true;
    }
    if (!hit) {
      continue;
    }

    const double vN = Dot(ball.vel, normal);
    if (vN < 0.0) {
      ball.vel -= normal * ((1.0 + kRestitutionRail) * vN);
    }
    const Vec2 t = Perp(normal);
      const double spinKick = ball.sideOmega * kBallRadius * 0.095;
    ball.vel += t * spinKick;
    ball.vel -= t * (Dot(ball.vel, t) * 0.035);
    ball.sideOmega *= 0.72;
    if (events) {
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
      const double radius = PocketRadiusForIndex(i);
      if (Distance(ball.pos, pockets[i]) < radius) {
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
  state_.message = breaker == 0 ? "Player 1 breaks" : "Player 2 breaks";
}

TurnDecision RulesEngine::ForceFoul(const std::string &reason) {
  SwitchTurn();
  state_.ballInHand = true;
  state_.message = reason + ". Ball in hand for Player " +
                   std::to_string(state_.currentPlayer + 1);
  return {Phase::BallInHand, state_.message};
}

TurnDecision RulesEngine::ChooseGroup(BallGroup group) {
  if (!state_.pendingGroupChoice ||
      (group != BallGroup::Solids && group != BallGroup::Stripes)) {
    return {Phase::GroupChoice, "Choose solids or stripes"};
  }
  AssignGroups(state_.currentPlayer, group);
  state_.pendingGroupChoice = false;
  state_.message = std::string("Player ") +
                   std::to_string(state_.currentPlayer + 1) + " takes " +
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
        return ForceFoul("Cue ball scratched on the break");
      }
      state_.message = "Eight on break is spotted. Shooter continues";
      return {Phase::Aiming, state_.message};
    }
    state_.winner = legalEight ? state_.currentPlayer : 1 - state_.currentPlayer;
    state_.message = std::string("Player ") + std::to_string(state_.winner + 1) +
                     " wins the rack";
    return {Phase::RackOver, state_.message};
  }

  bool foul = false;
  std::string reason;

  if (events.cuePocketed) {
    foul = true;
    reason = "Cue ball pocketed";
  } else if (events.firstContact < 0) {
    foul = true;
    reason = "No ball contacted";
  } else if (!IsLegalFirstContact(events.firstContact, world)) {
    foul = true;
    reason = "Wrong first contact";
  }

  if (!foul && !events.PottedObject() && events.railContacts.empty()) {
    foul = true;
    reason = "No ball potted and no cushion after contact";
  }

  if (state_.breakShot) {
    const bool legalBreak = events.PottedObject() || events.railContacts.size() >= 4;
    state_.breakShot = false;
    if (!legalBreak && !foul) {
      foul = true;
      reason = "Illegal break";
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
      state_.message = "Choose group: solids or stripes";
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
    state_.message = std::string("Player ") +
                     std::to_string(state_.currentPlayer + 1) + " continues";
    return {Phase::Aiming, state_.message};
  }

  SwitchTurn();
  state_.message = std::string("Turn passes to Player ") +
                   std::to_string(state_.currentPlayer + 1);
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
