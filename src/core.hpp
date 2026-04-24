#pragma once

#include <array>
#include <set>
#include <string>
#include <vector>

namespace hb {

constexpr double kTableWidth = 2.540;
constexpr double kTableHeight = 1.260;
constexpr double kBallRadius = 0.028575;
constexpr double kBallDiameter = kBallRadius * 2.0;
constexpr double kCornerPocketMouth = 0.0762;
constexpr double kSidePocketMouth = 0.0825;
constexpr double kCushionNoseInset = 0.035;
constexpr double kFixedStep = 1.0 / 240.0;

struct Vec2 {
  double x = 0.0;
  double y = 0.0;
};

Vec2 operator+(Vec2 a, Vec2 b);
Vec2 operator-(Vec2 a, Vec2 b);
Vec2 operator*(Vec2 a, double s);
Vec2 operator*(double s, Vec2 a);
Vec2 operator/(Vec2 a, double s);
Vec2 &operator+=(Vec2 &a, Vec2 b);
Vec2 &operator-=(Vec2 &a, Vec2 b);
Vec2 &operator*=(Vec2 &a, double s);
double Dot(Vec2 a, Vec2 b);
double Length(Vec2 a);
double LengthSq(Vec2 a);
double Distance(Vec2 a, Vec2 b);
Vec2 Normalize(Vec2 a);
Vec2 Perp(Vec2 a);
double Clamp(double v, double lo, double hi);

enum class BallGroup {
  Open,
  Solids,
  Stripes
};

enum class Phase {
  Aiming,
  Moving,
  BallInHand,
  GroupChoice,
  RackOver
};

struct Ball {
  int number = 0;
  Vec2 pos{};
  Vec2 vel{};
  Vec2 rollOmega{};
  double sideOmega = 0.0;
  double rollAngle = 0.0;
  Vec2 decal{};
  Vec2 sinkTarget{};
  bool sinking = false;
  bool pocketed = false;
  double pocketFade = 0.0;
};

struct ShotParams {
  Vec2 aim{1.0, 0.0};
  double power = 0.0;
  double tipX = 0.0;
  double tipY = 0.0;
};

struct ShotEvents {
  int firstContact = -1;
  std::vector<int> potted;
  std::set<int> railContacts;
  bool cuePocketed = false;
  bool eightPocketed = false;

  void Clear();
  bool PottedGroup(BallGroup group) const;
  bool PottedObject() const;
};

class PhysicsWorld {
public:
  PhysicsWorld();

  void ResetRack();
  void Step(double dt, ShotEvents *events);
  void StrikeCue(const ShotParams &shot);
  bool IsMoving(double threshold = 0.004) const;
  bool CanPlaceCue(Vec2 pos) const;
  void PlaceCue(Vec2 pos);
  void SpotBall(int number);

  const std::array<Ball, 16> &Balls() const { return balls_; }
  std::array<Ball, 16> &Balls() { return balls_; }
  const Ball &CueBall() const { return balls_[0]; }
  Ball &CueBall() { return balls_[0]; }

private:
  std::array<Ball, 16> balls_{};
  double accumulator_ = 0.0;

  void Integrate(double dt);
  void ApplyFriction(Ball &ball, double dt);
  void ResolveBallContacts(ShotEvents *events);
  void ResolveRailContacts(ShotEvents *events);
  void CheckPockets(ShotEvents *events);
};

struct Player {
  BallGroup group = BallGroup::Open;
  bool extensionUsed = false;
};

struct RulesState {
  int currentPlayer = 0;
  int winner = -1;
  bool breakShot = true;
  bool ballInHand = false;
  bool pendingGroupChoice = false;
  std::array<Player, 2> players{};
  std::string message = "1号玩家开球";
};

struct TurnDecision {
  Phase nextPhase = Phase::Aiming;
  std::string message;
};

class RulesEngine {
public:
  RulesState &State() { return state_; }
  const RulesState &State() const { return state_; }

  void ResetRack(int breaker = 0);
  TurnDecision ApplyShot(const ShotEvents &events, PhysicsWorld &world);
  TurnDecision ChooseGroup(BallGroup group);
  TurnDecision ForceFoul(const std::string &reason);

private:
  RulesState state_{};

  bool GroupCleared(BallGroup group, const PhysicsWorld &world) const;
  BallGroup NeededGroupForCurrent(const PhysicsWorld &world) const;
  bool IsLegalFirstContact(int ballNumber, const PhysicsWorld &world) const;
  bool IsOwnGroupBall(int ballNumber) const;
  void SwitchTurn();
  void AssignGroups(int player, BallGroup group);
};

const char *GroupName(BallGroup group);
bool IsSolid(int number);
bool IsStripe(int number);
BallGroup GroupForBall(int number);

} // namespace hb
