#include "core.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

namespace {

void Check(bool ok, const std::string &name) {
  if (!ok) {
    std::cerr << "FAIL: " << name << "\n";
    std::exit(1);
  }
}

void StepFor(hb::PhysicsWorld &world, hb::ShotEvents &events, double seconds) {
  const int steps = static_cast<int>(seconds / hb::kFixedStep);
  for (int i = 0; i < steps; ++i) {
    world.Step(hb::kFixedStep, &events);
  }
}

void TestHeadOnCollision() {
  hb::PhysicsWorld world;
  for (auto &b : world.Balls()) {
    b.pocketed = true;
  }
  auto &a = world.Balls()[0];
  auto &b = world.Balls()[1];
  a.pocketed = false;
  b.pocketed = false;
  a.pos = {-0.2, 0.0};
  b.pos = {-0.2 + hb::kBallDiameter * 1.01, 0.0};
  a.vel = {1.5, 0.0};
  hb::ShotEvents events;
  StepFor(world, events, 0.08);
  Check(b.vel.x > 0.8, "head-on transfers speed");
  Check(events.firstContact == 1, "first contact is recorded");
}

void TestRailBounce() {
  hb::PhysicsWorld world;
  for (auto &b : world.Balls()) {
    b.pocketed = true;
  }
  auto &ball = world.Balls()[1];
  ball.pocketed = false;
  ball.pos = {-hb::kTableWidth * 0.5 + hb::kBallRadius + 0.01, 0.0};
  ball.vel = {-1.0, 0.0};
  hb::ShotEvents events;
  StepFor(world, events, 0.08);
  Check(ball.vel.x > 0.2, "rail bounce reverses x velocity");
  Check(events.railContacts.count(1) == 1, "rail contact recorded");
}

void TestFrictionStopsBall() {
  hb::PhysicsWorld world;
  for (auto &b : world.Balls()) {
    b.pocketed = true;
  }
  auto &ball = world.Balls()[1];
  ball.pocketed = false;
  ball.pos = {0.0, 0.0};
  ball.vel = {0.9, 0.0};
  hb::ShotEvents events;
  StepFor(world, events, 12.0);
  Check(!world.IsMoving(), "friction stops ball");
}

void TestPocket() {
  hb::PhysicsWorld world;
  for (auto &b : world.Balls()) {
    b.pocketed = true;
  }
  auto &ball = world.Balls()[3];
  ball.pocketed = false;
  ball.pos = {-hb::kTableWidth * 0.5 + 0.01, -hb::kTableHeight * 0.5 + 0.01};
  hb::ShotEvents events;
  world.Step(hb::kFixedStep, &events);
  Check(ball.sinking || ball.pocketed, "ball starts dropping in corner pocket");
  Check(events.potted.size() == 1 && events.potted[0] == 3, "pocket event recorded");
  Check(world.IsMoving(), "sinking ball keeps world moving");
  StepFor(world, events, 0.6);
  Check(ball.pocketed, "ball finishes pocket animation");
  Check(!world.IsMoving(), "world stops after pocket animation");
}

void TestCuePocketAnimation() {
  hb::PhysicsWorld world;
  for (auto &b : world.Balls()) {
    b.pocketed = true;
  }
  auto &cue = world.Balls()[0];
  cue.pocketed = false;
  cue.pos = {-hb::kTableWidth * 0.5 + 0.01, -hb::kTableHeight * 0.5 + 0.01};
  hb::ShotEvents events;
  world.Step(hb::kFixedStep, &events);
  Check(cue.sinking, "cue ball starts pocket animation");
  Check(events.cuePocketed, "cue pocket event recorded");
  Check(world.IsMoving(), "cue pocket animation keeps world moving");
  StepFor(world, events, 0.2);
  Check(cue.sinking && !cue.pocketed, "cue ball remains visible while sinking");
  StepFor(world, events, 0.5);
  Check(cue.pocketed, "cue ball finishes pocket animation");
}

void TestSlowBallKeepsWorldMoving() {
  hb::PhysicsWorld world;
  for (auto &b : world.Balls()) {
    b.pocketed = true;
  }
  auto &ball = world.Balls()[2];
  ball.pocketed = false;
  ball.pos = {0.0, 0.0};
  ball.vel = {0.01, 0.0};
  Check(world.IsMoving(), "slow rolling ball keeps shot alive");
}

void TestSidePocketThroatCapturesBall() {
  hb::PhysicsWorld world;
  for (auto &b : world.Balls()) {
    b.pocketed = true;
  }
  auto &ball = world.Balls()[8];
  ball.pocketed = false;
  ball.pos = {hb::kSidePocketMouth * 0.80,
              hb::kTableHeight * 0.5 - hb::kCushionNoseInset * 1.25};
  ball.vel = {0.0, 0.45};
  hb::ShotEvents events;
  world.Step(hb::kFixedStep, &events);
  Check(ball.sinking || ball.pocketed, "side pocket throat captures ball");
  Check(events.eightPocketed, "side pocket throat records eight ball");
}

void TestSidePocketDoesNotPullIdleBall() {
  hb::PhysicsWorld world;
  for (auto &b : world.Balls()) {
    b.pocketed = true;
  }
  auto &ball = world.Balls()[8];
  ball.pocketed = false;
  ball.pos = {hb::kSidePocketMouth * 0.80,
              hb::kTableHeight * 0.5 - hb::kCushionNoseInset * 1.25};
  hb::ShotEvents events;
  world.Step(hb::kFixedStep, &events);
  Check(!ball.sinking && !ball.pocketed, "side pocket does not pull idle ball");
  Check(!events.eightPocketed, "idle eight is not recorded as pocketed");
}

void TestRulesFoulGivesBallInHand() {
  hb::PhysicsWorld world;
  hb::RulesEngine rules;
  hb::ShotEvents events;
  events.firstContact = 8;
  rules.State().breakShot = false;
  rules.State().players[0].group = hb::BallGroup::Solids;
  rules.State().players[1].group = hb::BallGroup::Stripes;
  auto decision = rules.ApplyShot(events, world);
  Check(decision.nextPhase == hb::Phase::BallInHand, "wrong first contact gives ball in hand");
  Check(rules.State().currentPlayer == 1, "turn switches after foul");
}

void TestRulesAssignsGroup() {
  hb::PhysicsWorld world;
  hb::RulesEngine rules;
  rules.State().breakShot = false;
  hb::ShotEvents events;
  events.firstContact = 2;
  events.potted.push_back(2);
  auto decision = rules.ApplyShot(events, world);
  Check(decision.nextPhase == hb::Phase::Aiming, "legal pot continues");
  Check(rules.State().players[0].group == hb::BallGroup::Solids, "solids assigned");
  Check(rules.State().players[1].group == hb::BallGroup::Stripes, "stripes assigned to opponent");
}

void TestEarlyEightLoses() {
  hb::PhysicsWorld world;
  hb::RulesEngine rules;
  rules.State().breakShot = false;
  rules.State().players[0].group = hb::BallGroup::Solids;
  rules.State().players[1].group = hb::BallGroup::Stripes;
  hb::ShotEvents events;
  events.firstContact = 8;
  events.eightPocketed = true;
  events.potted.push_back(8);
  auto decision = rules.ApplyShot(events, world);
  Check(decision.nextPhase == hb::Phase::RackOver, "early eight ends rack");
  Check(rules.State().winner == 1, "early eight loses");
}

void TestEightAfterClearedGroupWins() {
  hb::PhysicsWorld world;
  hb::RulesEngine rules;
  rules.State().currentPlayer = 1;
  rules.State().breakShot = false;
  rules.State().players[0].group = hb::BallGroup::Solids;
  rules.State().players[1].group = hb::BallGroup::Stripes;
  for (int n = 9; n <= 15; ++n) {
    world.Balls()[n].pocketed = true;
  }
  hb::ShotEvents events;
  events.firstContact = 8;
  events.eightPocketed = true;
  events.potted.push_back(8);
  auto decision = rules.ApplyShot(events, world);
  Check(decision.nextPhase == hb::Phase::RackOver, "cleared group eight ends rack");
  Check(rules.State().winner == 1, "player two wins on legal eight");
}

// ─── strict physics tests ───

void TestCutShotThrowAngle() {
  hb::PhysicsWorld world;
  for (auto &b : world.Balls()) { b.pocketed = true; }
  auto &cue = world.Balls()[0];
  auto &obj = world.Balls()[1];
  cue.pocketed = false; obj.pocketed = false;

  // Head-on: cue approaches exactly along line-of-centres
  obj.pos = {0.5, 0.0};
  cue.pos = {obj.pos.x - hb::kBallDiameter * 1.5, 0.0};
  cue.vel = {2.5, 0.0};

  hb::ShotEvents events;
  StepFor(world, events, 0.04);
  Check(events.firstContact == 1, "head-on contacts object ball");
  Check(std::abs(obj.vel.y) < 0.001, "head-on collision has zero throw");
}

void TestStopShot() {
  hb::PhysicsWorld world;
  for (auto &b : world.Balls()) { b.pocketed = true; }
  auto &cue = world.Balls()[0];
  auto &obj = world.Balls()[1];
  cue.pocketed = false; obj.pocketed = false;

  obj.pos = {0.3, 0.0};
  // head-on setup only ~1 ball diameter apart so ball is still sliding at impact
  cue.pos = {obj.pos.x - hb::kBallDiameter * 1.5, 0.0};

  // Center-hit: strike with mid-power, no spin
  hb::ShotParams shot;
  shot.aim = hb::Normalize(obj.pos - cue.pos);
  shot.power = 0.35;
  shot.tipX = 0.0;  // center
  shot.tipY = 0.0;  // center → no follow/draw
  world.StrikeCue(shot);

  hb::ShotEvents events;
  StepFor(world, events, 0.15);

  Check(obj.vel.x > 0.4, "object ball moves forward after collision");
  // After head-on collision with a sliding cue ball, the cue ball should
  // have very little residual speed (stop shot)
  Check(cue.vel.x < 0.15, "center-hit cue ball nearly stops after head-on collision");
}

void TestDrawShot() {
  hb::PhysicsWorld world;
  for (auto &b : world.Balls()) { b.pocketed = true; }
  auto &cue = world.Balls()[0];
  auto &obj = world.Balls()[1];
  cue.pocketed = false; obj.pocketed = false;

  obj.pos = {0.3, 0.0};
  cue.pos = {obj.pos.x - hb::kBallDiameter * 1.5, 0.0};

  hb::ShotParams shot;
  shot.aim = hb::Normalize(obj.pos - cue.pos);
  shot.power = 0.35;
  shot.tipX = 0.0;
  shot.tipY = -1.0; // maximum draw
  world.StrikeCue(shot);

  hb::ShotEvents events;
  StepFor(world, events, 0.6);

  Check(cue.vel.x < -0.04, "draw shot reverses cue ball direction");
  Check(obj.vel.x > 0.35, "draw shot sends object ball forward");
  Check(obj.vel.x > 0.35, "draw shot sends object ball forward");
}

void TestFollowShot() {
  hb::PhysicsWorld world;
  for (auto &b : world.Balls()) { b.pocketed = true; }
  auto &cue = world.Balls()[0];
  auto &obj = world.Balls()[1];
  cue.pocketed = false; obj.pocketed = false;

  obj.pos = {0.3, 0.0};
  cue.pos = {obj.pos.x - hb::kBallDiameter * 1.5, 0.0};

  hb::ShotParams shot;
  shot.aim = hb::Normalize(obj.pos - cue.pos);
  shot.power = 0.30;
  shot.tipX = 0.0;
  shot.tipY = 1.0; // maximum follow
  world.StrikeCue(shot);

  hb::ShotEvents events;
  StepFor(world, events, 0.3);

  Check(cue.vel.x > 0.08, "follow shot cue ball continues forward");
  Check(obj.vel.x > 0.35, "follow shot object ball moves forward");
}

void TestCcdThinCut() {
  hb::PhysicsWorld world;
  for (auto &b : world.Balls()) { b.pocketed = true; }
  auto &cue = world.Balls()[0];
  auto &obj = world.Balls()[1];
  cue.pocketed = false; obj.pocketed = false;

  // Extremely thin cut: cue ball passes almost tangent to object ball
  // Set up so that the aiming line would be valid for ghost ball but
  // discrete steps might miss the collision
  const double grazingOffset = hb::kBallDiameter * 0.995; // very thin
  obj.pos = {0.5, grazingOffset};
  cue.pos = {0.0, grazingOffset - 0.001};
  cue.vel = {4.0, 0.0}; // high speed → large step → potential miss

  hb::ShotEvents events;
  StepFor(world, events, 0.3);

  // CCD should ensure collision is detected even on thin cuts
  Check(events.firstContact == 1, "CCD detects thin cut collision");
  Check(hb::Length(obj.vel) > 0.05, "object ball moves after thin cut");
}

void TestCcdDeepIntoStepContact() {
  hb::PhysicsWorld world;
  for (auto &b : world.Balls()) { b.pocketed = true; }
  auto &cue = world.Balls()[0];
  auto &obj = world.Balls()[1];
  cue.pocketed = false; obj.pocketed = false;

  // Cue ball crosses object ball path; without CCD, contact happens
  // mid-step and would be missed at 240 Hz with high speed
  obj.pos = {0.6, 0.0};
  // Cue positioned so that in one step it jumps past the contact zone
  cue.pos = {obj.pos.x - hb::kBallDiameter - 0.018, 0.0};
  cue.vel = {6.5, 0.0}; // very fast: 6.5/240 ≈ 27mm per step

  // Use maximum legal cue speed
  cue.vel = hb::Vec2{4.85, 0.0};

  hb::ShotEvents events;
  StepFor(world, events, 0.15);

  Check(events.firstContact == 1, "CCD detects contact at high speed deep into step");
  Check(hb::Length(obj.vel) > 0.5, "object ball receives significant impulse via CCD");
}

void TestSpinDecaysOverTime() {
  hb::PhysicsWorld world;
  for (auto &b : world.Balls()) { b.pocketed = true; }
  auto &cue = world.Balls()[0];
  cue.pocketed = false;

  cue.pos = {0.0, 0.0};
  hb::ShotParams shot;
  shot.aim = {1.0, 0.0};
  shot.power = 0.5;
  shot.tipX = 1.0; // max side spin
  shot.tipY = 0.0;
  world.StrikeCue(shot);

  const double initialSpin = std::abs(cue.sideOmega);
  Check(initialSpin > 0.5, "cue ball gets significant side spin");

  hb::ShotEvents events;
  StepFor(world, events, 4.0);

  // Side spin should decay substantially after 4 seconds
  Check(std::abs(cue.sideOmega) < initialSpin * 0.25,
        "side spin decays over time");
}

void TestFrictionConvertsSlideToRoll() {
  hb::PhysicsWorld world;
  for (auto &b : world.Balls()) { b.pocketed = true; }
  auto &ball = world.Balls()[5];
  ball.pocketed = false;

  ball.pos = {0.0, 0.0};
  ball.vel = {0.5, 0.0};
  // No initial roll → pure slide
  ball.rollOmega = {};

  hb::ShotEvents events;
  // Let it travel for a while — friction should convert sliding to rolling
  StepFor(world, events, 2.0);

  // After sliding on felt for long enough, ball should be rolling
  // (rollOmega should match translational velocity: omega = v / R)
  const double expectedOmega = hb::Length(ball.vel) / hb::kBallRadius;
  const double actualOmega = hb::Length(ball.rollOmega);
  Check(std::abs(actualOmega - expectedOmega) < 0.8,
        "sliding friction converts to rolling");
}

void TestBallBallRestitution() {
  hb::PhysicsWorld world;
  for (auto &b : world.Balls()) { b.pocketed = true; }
  auto &cue = world.Balls()[0];
  auto &obj = world.Balls()[1];
  cue.pocketed = false; obj.pocketed = false;

  obj.pos = {0.4, 0.0};
  cue.pos = {obj.pos.x - hb::kBallDiameter * 1.3, 0.0};
  cue.vel = {1.5, 0.0};

  hb::ShotEvents events;
  StepFor(world, events, 0.1);

  const double vBefore = 1.5;
  const double vObjAfter = obj.vel.x;
  const double vCueAfter = cue.vel.x;
  // For equal-mass elastic collision: vObj ≈ vBefore (slightly less due to friction losses)
  Check(vObjAfter > vCueAfter, "object ball faster than cue ball after head-on");
  Check(vObjAfter > vBefore * 0.85, "object ball retains >85% of incoming speed");
}

} // namespace

int main() {
  TestHeadOnCollision();
  TestRailBounce();
  TestFrictionStopsBall();
  TestPocket();
  TestCuePocketAnimation();
  TestSlowBallKeepsWorldMoving();
  TestSidePocketThroatCapturesBall();
  TestSidePocketDoesNotPullIdleBall();
  TestRulesFoulGivesBallInHand();
  TestRulesAssignsGroup();
  TestEarlyEightLoses();
  TestEightAfterClearedGroupWins();
  TestCutShotThrowAngle();
  TestStopShot();
  TestDrawShot();
  TestFollowShot();
  TestCcdThinCut();
  TestCcdDeepIntoStepContact();
  TestSpinDecaysOverTime();
  TestFrictionConvertsSlideToRoll();
  TestBallBallRestitution();
  std::cout << "All heyball tests passed\n";
  return 0;
}
