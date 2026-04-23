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
  ball.pos = {-hb::kTableWidth * 0.5 + 0.02, -hb::kTableHeight * 0.5 + 0.02};
  hb::ShotEvents events;
  world.Step(hb::kFixedStep, &events);
  Check(ball.sinking || ball.pocketed, "ball starts dropping in corner pocket");
  Check(events.potted.size() == 1 && events.potted[0] == 3, "pocket event recorded");
  StepFor(world, events, 0.6);
  Check(ball.pocketed, "ball finishes pocket animation");
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

} // namespace

int main() {
  TestHeadOnCollision();
  TestRailBounce();
  TestFrictionStopsBall();
  TestPocket();
  TestRulesFoulGivesBallInHand();
  TestRulesAssignsGroup();
  TestEarlyEightLoses();
  std::cout << "All heyball tests passed\n";
  return 0;
}
