#include "network.hpp"
#include "core.hpp"
#include "mock_raylib.h"

#include <cstdio>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <string>

namespace {

int gFailures = 0;
void Check(bool ok, const char *name) {
  if (!ok) { std::cerr << "FAIL: " << name << "\n"; ++gFailures; }
}
void Msg(const char *m) { std::cout << "  " << m << "\n"; }
void Sleep0() { Sleep(1); }
const char *kLocalHost = "127.0.0.1";

// Poll for a fixed duration
void PollFor(double sec, hb::NetworkHost &h, hb::NetworkClient &c) {
  double t0 = GetTime();
  while (GetTime() - t0 < sec) { h.Poll(); c.Poll(); Sleep0(); }
  h.Poll(); c.Poll();
}

// Poll until a condition is true
bool Await(double timeoutSec, std::function<bool()> cond,
           hb::NetworkHost *h, hb::NetworkClient *c) {
  double t0 = GetTime();
  while (GetTime() - t0 < timeoutSec) {
    if (h) h->Poll();
    if (c) c->Poll();
    if (cond()) return true;
    Sleep0();
  }
  if (h) h->Poll();
  if (c) c->Poll();
  return cond();
}

// Connect and wait for accept
bool ConnectAndAccept(hb::NetworkHost &host, hb::NetworkClient &client,
                      const char *pwd) {
  if (!client.Connect(kLocalHost, pwd)) return false;
  return Await(1.0, [&]() { return client.IsAccepted(); }, &host, &client);
}

// ─── Test 1: Full join flow ───

void T1_JoinFlow() {
  Msg("T1_JoinFlow");
  hb::NetworkHost host;
  hb::NetworkClient client;
  Check(host.Start("Room", "", true), "host starts");
  Check(client.Start(), "client starts");
  PollFor(0.6, host, client);
  auto rooms = client.GetRooms();
  Check(!rooms.empty(), "client discovers room");
  if (rooms.empty()) { host.Stop(); client.Stop(); return; }
  Check(rooms[0].name == "Room", "room name matches");

  Check(ConnectAndAccept(host, client, ""), "client accepted");
  Check(host.HasClient(), "host has client");
  Check(!client.IsConnecting(), "client done connecting");

  client.Disconnect();
  host.Stop();
  client.Stop();
}

// ─── Test 2: Password reject ───

void T2_PasswordReject() {
  Msg("T2_PasswordReject");
  hb::NetworkHost host;
  hb::NetworkClient client;
  Check(host.Start("PwdRoom", "secret", true), "host starts");
  Check(client.Start(), "client starts");
  PollFor(0.6, host, client);
  auto rooms = client.GetRooms();
  Check(!rooms.empty(), "room found");
  if (rooms.empty()) { host.Stop(); client.Stop(); return; }

  Check(client.Connect(kLocalHost, "wrong"), "connect returns true");
  bool rejected = Await(2.0, [&]() { return client.Disconnected(); }, &host, &client);
  Check(rejected, "client disconnected (rejected)");
  Check(!client.IsAccepted(), "client not accepted");

  client.Disconnect();
  host.Stop();
  client.Stop();
}

// ─── Test 3: Ready / Unready ───

void T3_Ready() {
  Msg("T3_Ready");
  hb::NetworkHost host;
  hb::NetworkClient client;
  Check(host.Start("Ready", "", true), "host starts");
  Check(client.Start(), "client starts");
  PollFor(0.6, host, client);
  Check(ConnectAndAccept(host, client, ""), "accepted");

  client.SendReady();
  Check(Await(0.5, [&]() { return host.HasReady(); }, &host, &client), "host got READY");

  client.SendUnready();
  Check(Await(0.5, [&]() { return host.HasUnready(); }, &host, &client), "host got UNREADY");

  host.SendReady();
  Check(Await(0.5, [&]() { return client.HasReadyAck(); }, &host, &client), "client got READY_ACK");

  host.SendUnready();
  Check(Await(0.5, [&]() { return client.HasUnreadyAck(); }, &host, &client), "client got UNREADY_ACK");

  client.Disconnect();
  host.Stop();
  client.Stop();
}

// ─── Test 4: Chat ───

void T4_Chat() {
  Msg("T4_Chat");
  hb::NetworkHost host;
  hb::NetworkClient client;
  Check(host.Start("Chat", "", true), "host starts");
  Check(client.Start(), "client starts");
  PollFor(0.6, host, client);
  Check(ConnectAndAccept(host, client, ""), "accepted");

  client.SendChat("Hello from client");
  Check(Await(0.5, [&]() { return host.HasChat(nullptr); }, &host, &client), "host got chat");
  std::string m; host.HasChat(&m);
  Check(m == "Hello from client", "host chat correct");

  host.SendChat("Hi from host");
  Check(Await(0.5, [&]() { return client.HasChat(nullptr); }, &host, &client), "client got chat");
  client.HasChat(&m);
  Check(m == "Hi from host", "client chat correct");

  client.SendChat("\xe4\xbd\xa0\xe5\xa5\xbd\xe4\xb8\x96\xe7\x95\x8c"); // "你好世界"
  Check(Await(0.5, [&]() { return host.HasChat(nullptr); }, &host, &client), "host got CN chat");
  host.HasChat(&m);
  Check(m == "\xe4\xbd\xa0\xe5\xa5\xbd\xe4\xb8\x96\xe7\x95\x8c", "host CN intact");

  client.Disconnect();
  host.Stop();
  client.Stop();
}

// ─── Test 5: Assign / Turn / POS ───

void T5_GameSync() {
  Msg("T5_GameSync");
  // hostIsPlayer1 = true �?client is player 1
  hb::NetworkHost host;
  hb::NetworkClient client;
  Check(host.Start("Game1", "", true), "host starts P1");
  Check(client.Start(), "client starts");
  PollFor(0.6, host, client);
  Check(ConnectAndAccept(host, client, ""), "accepted");

  host.SendAssign(1);
  Check(Await(0.5, [&]() { return client.HasAssign(nullptr); }, &host, &client), "client got ASSIGN");
  int p; client.HasAssign(&p); Check(p == 1, "client is player 1");

  host.SendTurn(0);
  Check(Await(0.5, [&]() { return client.HasTurn(nullptr); }, &host, &client), "client got TURN");
  int t; client.HasTurn(&t); Check(t == 0, "turn = 0");

  std::array<hb::Ball, 16> balls{};
  balls[0].pos = {1.0, 2.0};
  balls[1].pos = {3.0, 4.0};
  host.SendPositions(balls);
  Check(Await(0.5, [&]() { return client.HasPositions(nullptr); }, &host, &client), "client got POS");
  std::array<hb::Ball, 16> synced{};
  client.HasPositions(&synced);
  Check(synced[0].pos.x == 1.0 && synced[0].pos.y == 2.0, "pos[0] ok");
  Check(synced[1].pos.x == 3.0 && synced[1].pos.y == 4.0, "pos[1] ok");

  client.Disconnect();
  host.Stop();
  client.Stop();

  // hostIsPlayer1 = false �?client is player 0
  Msg("T5_GameSync (hostIsP1=false)");
  hb::NetworkHost host2;
  Check(host2.Start("Game2", "", false), "host starts P2");
  Check(client.Start(), "client starts");
  PollFor(0.6, host2, client);
  Check(ConnectAndAccept(host2, client, ""), "accepted");

  host2.SendAssign(0);
  Check(Await(0.5, [&]() { return client.HasAssign(nullptr); }, &host2, &client), "client got ASSIGN");
  client.HasAssign(&p); Check(p == 0, "client is player 0");

  client.Disconnect();
  host2.Stop();
  client.Stop();
}

// ─── Test 6: Shot relay ───

void T6_Shot() {
  Msg("T6_Shot");
  hb::NetworkHost host;
  hb::NetworkClient client;
  Check(host.Start("Shot", "", true), "host starts");
  Check(client.Start(), "client starts");
  PollFor(0.6, host, client);
  Check(ConnectAndAccept(host, client, ""), "accepted");

  client.SendShot(0.5, -0.3, 0.75, 1.0, 0.0);
  Check(Await(0.5, [&]() { return host.HasShot(nullptr); }, &host, &client), "host got SHOT");
  hb::ShotParams shot;
  host.HasShot(&shot);
  Check(std::abs(shot.tipX - 0.5) < 0.001, "tipX ok");
  Check(std::abs(shot.tipY + 0.3) < 0.001, "tipY ok");
  Check(std::abs(shot.power - 0.75) < 0.001, "power ok");

  client.Disconnect();
  host.Stop();
  client.Stop();
}

// ─── Test 7: Aim sync ───

void T7_Aim() {
  Msg("T7_Aim");
  hb::NetworkHost host;
  hb::NetworkClient client;
  Check(host.Start("Aim", "", true), "host starts");
  Check(client.Start(), "client starts");
  PollFor(0.6, host, client);
  Check(ConnectAndAccept(host, client, ""), "accepted");

  client.SendAim(0.2, 0.8, 0.45);
  Check(Await(0.5, [&]() { return host.HasAim(nullptr,nullptr,nullptr); }, &host, &client), "host got AIM");
  double tx, ty, pw;
  host.HasAim(&tx, &ty, &pw);
  Check(std::abs(tx - 0.2) < 0.001, "aim tx ok");
  Check(std::abs(ty - 0.8) < 0.001, "aim ty ok");
  Check(std::abs(pw - 0.45) < 0.001, "aim pw ok");

  host.SendAim(-0.5, 0.0, 0.9);
  Check(Await(0.5, [&]() { return client.HasAim(nullptr,nullptr,nullptr); }, &host, &client), "client got AIM");
  client.HasAim(&tx, &ty, &pw);
  Check(std::abs(tx + 0.5) < 0.001, "host aim tx ok");
  Check(std::abs(pw - 0.9) < 0.001, "host aim pw ok");

  client.Disconnect();
  host.Stop();
  client.Stop();
}

// ─── Test 8: Disconnect / BYE ───

void T8_Disconnect() {
  Msg("T8_Disconnect");
  hb::NetworkHost host;
  hb::NetworkClient client;
  Check(host.Start("Bye", "", true), "host starts");
  Check(client.Start(), "client starts");
  PollFor(0.6, host, client);
  Check(ConnectAndAccept(host, client, ""), "accepted");

  client.SendBye();
  Check(Await(0.5, [&]() { return host.HasBye(); }, &host, &client), "host got BYE");

  client.Disconnect();
  Check(Await(1.0, [&]() { return host.ClientDisconnected(); }, &host, &client), "host sees disconnect");

  host.Stop();
  client.Stop();
}

// ─── Test 9: Room closed ───

void T9_RoomClosed() {
  Msg("T9_RoomClosed");
  hb::NetworkHost host;
  hb::NetworkClient client;
  Check(host.Start("Close", "", true), "host starts");
  Check(client.Start(), "client starts");
  PollFor(0.6, host, client);
  Check(ConnectAndAccept(host, client, ""), "accepted");

  host.SendRoomClosed();
  Check(Await(0.5, [&]() { return client.HasRoomClosed(); }, &host, &client), "client got ROOM_CLOSED");

  client.Disconnect();
  host.Stop();
  client.Stop();
}

// ─── Test 10: Rapid message burst ───

void T10_Burst() {
  Msg("T10_Burst");
  hb::NetworkHost host;
  hb::NetworkClient client;
  Check(host.Start("Burst", "", true), "host starts");
  Check(client.Start(), "client starts");
  PollFor(0.6, host, client);
  Check(ConnectAndAccept(host, client, ""), "accepted");

  // Send multiple messages with enough delay for separate TCP segments
  for (int i = 0; i < 5; ++i) {
    client.SendChat("burst" + std::to_string(i));
    Sleep(5);
  }

  int count = 0;
  bool ok = Await(1.0, [&]() {
    if (host.HasChat(nullptr)) { ++count; return true; }
    return false;
  }, &host, &client);
  Check(ok && count >= 1, "burst messages received");

  client.Disconnect();
  host.Stop();
  client.Stop();
}

// ─── Test 11: Poll integrity ───

void T11_PollIntegrity() {
  Msg("T11_PollIntegrity");
  hb::NetworkHost host;
  hb::NetworkClient client;
  Check(host.Start("Poll", "", true), "host starts");
  Check(client.Start(), "client starts");

  // Poll many times without client connected �?no crash
  for (int i = 0; i < 100; ++i) { host.Poll(); client.Poll(); Sleep(1); }
  Check(true, "no crash polling without client");

  PollFor(0.5, host, client);
  Check(ConnectAndAccept(host, client, ""), "accepted");

  for (int i = 0; i < 100; ++i) { host.Poll(); client.Poll(); Sleep(1); }
  Check(true, "no crash polling with client");

  client.Disconnect();
  host.Stop();
  client.Stop();
}

} // namespace

int main() {
  std::cout << "=== Network Tests ===\n" << std::flush;
  double t0 = GetTime();
  auto totalFailures = []() { return gFailures; };
  double ts;

  ts = GetTime(); T1_JoinFlow();
  std::cout << "  T1 done " << totalFailures() << "f " << (GetTime()-ts) << "s\n" << std::flush;
  ts = GetTime(); T2_PasswordReject();
  std::cout << "  T2 done " << totalFailures() << "f " << (GetTime()-ts) << "s\n" << std::flush;
  ts = GetTime(); T3_Ready();
  std::cout << "  T3 done " << totalFailures() << "f " << (GetTime()-ts) << "s\n" << std::flush;
  ts = GetTime(); T4_Chat();
  std::cout << "  T4 done " << totalFailures() << "f " << (GetTime()-ts) << "s\n" << std::flush;
  ts = GetTime(); T5_GameSync();
  std::cout << "  T5 done " << totalFailures() << "f " << (GetTime()-ts) << "s\n" << std::flush;
  ts = GetTime(); T6_Shot();
  std::cout << "  T6 done " << totalFailures() << "f " << (GetTime()-ts) << "s\n" << std::flush;
  ts = GetTime(); T7_Aim();
  std::cout << "  T7 done " << totalFailures() << "f " << (GetTime()-ts) << "s\n" << std::flush;
  ts = GetTime(); T8_Disconnect();
  std::cout << "  T8 done " << totalFailures() << "f " << (GetTime()-ts) << "s\n" << std::flush;
  ts = GetTime(); T9_RoomClosed();
  std::cout << "  T9 done " << totalFailures() << "f " << (GetTime()-ts) << "s\n" << std::flush;
  ts = GetTime(); T10_Burst();
  std::cout << "  T10 done " << totalFailures() << "f " << (GetTime()-ts) << "s\n" << std::flush;
  ts = GetTime(); T11_PollIntegrity();
  std::cout << "  T11 done " << totalFailures() << "f " << (GetTime()-ts) << "s\n" << std::flush;

  std::cout << "\nTotal: " << (GetTime()-t0) << "s, " << gFailures << " failures\n" << std::flush;
  return gFailures ? 1 : 0;
}
