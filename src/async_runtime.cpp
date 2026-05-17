#include "async_runtime.hpp"

PhysicsRunner::PhysicsRunner() : worker_(&PhysicsRunner::ThreadMain, this) {}

PhysicsRunner::~PhysicsRunner() { Shutdown(); }

void PhysicsRunner::Begin(const hb::PhysicsWorld &world, const hb::ShotParams *shot) {
  std::lock_guard<std::mutex> lock(mutex_);
  world_ = world;
  events_.Clear();
  if (shot) {
    world_.StrikeCue(*shot);
  }
  active_ = true;
  finished_ = false;
}

void PhysicsRunner::Stop() {
  std::lock_guard<std::mutex> lock(mutex_);
  active_ = false;
  finished_ = false;
}

void PhysicsRunner::Snapshot(hb::PhysicsWorld *world, hb::ShotEvents *events,
                             bool *finished) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (world) *world = world_;
  if (events) *events = events_;
  if (finished) *finished = finished_;
}

bool PhysicsRunner::Active() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return active_;
}

void PhysicsRunner::Shutdown() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    stop_ = true;
    active_ = false;
  }
  if (worker_.joinable()) {
    worker_.join();
  }
}

void PhysicsRunner::ThreadMain() {
  using Clock = std::chrono::steady_clock;
  auto last = Clock::now();
  for (;;) {
    bool shouldStep = false;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (stop_) break;
      shouldStep = active_;
    }
    if (!shouldStep) {
      last = Clock::now();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    const auto now = Clock::now();
    const double dt = std::chrono::duration<double>(now - last).count();
    last = now;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!active_) continue;
      world_.Step(dt, &events_);
      if (!world_.IsMoving()) {
        active_ = false;
        finished_ = true;
      }
    }
  }
}

AsyncNetworkHost::~AsyncNetworkHost() { Stop(); }

bool AsyncNetworkHost::Start(const std::string &roomName, const std::string &password,
                             bool hostIsPlayer1) {
  StopWorker();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!host_.Start(roomName, password, hostIsPlayer1)) {
      return false;
    }
    ClearFrameEventsLocked();
    ClearQueuedEventsLocked();
    running_ = true;
  }
  worker_ = std::thread(&AsyncNetworkHost::ThreadMain, this);
  return true;
}

void AsyncNetworkHost::Stop() {
  StopWorker();
  std::lock_guard<std::mutex> lock(mutex_);
  host_.Stop();
  ClearFrameEventsLocked();
  ClearQueuedEventsLocked();
}

void AsyncNetworkHost::Poll() {
  std::lock_guard<std::mutex> lock(mutex_);
  ClearFrameEventsLocked();
  frame_ = queued_;
  ClearQueuedEventsLocked();
}

bool AsyncNetworkHost::HasJoinRequest(std::string *password) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (frame_.hasJoin && password) *password = frame_.joinPwd;
  return frame_.hasJoin;
}

bool AsyncNetworkHost::HasReady() const { return LockedFlag(&HostEvents::hasReady); }

bool AsyncNetworkHost::HasUnready() const { return LockedFlag(&HostEvents::hasUnready); }

bool AsyncNetworkHost::HasCuePlacement(hb::Vec2 *pos, bool *confirmed) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (frame_.hasCuePlacement) {
    if (pos) *pos = frame_.cuePlacementPos;
    if (confirmed) *confirmed = frame_.cuePlacementConfirmed;
  }
  return frame_.hasCuePlacement;
}

bool AsyncNetworkHost::HasGroupChoice(hb::BallGroup *group) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (frame_.hasGroupChoice && group) *group = frame_.groupChoice;
  return frame_.hasGroupChoice;
}

bool AsyncNetworkHost::HasShot(hb::ShotParams *out) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (frame_.hasShot && out) *out = frame_.shot;
  return frame_.hasShot;
}

bool AsyncNetworkHost::HasAim(double *tipX, double *tipY, double *power,
                              double *aimX, double *aimY) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (frame_.hasAim) {
    if (tipX) *tipX = frame_.aimTipX;
    if (tipY) *tipY = frame_.aimTipY;
    if (power) *power = frame_.aimPower;
    if (aimX) *aimX = frame_.aimX;
    if (aimY) *aimY = frame_.aimY;
  }
  return frame_.hasAim;
}

bool AsyncNetworkHost::HasChat(std::string *msg) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (frame_.hasChat && msg) *msg = frame_.chatMsg;
  return frame_.hasChat;
}

bool AsyncNetworkHost::HasBye() const { return LockedFlag(&HostEvents::hasBye); }

bool AsyncNetworkHost::HasClient() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return host_.HasClient();
}

bool AsyncNetworkHost::ClientDisconnected() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return frame_.clientGone;
}

void AsyncNetworkHost::SendAccept() { WithHost([](hb::NetworkHost &h) { h.SendAccept(); }); }

void AsyncNetworkHost::SendReject(const std::string &reason) {
  WithHost([&](hb::NetworkHost &h) { h.SendReject(reason); });
}

void AsyncNetworkHost::SendReadyAck() {
  WithHost([](hb::NetworkHost &h) { h.SendReadyAck(); });
}

void AsyncNetworkHost::SendReady() { WithHost([](hb::NetworkHost &h) { h.SendReady(); }); }

void AsyncNetworkHost::SendUnready() {
  WithHost([](hb::NetworkHost &h) { h.SendUnready(); });
}

void AsyncNetworkHost::SendAssign(int player) {
  WithHost([&](hb::NetworkHost &h) { h.SendAssign(player); });
}

void AsyncNetworkHost::SendTurn(int player) {
  WithHost([&](hb::NetworkHost &h) { h.SendTurn(player); });
}

void AsyncNetworkHost::SendState(hb::Phase phase, const hb::RulesState &state) {
  WithHost([&](hb::NetworkHost &h) { h.SendState(phase, state); });
}

void AsyncNetworkHost::SendPositions(const std::array<hb::Ball, 16> &balls) {
  WithHost([&](hb::NetworkHost &h) { h.SendPositions(balls); });
}

void AsyncNetworkHost::SendShot(double tipX, double tipY, double power,
                                double aimX, double aimY) {
  WithHost([&](hb::NetworkHost &h) {
    h.SendShot(tipX, tipY, power, aimX, aimY);
  });
}

void AsyncNetworkHost::SendChat(const std::string &msg) {
  WithHost([&](hb::NetworkHost &h) { h.SendChat(msg); });
}

void AsyncNetworkHost::SendAim(double tipX, double tipY, double power,
                               double aimX, double aimY) {
  WithHost([&](hb::NetworkHost &h) {
    h.SendAim(tipX, tipY, power, aimX, aimY);
  });
}

void AsyncNetworkHost::SendBye() { WithHost([](hb::NetworkHost &h) { h.SendBye(); }); }

void AsyncNetworkHost::SendRoomClosed() {
  WithHost([](hb::NetworkHost &h) { h.SendRoomClosed(); });
}

bool AsyncNetworkHost::LockedFlag(bool HostEvents::*member) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return frame_.*member;
}

void AsyncNetworkHost::StopWorker() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    running_ = false;
  }
  if (worker_.joinable()) {
    worker_.join();
  }
}

void AsyncNetworkHost::ThreadMain() {
  while (true) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!running_) break;
      host_.Poll();
      CaptureEventsLocked();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(4));
  }
}

void AsyncNetworkHost::CaptureEventsLocked() {
  std::string pwd;
  if (host_.HasJoinRequest(&pwd)) {
    queued_.hasJoin = true;
    queued_.joinPwd = pwd;
  }
  if (host_.HasReady()) queued_.hasReady = true;
  if (host_.HasUnready()) queued_.hasUnready = true;
  hb::Vec2 cuePos{};
  bool cueConfirmed = false;
  if (host_.HasCuePlacement(&cuePos, &cueConfirmed)) {
    queued_.hasCuePlacement = true;
    queued_.cuePlacementPos = cuePos;
    queued_.cuePlacementConfirmed = cueConfirmed;
  }
  hb::BallGroup group = hb::BallGroup::Open;
  if (host_.HasGroupChoice(&group)) {
    queued_.hasGroupChoice = true;
    queued_.groupChoice = group;
  }
  hb::ShotParams shot;
  if (host_.HasShot(&shot)) {
    queued_.hasShot = true;
    queued_.shot = shot;
  }
  double tx, ty, power, ax, ay;
  if (host_.HasAim(&tx, &ty, &power, &ax, &ay)) {
    queued_.hasAim = true;
    queued_.aimTipX = tx;
    queued_.aimTipY = ty;
    queued_.aimPower = power;
    queued_.aimX = ax;
    queued_.aimY = ay;
  }
  std::string chat;
  if (host_.HasChat(&chat)) {
    queued_.hasChat = true;
    queued_.chatMsg = chat;
  }
  if (host_.HasBye()) queued_.hasBye = true;
  if (host_.ClientDisconnected()) queued_.clientGone = true;
}

void AsyncNetworkHost::ClearFrameEventsLocked() { frame_ = HostEvents{}; }

void AsyncNetworkHost::ClearQueuedEventsLocked() { queued_ = HostEvents{}; }

AsyncNetworkClient::~AsyncNetworkClient() { Stop(); }

bool AsyncNetworkClient::Start() {
  StopWorker();
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!client_.Start()) {
      return false;
    }
    ClearFrameEventsLocked();
    ClearQueuedEventsLocked();
    running_ = true;
  }
  worker_ = std::thread(&AsyncNetworkClient::ThreadMain, this);
  return true;
}

void AsyncNetworkClient::Stop() {
  StopWorker();
  std::lock_guard<std::mutex> lock(mutex_);
  client_.Stop();
  ClearFrameEventsLocked();
  ClearQueuedEventsLocked();
}

void AsyncNetworkClient::Poll() {
  std::lock_guard<std::mutex> lock(mutex_);
  ClearFrameEventsLocked();
  frame_ = queued_;
  ClearQueuedEventsLocked();
}

std::vector<hb::RoomInfo> AsyncNetworkClient::GetRooms() {
  std::lock_guard<std::mutex> lock(mutex_);
  return client_.GetRooms();
}

bool AsyncNetworkClient::Connect(const std::string &hostIP, const std::string &password,
                                 const std::string &roomId) {
  std::lock_guard<std::mutex> lock(mutex_);
  return client_.Connect(hostIP, password, roomId);
}

void AsyncNetworkClient::Disconnect() {
  WithClient([](hb::NetworkClient &c) { c.Disconnect(); });
}

bool AsyncNetworkClient::IsConnecting() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return client_.IsConnecting();
}

bool AsyncNetworkClient::IsAccepted() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return client_.IsAccepted();
}

std::string AsyncNetworkClient::RejectReason() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return client_.RejectReason();
}

bool AsyncNetworkClient::HasReadyAck() const {
  return LockedFlag(&ClientEvents::hasReadyAck);
}

bool AsyncNetworkClient::HasUnreadyAck() const {
  return LockedFlag(&ClientEvents::hasUnreadyAck);
}

bool AsyncNetworkClient::HasAssign(int *player) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (frame_.hasAssign && player) *player = frame_.assignPlayer;
  return frame_.hasAssign;
}

bool AsyncNetworkClient::HasTurn(int *player) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (frame_.hasTurn && player) *player = frame_.turnPlayer;
  return frame_.hasTurn;
}

bool AsyncNetworkClient::HasState(hb::Phase *phase, hb::RulesState *state) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (frame_.hasState) {
    if (phase) *phase = frame_.statePhase;
    if (state) *state = frame_.rulesState;
  }
  return frame_.hasState;
}

bool AsyncNetworkClient::HasPositions(std::array<hb::Ball, 16> *balls) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (frame_.hasPositions && balls) *balls = frame_.balls;
  return frame_.hasPositions;
}

bool AsyncNetworkClient::HasShot(hb::ShotParams *out) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (frame_.hasShot && out) *out = frame_.shot;
  return frame_.hasShot;
}

bool AsyncNetworkClient::HasAim(double *tipX, double *tipY, double *power,
                                double *aimX, double *aimY) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (frame_.hasAim) {
    if (tipX) *tipX = frame_.aimTipX;
    if (tipY) *tipY = frame_.aimTipY;
    if (power) *power = frame_.aimPower;
    if (aimX) *aimX = frame_.aimX;
    if (aimY) *aimY = frame_.aimY;
  }
  return frame_.hasAim;
}

bool AsyncNetworkClient::HasChat(std::string *msg) const {
  std::lock_guard<std::mutex> lock(mutex_);
  if (frame_.hasChat && msg) *msg = frame_.chatMsg;
  return frame_.hasChat;
}

bool AsyncNetworkClient::HasBye() const { return LockedFlag(&ClientEvents::hasBye); }

bool AsyncNetworkClient::HasRoomClosed() const {
  return LockedFlag(&ClientEvents::hasRoomClosed);
}

bool AsyncNetworkClient::Disconnected() const {
  return LockedFlag(&ClientEvents::disconnected);
}

void AsyncNetworkClient::SendReady() {
  WithClient([](hb::NetworkClient &c) { c.SendReady(); });
}

void AsyncNetworkClient::SendUnready() {
  WithClient([](hb::NetworkClient &c) { c.SendUnready(); });
}

void AsyncNetworkClient::SendCuePlacement(hb::Vec2 pos, bool confirmed) {
  WithClient([&](hb::NetworkClient &c) { c.SendCuePlacement(pos, confirmed); });
}

void AsyncNetworkClient::SendGroupChoice(hb::BallGroup group) {
  WithClient([&](hb::NetworkClient &c) { c.SendGroupChoice(group); });
}

void AsyncNetworkClient::SendShot(double tipX, double tipY, double power,
                                  double aimX, double aimY) {
  WithClient([&](hb::NetworkClient &c) {
    c.SendShot(tipX, tipY, power, aimX, aimY);
  });
}

void AsyncNetworkClient::SendAim(double tipX, double tipY, double power,
                                 double aimX, double aimY) {
  WithClient([&](hb::NetworkClient &c) {
    c.SendAim(tipX, tipY, power, aimX, aimY);
  });
}

void AsyncNetworkClient::SendChat(const std::string &msg) {
  WithClient([&](hb::NetworkClient &c) { c.SendChat(msg); });
}

void AsyncNetworkClient::SendBye() {
  WithClient([](hb::NetworkClient &c) { c.SendBye(); });
}

bool AsyncNetworkClient::LockedFlag(bool ClientEvents::*member) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return frame_.*member;
}

void AsyncNetworkClient::StopWorker() {
  {
    std::lock_guard<std::mutex> lock(mutex_);
    running_ = false;
  }
  if (worker_.joinable()) {
    worker_.join();
  }
}

void AsyncNetworkClient::ThreadMain() {
  while (true) {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!running_) break;
      client_.Poll();
      CaptureEventsLocked();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(4));
  }
}

void AsyncNetworkClient::CaptureEventsLocked() {
  if (client_.HasReadyAck()) queued_.hasReadyAck = true;
  if (client_.HasUnreadyAck()) queued_.hasUnreadyAck = true;
  int player = -1;
  if (client_.HasAssign(&player)) {
    queued_.hasAssign = true;
    queued_.assignPlayer = player;
  }
  if (client_.HasTurn(&player)) {
    queued_.hasTurn = true;
    queued_.turnPlayer = player;
  }
  hb::Phase phase = hb::Phase::Aiming;
  hb::RulesState state{};
  if (client_.HasState(&phase, &state)) {
    queued_.hasState = true;
    queued_.statePhase = phase;
    queued_.rulesState = state;
  }
  std::array<hb::Ball, 16> balls{};
  if (client_.HasPositions(&balls)) {
    queued_.hasPositions = true;
    queued_.balls = balls;
  }
  hb::ShotParams shot;
  if (client_.HasShot(&shot)) {
    queued_.hasShot = true;
    queued_.shot = shot;
  }
  double tx, ty, power, ax, ay;
  if (client_.HasAim(&tx, &ty, &power, &ax, &ay)) {
    queued_.hasAim = true;
    queued_.aimTipX = tx;
    queued_.aimTipY = ty;
    queued_.aimPower = power;
    queued_.aimX = ax;
    queued_.aimY = ay;
  }
  std::string chat;
  if (client_.HasChat(&chat)) {
    queued_.hasChat = true;
    queued_.chatMsg = chat;
  }
  if (client_.HasBye()) queued_.hasBye = true;
  if (client_.HasRoomClosed()) queued_.hasRoomClosed = true;
  if (client_.Disconnected()) queued_.disconnected = true;
}

void AsyncNetworkClient::ClearFrameEventsLocked() { frame_ = ClientEvents{}; }

void AsyncNetworkClient::ClearQueuedEventsLocked() { queued_ = ClientEvents{}; }
