#pragma once

#include "core.hpp"
#include "network.hpp"
#include "platform_threads.hpp"

#include <array>
#include <string>
#include <vector>

class PhysicsRunner {
public:
  PhysicsRunner();
  ~PhysicsRunner();

  PhysicsRunner(const PhysicsRunner &) = delete;
  PhysicsRunner &operator=(const PhysicsRunner &) = delete;

  void Begin(const hb::PhysicsWorld &world, const hb::ShotParams *shot = nullptr);
  void Stop();
  void Snapshot(hb::PhysicsWorld *world, hb::ShotEvents *events,
                bool *finished = nullptr) const;
  bool Active() const;

private:
  void Shutdown();
  void ThreadMain();

  mutable std::mutex mutex_;
  std::thread worker_;
  hb::PhysicsWorld world_;
  hb::ShotEvents events_;
  bool active_ = false;
  bool finished_ = false;
  bool stop_ = false;
};

class AsyncNetworkHost {
public:
  AsyncNetworkHost() = default;
  ~AsyncNetworkHost();

  AsyncNetworkHost(const AsyncNetworkHost &) = delete;
  AsyncNetworkHost &operator=(const AsyncNetworkHost &) = delete;

  bool Start(const std::string &roomName, const std::string &password,
             bool hostIsPlayer1);
  void Stop();
  void Poll();

  bool HasJoinRequest(std::string *password = nullptr) const;
  bool HasReady() const;
  bool HasUnready() const;
  bool HasShot(hb::ShotParams *out) const;
  bool HasAim(double *tipX, double *tipY, double *power, double *aimX,
              double *aimY) const;
  bool HasChat(std::string *msg = nullptr) const;
  bool HasBye() const;
  bool HasClient() const;
  bool ClientDisconnected() const;

  void SendAccept();
  void SendReject(const std::string &reason);
  void SendReadyAck();
  void SendReady();
  void SendUnready();
  void SendAssign(int player);
  void SendTurn(int player);
  void SendPositions(const std::array<hb::Ball, 16> &balls);
  void SendShot(double tipX, double tipY, double power, double aimX, double aimY);
  void SendChat(const std::string &msg);
  void SendAim(double tipX, double tipY, double power, double aimX, double aimY);
  void SendBye();
  void SendRoomClosed();

private:
  struct HostEvents {
    bool hasJoin = false;
    bool hasReady = false;
    bool hasUnready = false;
    bool hasShot = false;
    bool hasAim = false;
    bool hasChat = false;
    bool hasBye = false;
    bool clientGone = false;
    std::string joinPwd;
    hb::ShotParams shot;
    double aimTipX = 0.0;
    double aimTipY = 0.0;
    double aimPower = 0.0;
    double aimX = 1.0;
    double aimY = 0.0;
    std::string chatMsg;
  };

  bool LockedFlag(bool HostEvents::*member) const;

  template <typename Fn>
  void WithHost(Fn fn) {
    std::lock_guard<std::mutex> lock(mutex_);
    fn(host_);
  }

  void StopWorker();
  void ThreadMain();
  void CaptureEventsLocked();
  void ClearFrameEventsLocked();
  void ClearQueuedEventsLocked();

  mutable std::mutex mutex_;
  std::thread worker_;
  hb::NetworkHost host_;
  HostEvents queued_;
  HostEvents frame_;
  bool running_ = false;
};

class AsyncNetworkClient {
public:
  AsyncNetworkClient() = default;
  ~AsyncNetworkClient();

  AsyncNetworkClient(const AsyncNetworkClient &) = delete;
  AsyncNetworkClient &operator=(const AsyncNetworkClient &) = delete;

  bool Start();
  void Stop();
  void Poll();
  std::vector<hb::RoomInfo> GetRooms();
  bool Connect(const std::string &hostIP, const std::string &password);
  void Disconnect();
  bool IsConnecting() const;
  bool IsAccepted() const;
  std::string RejectReason() const;

  bool HasReadyAck() const;
  bool HasUnreadyAck() const;
  bool HasAssign(int *player = nullptr) const;
  bool HasTurn(int *player = nullptr) const;
  bool HasPositions(std::array<hb::Ball, 16> *balls = nullptr) const;
  bool HasShot(hb::ShotParams *out = nullptr) const;
  bool HasAim(double *tipX, double *tipY, double *power, double *aimX,
              double *aimY) const;
  bool HasChat(std::string *msg = nullptr) const;
  bool HasBye() const;
  bool HasRoomClosed() const;
  bool Disconnected() const;

  void SendReady();
  void SendUnready();
  void SendShot(double tipX, double tipY, double power, double aimX, double aimY);
  void SendAim(double tipX, double tipY, double power, double aimX, double aimY);
  void SendChat(const std::string &msg);
  void SendBye();

private:
  struct ClientEvents {
    bool hasReadyAck = false;
    bool hasUnreadyAck = false;
    bool hasAssign = false;
    bool hasTurn = false;
    bool hasPositions = false;
    bool hasShot = false;
    bool hasAim = false;
    bool hasChat = false;
    bool hasBye = false;
    bool hasRoomClosed = false;
    bool disconnected = false;
    int assignPlayer = -1;
    int turnPlayer = -1;
    std::array<hb::Ball, 16> balls{};
    hb::ShotParams shot;
    double aimTipX = 0.0;
    double aimTipY = 0.0;
    double aimPower = 0.0;
    double aimX = 1.0;
    double aimY = 0.0;
    std::string chatMsg;
  };

  bool LockedFlag(bool ClientEvents::*member) const;

  template <typename Fn>
  void WithClient(Fn fn) {
    std::lock_guard<std::mutex> lock(mutex_);
    fn(client_);
  }

  void StopWorker();
  void ThreadMain();
  void CaptureEventsLocked();
  void ClearFrameEventsLocked();
  void ClearQueuedEventsLocked();

  mutable std::mutex mutex_;
  std::thread worker_;
  hb::NetworkClient client_;
  ClientEvents queued_;
  ClientEvents frame_;
  bool running_ = false;
};
