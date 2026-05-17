#pragma once

#include "core.hpp"

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace hb {

struct RoomInfo {
  std::string name;
  std::string hostIP;
  std::string roomId;
  bool hasPassword = false;
  int playerCount = 1;
  double lastSeen = 0.0;
};

class NetworkHost {
public:
  NetworkHost();
  ~NetworkHost();

  bool Start(const std::string &roomName, const std::string &password,
             bool hostIsPlayer1);
  void Stop();
  void Poll();

  // Outgoing
  void SendAccept();
  void SendReject(const std::string &reason);
  void SendReadyAck();
  void SendReady();
  void SendUnready();
  void SendAssign(int player);
  void SendTurn(int player);
  void SendState(Phase phase, const RulesState &state);
  void SendPositions(const std::array<Ball, 16> &balls);
  void SendShot(double tipX, double tipY, double power, double aimX,
                double aimY);
  void SendChat(const std::string &msg);
  void SendAim(double tipX, double tipY, double power, double aimX, double aimY);
  void SendBye();
  void SendRoomClosed();

  // Incoming
  bool HasJoinRequest(std::string *password = nullptr) const;
  bool HasReady() const;
  bool HasUnready() const;
  bool HasCuePlacement(Vec2 *pos = nullptr, bool *confirmed = nullptr) const;
  bool HasGroupChoice(BallGroup *group = nullptr) const;
  bool HasShot(ShotParams *out) const;
  bool HasAim(double *tipX, double *tipY, double *power, double *aimX, double *aimY) const;
  bool HasChat(std::string *msg) const;
  bool HasBye() const;
  bool HasClient() const;
  bool ClientDisconnected() const;

private:
  uintptr_t udpSock_;
  uintptr_t tcpListen_;
  uintptr_t tcpClient_;
  std::string roomName_;
  std::string password_;
  std::string roomId_;
  bool hostIsPlayer1_;
  bool clientJoined_;
  double lastBroadcast_;
  double handshakeDeadline_;

  std::string recvBuf_;
  std::vector<std::string> pendingMsgs_;

  bool hasJoin_;
  bool hasReady_;
  bool hasUnready_;
  bool hasCuePlacement_;
  bool hasGroupChoice_;
  bool hasShot_;
  bool hasAim_;
  bool hasChat_;
  bool hasBye_;
  bool clientGone_;
  std::string joinPwd_;
  Vec2 cuePlacementPos_;
  bool cuePlacementConfirmed_;
  BallGroup groupChoice_;
  ShotParams shotParams_;
  double aimTipX_, aimTipY_, aimPower_, aimDirX_, aimDirY_;
  std::string chatMsg_;

  void Broadcast();
  void ProcessIncoming();
  void SendLine(const std::string &line);
  void CloseClient();
};

class NetworkClient {
public:
  NetworkClient();
  ~NetworkClient();

  bool Start();
  void Stop();
  void Poll();

  std::vector<RoomInfo> GetRooms();

  bool Connect(const std::string &hostIP, const std::string &password,
               const std::string &roomId = "");
  void Disconnect();
  bool IsConnecting() const;
  bool IsAccepted() const;
  std::string RejectReason() const;

  // Outgoing
  void SendReady();
  void SendUnready();
  void SendCuePlacement(Vec2 pos, bool confirmed);
  void SendGroupChoice(BallGroup group);
  void SendShot(double tipX, double tipY, double power, double aimX,
                double aimY);
  void SendAim(double tipX, double tipY, double power, double aimX, double aimY);
  void SendChat(const std::string &msg);
  void SendBye();

  // Incoming
  bool HasReadyAck() const;
  bool HasUnreadyAck() const;
  bool HasAssign(int *player) const;
  bool HasTurn(int *player) const;
  bool HasState(Phase *phase, RulesState *state) const;
  bool HasPositions(std::array<Ball, 16> *balls) const;
  bool HasShot(ShotParams *out) const;
  bool HasAim(double *tipX, double *tipY, double *power, double *aimX, double *aimY) const;
  bool HasChat(std::string *msg) const;
  bool HasBye() const;
  bool HasRoomClosed() const;
  bool Disconnected() const;

private:
  uintptr_t udpSock_;
  uintptr_t tcpSock_;
  bool connecting_;
  bool accepted_;
  bool disconnected_;
  std::string rejectReason_;

  std::string recvBuf_;
  std::vector<std::string> pendingMsgs_;
  std::vector<RoomInfo> rooms_;
  std::string lastPosData_;
  std::string joinPwd_;
  std::string joinRoomId_;

  bool hasReadyAck_;
  bool hasUnreadyAck_;
  bool hasAssign_;
  int assignPlayer_;
  bool hasTurn_;
  bool hasState_;
  bool hasPositions_;
  bool hasShot_;
  bool hasAim_;
  bool hasChat_;
  bool hasBye_;
  bool hasRoomClosed_;
  int turnPlayer_;
  Phase statePhase_;
  RulesState rulesState_;
  ShotParams shotParams_;
  double aimTipX_, aimTipY_, aimPower_, aimDirX_, aimDirY_;
  std::string chatMsg_;

  void ProcessIncoming();
  void SendLine(const std::string &line);
};

} // namespace hb
