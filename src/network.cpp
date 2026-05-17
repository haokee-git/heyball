#include "network.hpp"

#define WIN32_LEAN_AND_MEAN
#define NOGDI
#define NOUSER
#include <winsock2.h>
#include <ws2tcpip.h>

#ifdef HEYBALL_TEST_BUILD
#include "../tests/mock_raylib.h"
#else
#include <raylib.h>
#endif

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sstream>

namespace hb {
namespace {

constexpr uint16_t kUdpPort = 28561;
constexpr uint16_t kTcpPort = 28562;
constexpr double kBroadcastInterval = 0.5;
constexpr double kAimInterval = 0.067;
constexpr double kPosInterval = 1.0 / 30.0;
const char *kMagic = "HEYBALL";
const char *kDelim = "|";

struct WsaGuard {
  WsaGuard() {
    WSADATA wsa;
    WSAStartup(MAKEWORD(2, 2), &wsa);
  }
  ~WsaGuard() { WSACleanup(); }
};

WsaGuard gWsa;

SOCKET ToSock(uintptr_t s) {
  return reinterpret_cast<SOCKET>(s);
}
uintptr_t FromSock(SOCKET s) {
  return reinterpret_cast<uintptr_t>(s);
}

bool SetNonBlock(SOCKET s) {
  u_long mode = 1;
  return ioctlsocket(s, FIONBIO, &mode) == 0;
}

SOCKET CreateUdpSocket(bool bindPort) {
  SOCKET s = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (s == INVALID_SOCKET) return INVALID_SOCKET;
  int yes = 1;
  setsockopt(s, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char *>(&yes),
             sizeof(yes));
  if (bindPort) {
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(kUdpPort);
    addr.sin_addr.s_addr = INADDR_ANY;
    if (bind(s, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0) {
      closesocket(s);
      return INVALID_SOCKET;
    }
  } else {
    int bc = 1;
    setsockopt(s, SOL_SOCKET, SO_BROADCAST, reinterpret_cast<const char *>(&bc),
               sizeof(bc));
  }
  SetNonBlock(s);
  return s;
}

SOCKET CreateTcpListen() {
  SOCKET s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (s == INVALID_SOCKET) return INVALID_SOCKET;
  int yes = 1;
  setsockopt(s, SOL_SOCKET, SO_REUSEADDR, reinterpret_cast<const char *>(&yes),
             sizeof(yes));
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(kTcpPort);
  addr.sin_addr.s_addr = INADDR_ANY;
  if (bind(s, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0) {
    closesocket(s);
    return INVALID_SOCKET;
  }
  if (listen(s, 1) != 0) {
    closesocket(s);
    return INVALID_SOCKET;
  }
  SetNonBlock(s);
  return s;
}

bool IsLocalAddress(const std::string &ip) {
  if (ip == "127.0.0.1" || ip == "::1" || ip == "localhost") return true;
  // Compare with all local IPs
  char host[256];
  gethostname(host, sizeof(host));
  hostent *he = gethostbyname(host);
  if (he) {
    for (int i = 0; he->h_addr_list[i]; ++i) {
      in_addr addr;
      std::memcpy(&addr, he->h_addr_list[i], sizeof(addr));
      if (ip == inet_ntoa(addr)) return true;
    }
  }
  return false;
}

std::vector<std::string> SplitLines(std::string &buf) {
  std::vector<std::string> lines;
  for (;;) {
    auto pos = buf.find('\n');
    if (pos == std::string::npos) break;
    std::string line = buf.substr(0, pos);
    if (!line.empty() && line.back() == '\r') line.pop_back();
    lines.push_back(line);
    buf.erase(0, pos + 1);
  }
  return lines;
}

std::string NewRoomId() {
  static uint32_t sequence = 0;
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%.0f-%u", GetTime() * 1000.0, ++sequence);
  return buf;
}

std::string BroadcastField(std::string value) {
  for (char &ch : value) {
    if (ch == '|' || ch == '\r' || ch == '\n') ch = ' ';
  }
  return value;
}

std::string LineField(std::string value) {
  for (char &ch : value) {
    if (ch == '\r' || ch == '\n') ch = ' ';
  }
  return value;
}

int PhaseCode(Phase phase) {
  return static_cast<int>(phase);
}

Phase PhaseFromCode(int code) {
  switch (code) {
  case 0: return Phase::Aiming;
  case 1: return Phase::Moving;
  case 2: return Phase::BallInHand;
  case 3: return Phase::GroupChoice;
  case 4: return Phase::RackOver;
  default: return Phase::Aiming;
  }
}

int GroupCode(BallGroup group) {
  return static_cast<int>(group);
}

BallGroup GroupFromCode(int code) {
  switch (code) {
  case 1: return BallGroup::Solids;
  case 2: return BallGroup::Stripes;
  default: return BallGroup::Open;
  }
}

} // namespace

// ─── NetworkHost ────────────────────────────────────────────────────────────

NetworkHost::NetworkHost()
    : udpSock_(FromSock(INVALID_SOCKET)),
      tcpListen_(FromSock(INVALID_SOCKET)),
      tcpClient_(FromSock(INVALID_SOCKET)),
      hostIsPlayer1_(true),
      clientJoined_(false),
      lastBroadcast_(0.0),
      handshakeDeadline_(0.0),
      hasJoin_(false),
      hasReady_(false),
      hasUnready_(false),
      hasCuePlacement_(false),
      hasGroupChoice_(false),
      hasShot_(false),
      hasAim_(false),
      hasChat_(false),
      hasBye_(false),
      clientGone_(false),
      groupChoice_(BallGroup::Open) {}

NetworkHost::~NetworkHost() { Stop(); }

bool NetworkHost::Start(const std::string &roomName,
                        const std::string &password, bool hostIsPlayer1) {
  Stop();
  hasJoin_ = hasReady_ = hasUnready_ = hasCuePlacement_ = hasGroupChoice_ =
      hasShot_ = hasAim_ = hasChat_ = hasBye_ = false;
  joinPwd_.clear();
  chatMsg_.clear();
  SOCKET udp = CreateUdpSocket(false);
  if (udp == INVALID_SOCKET) return false;
  SOCKET tcp = CreateTcpListen();
  if (tcp == INVALID_SOCKET) {
    closesocket(udp);
    return false;
  }
  udpSock_ = FromSock(udp);
  tcpListen_ = FromSock(tcp);
  roomName_ = roomName;
  password_ = password;
  roomId_ = NewRoomId();
  hostIsPlayer1_ = hostIsPlayer1;
  clientJoined_ = false;
  lastBroadcast_ = 0.0;
  handshakeDeadline_ = 0.0;
  clientGone_ = false;
  return true;
}

void NetworkHost::CloseClient() {
  if (ToSock(tcpClient_) != INVALID_SOCKET) {
    closesocket(ToSock(tcpClient_));
    tcpClient_ = FromSock(INVALID_SOCKET);
  }
  clientJoined_ = false;
  handshakeDeadline_ = 0.0;
  pendingMsgs_.clear();
  recvBuf_.clear();
}

void NetworkHost::Stop() {
  CloseClient();
  if (ToSock(tcpListen_) != INVALID_SOCKET) {
    closesocket(ToSock(tcpListen_));
    tcpListen_ = FromSock(INVALID_SOCKET);
  }
  if (ToSock(udpSock_) != INVALID_SOCKET) {
    closesocket(ToSock(udpSock_));
    udpSock_ = FromSock(INVALID_SOCKET);
  }
  pendingMsgs_.clear();
  recvBuf_.clear();
  hasJoin_ = hasReady_ = hasUnready_ = hasCuePlacement_ = hasGroupChoice_ =
      hasShot_ = hasAim_ = hasChat_ = hasBye_ = false;
  joinPwd_.clear();
  chatMsg_.clear();
  roomId_.clear();
  clientGone_ = false;
}

void NetworkHost::Poll() {
  ProcessIncoming();

  double now = GetTime();
  if (now - lastBroadcast_ >= kBroadcastInterval) {
    lastBroadcast_ = now;
    Broadcast();
  }
}

void NetworkHost::Broadcast() {
  SOCKET udp = ToSock(udpSock_);
  if (udp == INVALID_SOCKET) return;
  bool hasClient = clientJoined_ && ToSock(tcpClient_) != INVALID_SOCKET;
  const std::string advertisedName = BroadcastField(roomName_);
  char buf[256];
  std::snprintf(buf, sizeof(buf), "%s%s%s%s%d%s%d%s%s", kMagic, kDelim,
                advertisedName.c_str(), kDelim,
                password_.empty() ? 0 : 1, kDelim,
                hasClient ? 2 : 1, kDelim, roomId_.c_str());
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(kUdpPort);
  addr.sin_addr.s_addr = INADDR_BROADCAST;
  sendto(udp, buf, static_cast<int>(std::strlen(buf)), 0,
         reinterpret_cast<sockaddr *>(&addr), sizeof(addr));
  addr.sin_addr.s_addr = inet_addr("127.0.0.1");
  sendto(udp, buf, static_cast<int>(std::strlen(buf)), 0,
         reinterpret_cast<sockaddr *>(&addr), sizeof(addr));
}

void NetworkHost::ProcessIncoming() {
  hasJoin_ = hasReady_ = hasUnready_ = hasCuePlacement_ = hasGroupChoice_ =
      hasShot_ = hasAim_ = hasChat_ = hasBye_ = false;
  const double now = GetTime();
  if (ToSock(tcpClient_) != INVALID_SOCKET && !clientJoined_ &&
      handshakeDeadline_ > 0.0 && now > handshakeDeadline_) {
    CloseClient();
  }

  // Accept new client
  SOCKET listenSock = ToSock(tcpListen_);
  if (listenSock != INVALID_SOCKET &&
      ToSock(tcpClient_) == INVALID_SOCKET) {
    SOCKET client = accept(listenSock, nullptr, nullptr);
    if (client != INVALID_SOCKET) {
      SetNonBlock(client);
      tcpClient_ = FromSock(client);
      clientJoined_ = false;
      handshakeDeadline_ = now + 2.0;
      pendingMsgs_.clear();
      recvBuf_.clear();
      clientGone_ = false;
    }
  }

  // Read from client
  SOCKET client = ToSock(tcpClient_);
  if (client == INVALID_SOCKET) return;

  char buf[4096];
  int n = recv(client, buf, sizeof(buf) - 1, 0);
  if (n > 0) {
    clientGone_ = false;
    buf[n] = '\0';
    recvBuf_ += buf;
    auto lines = SplitLines(recvBuf_);
    for (auto &line : lines) pendingMsgs_.push_back(line);
  } else if (n == 0 || (n == SOCKET_ERROR && WSAGetLastError() != WSAEWOULDBLOCK)) {
    clientGone_ = clientJoined_;
    CloseClient();
    return;
  }

  for (auto &msg : pendingMsgs_) {
    if (msg.rfind("JOIN2 ", 0) == 0) {
      const size_t idStart = 6;
      const size_t pwdStart = msg.find(' ', idStart);
      const std::string requestedRoomId =
          pwdStart == std::string::npos ? msg.substr(idStart)
                                        : msg.substr(idStart, pwdStart - idStart);
      const std::string requestedPassword =
          pwdStart == std::string::npos ? std::string() : msg.substr(pwdStart + 1);
      if (requestedRoomId != roomId_) {
        SendLine("REJECT room changed");
        CloseClient();
        break;
      }
      if (!password_.empty() && requestedPassword != password_) {
        SendLine("REJECT password");
        CloseClient();
        break;
      }
      clientJoined_ = true;
      handshakeDeadline_ = 0.0;
      hasJoin_ = true;
      joinPwd_ = requestedPassword;
      SendLine("ACCEPT");
      continue;
    }
    if (msg.rfind("JOIN ", 0) == 0) {
      hasJoin_ = true;
      joinPwd_ = msg.substr(5);
      // Auto-accept if password matches
      if (password_.empty() || joinPwd_ == password_) {
        clientJoined_ = true;
        handshakeDeadline_ = 0.0;
        SendLine("ACCEPT");
      } else {
        SendLine("REJECT 密码错误");
      }
      if (!clientJoined_) {
        hasJoin_ = false;
        joinPwd_.clear();
        CloseClient();
        break;
      }
    } else if (msg == "READY") {
      if (!clientJoined_) continue;
      hasReady_ = true;
    } else if (msg == "UNREADY") {
      if (!clientJoined_) continue;
      hasUnready_ = true;
    } else if (msg.rfind("CUE_PLACE ", 0) == 0) {
      if (!clientJoined_) continue;
      int confirm = 0;
      if (std::sscanf(msg.c_str(), "CUE_PLACE %lf %lf %d",
                      &cuePlacementPos_.x, &cuePlacementPos_.y,
                      &confirm) == 3) {
        hasCuePlacement_ = true;
        cuePlacementConfirmed_ = confirm != 0;
      }
    } else if (msg.rfind("GROUP_CHOICE ", 0) == 0) {
      if (!clientJoined_) continue;
      int groupCode = 0;
      if (std::sscanf(msg.c_str(), "GROUP_CHOICE %d", &groupCode) == 1) {
        const BallGroup group = GroupFromCode(groupCode);
        if (group == BallGroup::Solids || group == BallGroup::Stripes) {
          hasGroupChoice_ = true;
          groupChoice_ = group;
        }
      }
    } else if (msg.rfind("SHOT ", 0) == 0) {
      if (!clientJoined_) continue;
      hasShot_ = true;
      double pw;
      std::sscanf(msg.c_str(), "SHOT %lf %lf %lf %lf %lf", &shotParams_.tipX,
                  &shotParams_.tipY, &pw, &shotParams_.aim.x,
                  &shotParams_.aim.y);
      shotParams_.power = pw;
    } else if (msg.rfind("AIM ", 0) == 0) {
      if (!clientJoined_) continue;
      hasAim_ = true;
      double pw, ax, ay;
      std::sscanf(msg.c_str(), "AIM %lf %lf %lf %lf %lf", &aimTipX_, &aimTipY_,
                  &pw, &ax, &ay);
      aimPower_ = pw;
      aimDirX_ = ax;
      aimDirY_ = ay;
    } else if (msg.rfind("CHAT ", 0) == 0) {
      if (!clientJoined_) continue;
      hasChat_ = true;
      chatMsg_ = msg.substr(5);
    } else if (msg == "BYE") {
      if (!clientJoined_) continue;
      hasBye_ = true;
    }
  }
  pendingMsgs_.clear();
}

void NetworkHost::SendLine(const std::string &line) {
  SOCKET client = ToSock(tcpClient_);
  if (client == INVALID_SOCKET) return;
  std::string data = line + "\n";
  int total = static_cast<int>(data.size());
  int sent = 0;
  int tries = 0;
  while (sent < total && tries < 30) {
    int n = send(client, data.c_str() + sent, total - sent, 0);
    if (n > 0) {
      sent += n;
      tries = 0;
    } else if (WSAGetLastError() == WSAEWOULDBLOCK) {
      if (++tries > 4) Sleep(1);
    } else {
      break;
    }
  }
}

void NetworkHost::SendAccept() { SendLine("ACCEPT"); }
void NetworkHost::SendReject(const std::string &reason) {
  SendLine("REJECT " + reason);
}
void NetworkHost::SendReadyAck() { SendLine("READY_ACK"); }
void NetworkHost::SendReady() { SendLine("READY_ACK"); }
void NetworkHost::SendUnready() { SendLine("UNREADY_ACK"); }
void NetworkHost::SendAssign(int player) {
  SendLine("ASSIGN " + std::to_string(player));
}
void NetworkHost::SendTurn(int player) {
  SendLine("TURN " + std::to_string(player));
}

void NetworkHost::SendState(Phase phase, const RulesState &state) {
  std::string msg = "STATE " + std::to_string(PhaseCode(phase)) + " " +
                    std::to_string(state.currentPlayer) + " " +
                    std::to_string(state.winner) + " " +
                    std::to_string(state.breakShot ? 1 : 0) + " " +
                    std::to_string(state.ballInHand ? 1 : 0) + " " +
                    std::to_string(state.pendingGroupChoice ? 1 : 0) + " " +
                    std::to_string(GroupCode(state.players[0].group)) + " " +
                    std::to_string(GroupCode(state.players[1].group)) + " " +
                    LineField(state.message);
  SendLine(msg);
}

void NetworkHost::SendPositions(const std::array<Ball, 16> &balls) {
  std::string msg = "POS";
  char num[512];
  for (const Ball &b : balls) {
    int flags = 0;
    if (b.pocketed) flags |= 2;
    if (b.sinking) flags |= 1;
    std::snprintf(num, sizeof(num),
                  " %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f "
                  "%.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %.6f %d %.6f "
                  "%.6f %.6f",
                  b.pos.x, b.pos.y, b.vel.x, b.vel.y, b.rollOmega.x,
                  b.rollOmega.y, b.sideOmega, b.rollAngle, b.decal.x,
                  b.decal.y, b.orientation[0], b.orientation[1],
                  b.orientation[2], b.orientation[3], b.orientation[4],
                  b.orientation[5], b.orientation[6], b.orientation[7],
                  b.orientation[8], flags, b.pocketFade, b.sinkTarget.x,
                  b.sinkTarget.y);
    msg += num;
  }
  SendLine(msg);
}
void NetworkHost::SendShot(double tipX, double tipY, double power,
                           double aimX, double aimY) {
  char buf[256];
  std::snprintf(buf, sizeof(buf), "SHOT %.6f %.6f %.6f %.6f %.6f", tipX, tipY,
                power, aimX, aimY);
  SendLine(buf);
}
void NetworkHost::SendChat(const std::string &msg) {
  SendLine("CHAT " + msg);
}
void NetworkHost::SendAim(double tipX, double tipY, double power, double aimX, double aimY) {
  char buf[192];
  std::snprintf(buf, sizeof(buf), "AIM %.6f %.6f %.6f %.6f %.6f", tipX, tipY, power, aimX, aimY);
  SendLine(buf);
}
void NetworkHost::SendBye() { SendLine("BYE"); }
void NetworkHost::SendRoomClosed() { SendLine("ROOM_CLOSED"); }

bool NetworkHost::HasJoinRequest(std::string *password) const {
  if (hasJoin_ && password) *password = joinPwd_;
  return hasJoin_;
}
bool NetworkHost::HasReady() const { return hasReady_; }
bool NetworkHost::HasUnready() const { return hasUnready_; }
bool NetworkHost::HasCuePlacement(Vec2 *pos, bool *confirmed) const {
  if (hasCuePlacement_) {
    if (pos) *pos = cuePlacementPos_;
    if (confirmed) *confirmed = cuePlacementConfirmed_;
  }
  return hasCuePlacement_;
}
bool NetworkHost::HasGroupChoice(BallGroup *group) const {
  if (hasGroupChoice_ && group) *group = groupChoice_;
  return hasGroupChoice_;
}
bool NetworkHost::HasShot(ShotParams *out) const {
  if (hasShot_ && out) *out = shotParams_;
  return hasShot_;
}
bool NetworkHost::HasAim(double *tipX, double *tipY,
                          double *power, double *aimX, double *aimY) const {
  if (hasAim_) {
    if (tipX) *tipX = aimTipX_;
    if (tipY) *tipY = aimTipY_;
    if (power) *power = aimPower_;
    if (aimX) *aimX = aimDirX_;
    if (aimY) *aimY = aimDirY_;
  }
  return hasAim_;
}
bool NetworkHost::HasChat(std::string *msg) const {
  if (hasChat_ && msg) *msg = chatMsg_;
  return hasChat_;
}
bool NetworkHost::HasBye() const { return hasBye_; }
bool NetworkHost::HasClient() const {
  return clientJoined_ && ToSock(tcpClient_) != INVALID_SOCKET;
}
bool NetworkHost::ClientDisconnected() const {
  return clientGone_;
}

// ─── NetworkClient ──────────────────────────────────────────────────────────

NetworkClient::NetworkClient()
    : udpSock_(FromSock(INVALID_SOCKET)),
      tcpSock_(FromSock(INVALID_SOCKET)),
      connecting_(false),
      accepted_(false),
      disconnected_(false),
      hasReadyAck_(false),
      hasUnreadyAck_(false),
      hasAssign_(false),
      assignPlayer_(-1),
      hasTurn_(false),
      hasState_(false),
      hasPositions_(false),
      hasShot_(false),
      hasAim_(false),
      hasChat_(false),
      hasBye_(false),
      hasRoomClosed_(false) {}

NetworkClient::~NetworkClient() { Stop(); }

bool NetworkClient::Start() {
  Stop();
  rooms_.clear();
  pendingMsgs_.clear();
  recvBuf_.clear();
  rejectReason_.clear();
  hasReadyAck_ = hasUnreadyAck_ = hasAssign_ = hasTurn_ = hasState_ = hasPositions_ = hasShot_ = false;
  hasAim_ = hasChat_ = hasBye_ = hasRoomClosed_ = false;
  SOCKET udp = CreateUdpSocket(true);
  if (udp == INVALID_SOCKET) return false;
  udpSock_ = FromSock(udp);
  return true;
}

void NetworkClient::Stop() {
  Disconnect();
  if (ToSock(udpSock_) != INVALID_SOCKET) {
    closesocket(ToSock(udpSock_));
    udpSock_ = FromSock(INVALID_SOCKET);
  }
}

void NetworkClient::Poll() {
  // Listen for UDP broadcasts
  SOCKET udp = ToSock(udpSock_);
  if (udp != INVALID_SOCKET) {
    char buf[512];
    sockaddr_in from{};
    int fromLen = sizeof(from);
    int n = recvfrom(udp, buf, sizeof(buf) - 1, 0,
                     reinterpret_cast<sockaddr *>(&from), &fromLen);
    while (n > 0) {
      buf[n] = '\0';
      std::string data(buf);
      if (data.rfind(kMagic, 0) == 0) {
        auto parts = [&]() -> std::vector<std::string> {
          std::vector<std::string> p;
          size_t start = 0;
          while (start < data.size()) {
            size_t end = data.find(kDelim, start);
            p.push_back(data.substr(start, end - start));
            if (end == std::string::npos) break;
            start = end + 1;
          }
          return p;
        }();
        if (parts.size() >= 4) {
          RoomInfo ri;
          ri.name = parts[1];
          ri.hostIP = inet_ntoa(from.sin_addr);
          ri.hasPassword = (parts[2] == "1");
          ri.playerCount = (parts.size() >= 4) ? std::atoi(parts[3].c_str()) : 1;
          if (ri.playerCount < 1 || ri.playerCount > 2) ri.playerCount = 1;
          ri.roomId = (parts.size() >= 5) ? parts[4] : std::string();
          ri.lastSeen = GetTime();
          bool found = false;
          for (auto &r : rooms_) {
            if (r.hostIP == ri.hostIP ||
                (IsLocalAddress(r.hostIP) && IsLocalAddress(ri.hostIP))) {
              r = ri;
              found = true;
              break;
            }
          }
          if (!found) rooms_.push_back(ri);
        }
      }
      n = recvfrom(udp, buf, sizeof(buf) - 1, 0,
                   reinterpret_cast<sockaddr *>(&from), &fromLen);
    }
  }

  // Expire old rooms (none for now — keep until restart)

  const double now = GetTime();
  rooms_.erase(std::remove_if(rooms_.begin(), rooms_.end(),
                              [now](const RoomInfo &room) {
                                return now - room.lastSeen > 2.0;
                              }),
               rooms_.end());

  ProcessIncoming();
}

std::vector<RoomInfo> NetworkClient::GetRooms() {
  const double now = GetTime();
  rooms_.erase(std::remove_if(rooms_.begin(), rooms_.end(),
                              [now](const RoomInfo &room) {
                                return now - room.lastSeen > 2.0;
                              }),
               rooms_.end());
  return rooms_;
}

bool NetworkClient::Connect(const std::string &hostIP,
                            const std::string &password,
                            const std::string &roomId) {
  Disconnect();
  SOCKET s = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (s == INVALID_SOCKET) return false;
  SetNonBlock(s);
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(kTcpPort);
  // Use 127.0.0.1 for local connections (bypasses firewall on corporate networks)
  const char *actualIP = IsLocalAddress(hostIP) ? "127.0.0.1" : hostIP.c_str();
  addr.sin_addr.s_addr = inet_addr(actualIP);
  int cr = connect(s, reinterpret_cast<sockaddr *>(&addr), sizeof(addr));
  if (cr == SOCKET_ERROR) {
    int err = WSAGetLastError();
    if (err != WSAEWOULDBLOCK) {
      closesocket(s);
      return false;
    }
  }
  tcpSock_ = FromSock(s);
  connecting_ = true;
  accepted_ = false;
  disconnected_ = false;
  rejectReason_.clear();
  hasReadyAck_ = hasUnreadyAck_ = hasAssign_ = hasTurn_ = hasState_ = hasPositions_ = hasShot_ = false;
  hasAim_ = hasChat_ = hasBye_ = hasRoomClosed_ = false;
  joinPwd_ = password;
  joinRoomId_ = roomId;
  pendingMsgs_.clear();
  recvBuf_.clear();
  return true;
}

void NetworkClient::Disconnect() {
  if (ToSock(tcpSock_) != INVALID_SOCKET) {
    closesocket(ToSock(tcpSock_));
    tcpSock_ = FromSock(INVALID_SOCKET);
  }
  connecting_ = false;
  accepted_ = false;
  disconnected_ = true;
  pendingMsgs_.clear();
  recvBuf_.clear();
  joinPwd_.clear();
  joinRoomId_.clear();
  hasReadyAck_ = hasUnreadyAck_ = hasAssign_ = hasTurn_ = hasState_ = hasPositions_ = hasShot_ = false;
  hasAim_ = hasChat_ = hasBye_ = hasRoomClosed_ = false;
}

bool NetworkClient::IsConnecting() const { return connecting_ && !accepted_; }
bool NetworkClient::IsAccepted() const { return accepted_; }
std::string NetworkClient::RejectReason() const { return rejectReason_; }

void NetworkClient::ProcessIncoming() {
  SOCKET sock = ToSock(tcpSock_);
  if (sock == INVALID_SOCKET) {
    hasReadyAck_ = hasUnreadyAck_ = hasAssign_ = hasTurn_ = hasState_ = hasPositions_ = hasShot_ = false;
    hasAim_ = hasChat_ = hasBye_ = hasRoomClosed_ = false;
    return;
  }

  // Check if connect completed
  if (connecting_ && !accepted_) {
    fd_set wfds;
    FD_ZERO(&wfds);
    FD_SET(sock, &wfds);
    timeval tv = {0, 0};
    int sel = select(0, nullptr, &wfds, nullptr, &tv);
    if (sel > 0) {
      int err = 0;
      int len = sizeof(err);
      getsockopt(sock, SOL_SOCKET, SO_ERROR, reinterpret_cast<char *>(&err),
                 &len);
      if (err != 0) {
        closesocket(sock);
        tcpSock_ = FromSock(INVALID_SOCKET);
        connecting_ = false;
        disconnected_ = true;
        joinPwd_.clear();
        joinRoomId_.clear();
        return;
      }
      // Connection established — send JOIN now
      if (joinRoomId_.empty()) {
        SendLine("JOIN " + joinPwd_);
      } else {
        SendLine("JOIN2 " + joinRoomId_ + " " + joinPwd_);
      }
      joinPwd_.clear();
      joinRoomId_.clear();
    } else {
      // sel == 0: still connecting, sel < 0: error — wait and retry next poll
      return;
    }
  }

  char buf[4096];
  int n = recv(sock, buf, sizeof(buf) - 1, 0);
  if (n > 0) {
    buf[n] = '\0';
    recvBuf_ += buf;
    auto lines = SplitLines(recvBuf_);
    for (auto &line : lines) pendingMsgs_.push_back(line);
  } else if (n == 0 || (n == SOCKET_ERROR && WSAGetLastError() != WSAEWOULDBLOCK)) {
    closesocket(sock);
    tcpSock_ = FromSock(INVALID_SOCKET);
    connecting_ = false;
    accepted_ = false;
    disconnected_ = true;
  }

  hasReadyAck_ = hasUnreadyAck_ = hasAssign_ = hasTurn_ = hasState_ = hasPositions_ = hasShot_ = false;
  hasAim_ = hasChat_ = hasBye_ = hasRoomClosed_ = false;
  for (auto &msg : pendingMsgs_) {
    if (msg == "ACCEPT") {
      connecting_ = false;
      accepted_ = true;
    } else if (msg.rfind("REJECT ", 0) == 0) {
      connecting_ = false;
      rejectReason_ = msg.substr(7);
      disconnected_ = true;
      closesocket(sock);
      tcpSock_ = FromSock(INVALID_SOCKET);
    } else if (msg == "READY_ACK") {
      hasReadyAck_ = true;
    } else if (msg == "UNREADY_ACK") {
      hasUnreadyAck_ = true;
    } else if (msg.rfind("ASSIGN ", 0) == 0) {
      hasAssign_ = true;
      assignPlayer_ = std::stoi(msg.substr(7));
    } else if (msg.rfind("TURN ", 0) == 0) {
      hasTurn_ = true;
      turnPlayer_ = std::stoi(msg.substr(5));
    } else if (msg.rfind("STATE ", 0) == 0) {
      std::istringstream ss(msg.substr(6));
      int phase = 0;
      int currentPlayer = 0;
      int winner = -1;
      int breakShot = 0;
      int ballInHand = 0;
      int pendingGroupChoice = 0;
      int p0Group = 0;
      int p1Group = 0;
      if (ss >> phase >> currentPlayer >> winner >> breakShot >> ballInHand >>
          pendingGroupChoice >> p0Group >> p1Group) {
        std::string message;
        std::getline(ss, message);
        if (!message.empty() && message.front() == ' ') {
          message.erase(0, 1);
        }
        hasState_ = true;
        statePhase_ = PhaseFromCode(phase);
        rulesState_.currentPlayer = currentPlayer == 1 ? 1 : 0;
        rulesState_.winner = (winner >= 0 && winner <= 1) ? winner : -1;
        rulesState_.breakShot = breakShot != 0;
        rulesState_.ballInHand = ballInHand != 0;
        rulesState_.pendingGroupChoice = pendingGroupChoice != 0;
        rulesState_.players[0].group = GroupFromCode(p0Group);
        rulesState_.players[1].group = GroupFromCode(p1Group);
        rulesState_.message = message;
      }
    } else if (msg.rfind("POS ", 0) == 0) {
      hasPositions_ = true;
      lastPosData_ = msg.substr(4);
    } else if (msg.rfind("SHOT ", 0) == 0) {
      hasShot_ = true;
      double pw;
      std::sscanf(msg.c_str(), "SHOT %lf %lf %lf %lf %lf", &shotParams_.tipX,
                  &shotParams_.tipY, &pw, &shotParams_.aim.x,
                  &shotParams_.aim.y);
      shotParams_.power = pw;
    } else if (msg.rfind("AIM ", 0) == 0) {
      hasAim_ = true;
      double pw, ax, ay;
      std::sscanf(msg.c_str(), "AIM %lf %lf %lf %lf %lf", &aimTipX_, &aimTipY_,
                  &pw, &ax, &ay);
      aimPower_ = pw;
      aimDirX_ = ax;
      aimDirY_ = ay;
    } else if (msg.rfind("CHAT ", 0) == 0) {
      hasChat_ = true;
      chatMsg_ = msg.substr(5);
    } else if (msg == "BYE") {
      hasBye_ = true;
    } else if (msg == "ROOM_CLOSED") {
      hasRoomClosed_ = true;
    }
  }
  pendingMsgs_.clear();
}

void NetworkClient::SendLine(const std::string &line) {
  SOCKET sock = ToSock(tcpSock_);
  if (sock == INVALID_SOCKET) return;
  std::string data = line + "\n";
  int total = static_cast<int>(data.size());
  int sent = 0;
  int tries = 0;
  while (sent < total && tries < 30) {
    int n = send(sock, data.c_str() + sent, total - sent, 0);
    if (n > 0) {
      sent += n;
      tries = 0;
    } else if (WSAGetLastError() == WSAEWOULDBLOCK) {
      if (++tries > 4) Sleep(1);
    } else {
      break;
    }
  }
}

void NetworkClient::SendReady() { SendLine("READY"); }
void NetworkClient::SendUnready() { SendLine("UNREADY"); }
void NetworkClient::SendCuePlacement(Vec2 pos, bool confirmed) {
  char buf[128];
  std::snprintf(buf, sizeof(buf), "CUE_PLACE %.6f %.6f %d", pos.x, pos.y,
                confirmed ? 1 : 0);
  SendLine(buf);
}
void NetworkClient::SendGroupChoice(BallGroup group) {
  SendLine("GROUP_CHOICE " + std::to_string(GroupCode(group)));
}
void NetworkClient::SendShot(double tipX, double tipY, double power,
                             double aimX, double aimY) {
  char buf[256];
  std::snprintf(buf, sizeof(buf), "SHOT %.6f %.6f %.6f %.6f %.6f", tipX, tipY,
                power, aimX, aimY);
  SendLine(buf);
}
void NetworkClient::SendAim(double tipX, double tipY, double power, double aimX, double aimY) {
  char buf[192];
  std::snprintf(buf, sizeof(buf), "AIM %.6f %.6f %.6f %.6f %.6f", tipX, tipY, power, aimX, aimY);
  SendLine(buf);
}
void NetworkClient::SendChat(const std::string &msg) {
  SendLine("CHAT " + msg);
}
void NetworkClient::SendBye() { SendLine("BYE"); }

bool NetworkClient::HasReadyAck() const { return hasReadyAck_; }
bool NetworkClient::HasUnreadyAck() const { return hasUnreadyAck_; }
bool NetworkClient::HasAssign(int *player) const {
  if (hasAssign_ && player) *player = assignPlayer_;
  return hasAssign_;
}
bool NetworkClient::HasTurn(int *player) const {
  if (hasTurn_ && player) *player = turnPlayer_;
  return hasTurn_;
}
bool NetworkClient::HasState(Phase *phase, RulesState *state) const {
  if (hasState_) {
    if (phase) *phase = statePhase_;
    if (state) *state = rulesState_;
  }
  return hasState_;
}
bool NetworkClient::HasPositions(std::array<Ball, 16> *balls) const {
  if (hasPositions_ && balls) {
    std::istringstream ss(lastPosData_);
    for (int i = 0; i < 16; ++i) {
      Ball parsed = (*balls)[i];
      double x, y;
      double fade = 0.0;
      int flags = 0;
      if (!(ss >> x >> y)) break;
      parsed.pos.x = x;
      parsed.pos.y = y;

      std::streampos afterPosition = ss.tellg();
      double vx = 0.0;
      double vy = 0.0;
      double rollX = 0.0;
      double rollY = 0.0;
      double side = 0.0;
      double rollAngle = 0.0;
      double decalX = 0.0;
      double decalY = 0.0;
      std::array<double, 9> orientation{};
      double sinkX = 0.0;
      double sinkY = 0.0;
      bool extended = false;
      if (ss >> vx >> vy >> rollX >> rollY >> side >> rollAngle >> decalX >>
          decalY >> orientation[0] >> orientation[1] >> orientation[2] >>
          orientation[3] >> orientation[4] >> orientation[5] >> orientation[6] >>
          orientation[7] >> orientation[8] >> flags >> fade >> sinkX >> sinkY) {
        extended = true;
      }
      if (extended) {
        parsed.vel = {vx, vy};
        parsed.rollOmega = {rollX, rollY};
        parsed.sideOmega = side;
        parsed.rollAngle = rollAngle;
        parsed.decal = {decalX, decalY};
        parsed.orientation = orientation;
        parsed.sinkTarget = {sinkX, sinkY};
      } else {
        ss.clear();
        if (afterPosition == std::streampos(-1)) break;
        ss.seekg(afterPosition);
        if (!(ss >> flags >> fade)) break;
      }
      parsed.pocketed = (flags & 2) != 0;
      parsed.sinking = !parsed.pocketed && (flags & 1) != 0;
      parsed.pocketFade = parsed.pocketed ? 1.0 : fade;
      (*balls)[i] = parsed;
    }
  }
  return hasPositions_;
}
bool NetworkClient::HasShot(ShotParams *out) const {
  if (hasShot_ && out) *out = shotParams_;
  return hasShot_;
}
bool NetworkClient::HasAim(double *tipX, double *tipY,
                            double *power, double *aimX, double *aimY) const {
  if (hasAim_) {
    if (tipX) *tipX = aimTipX_;
    if (tipY) *tipY = aimTipY_;
    if (power) *power = aimPower_;
    if (aimX) *aimX = aimDirX_;
    if (aimY) *aimY = aimDirY_;
  }
  return hasAim_;
}
bool NetworkClient::HasChat(std::string *msg) const {
  if (hasChat_ && msg) *msg = chatMsg_;
  return hasChat_;
}
bool NetworkClient::HasBye() const { return hasBye_; }
bool NetworkClient::HasRoomClosed() const { return hasRoomClosed_; }
bool NetworkClient::Disconnected() const { return disconnected_; }

} // namespace hb
