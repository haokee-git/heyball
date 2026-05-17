void HandleChatInput(Game &game) {
  // Chat input handling
  if (IsKeyPressed(KEY_ENTER)) {
    if (!game.chatInputActive) {
      game.chatInputActive = true;
      game.chatInputBuf.clear();
      game.chatInputCursor = 0;
      game.textAllSelected = false;
    } else if (!game.chatInputBuf.empty()) {
      if (game.appMode == AppMode::PlayingOnline) {
        if (game.onlineHost) game.host.SendChat(game.chatInputBuf);
        else game.client.SendChat(game.chatInputBuf);
        game.chatHistory.push_back("我: " + game.chatInputBuf);
      } else if (game.appMode == AppMode::RoomHost) {
        game.host.SendChat(game.chatInputBuf);
        game.chatHistory.push_back("我: " + game.chatInputBuf);
      } else if (game.appMode == AppMode::RoomClient) {
        game.client.SendChat(game.chatInputBuf);
        game.chatHistory.push_back("我: " + game.chatInputBuf);
      }
      game.chatInputBuf.clear();
      game.chatInputActive = false;
      game.chatInputCursor = 0;
      game.textAllSelected = false;
    }
  }
  if (game.chatInputActive) {
    if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
      const float us = OnlineUiScale();
      const Rectangle chat = ChatWindowRect(game);
      const float cx = chat.x;
      const float cy = chat.y;
      const float cw = chat.width;
      const float ch = chat.height;
      Rectangle inputBox{cx + 4, cy + ch - 28.0f * us, cw - 8, 24.0f * us};
      Vector2 m = GetMousePosition();
      if (CheckCollisionPointRec(m, inputBox)) {
        game.chatInputCursor = TextCursorFromMouseX(game.font, game.chatInputBuf, m.x, cx + 10.0f * us, 15.0f * us);
        game.textAllSelected = false;
      }
    }
    int key = GetCharPressed();
    while (key > 0) {
      if (game.textAllSelected) {
        game.chatInputBuf.clear();
        game.chatInputCursor = 0;
        game.textAllSelected = false;
      }
      if (key >= 32 && key <= 0x9FFF && static_cast<int>(game.chatInputBuf.size()) < 120)
        TextInsertAt(game.chatInputBuf, game.chatInputCursor, key);
      key = GetCharPressed();
    }
    const double now = GetTime();
    if (game.leftRepeat.tick(IsKeyDown(KEY_LEFT), now)) {
      if (game.textAllSelected) {
        game.chatInputCursor = 0;
        game.textAllSelected = false;
      } else {
        game.chatInputCursor = Utf8PrevPos(game.chatInputBuf, game.chatInputCursor);
      }
    }
    if (game.rightRepeat.tick(IsKeyDown(KEY_RIGHT), now)) {
      if (game.textAllSelected) {
        game.chatInputCursor = static_cast<int>(game.chatInputBuf.size());
        game.textAllSelected = false;
      } else {
        game.chatInputCursor = Utf8NextPos(game.chatInputBuf, game.chatInputCursor);
      }
    }
    if (IsKeyPressed(KEY_HOME))
      { game.chatInputCursor = 0; game.textAllSelected = false; }
    if (IsKeyPressed(KEY_END))
      { game.chatInputCursor = static_cast<int>(game.chatInputBuf.size()); game.textAllSelected = false; }
    if (game.delRepeat.tick(IsKeyDown(KEY_DELETE), now)) {
      if (game.textAllSelected) {
        game.chatInputBuf.clear();
        game.chatInputCursor = 0;
        game.textAllSelected = false;
      } else if (game.chatInputCursor < static_cast<int>(game.chatInputBuf.size())) {
        TextEraseAfter(game.chatInputBuf, game.chatInputCursor);
      }
    }
    bool ctrl = IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL);
    if (ctrl && IsKeyPressed(KEY_A) && !game.chatInputBuf.empty()) {
      game.chatInputCursor = static_cast<int>(game.chatInputBuf.size());
      game.textAllSelected = true;
    }
    if (game.bsRepeat.tick(IsKeyDown(KEY_BACKSPACE), now)) {
      if (game.textAllSelected) {
        game.chatInputBuf.clear();
        game.chatInputCursor = 0;
        game.textAllSelected = false;
      } else if (game.chatInputCursor > 0) {
        TextEraseBefore(game.chatInputBuf, game.chatInputCursor);
      }
    }
    if (IsKeyPressed(KEY_ESCAPE)) {
      game.chatInputActive = false;
      game.chatInputBuf.clear();
      game.chatInputCursor = 0;
      game.textAllSelected = false;
    }
  }
}

bool UpdateLobbyMode(Game &game) {
  if (game.appMode == AppMode::Lobby) {
    if (HandleTitleBarInput(game)) return true;
    game.client.Poll();
    if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
      Vector2 mouse = GetMousePosition();
      float sw = static_cast<float>(GetScreenWidth()), sh = static_cast<float>(GetScreenHeight());
      Rectangle createBtn = LobbyCreateButtonRect();
      Rectangle refreshBtn = LobbyRefreshButtonRect();
      if (game.showCreateDlg) {
        const CreateRoomDialogLayout l = CreateRoomLayout();
        if (CheckCollisionPointRec(mouse, l.ok)) {
          if (game.createRoomName.empty()) {
            game.createRoomNotice = "房间名不能为空";
          } else {
            game.roomName = game.createRoomName;
            game.roomPassword = game.createRoomPwd;
            game.hostIsPlayer1 = game.createHostIsP1;
            game.client.Stop();
            if (game.host.Start(game.roomName, game.roomPassword, game.hostIsPlayer1)) {
              game.appMode = AppMode::RoomHost;
              game.amReady = false;
              game.opReady = false;
              game.chatHistory.clear();
            }
            CloseCreateRoomDialog(game);
          }
          return true;
        }
        if (CheckCollisionPointRec(mouse, l.cancel)) {
          game.host.Stop();
          CloseCreateRoomDialog(game);
          return true;
        }
        if (CheckCollisionPointRec(mouse, l.p1)) {
          game.createHostIsP1 = true;
          return true;
        }
        if (CheckCollisionPointRec(mouse, l.p2)) {
          game.createHostIsP1 = false;
          return true;
        }
        if (CheckCollisionPointRec(mouse, l.showPwd)) {
          game.showPwd = !game.showPwd;
          return true;
        }
        if (CheckCollisionPointRec(mouse, l.nameBox)) {
          game.createDlgFocus = 0;
          game.createNameCursor = TextCursorFromMouseX(
              game.font, game.createRoomName, mouse.x, l.nameBox.x + 10.0f * l.scale,
              21.0f * l.scale);
          return true;
        }
        if (CheckCollisionPointRec(mouse, l.pwdBox)) {
          game.createDlgFocus = 1;
          std::string display =
              game.showPwd ? game.createRoomPwd : std::string(game.createRoomPwd.size(), '*');
          game.createPwdCursor = TextCursorFromMouseX(
              game.font, display, mouse.x, l.pwdBox.x + 10.0f * l.scale,
              21.0f * l.scale);
          return true;
        }
        return true;
      }
      if (!game.showJoinPwdDlg && CheckCollisionPointRec(mouse, createBtn)) {
        ResetCreateRoomDialog(game);
        return true;
      }
      if (!game.showJoinPwdDlg && CheckCollisionPointRec(mouse, refreshBtn)) {
        game.client.Stop();
        game.client.Start();
        game.roomList.clear();
        return true;
      }
      if (game.showJoinPwdDlg) {
        const float us2 = OnlineUiScale();
        const float cw2 = 360.0f * us2, ch2 = 170.0f * us2;
        const float cx2 = sw * 0.5f - cw2 * 0.5f;
        const float cy2 = sh * 0.5f - ch2 * 0.5f;
        Rectangle okR2{cx2 + cw2 * 0.5f - 82.0f * us2, cy2 + ch2 - 48.0f * us2,
                       74.0f * us2, 32.0f * us2};
        Rectangle cancelR2{cx2 + cw2 * 0.5f + 10.0f * us2, okR2.y,
                           74.0f * us2, 32.0f * us2};
        if (CheckCollisionPointRec(mouse, okR2)) {
          game.client.Stop(); game.client.Start();
          if (game.client.Connect(game.pendingJoinHostIP, game.joinPwdInput,
                                  game.pendingJoinRoomId)) {
            game.appMode = AppMode::RoomClient;
            game.roomName = game.pendingJoinRoomName;
            game.amReady = false;
            game.opReady = false;
            game.chatHistory.clear();
          }
          CloseJoinPasswordDialog(game);
          return true;
        }
        if (CheckCollisionPointRec(mouse, cancelR2)) {
          CloseJoinPasswordDialog(game);
          return true;
        }
        Rectangle pwdBox3{cx2 + 20.0f * us2, cy2 + 62.0f * us2,
                          cw2 - 40.0f * us2, 34.0f * us2};
        if (CheckCollisionPointRec(mouse, pwdBox3)) {
          std::string display = std::string(game.joinPwdInput.size(), '*');
          game.joinPwdCursor =
              TextCursorFromMouseX(game.font, display, mouse.x, cx2 + 30.0f * us2,
                                   20.0f * us2);
          game.textAllSelected = false;
        }
        return true;
      }
      if (CheckCollisionPointRec(mouse, createBtn)) {
        ResetCreateRoomDialog(game);
      } else if (CheckCollisionPointRec(mouse, refreshBtn)) {
        game.client.Stop();
        game.client.Start();
        game.roomList.clear();
      } else if (game.showCreateDlg) {
        float cw = 360.0f, ch = 280.0f, cx = sw * 0.5f - cw * 0.5f, cy = sh * 0.5f - ch * 0.5f;
        Rectangle okR{cx + cw * 0.5f - 70, cy + ch - 44, 64, 28};
        Rectangle cancelR{cx + cw * 0.5f + 10, cy + ch - 44, 64, 28};
        Rectangle p1R{cx + 16, cy + 204, 80, 24};
        Rectangle p2R{cx + 106, cy + 204, 80, 24};
        if (CheckCollisionPointRec(mouse, okR)) {
          game.roomName = game.createRoomName;
          game.roomPassword = game.createRoomPwd;
          game.hostIsPlayer1 = game.createHostIsP1;
          game.client.Stop();
          if (game.host.Start(game.roomName, game.roomPassword, game.hostIsPlayer1)) {
            game.appMode = AppMode::RoomHost;
            game.amReady = false;
            game.opReady = false;
            game.chatHistory.clear();
          }
          game.showCreateDlg = false;
        } else if (CheckCollisionPointRec(mouse, cancelR)) {
          game.host.Stop();
          game.showCreateDlg = false;
        } else if (CheckCollisionPointRec(mouse, p1R)) {
          game.createHostIsP1 = true;
        } else if (CheckCollisionPointRec(mouse, p2R)) {
          game.createHostIsP1 = false;
        }
        float pwdBoxW2 = cw - 32 - 46;
        Rectangle showPwdR2{cx + 16 + pwdBoxW2 + 6, cy + 142, 40, 28};
        if (CheckCollisionPointRec(mouse, showPwdR2)) {
          game.showPwd = !game.showPwd;
        }
      } else {
        float y = 130;
        game.roomList = game.client.GetRooms();
        for (size_t i = 0; i < game.roomList.size(); ++i) {
          Rectangle row{40, y, sw - 80, 44};
          Rectangle joinBtn{row.x + row.width - 70, row.y + 8, 56, 28};
          if (CheckCollisionPointRec(mouse, joinBtn)) {
            if (game.roomList[i].hasPassword) {
              OpenJoinPasswordDialog(game, game.roomList[i]);
              game.roomList.clear(); // will re-poll
            } else {
              game.client.Stop(); game.client.Start();
              if (game.client.Connect(game.roomList[i].hostIP, "",
                                      game.roomList[i].roomId)) {
                game.appMode = AppMode::RoomClient;
                game.roomName = game.roomList[i].name;
                game.amReady = false;
                game.opReady = false;
                game.chatHistory.clear();
              }
            }
            break;
          }
          y += 52;
        }
      }
    }
    // Handle create dialog text input
    if (game.showCreateDlg) {
      // Mouse click to switch focus between room name and password fields
      if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
        Vector2 m = GetMousePosition();
        float scw = static_cast<float>(GetScreenWidth());
        float sch = static_cast<float>(GetScreenHeight());
        float cw3 = 360.0f, ch3 = 280.0f, cx3 = scw * 0.5f - cw3 * 0.5f,
              cy3 = sch * 0.5f - ch3 * 0.5f;
        Rectangle nameBox2{cx3 + 16, cy3 + 80, cw3 - 32, 28};
        Rectangle pwdBox2{cx3 + 16, cy3 + 142, cw3 - 32, 28};
        float us3 = UiScale();
        if (CheckCollisionPointRec(m, nameBox2)) {
          game.createDlgFocus = 0;
          game.createNameCursor = TextCursorFromMouseX(game.font, game.createRoomName, m.x, cx3 + 22, 18.0f * us3);
        } else if (CheckCollisionPointRec(m, pwdBox2)) {
          game.createDlgFocus = 1;
          std::string display = std::string(game.createRoomPwd.size(), '*');
          game.createPwdCursor = TextCursorFromMouseX(game.font, display, m.x, cx3 + 22, 18.0f * us3);
        }
      }
      int key = GetCharPressed();
      while (key > 0) {
        if (game.textAllSelected) {
          std::string &selectedText =
              game.createDlgFocus == 0 ? game.createRoomName : game.createRoomPwd;
          int &selectedCursor =
              game.createDlgFocus == 0 ? game.createNameCursor : game.createPwdCursor;
          selectedText.clear();
          selectedCursor = 0;
          game.textAllSelected = false;
        }
        if (game.createDlgFocus == 0 && key >= 32) {
          if (Utf8CodepointCount(game.createRoomName) < kMaxRoomNameChars) {
            TextInsertAt(game.createRoomName, game.createNameCursor, key);
            game.createRoomNotice.clear();
          } else {
            game.createRoomNotice = "房间名最多 24 个字符";
          }
        } else if (game.createDlgFocus == 1 && key > 0 &&
                   Utf8CodepointCount(game.createRoomPwd) < kMaxRoomPasswordChars) {
          TextInsertAt(game.createRoomPwd, game.createPwdCursor, key);
        }
        key = GetCharPressed();
      }
      int &cursor = game.createDlgFocus == 0 ? game.createNameCursor : game.createPwdCursor;
      std::string &text = game.createDlgFocus == 0 ? game.createRoomName : game.createRoomPwd;
      const double now = GetTime();
      if (game.leftRepeat.tick(IsKeyDown(KEY_LEFT), now)) {
        if (game.textAllSelected) {
          cursor = 0;
          game.textAllSelected = false;
        } else {
          cursor = Utf8PrevPos(text, cursor);
        }
      }
      if (game.rightRepeat.tick(IsKeyDown(KEY_RIGHT), now)) {
        if (game.textAllSelected) {
          cursor = static_cast<int>(text.size());
          game.textAllSelected = false;
        } else {
          cursor = Utf8NextPos(text, cursor);
        }
      }
      if (IsKeyPressed(KEY_HOME)) { cursor = 0; game.textAllSelected = false; }
      if (IsKeyPressed(KEY_END)) { cursor = static_cast<int>(text.size()); game.textAllSelected = false; }
      if (game.delRepeat.tick(IsKeyDown(KEY_DELETE), now)) {
        if (game.textAllSelected) {
          text.clear();
          cursor = 0;
          game.textAllSelected = false;
        } else if (cursor < static_cast<int>(text.size())) {
          TextEraseAfter(text, cursor);
        }
      }
      bool ctrl = IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL);
      if (ctrl && IsKeyPressed(KEY_A) && !text.empty()) {
        cursor = static_cast<int>(text.size());
        game.textAllSelected = true;
      }
      if (game.bsRepeat.tick(IsKeyDown(KEY_BACKSPACE), now)) {
        if (game.textAllSelected) {
          text.clear();
          cursor = 0;
          game.textAllSelected = false;
        } else if (cursor > 0) {
          TextEraseBefore(text, cursor);
        }
      }
      // Tab to switch focus
      if (IsKeyPressed(KEY_TAB)) {
        game.createDlgFocus = 1 - game.createDlgFocus;
        game.textAllSelected = false;
      }
    }
    // Handle join password dialog (also shown in lobby)
    if (game.showJoinPwdDlg && game.appMode == AppMode::Lobby) {
      // This is actually the join password dialog
      float sw2 = static_cast<float>(GetScreenWidth()), sh2 = static_cast<float>(GetScreenHeight());
      float us2 = OnlineUiScale();
      float cw2 = 360.0f * us2, ch2 = 170.0f * us2, cx2 = sw2 * 0.5f - cw2 * 0.5f, cy2 = sh2 * 0.5f - ch2 * 0.5f;
      Rectangle okR2{cx2 + cw2 * 0.5f - 82.0f * us2, cy2 + ch2 - 48.0f * us2,
                     74.0f * us2, 32.0f * us2};
      Rectangle cancelR2{cx2 + cw2 * 0.5f + 10.0f * us2, okR2.y,
                         74.0f * us2, 32.0f * us2};
      Vector2 mouse = GetMousePosition();
      if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
        if (CheckCollisionPointRec(mouse, okR2)) {
          game.client.Stop(); game.client.Start();
          if (game.client.Connect(game.pendingJoinHostIP, game.joinPwdInput,
                                  game.pendingJoinRoomId)) {
            game.appMode = AppMode::RoomClient;
            game.roomName = game.pendingJoinRoomName;
            game.amReady = false;
            game.opReady = false;
            game.chatHistory.clear();
          }
          CloseJoinPasswordDialog(game);
        } else if (CheckCollisionPointRec(mouse, cancelR2)) {
          CloseJoinPasswordDialog(game);
        } else {
          Rectangle pwdBox3{cx2 + 20.0f * us2, cy2 + 62.0f * us2,
                            cw2 - 40.0f * us2, 34.0f * us2};
          if (CheckCollisionPointRec(mouse, pwdBox3)) {
            std::string display = std::string(game.joinPwdInput.size(), '*');
            game.joinPwdCursor = TextCursorFromMouseX(game.font, display, mouse.x,
                                                      cx2 + 30.0f * us2, 20.0f * us2);
            game.textAllSelected = false;
          }
        }
      }
      int key2 = GetCharPressed();
      while (key2 > 0) {
        if (game.textAllSelected) {
          game.joinPwdInput.clear();
          game.joinPwdCursor = 0;
          game.textAllSelected = false;
        }
        if (key2 >= 32 && key2 <= 126 && static_cast<int>(game.joinPwdInput.size()) < 20)
          TextInsertAt(game.joinPwdInput, game.joinPwdCursor, key2);
        key2 = GetCharPressed();
      }
      double now2 = GetTime();
      if (game.leftRepeat.tick(IsKeyDown(KEY_LEFT), now2)) {
        if (game.textAllSelected) {
          game.joinPwdCursor = 0;
          game.textAllSelected = false;
        } else {
          game.joinPwdCursor = Utf8PrevPos(game.joinPwdInput, game.joinPwdCursor);
        }
      }
      if (game.rightRepeat.tick(IsKeyDown(KEY_RIGHT), now2)) {
        if (game.textAllSelected) {
          game.joinPwdCursor = static_cast<int>(game.joinPwdInput.size());
          game.textAllSelected = false;
        } else {
          game.joinPwdCursor = Utf8NextPos(game.joinPwdInput, game.joinPwdCursor);
        }
      }
      if (IsKeyPressed(KEY_HOME)) { game.joinPwdCursor = 0; game.textAllSelected = false; }
      if (IsKeyPressed(KEY_END)) { game.joinPwdCursor = static_cast<int>(game.joinPwdInput.size()); game.textAllSelected = false; }
      if (game.delRepeat.tick(IsKeyDown(KEY_DELETE), now2)) {
        if (game.textAllSelected) {
          game.joinPwdInput.clear();
          game.joinPwdCursor = 0;
          game.textAllSelected = false;
        } else if (game.joinPwdCursor < static_cast<int>(game.joinPwdInput.size())) {
          TextEraseAfter(game.joinPwdInput, game.joinPwdCursor);
        }
      }
      bool ctrl2 = IsKeyDown(KEY_LEFT_CONTROL) || IsKeyDown(KEY_RIGHT_CONTROL);
      if (ctrl2 && IsKeyPressed(KEY_A) && !game.joinPwdInput.empty()) {
        game.joinPwdCursor = static_cast<int>(game.joinPwdInput.size());
        game.textAllSelected = true;
      }
      if (game.bsRepeat.tick(IsKeyDown(KEY_BACKSPACE), now2)) {
        if (game.textAllSelected) {
          game.joinPwdInput.clear();
          game.joinPwdCursor = 0;
          game.textAllSelected = false;
        } else if (game.joinPwdCursor > 0) {
          TextEraseBefore(game.joinPwdInput, game.joinPwdCursor);
        }
      }
    }
    return true;
  }
  return false;
}

void SendAuthoritativeOnlineState(Game &game, bool includePositions);
void ApplyClientOnlineState(Game &game, Phase phase,
                            const hb::RulesState &state);

bool UpdateRoomHostMode(Game &game) {
  if (game.appMode == AppMode::RoomHost) {
    game.host.Poll();
    auto syncHostReadyToClient = [&]() {
      if (game.amReady) game.host.SendReady();
      else game.host.SendUnready();
    };
    if (game.host.HasJoinRequest(nullptr)) {
      game.opReady = false;
      syncHostReadyToClient();
    }
    if (game.host.HasReady()) {
      game.opReady = true;
      syncHostReadyToClient();
    }
    if (game.host.HasUnready()) {
      game.opReady = false;
      syncHostReadyToClient();
    }
    if (game.host.HasChat(nullptr)) {
      std::string msg; game.host.HasChat(&msg);
      game.chatHistory.push_back("对手: " + msg);
    }
    if (game.host.HasBye() || game.host.ClientDisconnected()) {
      game.opReady = false;
      game.amReady = false;
      game.chatHistory.clear();
    }
    if (game.host.HasAim(nullptr, nullptr, nullptr, nullptr, nullptr)) {
      game.host.HasAim(&game.opAimTipX, &game.opAimTipY, &game.opPower, &game.opAimX, &game.opAimY);
    }
    if (game.amReady && game.opReady) {
      int clientPlayer = game.hostIsPlayer1 ? 1 : 0;
      game.host.SendAssign(clientPlayer);
      game.physics.Stop();
      game.world.ResetRack();
      game.rules.ResetRack(0);
      game.phase = Phase::Aiming;
      game.onlineTurn = 0;
      if (!game.hostIsPlayer1) game.onlineTurn = 1;
      game.onlineHost = true;
      game.onlineCuePlacementActive = false;
      game.appMode = AppMode::PlayingOnline;
      SendAuthoritativeOnlineState(game, true);
    }
    // Ready button
    if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
      Vector2 mouse = GetMousePosition();
      float sw = static_cast<float>(GetScreenWidth()), sh = static_cast<float>(GetScreenHeight());
      Rectangle readyBtn{sw * 0.5f - 60, sh - 120, 120, 34};
      if (CheckCollisionPointRec(mouse, readyBtn)) {
        game.amReady = !game.amReady;
        if (game.amReady) game.host.SendReady();
        else game.host.SendUnready();
      }
    }
    return true;
  }
  return false;
}

bool UpdateRoomClientMode(Game &game) {
  if (game.appMode == AppMode::RoomClient) {
    game.client.Poll();
    bool receivedLobbyState = false;
    if (game.client.HasAssign(nullptr)) {
      game.client.HasAssign(&game.assignedPlayer);
    }
    int lobbyTurn = -1;
    if (game.client.HasTurn(&lobbyTurn)) {
      int turn = lobbyTurn;
      game.rules.State().currentPlayer = turn;
    }
    Phase lobbyPhase = Phase::Aiming;
    hb::RulesState lobbyState{};
    if (game.client.HasState(&lobbyPhase, &lobbyState)) {
      ApplyClientOnlineState(game, lobbyPhase, lobbyState);
      receivedLobbyState = true;
    }
    if (game.client.HasPositions(nullptr)) {
      game.client.HasPositions(&game.syncedBalls);
    }
    if (game.client.HasReadyAck()) { game.opReady = true; }
    if (game.client.HasUnreadyAck()) { game.opReady = false; }
    if (game.client.HasChat(nullptr)) {
      std::string msg; game.client.HasChat(&msg);
      game.chatHistory.push_back("对手: " + msg);
    }
    if (game.client.HasBye() || game.client.Disconnected()) {
      game.opReady = false;
      game.amReady = false;
      game.assignedPlayer = -1;
      game.chatHistory.clear();
    }
    // Auto-return to lobby if connection was rejected or failed
    if (!game.client.IsAccepted() && !game.client.IsConnecting()) {
      game.client.Stop(); game.client.Start();
      game.appMode = AppMode::Lobby;
      game.showCreateDlg = false;
      game.showJoinPwdDlg = false;
      game.roomPassword.clear();
      game.assignedPlayer = -1;
      return true;
    }
    if (game.client.HasAim(nullptr, nullptr, nullptr, nullptr, nullptr)) {
      game.client.HasAim(&game.opAimTipX, &game.opAimTipY, &game.opPower, &game.opAimX, &game.opAimY);
    }
    if (game.amReady && game.opReady && game.client.IsAccepted() && game.assignedPlayer >= 0) {
      game.onlineTurn = game.assignedPlayer;
      game.onlineHost = false;
      game.onlineCuePlacementActive = false;
      game.physics.Stop();
      game.world.ResetRack();
      // Apply synced positions to world
      if (game.syncedBalls[0].pos.x != 0.0 || game.syncedBalls[0].pos.y != 0.0) {
        ApplySyncedBalls(game);
      }
      if (!receivedLobbyState) game.phase = Phase::Aiming;
      game.appMode = AppMode::PlayingOnline;
    }
    if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON) && game.client.IsAccepted()) {
      Vector2 mouse = GetMousePosition();
      float sw = static_cast<float>(GetScreenWidth()), sh = static_cast<float>(GetScreenHeight());
      Rectangle readyBtn{sw * 0.5f - 60, sh - 120, 120, 34};
      if (CheckCollisionPointRec(mouse, readyBtn)) {
        game.amReady = !game.amReady;
        if (game.amReady) game.client.SendReady();
        else game.client.SendUnready();
      }
    }
    return true;
  }
  return false;
}

void SendAuthoritativeOnlineState(Game &game, bool includePositions) {
  game.host.SendTurn(game.rules.State().currentPlayer);
  game.host.SendState(game.phase, game.rules.State());
  if (includePositions) {
    game.host.SendPositions(game.world.Balls());
    game.syncedBalls = game.world.Balls();
  }
}

void ApplyClientOnlineState(Game &game, Phase phase,
                            const hb::RulesState &state) {
  game.rules.State() = state;
  game.phase = phase;
  game.onlineCuePlacementActive = false;
}

bool UpdateOnlineMatch(Game &game, const View &view) {
  // Playing Online
  if (game.appMode == AppMode::PlayingOnline) {
    if (HandleTitleBarInput(game)) return true;
    bool amHost = game.onlineHost;
    if (amHost) game.host.Poll(); else game.client.Poll();
    // Handle opponent bye/disconnect
    if (amHost) {
      if (game.host.HasBye() || game.host.ClientDisconnected()) {
        game.appMode = AppMode::RoomHost; game.amReady = false; game.opReady = false; game.chatHistory.clear(); return true;
      }
      if (game.host.HasChat(nullptr)) { std::string m; game.host.HasChat(&m); game.chatHistory.push_back("对手: " + m); }
      if (game.host.HasAim(nullptr, nullptr, nullptr, nullptr, nullptr)) { game.host.HasAim(&game.opAimTipX, &game.opAimTipY, &game.opPower, &game.opAimX, &game.opAimY); }
    } else {
      if (game.client.HasBye() || game.client.Disconnected()) {
        game.appMode = AppMode::RoomClient; game.amReady = false; game.opReady = false; game.assignedPlayer = -1; game.chatHistory.clear(); return true;
      }
      if (game.client.HasChat(nullptr)) { std::string m; game.client.HasChat(&m); game.chatHistory.push_back("对手: " + m); }
      if (game.client.HasAim(nullptr, nullptr, nullptr, nullptr, nullptr)) { game.client.HasAim(&game.opAimTipX, &game.opAimTipY, &game.opPower, &game.opAimX, &game.opAimY); }
    }
    int onlineTurnMsg = -1;
    if (!amHost && game.client.HasTurn(&onlineTurnMsg)) {
      int turn = onlineTurnMsg;
      game.rules.State().currentPlayer = turn;
      if (game.phase == Phase::Moving) {
        game.phase = Phase::Aiming;
      }
    }
    Phase onlinePhase = Phase::Aiming;
    hb::RulesState onlineState{};
    if (!amHost && game.client.HasState(&onlinePhase, &onlineState)) {
      ApplyClientOnlineState(game, onlinePhase, onlineState);
    }
    bool isMyTurn = (game.rules.State().currentPlayer == game.onlineTurn);
    bool isOpponentTurn = !isMyTurn;
    const Phase phaseBeforeUi = game.phase;
    const int playerBeforeUi = game.rules.State().currentPlayer;
    const bool uiHandled = isMyTurn ? HandleUiInput(game) : false;
    if (uiHandled && isMyTurn && phaseBeforeUi == Phase::GroupChoice &&
        game.phase != Phase::GroupChoice &&
        playerBeforeUi >= 0 && playerBeforeUi < 2) {
      const BallGroup group = game.rules.State().players[playerBeforeUi].group;
      if (group == BallGroup::Solids || group == BallGroup::Stripes) {
        if (amHost) {
          SendAuthoritativeOnlineState(game, false);
        } else {
          game.client.SendGroupChoice(group);
        }
      }
    }
    const float dt = GetFrameTime();
    double now = GetTime();
    if (isMyTurn && !uiHandled && game.phase == Phase::Aiming && !game.world.CueBall().pocketed &&
        !game.world.CueBall().sinking) {
      const Vec2 mouseWorld = ScreenToWorld(view, GetMousePosition());
      game.aim = hb::Normalize(mouseWorld - game.world.CueBall().pos);
      const float wheel = GetMouseWheelMove();
      if (wheel < 0.0f) {
        game.power = hb::Clamp(game.power + (-wheel) * 0.055, 0.0, 1.0);
      } else if (wheel > 0.0f && game.power > 0.015) {
        ShotParams shot{game.aim, game.power, game.tipX, game.tipY};
        StartPhysicsShot(game, shot);
        game.phase = Phase::Moving;
        game.power = 0.0;
        if (amHost) {
          game.host.SendShot(game.tipX, game.tipY, shot.power, game.aim.x, game.aim.y);
          game.host.SendPositions(game.world.Balls());
          game.syncedBalls = game.world.Balls();
        } else {
          game.client.SendShot(game.tipX, game.tipY, shot.power, game.aim.x, game.aim.y);
        }
      }
      // Send aim updates
      if (now - game.lastAimSend > 0.067) {
        game.lastAimSend = now;
        if (amHost) game.host.SendAim(game.tipX, game.tipY, game.power, game.aim.x, game.aim.y);
        else game.client.SendAim(game.tipX, game.tipY, game.power, game.aim.x, game.aim.y);
        // also send aim direction? For simplicity, just tipX/tipY/power
      }
    } else if (isMyTurn && game.phase == Phase::BallInHand) {
      Vec2 pos = ClampCuePlacement(ScreenToWorld(view, GetMousePosition()));
      if (!uiHandled && game.world.CanPlaceCue(pos)) {
        game.physics.Stop();
        game.world.PlaceCue(pos);
        if (!amHost) {
          game.onlineCuePlacementActive = true;
        }
        const bool confirmed = IsMouseButtonPressed(MOUSE_LEFT_BUTTON);
        const bool shouldSendPlacement =
            confirmed || now - game.lastPosSend > 1.0 / 30.0;
        if (shouldSendPlacement) {
          game.lastPosSend = now;
          if (amHost) {
            game.host.SendPositions(game.world.Balls());
            game.syncedBalls = game.world.Balls();
          } else {
            game.client.SendCuePlacement(pos, confirmed);
          }
        }
        if (confirmed) {
          game.rules.State().ballInHand = false;
          game.phase = Phase::Aiming;
          game.syncedBalls = game.world.Balls();
          if (amHost) {
            game.onlineCuePlacementActive = false;
            SendAuthoritativeOnlineState(game, true);
          }
        }
      }
    } else if (isMyTurn && game.phase == Phase::Moving) {
      (void)dt;
      if (SyncPhysics(game)) {
        auto decision = game.rules.ApplyShot(game.shotEvents, game.world);
        game.phase = decision.nextPhase;
        if (game.phase == Phase::BallInHand) {
          game.physics.Stop();
          game.world.PlaceCue({hb::kHeadSpotX, 0.0});
        }
        if (amHost) {
          SendAuthoritativeOnlineState(game, true);
        }
      }
    } else if (isMyTurn && game.phase == Phase::RackOver && !game.chatInputActive && IsKeyPressed(KEY_SPACE)) {
      int nb = game.rules.State().winner >= 0 ? game.rules.State().winner : 0;
      game.physics.Stop();
      game.world.ResetRack();
      game.rules.ResetRack(nb);
      game.phase = Phase::Aiming;
      if (amHost) {
        SendAuthoritativeOnlineState(game, true);
      }
    }
    if (amHost && isOpponentTurn && game.phase == Phase::GroupChoice) {
      BallGroup group = BallGroup::Open;
      if (game.host.HasGroupChoice(&group)) {
        game.phase = game.rules.ChooseGroup(group).nextPhase;
        SendAuthoritativeOnlineState(game, false);
      }
    }
    if (amHost && isOpponentTurn && game.phase == Phase::BallInHand) {
      Vec2 cuePos{};
      bool confirmed = false;
      if (game.host.HasCuePlacement(&cuePos, &confirmed)) {
        const Vec2 pos = ClampCuePlacement(cuePos);
        bool accepted = false;
        if (game.world.CanPlaceCue(pos)) {
          game.physics.Stop();
          game.world.PlaceCue(pos);
          accepted = true;
          if (confirmed) {
            game.rules.State().ballInHand = false;
            game.phase = Phase::Aiming;
          }
        }
        if (confirmed || !accepted) {
          SendAuthoritativeOnlineState(game, true);
        } else {
          game.host.SendPositions(game.world.Balls());
          game.syncedBalls = game.world.Balls();
        }
      }
    }
    // Host: compute physics for opponent's shot if received
    if (amHost && isOpponentTurn && game.phase == Phase::Aiming) {
      if (game.host.HasShot(nullptr)) {
        ShotParams opShot;
        game.host.HasShot(&opShot);
        StartPhysicsShot(game, opShot);
        game.phase = Phase::Moving;
      }
    }
    // Also handle opponent's Moving phase on host
    if (amHost && isOpponentTurn && game.phase == Phase::Moving) {
      (void)dt;
      if (SyncPhysics(game)) {
        auto decision = game.rules.ApplyShot(game.shotEvents, game.world);
        game.phase = decision.nextPhase;
        if (game.phase == Phase::BallInHand) {
          game.physics.Stop();
          game.world.PlaceCue({hb::kHeadSpotX, 0.0});
        }
        SendAuthoritativeOnlineState(game, true);
      }
    }
    // Client: apply received shot
    if (!amHost && isOpponentTurn && game.phase == Phase::Aiming) {
      if (game.client.HasShot(nullptr)) {
        ShotParams opShot;
        game.client.HasShot(&opShot);
        StartPhysicsShot(game, opShot);
        game.phase = Phase::Moving;
      }
    }
    // Host: send positions during moving phase
    if (amHost && game.phase == Phase::Moving) {
      if (now - game.lastPosSend > 1.0 / 30.0) {
        game.lastPosSend = now;
        game.host.SendPositions(game.world.Balls());
        game.syncedBalls = game.world.Balls();
      }
    }
    // Client: receive and render positions
    if (!amHost) {
      if (game.client.HasPositions(nullptr)) {
        game.client.HasPositions(&game.syncedBalls);
      }
      // Override world ball positions with synced data when not actively computing physics
      const bool waitingForCuePlacementAck =
          isMyTurn && game.onlineCuePlacementActive;
      if (!waitingForCuePlacementAck && game.phase != Phase::Moving) {
        ApplySyncedBalls(game);
      }
    }
    // Handle turn messages for client
    int finalTurnMsg = -1;
    if (!amHost && game.client.HasTurn(&finalTurnMsg)) {
      int turn = finalTurnMsg;
      game.rules.State().currentPlayer = turn;
    }
    if (!game.chatInputActive && IsKeyPressed(KEY_C)) { game.tipX = 0.0; game.tipY = 0.0; }
    return true;
  }
  return false;
}

void UpdateSinglePlayer(Game &game, const View &view) {
  // Single-player (original)
  if (!game.chatInputActive && IsKeyPressed(KEY_ESCAPE)) {
    if (game.power > 0.0) {
      game.power = 0.0;
      return;
    }
  }
  if (HandleTitleBarInput(game)) {
    return;
  }
  const bool uiHandled = HandleUiInput(game);
  const float dt = GetFrameTime();
  if (!uiHandled && game.phase == Phase::Aiming && !game.world.CueBall().pocketed &&
      !game.world.CueBall().sinking) {
    const Vec2 mouseWorld = ScreenToWorld(view, GetMousePosition());
    game.aim = hb::Normalize(mouseWorld - game.world.CueBall().pos);
    const float wheel = GetMouseWheelMove();
    if (wheel < 0.0f) {
      game.power = hb::Clamp(game.power + (-wheel) * 0.055, 0.0, 1.0);
    } else if (wheel > 0.0f && game.power > 0.015) {
      ShotParams shot{game.aim, game.power, game.tipX, game.tipY};
      StartPhysicsShot(game, shot);
      game.phase = Phase::Moving;
      game.power = 0.0;
    }
  } else if (game.phase == Phase::BallInHand) {
    Vec2 pos = ClampCuePlacement(ScreenToWorld(view, GetMousePosition()));
    if (!uiHandled && game.world.CanPlaceCue(pos)) {
      game.physics.Stop();
      game.world.PlaceCue(pos);
      if (IsMouseButtonPressed(MOUSE_LEFT_BUTTON)) {
        game.rules.State().ballInHand = false;
        game.phase = Phase::Aiming;
      }
    }
  } else if (game.phase == Phase::Moving) {
    (void)dt;
    if (SyncPhysics(game)) {
      auto decision = game.rules.ApplyShot(game.shotEvents, game.world);
      game.phase = decision.nextPhase;
      if (game.phase == Phase::BallInHand) {
        game.physics.Stop();
        game.world.PlaceCue({hb::kHeadSpotX, 0.0});
      }
    }
  } else if (game.phase == Phase::RackOver && !game.chatInputActive && IsKeyPressed(KEY_SPACE)) {
    const int nextBreaker = game.rules.State().winner >= 0 ? game.rules.State().winner : 0;
    game.physics.Stop();
    game.world.ResetRack();
    game.rules.ResetRack(nextBreaker);
    game.phase = Phase::Aiming;
  }
  if (!game.chatInputActive && IsKeyPressed(KEY_C)) {
    game.tipX = 0.0;
    game.tipY = 0.0;
  }
}

void UpdateGame(Game &game, const View &view) {
  if (HandleTitleBarInput(game)) return;
  HandleChatInput(game);
  if (UpdateLobbyMode(game)) return;
  if (UpdateRoomHostMode(game)) return;
  if (UpdateRoomClientMode(game)) return;
  if (UpdateOnlineMatch(game, view)) return;
  UpdateSinglePlayer(game, view);
}
