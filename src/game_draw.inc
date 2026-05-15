bool DrawLobbyOrRoom(Game &game) {
  if (game.appMode == AppMode::Lobby) {
    DrawLobby(game);
    if (game.showCreateDlg || game.showJoinPwdDlg) {
      DrawBackdrop();
      if (game.showJoinPwdDlg) {
        DrawJoinPasswordDialog(game);
      } else if (game.showCreateDlg) {
        DrawCreateRoomDialog(game);
      }
    }
    return true;
  }
  if (game.appMode == AppMode::RoomHost || game.appMode == AppMode::RoomClient) {
    DrawRoomWait(game);
    return true;
  }
  return false;
}

void DrawTableScene(Game &game, const View &view) {
  ClearBackground(BLACK);
  DrawTable(view);
  DrawPockets(view);
  for (const Ball &ball : game.world.Balls()) {
    if (!ball.sinking) {
      DrawBall(game.font, view, ball);
    }
  }
  for (const Ball &ball : game.world.Balls()) {
    if (ball.sinking) {
      DrawBall(game.font, view, ball);
    }
  }
}

void DrawAimAndOnlineChat(Game &game, const View &view) {
  if (game.appMode == AppMode::PlayingOnline) {
    bool isMyTurn = (game.rules.State().currentPlayer == game.onlineTurn);
    if (isMyTurn && game.phase == Phase::Aiming) {
      DrawCueAndAim(game, view);
      DrawCueShadow(game, view);
    } else if (!isMyTurn && game.phase == Phase::Aiming) {
      const Vector2 oc = WorldToScreen(view, game.world.CueBall().pos);
      Vec2 opAim = hb::Normalize({game.opAimX, game.opAimY});
      Vector2 aimDir{static_cast<float>(opAim.x), static_cast<float>(opAim.y)};
      const float ballR = static_cast<float>(hb::kBallRadius * view.scale);
      DrawDetailedCue(oc, aimDir, ballR, static_cast<float>(game.opPower), view);
      // Draw opponent aim line
      Vec2 ghostPos{};
      const bool hasGhost = FirstAimTarget(game.world, game.world.CueBall().pos, opAim, &ghostPos) > 0;
      const Vec2 aimStop = hasGhost ? ghostPos : RayToTableEdge(game.world.CueBall().pos, opAim);
      const Vector2 aimEnd = WorldToScreen(view, aimStop);
      DrawLineEx(oc, aimEnd, 1.0f, {214, 225, 210, 70});
      if (hasGhost) {
        const Vector2 ghost = WorldToScreen(view, ghostPos);
        const float ghostR = static_cast<float>(hb::kBallRadius * view.scale);
        DrawCircleLines(static_cast<int>(ghost.x), static_cast<int>(ghost.y), ghostR,
                        {236, 241, 228, 205});
      }
    }
    DrawChatWindow(game);
  } else {
    DrawCueAndAim(game, view);
    DrawCueShadow(game, view);
  }
}

void DrawGameOverlays(Game &game, const View &view) {
  DrawTopStatus(game);
  DrawFpsCounter(game.font);
  if (game.phase == Phase::BallInHand) {
    const Vector2 p = WorldToScreen(view, game.world.CueBall().pos);
    DrawCircleLines(static_cast<int>(p.x), static_cast<int>(p.y),
                    static_cast<float>(hb::kBallRadius * view.scale * 1.45),
                    {236, 238, 228, 150});
  }
  DrawUI(game);
  if (game.helpOpen) DrawBackdrop();
  DrawHelpWindow(game);
  if (game.phase == Phase::RackOver) {
    DrawBackdrop();
    Rectangle r{static_cast<float>(GetScreenWidth()) * 0.5f - 190.0f,
                static_cast<float>(GetScreenHeight()) * 0.5f - 45.0f, 380.0f, 90.0f};
    DrawRoundedFillWithBorder(r, 0.05f, 12, {11, 12, 12, 240},
                              {70, 82, 76, 210});
    const std::string rackMessage = LocalStatusMessage(game);
    DrawTextF(game.font, rackMessage.c_str(), {r.x + 35, r.y + 15}, 28,
              {236, 238, 228, 255});
    DrawTextF(game.font, "按空格开始新一局", {r.x + 63, r.y + 52}, 22,
              {180, 190, 182, 255});
  }
  DrawTitleBar(game);
}

void DrawGame(Game &game, const View &view) {
  if (DrawLobbyOrRoom(game)) return;
  DrawTableScene(game, view);
  DrawAimAndOnlineChat(game, view);
  DrawGameOverlays(game, view);
}