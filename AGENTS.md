# Heyball 开发接手指南

这份文档给后续进入 `haokee-git/heyball` 的开发者使用。请优先阅读这里，再动代码。

## 项目目标

这是一个用 C++17 + raylib 实现的中式八球/Heyball 2D 视角台球游戏。画面是正俯视 2D，但球体、球杆、袋口、库边和滚动数字贴花都按接近 3D 的视觉效果绘制。项目目标是保持高帧率、真实物理手感、中文 UI、本地双人规则流程和局域网联机对战。

当前玩法已经包含：

- 单机双人轮流对战。
- 中式八球核心规则：开局、球组判定、自由球、犯规换手、8 号球胜负。
- 自定义球桌、球、库边、袋口、球杆、瞄准线和 UI。
- 局域网大厅、创建房间、密码房、就绪、聊天、同步球位、同步瞄准、同步击球。
- 物理计算和网络轮询已从主 UI 线程中分离。

## 当前最新提交

截至 2026-05-17，`main` 最新一轮更新为联机同步、图标、袋口几何和发布目录清理。最终提交哈希请以 `git log -1 --oneline` 为准。

上一轮已推送的远端提交是：

```text
2c5e2c7 Fix online match synchronization
```

最近重要进展：

- `2c5e2c7 Fix online match synchronization`
  - 修复联机房间创建/取消/改密码/切换先后手后，另一玩家加入但房主仍显示等待的问题。
  - 修复联机先后手、准备状态、聊天框布局、联机重摆球按钮、加塞同步和状态显示问题。
  - 修复联机自由球阶段客户端球位同步策略，避免球权方摆放白球时被旧位置覆盖；已记录并继续关注复杂自由球场景下的边界同步。
- `5989af0 Split runtime helpers from main`
  - 拆出异步物理/网络运行时。
  - 拆出 UTF-8 文本工具。
  - 拆出旧 MinGW 线程兼容层。
- `604c00f Split game update and draw flow`
  - 将 `UpdateGame` 细拆到 `src/game_update.inc`。
  - 将 `DrawGame` 细拆到 `src/game_draw.inc`。
  - `src/main.cpp` 从约 2739 行降到约 1864 行。

本轮已处理内容：

- 联机自由球同步：
  - `ApplySyncedBalls(Game&, bool preserveCueBall = false)` 支持摆球方保留本地白球位置，同时继续接收权威目标球位置。
  - 客户端只在真正收到 `Positions` 包时覆盖本地球位，不再每帧用旧同步缓存覆盖。
- 袋口和库边：
  - 库边入口形状改为“平行库边直线、一小段曲线、约 45 度入袋直线”的三段结构。
  - 视觉库边和物理碰撞体同步更新，贝塞尔分段增加，袋口过渡更平滑。
  - 袋口常量改为完整开口语义：角袋 `85mm`，中袋 `86mm`，球径仍为 `57.15mm`。
  - 可见袋洞、库边断点、`PocketRadiusForIndex` 和进袋判定统一使用 `kCornerPocketAxisGap` / `kSidePocketHalfMouth`，避免视觉与物理脱节。
- 应用图标：
  - 新增 `assets/heyball_icon.png`、`assets/heyball_icon.ico`、`heyball.rc`。
  - CMake 将 ICO 资源嵌入 exe，并把 PNG 复制到运行目录。
  - 启动时同时使用 raylib/GLFW `SetWindowIcons` 和 Windows `WM_SETICON` / 窗口类图标，避免任务栏回退为默认 exe 图标。
  - 图标透明留白已收紧，有效覆盖约 `90.8% x 93.0%`，避免任务栏图标比其他应用小一圈。
- Release 输出：
  - `build/` 不再作为仓库内容维护，只保留为本地 CMake 构建树。
  - Release 玩家运行文件输出到根目录 `release/`，该目录只应包含 `heyball.exe`、`glfw3.dll`、`PingFangSC.otf`、`heyball_icon.png` 等运行必需文件。
  - `release/` 和 `build/` 都在 `.gitignore` 中，避免提交中间文件、测试程序、静态库、Ninja/CMake 缓存和对象文件。

## 构建环境

不要把某一台机器上的工具路径当成永久事实。后续开发者可能在另一台机器上工作，只要路径和工具链有效，就应优先使用仓库已有的标准 CMake 预设。

首选验证命令：

```powershell
D:\msys64\ucrt64\bin\cmake.exe --preset ucrt64-debug
D:\msys64\ucrt64\bin\cmake.exe --build --preset ucrt64-debug
D:\msys64\ucrt64\bin\ctest.exe --preset ucrt64-debug --output-on-failure
D:\msys64\ucrt64\bin\cmake.exe --preset ucrt64-release
D:\msys64\ucrt64\bin\cmake.exe --build --preset ucrt64-release
D:\msys64\ucrt64\bin\ctest.exe --preset ucrt64-release --output-on-failure
```

`CMakePresets.json` 默认使用：

- MSYS2 UCRT64 根目录：`D:/msys64/ucrt64`
- Debug 构建目录：`build/ucrt64-debug`
- Release 构建目录：`build/ucrt64-release`
- Release 玩家输出目录：`release/`

注意区分 `build/ucrt64-release` 和 `release/`：

- `build/ucrt64-release` 是 CMake/Ninja 本地构建树，可以包含缓存、对象文件、测试程序和临时文件，不面向玩家分发。
- `release/` 是 Release 构建生成的玩家目录，只放正常游玩需要的运行文件。
- 不要把 `build/` 或 `release/` 内容提交进仓库；需要发布二进制时，应从干净的 `release/` 目录单独打包。

如果当前机器上的 `D:\msys64\ucrt64` 损坏或不可用，才使用本机备用工具链。不要把备用路径写入源码或 CMake 固定配置，除非用户明确要求。当前这台机器曾验证过的备用组合如下，仅作应急参考：

- CMake：`D:\Program Files\CMake\bin\cmake.exe`
- MinGW：`C:\PROGRA~2\mingw64\bin`
- raylib 前缀：`D:\haokee\heyball\build\raylib-mingw-prefix`
- Debug 构建目录：`build/mingw64-debug-full`
- Release 构建目录：`build/mingw64-release-full`

备用验证命令示例：

```powershell
$env:PATH='C:\PROGRA~2\mingw64\bin;D:\haokee\heyball\build\raylib-mingw-prefix\bin;C:\Windows\System32;C:\Windows;C:\Windows\System32\Wbem'
& 'D:\Program Files\CMake\bin\cmake.exe' --build build/mingw64-debug-full -- -j4
& 'D:\Program Files\CMake\bin\cmake.exe' --build build/mingw64-release-full -- -j4
& 'D:\Program Files\CMake\bin\ctest.exe' --test-dir build/mingw64-debug-full --output-on-failure
& 'D:\Program Files\CMake\bin\ctest.exe' --test-dir build/mingw64-release-full --output-on-failure
```

注意：

- `raymath.h` 会产生大量 `missing initializer` 警告，这是外部头文件的既有警告。
- Debug 下 `heyball_network_tests` 的 `T10_Burst` 偶尔会因网络时序失败。若第一次失败，请单独重跑：

```powershell
& 'D:\Program Files\CMake\bin\ctest.exe' --test-dir build/mingw64-debug-full -R heyball_network_tests --output-on-failure
```

此前多次观察到单独重跑通过，然后 Debug 全套重跑也通过。不要在没有复现稳定失败前贸然改网络协议。

## 代码结构

主要源码位于 `src/`。

### `core.hpp` / `core.cpp`

核心规则和物理：

- `hb::PhysicsWorld`
- 球、袋口、库边、碰撞、滚动/滑动摩擦、进袋事件。
- `hb::RulesEngine`
- 中式八球规则状态机和击球结果判定。

这里被测试覆盖最多，改动前请先理解 `tests/test_runner.cpp`。

### `network.hpp` / `network.cpp`

局域网协议和同步：

- `hb::NetworkHost`
- `hb::NetworkClient`
- 房间广播、加入、密码校验、就绪、聊天、瞄准、击球、球位同步、断开。

`tests/test_network.cpp` 覆盖网络行为。注意网络测试存在少量时序敏感性。

### `async_runtime.hpp` / `async_runtime.cpp`

主线程外的运行时封装：

- `PhysicsRunner`
  - 单独线程推进 `hb::PhysicsWorld::Step`。
  - 主线程通过 `Begin`、`Stop`、`Snapshot` 与它交互。
- `AsyncNetworkHost`
  - 后台轮询 `hb::NetworkHost`。
  - 将网络事件缓存到 frame event，主线程每帧 `Poll`。
- `AsyncNetworkClient`
  - 后台轮询 `hb::NetworkClient`。
  - API 与 Host 尽量对称。

原则：

- 不要在主 UI/绘制循环里直接阻塞等待网络。
- 不要让物理长时间占用主线程。
- 与异步类交互时保持数据快照思维，不要跨线程持有引用。

### `platform_threads.hpp`

旧 MinGW 的线程兼容层。部分 MinGW/libstdc++ 组合没有完整 gthreads 支持，因此这里在 Windows 且没有 `_GLIBCXX_HAS_GTHREADS` 时补了最小 `std::mutex`、`std::thread`、`std::this_thread::sleep_for` 实现。

不要随便扩大这个兼容层的功能面。它只为当前项目用到的后台类服务。

### `text_utils.hpp` / `text_utils.cpp`

UTF-8 输入和文本编辑工具：

- `BuildFontCodepoints`
- `Utf8PrevPos`
- `Utf8NextPos`
- `Utf8CodepointCount`
- `TextInsertAt`
- `TextEraseBefore`
- `TextEraseAfter`
- `TextCursorFromMouseX`
- `BackspaceRepeat`

所有中文输入框、聊天输入框、密码框的光标和重复删除逻辑都应该走这里。

### `main.cpp`

仍是游戏入口和多数渲染 helper 所在文件，但已显著瘦身。当前保留：

- `Game` 状态结构。
- `View`、窗口行为、坐标转换、UI 尺寸布局。
- 球桌、球、球杆、标题栏、帮助窗口、大厅/房间 UI 的绘制 helper。
- 创建房间、加入密码等弹窗布局 helper。
- `main` 启动流程。

`main.cpp` 末尾通过 include 引入两个分片：

```cpp
#include "game_update.inc"
#include "game_draw.inc"
```

这两个 `.inc` 文件处于同一个匿名命名空间内，可以直接访问 `Game`、`View` 和上方 helper。这样做是为了先降低主文件复杂度，同时避免一次性把大量内部 helper 提升到公共头文件造成更大风险。

### `game_update.inc`

从原 `UpdateGame` 拆出来的更新逻辑。当前函数边界：

- `HandleChatInput(Game&)`
- `UpdateLobbyMode(Game&)`
- `UpdateRoomHostMode(Game&)`
- `UpdateRoomClientMode(Game&)`
- `UpdateOnlineMatch(Game&, const View&)`
- `UpdateSinglePlayer(Game&, const View&)`
- `UpdateGame(Game&, const View&)`

维护建议：

- 修改联机大厅行为，优先看 `UpdateLobbyMode`。
- 修改房间等待/就绪，优先看 `UpdateRoomHostMode` 和 `UpdateRoomClientMode`。
- 修改联机对局同步，优先看 `UpdateOnlineMatch`。
- 修改单机击球/自由球/重开，优先看 `UpdateSinglePlayer`。
- 聊天输入、退格、左右光标、全选、取消输入，优先看 `HandleChatInput` 和 `text_utils`。

### `game_draw.inc`

从原 `DrawGame` 拆出来的绘制调度。当前函数边界：

- `DrawLobbyOrRoom(Game&)`
- `DrawTableScene(Game&, const View&)`
- `DrawAimAndOnlineChat(Game&, const View&)`
- `DrawGameOverlays(Game&, const View&)`
- `DrawGame(Game&, const View&)`

维护建议：

- 大厅和房间等待页显示问题，看 `DrawLobbyOrRoom` 以及 `DrawLobby` / `DrawRoomWait`。
- 球桌、球体绘制顺序问题，看 `DrawTableScene`。
- 当前玩家/对手瞄准显示和聊天窗口，看 `DrawAimAndOnlineChat`。
- 状态栏、FPS、自由球圈、控制面板、帮助窗、结束弹窗、标题栏，看 `DrawGameOverlays`。

## 重要行为和已知设计

### 渲染

- 使用 raylib/OpenGL 路径渲染，不是 Vulkan。
- 球体是程序化逐像素绘制：全色球、半色球、数字贴花、滚动方向、受光和高光都在 CPU 侧计算后用 raylib 画出。
- 画面整体是 2D 顶视，但球、球杆和袋口尽量模拟立体质感。

### 物理

- 当前物理在 `PhysicsRunner` 后台线程推进。
- 主线程每帧用 `SyncPhysics` 拿快照。
- 击球入口统一用 `StartPhysicsShot`。
- 若收到联机球位同步，使用 `ApplySyncedBalls` 覆盖本地球位，并停止本地物理线程。

### 联机

- Host 和 Client 均由异步 wrapper 轮询。
- 在线对局里用 `game.onlineHost` 区分本机是否为 Host，不要再用 `host.HasClient()` 推断身份。
- Host 负责权威物理和 `SendPositions`。
- Client 接收 `Positions` 后同步球位。
- 双方都会同步瞄准参数，当前包括 `tipX/tipY/power/aimX/aimY`。

### UI 与输入

- `Enter` 打开/发送聊天。
- 鼠标控制瞄准方向。
- 鼠标滚轮向下蓄力，向上释放击球。
- `C` 归中击点。
- 控制面板可还原击点、重摆球。
- 无边框窗口使用自绘标题栏，窗口移动/缩放逻辑在 `HandleTitleBarInput`。

## 开发守则

- 修改代码后必须同时验证 Debug 和 Release。
- 不要在没验证的情况下提交或推送。
- 不要把 GitHub token、个人密钥或本地绝对私密路径写进代码或 remote。
- 当前仓库 remote 应保持：

```text
origin https://github.com/haokee-git/heyball.git
```

- 若需要推送且 HTTPS 认证失败，不要把个人凭证写进 remote 或文档。某些本机环境可能在 `D:\installer\github_token.txt` 放有临时 token；只有用户明确允许且该文件存在时，才可作为一次性 push URL 使用：

```powershell
$token = (Get-Content -LiteralPath 'D:\installer\github_token.txt' -Raw).Trim()
$remote = "https://x-access-token:$token@github.com/haokee-git/heyball.git"
git push $remote main
```

- 提交作者当前已在仓库本地配置为：

```text
haokee-git <19173155158@163.com>
```

## 后续建议

推荐继续拆分，但要小步走：

- 将大厅/房间绘制 helper 从 `main.cpp` 迁入单独 UI 文件。
- 将标题栏和窗口拖拽/缩放逻辑拆成窗口模块。
- 将球桌/球/球杆渲染拆成 render 模块。
- 将 `Game` 状态和 `AppMode` 从 `main.cpp` 提升到 `game_state.hpp`，再把 `.inc` 文件逐步改成正常 `.cpp`。
- 为联机对局流程补更多测试，尤其是断线、重连、连续击球、球位同步边界。
- 继续手动验收真实袋口拒球、库边鼻线、旋转转移、击球手感和规则边角案例。
