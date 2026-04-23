# Heyball Raylib

一个使用 raylib 和 C++17 制作的本地双人中式八球/Heyball 桌面游戏。画面采用纯黑背景、自绘无系统标题栏和正俯视 2.5D 球台，球体、数字、半色球色带和滚动贴花均为程序化绘制。

## 当前功能

- 本地两名玩家轮流对战。
- 规则按 WPA Heyball 2025 风格实现核心流程：开放球局、球组判定、合法进球继续、犯规自由球、8 号球胜负、45 秒计时和一次延时。
- 鼠标拖动控制击球方向，滚轮下滑蓄力，滚轮上滑出杆。
- 右上角击点盘支持高杆、低杆、左塞、右塞，默认中心击球。
- 专用台球物理：固定步长、球-球碰撞、球-库碰撞、摩擦、旋转、库边塞效应、落袋判定和落袋动画。
- 瞄准线会延伸到球台边框，并在目标球撞点处绘制白球空心圆位置提示。
- 自绘黑色标题栏，与背景融合；支持拖动窗口和边缘/角落缩放；默认不打开控制台，使用 `--console` 或 `--debug-console` 才显示调试控制台。
- 使用粗体等线字体和双线性过滤改善文字清晰度；球杆绘制在所有球之上，球号圆牌放大以便读取。

## 构建

项目要求使用 MSYS2 UCRT64 工具链，不使用系统 PATH 中的其它 `g++`。

```powershell
D:\msys64\ucrt64\bin\cmake.exe -S . -B build/ucrt64-debug -G Ninja -DCMAKE_CXX_COMPILER=D:/msys64/ucrt64/bin/g++.exe -DCMAKE_MAKE_PROGRAM=D:/msys64/ucrt64/bin/ninja.exe
D:\msys64\ucrt64\bin\cmake.exe --build build/ucrt64-debug
```

构建会生成：

- `build/ucrt64-debug/heyball.exe`
- `build/ucrt64-debug/heyball_tests.exe`

构建后会自动复制 `glfw3.dll` 到主程序输出目录。

## 运行

```powershell
.\build\ucrt64-debug\heyball.exe
```

带调试控制台运行：

```powershell
.\build\ucrt64-debug\heyball.exe --console
```

## 测试

```powershell
.\build\ucrt64-debug\heyball_tests.exe
```

测试覆盖基础球-球碰撞、库边反弹、摩擦停球、落袋事件和若干规则判定。

## 规格与比例

- 台面：`2540 x 1260mm`
- 球直径：`57.15mm`
- 角袋口：约 `76.2mm`
- 腰袋口：约 `82.5mm`

这些比例用于让中式/Heyball 的窄袋和标准球尺寸在画面与物理中保持协调。

## 当前调校重点

- 继续优化袋口旁边库边曲线、角袋/中袋 jaws 和袋口拒球表现。
- 继续调校高低杆、左右塞、旋转转移和库边反弹手感。
- 补充更多 WPA Heyball 2025 规则边角案例与整局手动验收。
