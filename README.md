# X-Plane 12 GNSS to Android

将 **X-Plane 12** 中飞机的实时位置、速度、航向、高度读取出来，通过 UDP 推送给安卓手机，由手机 App 把该位置注入为**模拟定位**（Mock GPS），从而在第三方地图 / EFB 中实时导航。

本工程参考并移植自同目录下的两个项目：

- [`MSFS-SimGPStoAndroid-main`](https://github.com/JinShichang/MSFS-SimGPStoAndroid/tree/main) —— 提供“读取游戏数据 → UDP 推送到安卓 → 安卓注入模拟定位”的整套方法与**网络协议**（Android 端协议解析、平滑插值、悬浮窗全部沿用）。
- [`XPlaneConnect-1.2.1`](https://github.com/nasa/XPlaneConnect) —— 提供 X-Plane 插件运行机制、底层 SDK（XPLM）头文件与导入库，以及 dataref 读写范例。

与 MSFS 版的对应关系：

| MSFS 版 | 本工程 |
| --- | --- |
| 运行在电脑上的独立 EXE，通过 **SimConnect** 读取游戏数据 | 运行在游戏内的 **X-Plane 插件（.xpl）**，通过 **XPLM SDK** 直接读 dataref |
| `main.cpp` 中的 UDP 服务端（HELLO / PONG / HEARTBEAT / 数据帧） | `src/GpsUdpServer.cpp`（协议逐字节兼容） |
| SimConnect 数据定义（经纬度/高度/航向/俯仰/横滚/地速/空速） | `src/XP12GNSS.cpp` 中的 X-Plane dataref 映射 |

> 网络协议与 `MSFS-SimGPStoAndroid` 完全一致，因此该项目的 **Android APK 无需任何改动**即可直接连接本插件，即使用[`MSFS-SimGPStoAndroid-main`](https://github.com/JinShichang/MSFS-SimGPStoAndroid/tree/main)中提供的APK。  
> 在此感谢 `MSFS-SimGPStoAndroid-main` 项目及其开发者 **JinShichang** ！

---

## 原理

### 1. 读取 X-Plane 12 数据

插件启动时用 `XPLMFindDataRef` 找到以下 dataref（每次启动查询一次，随后每帧直接读取），在飞行循环回调（`XPLMRegisterFlightLoopCallback`）中按设定的刷新间隔采集：

| 字段 | dataref | 类型 | 单位 → 线上单位 |
| --- | --- | --- | --- |
| 纬度 | `sim/flightmodel/position/latitude` | double | 度 |
| 经度 | `sim/flightmodel/position/longitude` | double | 度 |
| 高度 | `sim/flightmodel/position/elevation` | double | 米 → **英尺**（×3.2808） |
| 航向（真航向） | `sim/flightmodel/position/true_psi`（回退 `psi`） | float | 度 |
| 俯仰 | `sim/flightmodel/position/theta` | float | 度 |
| 横滚 | `sim/flightmodel/position/phi` | float | 度 |
| 地速 | `sim/flightmodel/position/groundspeed` | float | m/s |
| 空速（IAS） | `sim/flightmodel/position/indicated_airspeed` | float | m/s |

### 2. UDP 推送到安卓（与 MSFS 版同一协议）

```
手机 → 电脑 :  "HELLO:<手机开机毫秒时间戳>"   每 2 秒一次（注册 + 保活）
电脑 → 手机 :  "PONG:<回显的时间戳>"          延迟测量
电脑 → 手机 :  "HEARTBEAT"                    每 2 秒一次
电脑 → 手机 :  "SERVER_FULL"                  客户端已满（最多 3 台）
电脑 → 手机 :  "纬度,经度,高度_ft,航向,俯仰,横滚,地速_mps,空速_mps\n"  数据帧
```

数据帧格式（与 `MSFSSimConnect/main.cpp` 相同）：

```
%.6f,%.6f,%.1f,%.2f,%.2f,%.2f,%.2f,%.2f\n
```

### 3. 安卓注入模拟定位

手机端 App（`com.msfs.simconnect.client`）把收到的 8 个字段解析为 `FlightData`，通过 `LocationManager.addTestProvider` 的模拟位置提供者注入 GPS / NETWORK，并在两帧之间做 5~1000Hz 的平滑预测插值。以上逻辑完全复用 MSFS 版安卓工程。

---

## 目录结构

```
XPlane12-GNSS2Android
├─ src/
│  ├─ XP12GNSS.cpp     插件入口 + dataref 读取 + 数据帧格式化 + 飞行循环
│  ├─ XP12GNSS.h
│  ├─ GpsUdpServer.cpp UDP 服务端（协议线程 + 心跳 + 客户端管理）
│  └─ GpsUdpServer.h
├─ SDK/                内置的 X-Plane 插件 SDK（自 XPlaneConnect 提取）
│  ├─ CHeaders/XPLM/   XPLM 头文件
│  ├─ Libraries/Win/   XPLM.lib / XPLM_64.lib
│  └─ license.txt
├─ XP12GNSS.slnx        Visual Studio 解决方案（x64）
├─ XP12GNSS.vcxproj     Visual Studio 工程（输出 win.xpl）
├─ CMakeLists.txt       跨平台构建（Windows / Linux / macOS）
└─ README.md
```

---

## 构建

### Visual Studio（Windows）

用 VS 打开 `XP12GNSS.slnx`，选择 **Release | x64** 构建。产物：

```
x64\Release\win.xpl
```

命令行构建：

```bat
MSBuild XP12GNSS.vcxproj /p:Configuration=Release /p:Platform=x64
```

### CMake（可选，跨平台）

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build
```

产物按平台为 `win.xpl` / `lin.xpl` / `mac.xpl`。

> 说明：随附的 SDK 头文件来自 X-Plane SDK 2.1.2（与 X-Plane 12 的 XPLM 插件 ABI 向后兼容）。如需使用本机新版 SDK，请将 `SDK/CHeaders/XPLM` 替换为你 `X-Plane 12/Resources/plugins/SDK/CHeaders/XPLM` 的内容，并把 `SDK/Libraries/Win` 换成新版 `Libraries/Win/XPLM_64.lib`。

---

## 安装与使用

1. 在 `X-Plane 12/Resources/plugins/` 下新建插件文件夹，例如：
   ```
   X-Plane 12/Resources/plugins/XPlane12-GNSS2Android/
   └─ win_x64/
      └─ win.xpl
   ```
2. 启动 X-Plane 12，插件会自动开始监听 UDP 端口（默认 **36666**），并在 `Log.txt` 中写入 `XP12-GNSS2Android: UDP server listening on port 36666`。
3. 手机上安装 `MSFS-SimGPStoAndroid` 的 APK，在 系统设置 → 开发者选项 → 选择模拟位置信息应用 中选中它。
4. 手机 App 填写电脑的局域网 IP 与端口 36666，连接成功即开始注入模拟定位。
5. 打开手机的定位服务。
6. 打开高德/谷歌地图等地图软件即可看到飞机实时位置。

> 首次使用请允许 Windows 防火墙放行 UDP 36666（或插件所在进程）。

---

## 配置

可通过两个途径修改配置（默认无需任何配置即可使用）：

1. **INI 文件**：在插件所在目录（`win.xpl` 旁）新建 `XP12-GNSS2Android.ini`：
   ```ini
   udp_port=36666
   refresh_interval_ms=50
   enabled=1
   ```
2. **X-Plane dataref（运行时热改，可用 DataRefTool 查看/修改）**：
   | dataref | 类型 | 默认 | 说明 |
   | --- | --- | --- | --- |
   | `sim/GNSS2Android/udp_port` | int | 36666 | 监听端口（改动后下一帧自动重启监听） |
   | `sim/GNSS2Android/refresh_interval_ms` | int | 50 | 电脑端数据刷新间隔（ms） |
   | `sim/GNSS2Android/enabled` | int | 1 | 1=发送数据，0=暂停 |

---

## 常见问题

- **位置卡顿**：适当调低 `refresh_interval_ms`（如 20ms），或在手机端调高注入频率。
- **手机收不到数据**：确认同一局域网、Windows 防火墙已放行 UDP 36666、X-Plane 插件已启用（插件管理器中查看）。
- **`Log.txt` 提示 dataref 未找到**：确认正在运行的是 X-Plane 12（插件读取的 dataref 均为 X-Plane 11/12 通用路径）。
- **航向显示为真航向（True）**：如需地速方向（地面航迹）代替机头方向，可将 `XP12GNSS.cpp` 中的航向读取改为 `sim/flightmodel/position/hpath`。

---

## 许可证

- 插件逻辑参照 `MSFS-SimGPStoAndroid-main`（CC BY-NC-SA 4.0），**不得用于商业用途**。
- `SDK/` 内的 X-Plane 插件 SDK 版权归 Sandy Barbour / Ben Supnik / Laminar Research，见 `SDK/license.txt`。

---

### 注释

- X-Plane 12 及其系列产品与本项目无关，本项目无关该游戏本体的分发，也没有相应的授权。请到官方游戏平台下载该游戏本体。
- 本项目代码及该README中使用了一定的人工智能（AI）。
