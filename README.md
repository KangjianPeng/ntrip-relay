# GNSS NTRIP Relay

将 GNSS 接收机实际输出的 NMEA GGA 通过 NTRIP 上报给 CORS 服务商，再把服务返回的 RTCM 数据通过串口写回接收机。节点读取 GGA、GSA、GST 和 RMC，组合成 FAST-LIVO2-RTK 使用的 GNSS PVT 话题。适用于能够输出这些 NMEA 报文并接收 RTCM 的接收机和提供兼容 NTRIP 数据流的服务；接收机型号、串口适配器、服务域名、端口和挂载点均由使用者选择。

```text
接收机 TX (GGA/GSA/GST/RMC) -> 串口适配器 RX -> 本节点
                                                +-> ROS 1 /gnss/pvt
                                                +-> NTRIP (仅 GGA)
接收机 RX (RTCM) <- 串口适配器 TX <- NTRIP
```

## 接收机与接线

- 接收机 UART TX 接串口适配器 RX，接收机 UART RX 接适配器 TX，GND 共地。
- 按接收机手册配置同一 UART 输出带校验和的 NMEA 报文，并接收服务返回的 RTCM；脚本不会发送厂商专用配置命令或更改接收机输出频率。
- 接收机与脚本波特率一致；适配器的电平必须兼容接收机 UART，不能直接混接 TTL 与 RS-232。
- 接收机需要连接 GNSS 天线，并获得真实定位。单点定位即可开始请求差分，无需预先取得 RTK FIX。
- Linux 下可用 `ls -l /dev/serial/by-id/` 查找设备，长期运行建议使用稳定的 `by-id` 或 `by-path` 路径。

### NMEA 0183 输出字段

报文需包含正确校验和；`GP`、`GN` 等 talker ID 均可。下表列出节点需要的报文和读取字段：

| 报文    | 必需程度 / 建议频率 | 节点使用的字段                                                                                                             | 用途及缺失行为                                                                                     |
| ------- | ------------------- | -------------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------- |
| `GGA` | 必需；10 Hz         | UTC 时间、纬度及 N/S、经度及 E/W、定位质量；完整 PVT 还要求海拔及单位`M`、大地水准面差及单位 `M`。卫星数和 HDOP 可选。 | 用于 NTRIP 上报位置。缺少高度或大地水准面差时仍可上报 GGA，但不发布 PVT。                          |
| `RMC` | 必需；10 Hz         | UTC 时间和日期；状态、地面速度（节）、航迹角（度）用于速度输出。                                                           | 提供 PVT 日期和速度。速度或航迹角为空时按零处理；状态不是`A` 时速度按零处理。                    |
| `GSA` | 建议；1 Hz          | 定位类型`mode_fix_type`（1/2/3）和 PDOP。首个 `A/M` 字段表示自动/手动模式，不是 2D/3D 定位类型。                       | 提供`fix_type` 和 PDOP；报文缺失时分别使用 3 和 `0.0`。近期 GSA 报告 1（无定位）时不发布 PVT。 |
| `GST` | 建议；10 Hz         | 纬度、经度和海拔误差，单位为米。                                                                                           | 纬度和经度误差合成为水平精度，海拔误差作为垂直精度；字段为空或未及时收到 GST 时相应精度为`0.0`。 |

这些报文应由同一 UART 输出；节点不会通过串口命令更改接收机配置。

Linux 下运行节点的用户必须有串口设备的读写权限。先检查设备属组和当前用户组：

```bash
ls -l /dev/ttyUSB0
id -nG
```

常见发行版将串口设备分配给 `dialout` 组，可将当前用户加入该组：

```bash
sudo usermod -aG dialout "$USER"
```

然后注销并重新登录，再启动节点。其他发行版可能使用 `uucp` 等设备属组，请以 `ls -l` 显示的属组为准。重新连接设备后权限可能由系统规则重设；不要依赖手动修改设备节点权限。

## ROS 1 安装与运行

本分支面向 ROS Noetic 和 Ubuntu 20.04（Python 3.8），需要已安装的 ROS Noetic、`catkin`、`rosdep`、可访问 CORS 服务商的网络，以及串口读写权限。`gnss_comm` 使用其 ROS 1 `main` 分支。本分支不能在 ROS 2 环境中构建；ROS 2 用户请切换到 `ros2` 分支。

从干净的 Ubuntu 20.04 开始，先配置 ROS Noetic apt 源。Noetic 已结束生命周期；首次使用 `rosdep` 时初始化一次，更新规则时包含 EOL 发行版：

```bash
sudo apt install ros-noetic-ros-base ros-noetic-rospy python3-rosdep libeigen3-dev libgoogle-glog-dev
sudo rosdep init  # 仅首次初始化时运行；已初始化则跳过
rosdep update --include-eol-distros --rosdistro noetic
source /opt/ros/noetic/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone --branch ros1 https://github.com/KangjianPeng/ntrip-relay.git
git clone --branch main https://github.com/KangjianPeng/gnss_comm.git
cd ntrip-relay
cp .env.example .env
chmod 600 .env
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src --rosdistro noetic -r -y
catkin_make
source devel/setup.bash
```

编辑 `src/ntrip-relay/.env` 填写串口和 CORS 服务商提供的连接参数、账号密码；先在一个终端运行 `roscore`，再在另一个已 source ROS Noetic 和工作空间的终端运行节点：

```bash
rosrun ntrip_gnss_relay ntrip_gnss_relay.py
```

节点默认自动读取脚本所在包目录的 `.env`，即 `src/ntrip-relay/.env`；当前工作目录不影响查找位置。需要使用其他配置文件时，再传入 `--env-file /path/to/.env`。`package.xml` 声明了 `rospy`、`gnss_comm`、`pyserial`、`pynmea2` 和 `python-dotenv` 依赖；`rosdep install` 会安装对应 ROS 与系统 Python 包。使用 ROS 自带的 Python 环境，不需要另建虚拟环境。

也可以在启动时覆盖 `.env` 中的配置：

```bash
rosrun ntrip_gnss_relay ntrip_gnss_relay.py \
  --serial /dev/ttyUSB0 \
  --baud 115200 \
  --host caster.example.com \
  --caster-port 2101 \
  --mount YOUR_MOUNTPOINT \
  --username YOUR_USERNAME \
  /gnss/pvt:=/ublox_driver/receiver_pvt
```

`caster.example.com` 和 `YOUR_MOUNTPOINT` 是占位示例，需替换为实际服务参数。未设置密码时会交互询问；非交互运行需配置 `NTRIP_USER` 和 `NTRIP_PASSWORD`。凭据使用 TCP Basic Authentication，当前不支持 TLS。
FAST-LIVO2-RTK 当前配置默认订阅 `/ublox_driver/receiver_pvt`，上面的重映射可直接连接该订阅；其他消费者可使用通用默认话题 `/gnss/pvt`。

配置优先级：命令行参数 > 已有环境变量 > `.env` > 程序默认值。服务器和挂载点没有预设值，必须配置；端口默认 `2101`，支持 `1`～`65535`。密码只通过 `.env`、环境变量或交互提示提供。

| 参数                  | 环境变量 /`.env` 键 | 默认值                 |
| --------------------- | --------------------- | ---------------------- |
| `--env-file`        | —                    | 脚本所在目录的`.env` |
| `--serial`          | `SERIAL_DEVICE`     | 必填                   |
| `--baud`            | `SERIAL_BAUD`       | `115200`             |
| `--host`            | `NTRIP_HOST`        | 必填                   |
| `--caster-port`     | `NTRIP_PORT`        | `2101`               |
| `--mount`           | `NTRIP_MOUNT`       | 必填                   |
| `--username`        | `NTRIP_USER`        | 交互询问               |
| 密码                  | `NTRIP_PASSWORD`    | 交互询问               |
| `--gga-interval`    | —                    | `5` 秒               |
| `--gga-timeout`     | —                    | `15` 秒              |
| `--connect-timeout` | —                    | `10` 秒              |
| `--rtcm-timeout`    | —                    | `15` 秒              |
| `--verbose`         | —                    | 关闭                   |

## ROS 1 话题

UTC 日期和速度取自最近的有效 RMC，GGA UTC 作为定位历元；GPS 周和周内秒按 18 秒 GPS-UTC 闰秒差换算。椭球高由 GGA 海拔（MSL）加大地水准面差得到，仅在两个高度字段均以米提供且 RMC 日期可用时发布。`fix_type` 使用 GSA 的 2D/3D 模式（缺失时按 3D 处理；近期 GSA 报告无定位时不发布）；差分与载波状态取自 GGA 定位质量，DOP 来自 GSA，水平/垂直精度来自 GST。未收到 GST 精度时 `h_acc` 和 `v_acc` 为 `0.0`；RMC 无效时速度为零。无效定位不会发布 PVT，避免下游将无效坐标用于初始化。

| 话题          | 类型 / 来源                  |       频率 |
| ------------- | ---------------------------- | ---------: |
| `/gnss/pvt` | `gnss_comm/GnssPVTSolnMsg` | 最多 10 Hz |

校验失败或格式错误的辅助 NMEA 报文会被忽略。

```bash
rostopic echo /gnss/pvt
```

节点会在启动终端输出串口、定位、PVT 发布和 CORS/NTRIP 响应状态；日志不会显示鉴权凭据。`--verbose` 会额外显示调试信息和原始 GGA。

## 定位状态与恢复

脚本严格验证 GGA 校验和；上报给服务的是接收机原始 GGA，仅统一行尾为 CRLF，不生成或补填坐标。

- quality=1 表示单点定位，quality=5 表示 RTK FLOAT，quality=4 表示 RTK FIX。
- quality=0 或坐标、UTC 不完整时，立即清除上一位置并暂停差分流；人工、模拟和推算位置不用于请求差分。
- 每 5 秒发送一次有效 GGA。15 秒没有新的有效定位历元时，断开并等待新位置；可调整 `--gga-interval` 和 `--gga-timeout`。
- 后台串口线程在网络连接和重试期间持续更新位置。串口重新打开或读取长时间暂停时，丢弃积压报文。
- 重复的 UTC 历元不会刷新位置有效期；支持 UTC 跨午夜，时间回退时清除位置。
- `--rtcm-timeout` 监测原始流字节活动。首包或后续数据超过期限时断开重连，不等同于 RTCM 校验或接收机成功解算。
- 网络故障采用 1、2、4、8、16、30 秒退避，成功转发数据后复位。串口故障会重新打开设备。
- 鉴权失败、挂载点不可用或返回源表时，以退出码 2 停止；临时服务故障和超时重试。按 `Ctrl+C` 关闭连接、串口和读取线程。

例如：

```text
$GNGGA,,,,,,0,,,,,,,,*78
```

这是校验正确的未定位报文，不能作为零坐标或虚拟位置使用。如果持续收到它，请检查天线连接、供电和天空可见性，等待接收机获得真实单点定位。完整的串口报文只说明通信可读，不代表已经定位。

## 坐标框架与数据兼容性

挂载点、坐标框架和参考历元由 CORS 服务商定义，应与地图、控制点和下游系统一致。端口编号没有通用的框架含义。RTK FIX 不能证明坐标框架正确，应使用同框架、同参考历元的已知控制点验证绝对坐标。

服务的星座、频点和 RTCM 消息必须兼容接收机。能够连接并收到数据，不保证接收机能解算或固定。网络/VRS 服务需要持续上报真实位置；距离过远的实体参考站也可能降低精度与固定可靠性。

## 常见问题

| 现象                             | 检查项                                                                                                                     |
| -------------------------------- | -------------------------------------------------------------------------------------------------------------------------- |
| 串口打不开                       | 设备路径、USB 连接、用户串口权限                                                                                           |
| 一直等待 GGA                     | TX/RX 接线、波特率、接收机 GGA 输出配置                                                                                    |
| quality=0                        | 天线连接、供电、天空可见性                                                                                                 |
| 定位历元未推进                   | 接收机 UTC 输出，是否反复发送同一报文                                                                                      |
| 401/403 或挂载点错误             | 账号有效期、密码、挂载点名称和授权                                                                                         |
| RTCM 超时                        | 网络、服务状态、位置覆盖和实际数据周期                                                                                     |
| 有差分但没有 FIX                 | 串口 RTCM 输入、支持的消息、信号质量和差分龄期                                                                             |
| `catkin_make` 找不到 `rospy` | 确认使用 ROS Noetic，先 source `/opt/ros/noetic/setup.bash`，并安装 `ros-noetic-rospy`；ROS 2 环境请构建 `ros2` 分支 |
| `GnssPVTSolnMsg` 字段 AttributeError | 检查 `rosmsg show gnss_comm/GnssPVTSolnMsg` 中是否为 `latitude`，并确认 `rospack find gnss_comm` 指向当前工作空间；重新构建并 source 该工作空间。节点启动日志会显示实际加载的脚本和消息文件路径 |

可添加 `--verbose` 查看 GGA 原始报文和调试日志。测试时保持一个程序读取串口，避免争抢数据。

## 项目结构与测试

```text
ntrip_gnss_relay.py              NMEA 接收解析、ROS 1 发布、NTRIP 转发与恢复
CMakeLists.txt / package.xml    catkin 包构建和运行依赖
.env.example               不含凭据的配置模板
pyproject.toml              Ruff 检查规则
tests/test_config.py       配置优先级、通用端口和挂载点验证
tests/test_gga.py          GGA 与定位状态验证
tests/test_recovery.py     断线恢复、协议响应、TCP/伪终端集成测试
```

```bash
python3 -m unittest discover -s tests -v
python3 -m py_compile ntrip_gnss_relay.py
ruff check ntrip_gnss_relay.py tests
ruff format --check ntrip_gnss_relay.py tests
```

测试使用隔离样例、本机 TCP 服务和模拟串口，不连接实际差分账号。TCP/伪终端集成测试仅在 POSIX 系统运行。

当前支持明文 TCP Basic 鉴权，不支持 TLS、HTTP chunked 或压缩响应体。RTCM 按原始字节转发，未做帧 CRC 校验、坐标转换或基站验证。NTRIP 使用的 GGA 不含日期，无法独立识别整日重放；PVT 日期取自最近的 RMC。
