# GNSS NTRIP Relay

将 GNSS 接收机实际输出的 NMEA GGA 通过 NTRIP 上报给 CORS 服务商，再把服务返回的 RTCM 数据通过串口写回接收机。节点读取 GGA、GSA、GST 和 RMC，组合成 FAST-LIVO2-RTK 使用的 GNSS PVT 话题。适用于能够输出这些 NMEA 报文并接收 RTCM 的接收机和提供兼容 NTRIP 数据流的服务；接收机型号、串口适配器、服务域名、端口和挂载点均由使用者选择。

```text
接收机 TX (GGA/GSA/GST/RMC) -> 串口适配器 RX -> 本节点
                                                +-> ROS 2 /ublox_driver/receiver_pvt
                                                +-> NTRIP (仅 GGA)
接收机 RX (RTCM) <- 串口适配器 TX <- NTRIP
```

## 接收机与接线

- 接收机 UART TX 接串口适配器 RX，接收机 UART RX 接适配器 TX，GND 共地。
- 按接收机手册配置同一 UART 输出带校验和的 NMEA GGA（10 Hz）、GSA（1 Hz）、GST（10 Hz）和 RMC（10 Hz），并接受服务返回的 RTCM 消息。脚本不会发送厂商专用配置命令，也不会更改接收机输出频率。
- 接收机与脚本波特率一致；适配器的电平必须兼容接收机 UART，不能直接混接 TTL 与 RS-232。
- 接收机需要连接 GNSS 天线，并获得真实定位。单点定位即可开始请求差分，无需预先取得 RTK FIX。
- Linux 可用 `ls -l /dev/serial/by-id/` 查找设备，长期运行建议使用稳定的 `by-id` 或 `by-path` 路径。Windows 可使用 `COM3` 等名称。

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

## ROS 2 安装与运行

需要已安装的 ROS 2、`colcon`、`rosdep`、`uv`、可访问 CORS 服务商的网络，以及串口读写权限。

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/KangjianPeng/ntrip-relay.git
git clone --branch humble https://github.com/KangjianPeng/gnss_comm.git
cd ntrip-relay
uv venv --system-site-packages --python /usr/bin/python3
uv sync --locked
cp .env.example .env
chmod 600 .env
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select gnss_comm ntrip_gnss_relay
source install/setup.bash
source src/ntrip-relay/.venv/bin/activate
```

编辑 `src/ntrip-relay/.env`，填写串口和 CORS 服务商提供的连接参数、账号密码，然后启动节点：

```bash
ros2 run ntrip_gnss_relay ntrip_gnss_relay \
  --env-file "$HOME/ros2_ws/src/ntrip-relay/.env"
```

本项目是 `ament_cmake` ROS 2 包。CMake 安装 Python 入口为 `ntrip_gnss_relay`；虚拟环境启用系统 site-packages，以便访问 ROS 2 的 `rclpy` 和 `gnss_comm`。构建并 source 提供 `gnss_comm/msg/GnssPVTSolnMsg` 的 ROS 2 工作空间后再启动节点。Python 依赖包括 `pyserial`、`pynmea2` 和 `python-dotenv`。

默认读取脚本所在目录的 `.env`，可通过 `--env-file /path/to/.env` 指定其他配置文件。也可以在启动时覆盖配置：

```bash
ros2 run ntrip_gnss_relay ntrip_gnss_relay \
  --serial /dev/ttyUSB0 \
  --baud 115200 \
  --host caster.example.com \
  --caster-port 2101 \
  --mount YOUR_MOUNTPOINT \
  --username YOUR_USERNAME \
  --ros-args \
  -r /ublox_driver/receiver_pvt:=/robot/gnss/receiver_pvt
```

`caster.example.com` 和 `YOUR_MOUNTPOINT` 是占位示例，需替换为实际服务参数。未设置密码时会交互询问；非交互运行需配置 `NTRIP_USER` 和 `NTRIP_PASSWORD`。凭据使用 TCP Basic Authentication，当前不支持 TLS。

配置优先级：命令行参数 > 已有环境变量 > `.env` > 程序默认值。服务器和挂载点没有预设值，必须配置；端口默认 `2101`，支持 `1`～`65535`，不约定端口与坐标框架的关系。密码只通过 `.env`、环境变量或交互提示提供。

| 参数 | 环境变量 / `.env` 键 | 默认值 |
|---|---|---|
| `--env-file` | — | 脚本所在目录的 `.env` |
| `--serial` | `SERIAL_DEVICE` | 必填 |
| `--baud` | `SERIAL_BAUD` | `115200` |
| `--host` | `NTRIP_HOST` | 必填 |
| `--caster-port` | `NTRIP_PORT` | `2101` |
| `--mount` | `NTRIP_MOUNT` | 必填 |
| `--username` | `NTRIP_USER` | 交互询问 |
| 密码 | `NTRIP_PASSWORD` | 交互询问 |
| `--gga-interval` | — | `5` 秒 |
| `--gga-timeout` | — | `15` 秒 |
| `--connect-timeout` | — | `10` 秒 |
| `--rtcm-timeout` | — | `15` 秒 |
| `--verbose` | — | 关闭 |

## ROS 2 话题

节点校验 GGA、GSA、GST 和 RMC，并将有效定位转为 `gnss_comm/msg/GnssPVTSolnMsg`，默认发布到 `/ublox_driver/receiver_pvt`，可通过 ROS 参数重映射。话题采用可靠 QoS，与 FAST-LIVO2-RTK 默认订阅配置匹配。UTC 日期和速度取自最近的有效 RMC，GGA UTC 作为定位历元；GPS 周和周内秒按 18 秒 GPS-UTC 闰秒差换算。椭球高由 GGA 海拔（MSL）加大地水准面差得到，因此仅在这两个字段均以米提供且 RMC 日期可用时发布。`fix_type` 使用 GSA 的 2D/3D 模式，差分与载波状态使用 GGA 定位质量，DOP 来自 GSA，水平/垂直精度来自 GST。未收到 GST 精度时 `h_acc` 和 `v_acc` 为 `0.0`；RMC 无效时速度为零。无效定位不会发布 PVT，避免下游将无效坐标用于初始化。

| 话题 | 类型 / 来源 | 频率 |
|---|---|---:|
| `/ublox_driver/receiver_pvt` | `gnss_comm/msg/GnssPVTSolnMsg` | 最多 10 Hz |

无效、无定位或不完整的 GGA 不会发布 PVT；校验失败或格式错误的辅助 NMEA 报文会被忽略。

```bash
ros2 topic echo /ublox_driver/receiver_pvt gnss_comm/msg/GnssPVTSolnMsg
```

## 定位状态与恢复

脚本严格验证 GGA 校验和，解析 UTC、经纬度、定位质量、卫星数、HDOP 和海拔。日志中的经纬度为十进制度，上报给服务的仍是接收机原始 GGA，仅统一行尾为 CRLF。不会生成或补填坐标。

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

| 现象 | 检查项 |
|---|---|
| 串口打不开 | 设备路径、USB 连接、用户串口权限 |
| 一直等待 GGA | TX/RX 接线、波特率、接收机 GGA 输出配置 |
| quality=0 | 天线连接、供电、天空可见性 |
| 定位历元未推进 | 接收机 UTC 输出，是否反复发送同一报文 |
| 401/403 或挂载点错误 | 账号有效期、密码、挂载点名称和授权 |
| RTCM 超时 | 网络、服务状态、位置覆盖和实际数据周期 |
| 有差分但没有 FIX | 串口 RTCM 输入、支持的消息、信号质量和差分龄期 |

可添加 `--verbose` 查看 GGA 原始报文和调试日志。测试时保持一个程序读取串口，避免争抢数据。

## 项目结构与测试

```text
ntrip_gnss_relay.py              NMEA 接收解析、ROS 2 发布、NTRIP 转发与恢复
CMakeLists.txt / package.xml    ROS 2 包构建和运行依赖
.env.example               不含凭据的配置模板
pyproject.toml / uv.lock    Python 依赖和锁定版本
tests/test_config.py       配置优先级、通用端口和挂载点验证
tests/test_gga.py          GGA 与定位状态验证
tests/test_recovery.py     断线恢复、协议响应、TCP/伪终端集成测试
```

```bash
uv sync --locked
uv run python -m unittest discover -s tests -v
uv run python -m py_compile ntrip_gnss_relay.py
uv lock --check
uvx ruff check ntrip_gnss_relay.py tests
uvx ruff format --check ntrip_gnss_relay.py tests
```

测试使用隔离样例、本机 TCP 服务和模拟串口，不连接实际差分账号。TCP/伪终端集成测试仅在 POSIX 系统运行。

当前支持明文 TCP Basic 鉴权，不支持 TLS、HTTP chunked 或压缩响应体。RTCM 按原始字节转发，未做帧 CRC 校验、坐标转换或基站验证。NTRIP 使用的 GGA 不含日期，无法独立识别整日重放；PVT 日期取自最近的 RMC。
