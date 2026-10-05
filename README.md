# UM960 NTRIP RTCM Relay

这个项目把 UM960 的 NMEA GGA 位置通过 NTRIP 上报给苍穹北斗服务，再将服务返回的 RTCM 3.x 差分数据通过 CH340 USB-TTL 写回 UM960。它适用于没有蜂窝网络 NTRIP 客户端的接收机：运行脚本的计算机负责网络连接，UM960 负责 GNSS/RTK 解算。

数据路径如下：

```text
UM960 TX (GGA) -> CH340 RX -> Python/NTRIP -> CH340 TX (RTCM) -> UM960 RX
```

脚本只使用接收机实际输出的 GGA；不会生成测试坐标，也不会修改 RTCM 二进制内容。

## 接线与接收机设置

- CH340 TXD 接 UM960 UART RX，CH340 RXD 接 UM960 UART TX，GND 与 GND 相连。
- 确认 USB-TTL 的逻辑电平与 UM960 UART 电平兼容；不要把 RS-232 电平接到 TTL UART，也不要在未确认前连接 VCC。
- 在 UM960 中将该 UART 配置为输出 NMEA GGA，并允许该 UART 输入 RTCM 3.x。接收机与脚本的波特率必须一致。
- 如果使用 COM2，可在 UM960 命令行中按设备手册配置：

  ```text
  UNLOGALL COM2
  GPGGA COM2 1
  SAVECONFIG
  ```

  `GPGGA`、串口号和波特率必须与实际端口一致；如果设备输出的是 `GNGGA`，脚本同样支持。确认该端口同时允许接收 RTCM 输入。
- Linux 下可用 `ls /dev/serial/by-id/` 查找 CH340 设备，常见设备名为 `/dev/ttyUSB0`。
- 长时间运行优先使用 `/dev/serial/by-id/` 下的设备路径，避免拔插后 `ttyUSB` 编号变化；若 CH340 没有唯一 ID，可使用合适的 `by-path` 路径。

## 安装和运行

需要 Python 3.10 或更高版本、已安装的 `uv`、可访问 NTRIP 服务的网络，以及具有串口读写权限的运行用户。Linux 上通常使用 `/dev/ttyUSB*`，Windows 可使用 `COM3` 等名称。接收机需要连接 GNSS 天线并获得单点定位。

```bash
git clone https://github.com/KangjianPeng/ntrip-relay.git
cd ntrip-relay
uv sync --locked
uv run python ntrip_um960_relay.py \
  --serial /dev/ttyUSB0 \
  --baud 115200 \
  --caster-port 8002 \
  --mount RTCM33_GRCEJ \
  --username '你的卡号'
```

`uv sync` 会根据 `pyproject.toml` 和 `uv.lock` 创建项目环境并安装 `pyserial`（串口通信）和 `pynmea2`（NMEA 解析）。

运行时按提示输入密码。默认连接 `rtkcq.cn`；端口 `8001` 为 ITRF2008，`8002` 为 WGS84（参考历元 2005），`8003` 为 CGCS2000（参考历元 2000）。挂载点可选 `RTCM33_GRC` 或 `RTCM33_GRCEJ`。

非交互运行也可以通过环境变量提供凭据；交互使用建议按提示输入密码，避免把密码写入 shell 历史：

```bash
export NTRIP_USER='你的卡号'
export NTRIP_PASSWORD='你的密码'
uv run python ntrip_um960_relay.py --serial /dev/serial/by-id/你的CH340 --baud 115200
```

密码会以 Basic Authentication 发送到服务端；当前服务端口是明文 TCP，不要在不可信网络中暴露凭据。

脚本只使用 UM960 实际输出的 GGA，严格验证接收机报文的校验和，并解析 UTC、经纬度、定位质量、卫星数、HDOP 和海拔。日志中的经纬度转换为十进制度；发给服务器的仍是接收机原始 GGA，仅统一行尾为 CRLF。脚本不会生成或补填坐标，已移除固定位置的 `--gga` 参数。

脚本会每 5 秒发送一次 UM960 的有效 GGA。若接收机报告未定位，立即清除上一位置并暂停差分流；若 15 秒没有新的有效定位历元，也会断开并等待新位置。可用 `--gga-interval` 和 `--gga-timeout` 调整这两个时间。单点定位（quality=1）即可启动，不需要预先获得 RTK FIX；人工、模拟和推算位置不用于请求差分流。

## 断线恢复与位置时效

- 后台串口线程持续读取 GGA，网络连接或重试期间仍更新最新位置。串口首次打开、重新打开或读取曾暂停超过 GGA 有效期时，清除积压数据并等待新报文。
- 有效位置必须带 UTC。重复报送同一个 UTC 历元不会刷新位置有效期；UTC 跨午夜可正常处理，时间回退则清除位置并等待下一历元。GGA 不含日期，无法单靠它检测整日重放，也无法证明服务端坐标框架是否正确。
- `--rtcm-timeout` 默认 15 秒：在此时间内未收到首包，或后续数据持续中断，都会断开 NTRIP 并重连。此处监测原始流字节活动，不等同于 RTCM CRC 校验或接收机成功解算。
- 网络异常采用 1、2、4、8、16、30 秒退避；首次成功收到并转发数据后复位。网络重试保持串口读取线程运行。
- 串口读写失败时关闭旧对象，再尝试重新打开设备，并创建新的 GGA 读取器。启动时设备不存在也会等待重试。配置的设备路径必须在拔插后仍指向同一个 CH340。
- `401/403`、错误挂载点或返回源表时，输出明确错误并以退出码 2 停止。服务器 `5xx` 和网络超时按临时故障重试。按 `Ctrl+C` 关闭线程、网络连接和串口。

常用参数：

| 参数 | 默认值 | 作用 |
|---|---:|---|
| `--serial` | 必填 | CH340 设备路径 |
| `--baud` | `115200` | UM960 与 CH340 的串口波特率 |
| `--host` | `rtkcq.cn` | NTRIP 服务域名或 IP |
| `--caster-port` | `8002` | `8001` ITRF2008、`8002` WGS84、`8003` CGCS2000 |
| `--mount` | `RTCM33_GRCEJ` | RTCM 挂载点 |
| `--username` | 环境变量或提示输入 | NTRIP 卡号，可用 `NTRIP_USER` 提供 |
| `--gga-interval` | `5` 秒 | 发送 GGA 的周期 |
| `--gga-timeout` | `15` 秒 | GGA 定位历元的最大年龄 |
| `--connect-timeout` | `10` 秒 | 网络连接和响应头等待时间 |
| `--rtcm-timeout` | `15` 秒 | RTCM 首包/后续数据最大等待时间 |
| `--verbose` | 关闭 | 输出原始 GGA 和调试日志 |

## 坐标框架与挂载点

坐标框架应与地图、控制点和下游系统一致。选错 WGS84/CGCS2000/ITRF 端口时，UM960 仍可能得到 RTK 固定解，但绝对坐标会跟随基准站的框架和参考历元，与目标数据存在系统偏差。FIX 不证明坐标框架正确；NMEA 格式或接收机输出标签也不能自动把差分结果转换为目标框架。切换端口可能造成位置跳变，应在同框架、同参考历元的已知控制点上确认。

挂载点不存在或没有访问权限时无法获得流。挂载点存在但观测星座、频点或消息类型不适配接收机时，可能只有部分消息可用，表现为固定较慢、FLOAT 或无法 RTK；较少星座的挂载点也可能正常固定。选择距离很远的实体参考站时，公共卫星和大气误差相关性变差，精度与固定可靠性可能下降；网络/VRS 服务需持续上报真实 GGA。能够连接和收到 RTCM 不保证基站坐标或最终定位正确。

## 原始 GGA 与未定位状态

```text
$GNGGA,,,,,,0,,,,,,,,*78
```

这是校验和正确的 GGA，但 `quality=0` 且经纬度为空，表示当前未定位。卫星数、HDOP、海拔和 UTC 同样未提供，不能当成零坐标。脚本会解析出未定位状态并等待有效坐标，不会虚构位置来请求 RTCM。

如果一直收到这条报文，请检查 GNSS 天线连接、天线供电和天空可见性，并在室外等待接收机获得单点定位。持续收到完整且校验正确的 GGA 说明串口数据可读取，但不代表天线或定位已正常。添加 `--verbose` 可查看原始 GGA 日志。

## 常见问题

| 现象 | 检查方法 |
|---|---|
| 串口不存在或一直重开 | 检查 USB 连接、设备路径；优先使用稳定的 `by-id` 路径 |
| Permission denied | 用 `ls -l` 和 `id` 检查串口属组与用户权限；不要同时运行其他占用串口的软件 |
| 一直等待 GGA | 检查 TX/RX 交叉接线、波特率、UART 的 GGA 输出配置；用 `--verbose` 查看报文 |
| GGA 质量为 0 | 检查天线连接、供电和天空可见性，在室外等待单点定位 |
| GGA 定位历元未推进 | 检查接收机时间输出，避免重放文件或反复发送同一报文 |
| 401/403、Bad MountPoint | 检查账号有效期、密码、挂载点和账号授权；修正后重新运行 |
| RTCM 超时 | 检查网络和服务状态、挂载点是否覆盖当前位置；根据实际数据周期调整超时 |
| 有数据但没有 FIX | 检查串口 RTCM 输入、卫星信号、差分龄期和 UM960 支持的消息/频点 |

查看串口和参数：

```bash
ls -l /dev/serial/by-id/
uv run python ntrip_um960_relay.py --help
```

## 项目结构与测试

```text
ntrip_um960_relay.py   GGA 解析、NTRIP 连接、串口转发与恢复逻辑
pyproject.toml        Python 版本和运行依赖
uv.lock               锁定依赖版本
tests/test_gga.py     定位报文与有效位置转发测试
tests/test_recovery.py 断线恢复、协议响应、TCP/伪终端集成测试
Uprecise.md           UM960 接收机配置速查
```

运行验证：

```bash
uv sync --locked
uv run python -m unittest discover -s tests -v
uv run python -m py_compile ntrip_um960_relay.py
uv lock --check
uvx ruff check ntrip_um960_relay.py tests
uvx ruff format --check ntrip_um960_relay.py tests
```

测试使用隔离的样例 GGA、本机 TCP 服务和模拟串口；不会连接真实 CORS 账号。TCP/伪终端集成测试仅在 POSIX 系统运行，Windows 会跳过该项。

## 当前限制

- 服务商提供的三个端口和两个挂载点是本项目默认配置，框架/历元信息以服务商确认结果为准。
- 当前使用明文 TCP Basic 鉴权，不支持 TLS、HTTP chunked 或压缩响应体。
- RTCM 按原始字节转发，未进行帧 CRC 校验、坐标转换或基站真实性验证。日志中的转发字节数不证明接收机已成功解算。
- GGA 不含日期，无法独立识别整日重放或所有陈旧定位情况。

串口号、波特率、接收机 NMEA 输出和权限因设备与系统而异；需要在实际 UM960/CH340 链路上确认 RTK 状态进入 FLOAT/FIX，并使用同框架、同参考历元的控制点检查绝对坐标。
