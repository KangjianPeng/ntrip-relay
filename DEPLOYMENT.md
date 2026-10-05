# Linux 原生部署

使用项目 Python 虚拟环境和 systemd 运行串口转发服务。示例服务定义采用 `/opt/ntrip-relay` 和普通运行用户 `ntrip-relay`；使用已有项目目录和用户时，需要相应修改 `User`、`Group`、`WorkingDirectory`、`ExecStart`。

## 准备配置

在实际项目目录安装依赖，准备 `.env`：

```bash
uv sync --locked
cp .env.example .env
chmod 600 .env
```

填写 `SERIAL_DEVICE`、`SERIAL_BAUD`、`NTRIP_HOST`、`NTRIP_PORT`、`NTRIP_MOUNT`、`NTRIP_USER` 和 `NTRIP_PASSWORD`。服务器、端口、挂载点和账号以服务商提供的信息为准；串口波特率、GGA 输出和 RTCM 输入按接收机手册配置。

服务必须能读取项目、虚拟环境和 `.env`。如果 `.env` 是符号链接，目标文件及其父目录也必须允许运行用户访问。实际 `.env` 不应提交到 Git。

## 安装服务

`deploy/ntrip-relay.service` 是可修改的示例。在安装前，确认服务中的运行账户已存在、项目路径正确。若采用示例专用用户和路径，代码应已放置在 `/opt/ntrip-relay`，并完成依赖安装与配置：

```bash
sudo useradd --system --user-group --home-dir /opt/ntrip-relay --shell /usr/sbin/nologin ntrip-relay
sudo chown -R ntrip-relay:ntrip-relay /opt/ntrip-relay
```

已存在的账户无需重复创建。对于 Linux 上属于 `dialout` 组的串口，示例通过 `SupplementaryGroups=dialout` 授予服务访问权限；其他发行版请按设备实际属组调整。

在项目目录安装确认后的服务定义：

```bash
sudo install -m 644 deploy/ntrip-relay.service /etc/systemd/system/ntrip-relay.service
sudo systemctl daemon-reload
sudo systemctl enable --now ntrip-relay
systemctl status ntrip-relay --no-pager
```

服务会在启动时读取 `.env`。临时串口或网络故障由程序内部处理；异常退出由 systemd 重启。鉴权失败或挂载点错误以退出码 2 停止，修正配置后重启。

## 管理与前台运行

```bash
journalctl -u ntrip-relay -f  # 查看实时状态
journalctl -u ntrip-relay -n 50 --no-pager
sudo systemctl restart ntrip-relay  # 修改 .env 后重新加载
sudo systemctl stop ntrip-relay
sudo systemctl start ntrip-relay
```

前台测试时先停止服务，以具有串口权限的用户在项目目录运行，避免两个程序同时读取串口：

```bash
sudo systemctl stop ntrip-relay
uv run python ntrip_relay.py
# Ctrl+C 停止前台程序后恢复后台服务
sudo systemctl start ntrip-relay
```

如果当前用户没有串口权限，可使用 `sudo -u 运行用户 -g 串口属组 .venv/bin/python ntrip_relay.py`，或按系统规则为用户授予串口访问权限。

## 实际验证

确认串口适配器 RX 接接收机 TX，适配器 TX 接接收机 RX，并共地。接收机应在该 UART 输出 GGA 并接收 RTCM。把 GNSS 天线放到天空视野较好的位置，按日志检查：

1. 持续收到校验正确的 GGA。
2. 获得真实单点定位，通常为 quality=1。
3. 出现“NTRIP 已连接”和“已转发 … 字节 RTCM”。
4. 接收机进入 RTK FLOAT（quality=5）或 FIX（quality=4）。

仅收到未定位 GGA 或完成 NTRIP 握手，尚不能证明差分转发和定位正常。RTK FIX 也需要结合服务坐标框架和已知控制点验证。
