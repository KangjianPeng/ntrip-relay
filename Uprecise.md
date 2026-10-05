# UM960 从站接收机设置速查

脚本需要 UM960 在与 CH340 相连的同一个 UART 上输出带校验和的 GGA，同时从该 UART 接收 RTCM 3.x。下面以 COM2 为例，具体命令和波特率以当前 UM960 固件手册为准：

```text
UNLOGALL COM2
GPGGA COM2 1
SAVECONFIG
```

检查要点：

1. CH340 TXD 接 UM960 RX，CH340 RXD 接 UM960 TX，GND 共地。
2. 逻辑电平必须兼容；CH340 TTL 不能直接接 RS-232 电平。
3. 脚本 `--baud` 与 COM2 波特率一致。
4. 能看到 `$GNGGA,...*hh` 或 `$GPGGA,...*hh`，带有效 UTC 和经纬度，且 `quality` 为 1～5 时，脚本才会向 NTRIP 上报位置。
5. `quality=0` 且经纬度为空表示尚未定位，不应使用虚拟 GGA。

`UNLOGALL COM2` 会停止 COM2 已配置的输出，请确认其他应用是否依赖这些输出，再按需要执行。脚本不会自动发送接收机配置命令。项目安装和运行方法见 [README.md](README.md)。
