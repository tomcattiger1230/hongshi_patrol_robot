# 海康摄像头公网视频中继

数据链路：

```text
海康摄像头 --RTSP/LAN--> 机器人 NUC --RTSP/TCP--> 公网服务器 --RTSP/TCP--> 本地电脑
```

随车 NUC 主动向公网服务器推流，所以机器人侧无需公网 IP、DDNS 或路由器端口映射。默认使用 FFmpeg `copy` 模式，不解码和重新编码视频，适合 Intel NUC，CPU 占用较低。公网服务器使用 MediaMTX，仅开放 `8554/tcp`。

## 一、准备公网服务器

要求：Linux 公网服务器、Docker Engine、Docker Compose 插件，以及安全组/防火墙放行 `8554/tcp`。

```bash
cd video_relay/server
cp .env.example .env
# 编辑 .env，务必替换两个密码
docker compose up -d
docker compose logs -f
```

不要把 `.env` 提交到 Git。

### 账号含义与配置示例

公网服务器使用两组相互独立的账号：

| 配置项 | 使用位置 | 用途 |
| --- | --- | --- |
| `PUBLISH_USER` | 公网服务器、机器人 NUC | 推流用户名，允许 NUC 向 `/robot` 发送视频 |
| `PUBLISH_PASSWORD` | 公网服务器、机器人 NUC | 推流密码，两端必须完全一致 |
| `READ_USER` | 公网服务器、本地电脑 | 只读用户名，允许本地电脑观看 `/robot` |
| `READ_PASSWORD` | 公网服务器、本地电脑 | 只读密码，两端必须完全一致 |

公网服务器 `server/.env` 配置全部四项：

```dotenv
PUBLISH_USER=robot_publisher
PUBLISH_PASSWORD=replace-with-a-strong-publish-password
READ_USER=local_viewer
READ_PASSWORD=replace-with-a-strong-read-password
```

机器人 NUC 的 `robot/.env` 使用服务器上的推流账号：

```dotenv
PUBLISH_USER=robot_publisher
PUBLISH_PASSWORD=replace-with-a-strong-publish-password
```

本地电脑的 `local/.env` 使用服务器上的只读账号：

```dotenv
READ_USER=local_viewer
READ_PASSWORD=replace-with-a-strong-read-password
```

两组账号应使用不同的强密码。只读账号即使泄露，也不能冒充机器人覆盖服务器上的视频流。

### 服务器端口

当前配置只需要开放入站端口 `8554/tcp`，机器人 NUC 和本地电脑都主动连接此端口，不需要开放 RTSP UDP 端口。

若服务器启用了 UFW，可执行：

```bash
sudo ufw allow 8554/tcp
sudo ufw status
```

云服务器还需要在云厂商控制台的安全组中放行相同端口。可以使用以下命令检查监听状态：

```bash
sudo ss -lntp | grep 8554
docker compose ps
```

服务器拉取 Docker 镜像需要正常的出站 HTTPS 网络，但不需要为此开放入站 `443`。如果以后启用 RTSPS，通常还需要开放 `8322/tcp`；当前配置没有启用该端口。服务器地址记为 `SERVER_PUBLIC_IP`。

## 二、配置海康摄像头

确保机器人能在局域网内访问摄像头。常见地址：

- 主码流：`rtsp://CAMERA_IP:554/Streaming/Channels/101`
- 子码流：`rtsp://CAMERA_IP:554/Streaming/Channels/102`

先在机器人上验证：

```bash
ffprobe -rtsp_transport tcp 'rtsp://admin:摄像头密码@CAMERA_IP:554/Streaming/Channels/101'
```

建议在海康管理页面中把视频设为 H.264。H.265 更省带宽，但部分本地播放器兼容性较差。

## 三、机器人 NUC 端推流

Ubuntu/Debian 安装依赖：

```bash
sudo apt update
sudo apt install -y ffmpeg python3
cd video_relay/robot
cp .env.example .env
# 编辑 .env：摄像头、服务器和推流账号必须与 server/.env 对应
python3 push_stream.py
```

程序会在摄像头断线、网络切换或服务器重启后自动重连。日志中的 URL 会隐藏密码。

长期运行时，将仓库放在 NUC 的 `~/Develop/github_ws/hongshi_patrol_robot`。服务是模板形式，下面的 `$USER` 会自动使用当前 NUC 登录用户：

```bash
cd ~/Develop/github_ws/hongshi_patrol_robot/video_relay/robot
chmod 600 .env
sudo cp hikvision-video-relay@.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now "hikvision-video-relay@$USER"
sudo journalctl -u "hikvision-video-relay@$USER" -f
```

若 `copy` 模式无法在本地播放，将 NUC `.env` 中的 `VIDEO_MODE` 改成 `transcode`，程序会转成低延迟 H.264。默认 `VIDEO_ENCODER=libx264` 使用 CPU；Intel NUC 可先运行 `ffmpeg -encoders | grep h264_qsv` 检查支持情况，存在该编码器时改成 `VIDEO_ENCODER=h264_qsv`，使用 Intel Quick Sync 降低 CPU 压力。修改后执行 `sudo systemctl restart "hikvision-video-relay@$USER"`。

## 四、本地电脑查看和录制

### 命令行快速验证

本地电脑安装 FFmpeg，然后先用 `probe` 确认是否收到视频数据，再进行播放或录制：

```bash
cd video_relay/local
cp .env.example .env
# 编辑 .env：填写公网服务器和 server/.env 中的只读账号
python3 client.py probe
python3 client.py play
python3 client.py record
```

`probe` 成功时会输出 `Video: h264` 或 `Video: hevc`、分辨率和帧率；`play` 会打开低延迟播放窗口；`record` 会把原始码流无损保存成带时间戳的 MKV 文件。

也可直接用 VLC 打开：

```text
rtsp://local_viewer:只读密码@SERVER_PUBLIC_IP:8554/robot
```

### Python Web 查看器

项目提供了一个简易 Web 页面。视频仍通过 RTSP 从公网服务器获取，由本地 Python/OpenCV 解码成 MJPEG，因此公网服务器不需要增加端口：

```bash
cd video_relay/local
python3 -m venv .venv
source .venv/bin/activate       # Windows PowerShell 使用：.venv\Scripts\Activate.ps1
python3 -m pip install -r requirements.txt
cp .env.example .env           # 如果之前已配置则不要覆盖
# 编辑 .env，填写 SERVER_HOST、READ_USER 和 READ_PASSWORD
python3 web_viewer.py
```

浏览器打开 <http://127.0.0.1:8081>。页面会显示连接状态、画面分辨率、接收帧率和最新帧时间，同时提供以下数据接口：

- `/video`：浏览器可直接显示的 MJPEG 视频流。
- `/snapshot.jpg`：获取当前帧 JPEG 图片。
- `/api/status`：获取连接状态、分辨率、FPS 和累计帧数 JSON。

Web 服务默认只监听 `127.0.0.1`，其他电脑无法访问。如果确实需要局域网访问，可在 `local/.env` 中设置 `WEB_HOST=0.0.0.0`，但该页面没有登录认证，不应直接暴露到公网。

公网链路统一强制 RTSP over TCP，防火墙配置简单，并避免 UDP 在 NAT 环境下丢包。端到端带宽约等于摄像头码率；例如 2.5 Mbit/s 连续运行约产生 810 GB/月的服务器入站流量，每增加一个观看端还会产生约 810 GB/月出站流量。

## 安全和故障排查

当前基线使用 RTSP Basic 认证，账号有读写权限隔离，但普通 RTSP **不加密视频和密码**。生产环境建议让机器人和本地电脑通过 WireGuard/Tailscale 接入服务器私网，并只允许 VPN 网段访问 8554；或进一步配置 MediaMTX 的 RTSPS 和可信 TLS 证书。

- 服务器日志出现 `authentication failed`：检查三处 `.env` 的账号密码是否完全对应。
- 机器人无法连接服务器：检查服务器安全组、系统防火墙和 `8554/tcp`。
- 机器人无法读取摄像头：检查摄像头 IP、密码、RTSP 功能和 `/101`、`/102` 通道。
- 延迟持续增长：先改用子码流 `/102`，降低海康码率并缩短关键帧间隔。
- 快速自检：在仓库根目录执行 `python3 -m unittest discover -s video_relay/tests -v`。
