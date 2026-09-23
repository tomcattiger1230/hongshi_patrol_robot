"""Read-only standalone DDS LiDAR/map preview; no vehicle command writers."""
from collections import deque
import base64
import json
import math
import os
import socket
import tempfile
import sys
import time
import uuid
import zlib
from pathlib import Path

from PySide6.QtCore import QPointF, QProcess, QRectF, Qt, QTimer, Signal
from PySide6.QtGui import QColor, QImage, QPainter, QPen, QPolygonF
from PySide6.QtWidgets import QFileDialog, QHBoxLayout, QLabel, QMessageBox, QPushButton, QVBoxLayout, QWidget

from robot320_interfaces.fastdds_transport import FastDdsParticipant
from .map_model import MapGeometry, MapSnapshot, load_map_yaml, save_map_yaml


WAN_MAX_LINE_BYTES = 8 * 1024 * 1024


def available_loopback_port():
    with socket.socket() as candidate:
        candidate.bind(('127.0.0.1', 0))
        return candidate.getsockname()[1]


def parse_spatial(kind, payload):
    data = json.loads(payload)
    if not isinstance(data, dict) or not isinstance(data.get('frame'), str) or not data['frame']:
        raise ValueError('missing coordinate frame')
    if kind == 'scan':
        points = data['points']
        if len(points) > 1200 or any(len(p) != 2 or not all(math.isfinite(v) for v in p) for p in points):
            raise ValueError('invalid scan')
    elif kind == 'pose':
        if len(data['pose']) != 3 or not all(math.isfinite(v) for v in data['pose']):
            raise ValueError('invalid pose')
    elif kind == 'map':
        geom = MapGeometry(data['width'], data['height'], data['resolution'], *data['origin'])
        count = geom.width * geom.height
        if count > 1_000_000 or not all(math.isfinite(v) for v in [geom.resolution, *data['origin']]):
            raise ValueError('invalid map geometry')
        inflater = zlib.decompressobj()
        cells = inflater.decompress(base64.b64decode(data['cells'], validate=True), count + 1)
        if len(cells) != count or not inflater.eof or any(v > 101 for v in cells):
            raise ValueError('invalid map cells')
        data['geometry'], data['decoded_cells'] = geom, cells
    else:
        raise ValueError('unknown spatial payload')
    return data


class SpatialCanvas(QWidget):
    initial_pose_selected = Signal(float, float, float)

    def __init__(self):
        super().__init__()
        self.setMinimumSize(260, 240)
        self.scan = None
        self.pose = None
        self.map = None
        self.trajectory = deque(maxlen=3000)
        self.trajectory_frame = ''
        self.selected_initial_pose = None
        self._selection_start = None
        self._view = None
        self.show_full_map = False

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor('#111c2c'))
        frame = self.map['frame'] if self.map else (self.scan['frame'] if self.scan else '')
        if not frame:
            painter.setPen(QColor('#a8bad0'))
            painter.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter, '等待真实雷达数据（未显示模拟点云）')
            return
        matching_map = self.map and self.map['frame'] == frame
        g = self.map['geometry'] if matching_map else None
        following_pose = (
            matching_map and not self.show_full_map and self.pose
            and self.pose['frame'] == frame
        )
        if following_pose:
            cx, cy = self.pose['pose'][:2]
            xmin, xmax, ymin, ymax = cx-12, cx+12, cy-12, cy+12
        elif matching_map:
            corners = [g.grid_to_world(x, y) for x, y in [(0, 0), (g.width, 0), (0, g.height), (g.width, g.height)]]
            xmin, xmax = min(p[0] for p in corners), max(p[0] for p in corners)
            ymin, ymax = min(p[1] for p in corners), max(p[1] for p in corners)
        else:
            center = self.pose['pose'][:2] if self.pose and self.pose['frame'] == frame else (0, 0)
            xmin, xmax, ymin, ymax = center[0]-15, center[0]+15, center[1]-15, center[1]+15
        scale = min((self.width()-32)/max(xmax-xmin, .01), (self.height()-32)/max(ymax-ymin, .01))
        cx, cy = (xmin+xmax)/2, (ymin+ymax)/2
        self._view = (scale, cx, cy)
        painter.translate(self.width()/2, self.height()/2)
        painter.scale(scale, -scale)
        painter.translate(-cx, -cy)
        if matching_map:
            painter.save()
            painter.translate(g.origin_x, g.origin_y)
            painter.rotate(math.degrees(g.origin_yaw))
            painter.drawImage(QRectF(0, 0, g.width*g.resolution, g.height*g.resolution), self.map['image'])
            painter.restore()
        else:
            painter.setPen(QPen(QColor('#23344a'), 0))
            for i in range(math.floor(xmin), math.ceil(xmax)+1):
                painter.drawLine(QPointF(i, ymin), QPointF(i, ymax))
            for i in range(math.floor(ymin), math.ceil(ymax)+1):
                painter.drawLine(QPointF(xmin, i), QPointF(xmax, i))
        if self.trajectory_frame == frame:
            painter.setPen(QPen(QColor('#4bd6a4'), 2/scale))
            painter.drawPolyline(QPolygonF([QPointF(x, y) for x, y in self.trajectory]))
        if self.scan and self.scan['frame'] == frame:
            scan_points = QPolygonF([QPointF(x, y) for x, y in self.scan['points']])
            halo = QPen(QColor('#142033'))
            halo.setWidthF(7.0)
            halo.setCosmetic(True)
            painter.setPen(halo)
            painter.drawPoints(scan_points)
            foreground = QPen(QColor('#ffb000'))
            foreground.setWidthF(4.0)
            foreground.setCosmetic(True)
            painter.setPen(foreground)
            painter.drawPoints(scan_points)
        if self.pose and self.pose['frame'] == frame:
            x, y, yaw = self.pose['pose']
            painter.setPen(QPen(QColor('#60a5fa'), 3/scale))
            painter.drawEllipse(QPointF(x, y), .3, .3)
            painter.drawLine(QPointF(x, y), QPointF(x+math.cos(yaw), y+math.sin(yaw)))
        if self.selected_initial_pose is not None:
            x, y, yaw = self.selected_initial_pose
            painter.setPen(QPen(QColor('#e879f9'), 4/scale))
            painter.drawEllipse(QPointF(x, y), .35, .35)
            painter.drawLine(QPointF(x, y), QPointF(x+1.2*math.cos(yaw), y+1.2*math.sin(yaw)))

    def screen_to_world(self, point):
        if self._view is None:
            return None
        scale, cx, cy = self._view
        return ((point.x() - self.width()/2) / scale + cx,
                -(point.y() - self.height()/2) / scale + cy)

    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton and self.map is not None:
            self._selection_start = self.screen_to_world(event.position())
            event.accept()
            return
        super().mousePressEvent(event)

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton and self._selection_start is not None:
            end = self.screen_to_world(event.position())
            start, self._selection_start = self._selection_start, None
            if end is None:
                return
            dx, dy = end[0]-start[0], end[1]-start[1]
            fallback = self.pose['pose'][2] if self.pose else 0.0
            yaw = math.atan2(dy, dx) if math.hypot(dx, dy) > .15 else fallback
            self.selected_initial_pose = (start[0], start[1], yaw)
            self.initial_pose_selected.emit(*self.selected_initial_pose)
            self.update()
            event.accept()
            return
        super().mouseReleaseEvent(event)


class SpatialPanel(QWidget):
    received = Signal(str, object)

    def __init__(self, domain_id=20, auto_connect=True, transport=None):
        super().__init__()
        self.domain_id = domain_id
        default_transport = 'wan' if sys.platform in ('darwin', 'win32') else 'dds'
        self.transport = transport or os.getenv('ROBOT_SPATIAL_TRANSPORT', default_transport)
        if self.transport not in ('wan', 'dds', 'local'):
            raise ValueError('ROBOT_SPATIAL_TRANSPORT must be wan, dds or local')
        self.participant = None
        self.last_scan = 0.0
        self.scan_sequence = 0
        self.mapping_writer = None
        self.pending_mapping_request = None
        self.last_mapping_status_stamp = 0.0
        self.map_upload = None
        self.map_upload_directory = None
        self.auto_relocalize = False
        self.data_source = {
            'wan': '公网 SSH', 'dds': '只读 DDS', 'local': 'NUC 本机 DDS',
        }[self.transport]
        self.cloud_host = os.getenv('ROBOT_CLOUD_SSH_HOST', 'hsjc_ecs')
        configured_port = os.getenv('ROBOT_SPATIAL_WAN_SSH_PORT', '').strip()
        self.local_port = int(configured_port) if configured_port else available_loopback_port()
        self.cloud_reverse_port = int(os.getenv('ROBOT_CLOUD_REVERSE_PORT', '12220'))
        self.host_key_alias = os.getenv('ROBOT_SSH_HOST_KEY_ALIAS', '192.168.88.108')
        self.tunnel = QProcess(self)
        self.stream = QProcess(self)
        self._stdout_buffer = b''
        self._shutting_down = False
        layout = QVBoxLayout(self)
        row = QHBoxLayout()
        self.connect_button = QPushButton(
            '连接远程空间数据' if self.transport == 'wan'
            else ('连接 NUC 本机空间数据' if self.transport == 'local' else '连接只读 DDS 数据')
        )
        self.connect_button.clicked.connect(self.connect_data)
        row.addWidget(self.connect_button)
        self.status = QLabel('未连接 · 不发送车辆指令')
        self.status.setWordWrap(True)
        row.addWidget(self.status, 1)
        layout.addLayout(row)
        tools = QHBoxLayout()
        self.start_mapping_button = QPushButton('开始雷达建图')
        self.stop_mapping_button = QPushButton('停止建图')
        self.start_mapping_button.setEnabled(False)
        self.stop_mapping_button.setEnabled(False)
        self.start_mapping_button.clicked.connect(lambda: self.mapping_control('start'))
        self.stop_mapping_button.clicked.connect(lambda: self.mapping_control('stop'))
        self.save_button = QPushButton('保存地图到本机…')
        self.save_button.setEnabled(False)
        self.save_button.clicked.connect(self.save_local_map)
        self.load_button = QPushButton('载入地图并自动定位…')
        self.load_button.clicked.connect(self.load_local_map)
        self.initial_pose_button = QPushButton('发送手动初始位置')
        self.initial_pose_button.setEnabled(False)
        self.initial_pose_button.clicked.connect(self.send_initial_pose)
        for button in (self.start_mapping_button, self.stop_mapping_button, self.save_button,
                       self.load_button, self.initial_pose_button):
            tools.addWidget(button)
        tools.addStretch()
        layout.addLayout(tools)
        self.map_operation = QLabel('保存位置为运行 GUI 的本机；保存栅格地图，不包含 Cartographer pose graph。')
        self.map_operation.setWordWrap(True)
        layout.addWidget(self.map_operation)
        view_tools = QHBoxLayout()
        self.view_button = QPushButton('显示完整地图')
        self.view_button.setCheckable(True)
        self.view_button.toggled.connect(self._toggle_map_view)
        view_tools.addWidget(self.view_button)
        self.view_hint = QLabel('当前：局部跟随机器人，便于观察实时点云')
        view_tools.addWidget(self.view_hint, 1)
        layout.addLayout(view_tools)
        self.canvas = SpatialCanvas()
        self.canvas.initial_pose_selected.connect(self._initial_pose_selected)
        layout.addWidget(self.canvas, 1)
        self.details = QLabel('地图：等待 /map · 轨迹：等待真实 map/odom → base_link TF')
        self.details.setWordWrap(True)
        layout.addWidget(self.details)
        self.received.connect(self.update_data)
        self.tunnel.started.connect(lambda: QTimer.singleShot(500, self._start_wan_stream))
        self.tunnel.errorOccurred.connect(lambda _error: self._wan_failed('公网隧道启动失败，正在重连……'))
        self.tunnel.finished.connect(self._tunnel_finished)
        self.stream.readyReadStandardOutput.connect(self._read_wan_output)
        self.stream.readyReadStandardError.connect(self._read_wan_error)
        self.stream.errorOccurred.connect(lambda _error: self._wan_failed('空间数据连接失败，正在重连……'))
        self.stream.finished.connect(self._stream_finished)
        self.timer = QTimer(self)
        self.timer.setInterval(1000)
        self.timer.timeout.connect(self.check_stale)
        self.timer.start()
        if auto_connect:
            QTimer.singleShot(0, self.connect_data)

    def connect_data(self):
        if self.transport == 'wan':
            self.start_wan_tunnel()
        else:
            self.connect_dds()

    def connect_dds(self):
        if self.participant or self._shutting_down:
            return
        try:
            if self.transport == 'local':
                from .ros2_string_transport import Ros2StringParticipant
                self.participant = Ros2StringParticipant(
                    self.domain_id, 'robot320_nuc_spatial_gui',
                )
            else:
                self.participant = FastDdsParticipant(self.domain_id, 'robot320_readonly_spatial_gui')
            for kind in ('scan', 'map', 'pose'):
                self.participant.create_reader('/robot320/spatial_'+kind, lambda payload, k=kind: self.decode(k, payload))
            self.participant.create_reader('/robot320/mapping_status', lambda payload: self.decode('mapping_status', payload))
            self.mapping_writer = self.participant.create_writer('/robot320/mapping_control')
            self.connect_button.setEnabled(False)
            if self.transport == 'local':
                self.status.setText(f'NUC 本机 DDS Domain {self.domain_id} · 等待空间数据')
            else:
                self.status.setText(f'DDS Domain {self.domain_id} · 等待车上空间数据（需同网、VPN 或已配置路由）')
        except Exception as exc:
            self.shutdown()
            self.status.setText(f'DDS 连接失败：{exc}')

    def start_wan_tunnel(self):
        if self._shutting_down or self.tunnel.state() != QProcess.ProcessState.NotRunning:
            return
        self.status.setText('正在建立雷达公网安全通道（Wi-Fi 优先，SIM 自动备用）……')
        self.tunnel.setProgram('ssh')
        self.tunnel.setArguments([
            '-N', '-T', '-o', 'BatchMode=yes', '-o', 'ExitOnForwardFailure=yes',
            '-o', 'ServerAliveInterval=20', '-o', 'ServerAliveCountMax=3',
            '-L', f'127.0.0.1:{self.local_port}:127.0.0.1:{self.cloud_reverse_port}',
            self.cloud_host,
        ])
        self.tunnel.start()

    def _start_wan_stream(self):
        if (self._shutting_down or self.tunnel.state() != QProcess.ProcessState.Running
                or self.stream.state() != QProcess.ProcessState.NotRunning):
            return
        self.status.setText('公网通道已建立，正在订阅车上降采样雷达数据……')
        self.stream.setProgram('ssh')
        self.stream.setArguments([
            '-T', '-p', str(self.local_port),
            '-o', f'HostKeyAlias={self.host_key_alias}',
            '-o', 'StrictHostKeyChecking=yes', '-o', 'BatchMode=yes',
            '-o', 'ConnectTimeout=8', 'hs@127.0.0.1',
            '/usr/bin/bash /home/hs/robot320_remote_spatial/start_spatial_stream.sh',
        ])
        self.stream.start()

    def _tunnel_finished(self, *_args):
        if self.stream.state() != QProcess.ProcessState.NotRunning:
            self.stream.terminate()
        self._wan_failed('雷达公网隧道已断开，正在重连……')
        if not self._shutting_down:
            QTimer.singleShot(2000, self.start_wan_tunnel)

    def _stream_finished(self, *_args):
        self._wan_failed('雷达数据流已断开，正在重连……')
        if not self._shutting_down and self.tunnel.state() == QProcess.ProcessState.Running:
            QTimer.singleShot(1500, self._start_wan_stream)

    def _wan_failed(self, message):
        self.connect_button.setEnabled(True)
        self.start_mapping_button.setEnabled(False)
        self.stop_mapping_button.setEnabled(False)
        self.status.setText(message)

    def _read_wan_error(self):
        error = bytes(self.stream.readAllStandardError()).decode(errors='replace').strip()
        if error and not self.last_scan:
            self.status.setText('空间数据连接提示：' + error[-180:])

    def _read_wan_output(self):
        self._stdout_buffer += bytes(self.stream.readAllStandardOutput())
        if len(self._stdout_buffer) > WAN_MAX_LINE_BYTES and b'\n' not in self._stdout_buffer:
            self._stdout_buffer = b''
            self.status.setText('收到超出安全上限的空间数据，已丢弃并等待下一帧')
            return
        while b'\n' in self._stdout_buffer:
            line, self._stdout_buffer = self._stdout_buffer.split(b'\n', 1)
            self._handle_wan_line(line)

    def _handle_wan_line(self, line):
        if len(line) > WAN_MAX_LINE_BYTES:
            return
        try:
            envelope = json.loads(line)
            if envelope.get('ready') is True and envelope.get('protocol') == 1:
                self.connect_button.setEnabled(False)
                self.status.setText('公网空间数据通道已连接 · Wi-Fi 优先 / SIM 自动备用')
                return
            kind, data = envelope['kind'], envelope['data']
            if kind == 'control_ack':
                if (not data.get('ok') and data.get('request_id') == self.pending_mapping_request):
                    self.pending_mapping_request = None
                    self.map_operation.setText('车端拒绝了建图请求')
                return
            if kind not in ('scan', 'pose', 'map', 'mapping_status') or not isinstance(data, dict):
                return
            self.decode(kind, json.dumps(data, allow_nan=False, separators=(',', ':')))
        except (UnicodeDecodeError, json.JSONDecodeError, TypeError, KeyError, ValueError):
            return

    def decode(self, kind, payload):
        try:
            if kind == 'mapping_status':
                data = json.loads(payload)
                if isinstance(data, dict) and isinstance(data.get('message'), str):
                    self.received.emit(kind, data)
                return
            self.received.emit(kind, parse_spatial(kind, payload))
        except (ValueError, TypeError, KeyError, OverflowError, zlib.error):
            return

    def update_data(self, kind, data):
        if kind == 'mapping_status':
            stamp = data.get('stamp', 0.0)
            if isinstance(stamp, (int, float)) and stamp < self.last_mapping_status_stamp:
                return
            if isinstance(stamp, (int, float)):
                self.last_mapping_status_stamp = stamp
            ready = data.get('available') is True
            self.start_mapping_button.setEnabled(ready)
            self.stop_mapping_button.setEnabled(ready)
            if self.pending_mapping_request:
                if data.get('request_id') != self.pending_mapping_request:
                    return
                self.pending_mapping_request = None
            self.map_operation.setText(data['message'])
            if self.auto_relocalize and data.get('localization_state') == 'active':
                self.auto_relocalize = False
                QTimer.singleShot(500, lambda: self.mapping_control('relocalize'))
            return
        if kind == 'scan':
            self.canvas.scan = data
            self.last_scan = time.monotonic()
            self.scan_sequence += 1
            self.status.setText(
                f'实时雷达 #{self.scan_sequence} · {len(data["points"])} 点 · '
                f'坐标系 {data["frame"]} · {self.data_source}'
            )
        elif kind == 'pose':
            self.canvas.pose = data
            if self.canvas.trajectory_frame != data['frame']:
                self.canvas.trajectory.clear()
                self.canvas.trajectory_frame = data['frame']
            point = tuple(data['pose'][:2])
            if not self.canvas.trajectory or math.dist(point, self.canvas.trajectory[-1]) > .03:
                self.canvas.trajectory.append(point)
        else:
            g = data['geometry']
            pixels = bytes(130 if v == 0 else 255-round((v-1)*2.55) for v in data['decoded_cells'])
            data['image'] = QImage(pixels, g.width, g.height, g.width, QImage.Format.Format_Grayscale8).copy()
            self.canvas.map = data
            self.save_button.setEnabled(True)
        map_text = '已接收' if self.canvas.map else '等待 /map（SLAM 未提供地图）'
        pose_text = f'{len(self.canvas.trajectory)} 个轨迹点 [{self.canvas.trajectory_frame}]' if self.canvas.pose else '等待定位 TF'
        self.details.setText(f'地图：{map_text} · 轨迹：{pose_text} · 仅叠加同一坐标系的数据')
        self.canvas.update()

    def _toggle_map_view(self, show_full_map):
        self.canvas.show_full_map = show_full_map
        self.view_button.setText('跟随机器人' if show_full_map else '显示完整地图')
        self.view_hint.setText(
            '当前：完整地图，可点击选择初始位置' if show_full_map
            else '当前：局部跟随机器人，便于观察实时点云'
        )
        self.canvas.update()

    def mapping_control(self, action, extra=None):
        if action not in ('start', 'stop', 'localize', 'relocalize', 'stop_localization',
                          'initial_pose'):
            return
        if action == 'start' and QMessageBox.question(
            self, '开始新建图会话',
            '开始独立雷达建图，不启动车辆控制或导航。\n如已存在地图，请先保存；停止后重启会创建新会话。继续？',
        ) != QMessageBox.StandardButton.Yes:
            return
        request_id = uuid.uuid4().hex
        if self.transport == 'wan':
            if self.stream.state() != QProcess.ProcessState.Running:
                self.map_operation.setText('公网空间数据通道不可用，建图请求未发送')
                return
            request = {'action': action, 'request_id': request_id}
            if extra:
                request.update(extra)
            payload = (json.dumps(request, separators=(',', ':')) + '\n').encode()
            if self.stream.write(payload) != len(payload):
                self.map_operation.setText('公网空间数据通道写入失败，建图请求未发送')
                return
        elif self.participant and self.mapping_writer is not None:
            request = {'action': action, 'stamp': time.time(), 'request_id': request_id}
            if extra:
                request.update(extra)
            self.participant.write_string(self.mapping_writer, json.dumps(request))
        else:
            self.map_operation.setText('DDS 建图管理通道不可用，请求未发送')
            return
        self.pending_mapping_request = request_id
        self.map_operation.setText('已发送建图服务请求，等待车上状态确认…')

    def _initial_pose_selected(self, x, y, yaw):
        self.initial_pose_button.setEnabled(True)
        self.map_operation.setText(
            f'已选择初始位置 x={x:.2f} m, y={y:.2f} m, 航向={math.degrees(yaw):.1f}°；'
            '确认现场朝向后发送。'
        )

    def send_initial_pose(self):
        pose = self.canvas.selected_initial_pose
        if pose is None:
            return
        if QMessageBox.question(
            self, '修正机器人初始位置',
            '将此位置和车头方向发送给 AMCL，并强制使用连续雷达扫描校正。\n'
            '该操作不会让车辆运动。继续？',
        ) != QMessageBox.StandardButton.Yes:
            return
        self.mapping_control('initial_pose', {'pose': list(pose)})

    def save_snapshot(self, path):
        if self.canvas.map is None:
            raise ValueError('尚未收到栅格地图，不能把点云保存成地图')
        data = self.canvas.map
        snapshot = MapSnapshot(data['geometry'], tuple(v-1 for v in data['decoded_cells']), data['frame'])
        return save_map_yaml(snapshot, path)

    def save_local_map(self):
        if not self.canvas.map:
            return
        default = Path.home() / 'Documents' / 'Robot320Maps' / time.strftime('map_%Y%m%d_%H%M%S.yaml')
        path, _ = QFileDialog.getSaveFileName(self, '保存地图到 GUI 本机', str(default), 'ROS 地图 (*.yaml *.yml)')
        if not path:
            return
        yaml_path = Path(path)
        if yaml_path.suffix.lower() not in ('.yaml', '.yml'):
            yaml_path = yaml_path.with_suffix('.yaml')
        if (yaml_path.exists() or yaml_path.with_suffix('.pgm').exists()) and QMessageBox.question(
            self, '覆盖地图文件', '同名 YAML 或 PGM 已存在，是否覆盖这两个文件？',
        ) != QMessageBox.StandardButton.Yes:
            return
        try:
            yaml_path, pgm_path = self.save_snapshot(yaml_path)
            self.map_operation.setText(f'已保存到本机：{yaml_path} 和 {pgm_path.name}')
        except (OSError, ValueError) as exc:
            QMessageBox.critical(self, '地图保存失败', str(exc))

    def load_local_map(self):
        path, _ = QFileDialog.getOpenFileName(self, '打开本机地图（仅预览）', '', 'ROS 地图 (*.yaml *.yml)')
        if not path:
            return
        try:
            snapshot = load_map_yaml(path)
            self.update_data('map', {'frame': snapshot.frame_id, 'geometry': snapshot.geometry,
                                    'decoded_cells': bytes(v+1 for v in snapshot.data)})
            if self.transport == 'local':
                target = Path(os.getenv(
                    'ROBOT_LOCAL_MAP_PATH',
                    '/home/hs/robot320_remote_spatial/maps/active.yaml',
                ))
                target.parent.mkdir(parents=True, exist_ok=True)
                save_map_yaml(snapshot, target)
                self.auto_relocalize = True
                self.map_operation.setText(f'地图已保存到 NUC 本机：{target}；正在启动定位……')
                self.mapping_control('localize')
                return
            if self.transport != 'wan':
                self.map_operation.setText(f'本机地图预览：{path}；DDS 模式暂不支持安全上传。')
                return
            if QMessageBox.question(
                self, '上传地图并自动定位',
                '将把此地图上传到机器人、停止冲突的建图会话并启动 AMCL 全局匹配。\n'
                '不会启动导航或让车辆运动。继续？',
            ) != QMessageBox.StandardButton.Yes:
                self.map_operation.setText(f'本机地图预览：{path}；尚未切换车端定位。')
                return
            self.upload_localization_map(snapshot)
        except (OSError, ValueError) as exc:
            QMessageBox.critical(self, '地图读取失败', str(exc))

    def upload_localization_map(self, snapshot):
        if self.stream.state() != QProcess.ProcessState.Running:
            self.map_operation.setText('公网空间通道不可用，地图未上传')
            return
        self.map_upload_directory = tempfile.TemporaryDirectory(prefix='robot320-map-')
        stage = Path(self.map_upload_directory.name)
        yaml_path, pgm_path = save_map_yaml(snapshot, stage / 'active.yaml')
        process = QProcess(self)
        process.setProgram('scp')
        process.setArguments([
            '-q', '-P', str(self.local_port), '-o', f'HostKeyAlias={self.host_key_alias}',
            '-o', 'StrictHostKeyChecking=yes', '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=8',
            str(yaml_path), str(pgm_path),
            'hs@127.0.0.1:/home/hs/robot320_remote_spatial/maps/',
        ])
        process.finished.connect(lambda code, _status, p=process: self._map_upload_finished(p, code))
        self.map_upload = process
        self.load_button.setEnabled(False)
        self.map_operation.setText('正在通过安全通道上传地图到机器人……')
        process.start()

    def _map_upload_finished(self, process, code):
        error = bytes(process.readAllStandardError()).decode(errors='replace').strip()
        self.map_upload = None
        self.load_button.setEnabled(True)
        if self.map_upload_directory is not None:
            self.map_upload_directory.cleanup()
            self.map_upload_directory = None
        process.deleteLater()
        if code != 0:
            self.map_operation.setText('地图上传失败：' + (error[-180:] or f'scp exit {code}'))
            return
        self.auto_relocalize = True
        self.map_operation.setText('地图已上传，正在启动车端静态地图定位……')
        self.mapping_control('localize')

    def check_stale(self):
        if self.last_scan and time.monotonic()-self.last_scan > 2:
            self.status.setText('雷达数据已超时 · 显示最后一帧，不代表实时数据')

    def shutdown(self):
        self._shutting_down = True
        if self.participant:
            self.participant.close()
            self.participant = None
        for process in (self.stream, self.tunnel):
            if process.state() != QProcess.ProcessState.NotRunning:
                process.terminate()
                if not process.waitForFinished(1000):
                    process.kill()
        self.mapping_writer = None
        if self.map_upload is not None and self.map_upload.state() != QProcess.ProcessState.NotRunning:
            self.map_upload.kill()
        if self.map_upload_directory is not None:
            self.map_upload_directory.cleanup()
            self.map_upload_directory = None
        self.start_mapping_button.setEnabled(False)
        self.stop_mapping_button.setEnabled(False)
