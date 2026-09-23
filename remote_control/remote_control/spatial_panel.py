"""Read-only standalone DDS LiDAR/map preview; no vehicle command writers."""
from collections import deque
import base64
import json
import math
import time
import zlib
from pathlib import Path

from PySide6.QtCore import QPointF, QRectF, Qt, QTimer, Signal
from PySide6.QtGui import QColor, QImage, QPainter, QPen, QPolygonF
from PySide6.QtWidgets import QFileDialog, QHBoxLayout, QLabel, QMessageBox, QPushButton, QVBoxLayout, QWidget

from robot320_interfaces.fastdds_transport import FastDdsParticipant
from .map_model import MapGeometry, MapSnapshot, load_map_yaml, save_map_yaml


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
    def __init__(self):
        super().__init__()
        self.setMinimumSize(260, 240)
        self.scan = None
        self.pose = None
        self.map = None
        self.trajectory = deque(maxlen=3000)
        self.trajectory_frame = ''

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor('#111c2c'))
        frame = self.map['frame'] if self.map else (self.scan['frame'] if self.scan else '')
        if not frame:
            painter.setPen(QColor('#a8bad0'))
            painter.drawText(self.rect(), Qt.AlignmentFlag.AlignCenter, '等待真实雷达数据（未显示模拟点云）')
            return
        matching_map = self.map and self.map['frame'] == frame
        if matching_map:
            g = self.map['geometry']
            corners = [g.grid_to_world(x, y) for x, y in [(0, 0), (g.width, 0), (0, g.height), (g.width, g.height)]]
            xmin, xmax = min(p[0] for p in corners), max(p[0] for p in corners)
            ymin, ymax = min(p[1] for p in corners), max(p[1] for p in corners)
        else:
            center = self.pose['pose'][:2] if self.pose and self.pose['frame'] == frame else (0, 0)
            xmin, xmax, ymin, ymax = center[0]-15, center[0]+15, center[1]-15, center[1]+15
        scale = min((self.width()-32)/max(xmax-xmin, .01), (self.height()-32)/max(ymax-ymin, .01))
        cx, cy = (xmin+xmax)/2, (ymin+ymax)/2
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
            painter.setPen(QPen(QColor('#ffb45b'), 3/scale))
            painter.drawPoints(QPolygonF([QPointF(x, y) for x, y in self.scan['points']]))
        if self.pose and self.pose['frame'] == frame:
            x, y, yaw = self.pose['pose']
            painter.setPen(QPen(QColor('#60a5fa'), 3/scale))
            painter.drawEllipse(QPointF(x, y), .3, .3)
            painter.drawLine(QPointF(x, y), QPointF(x+math.cos(yaw), y+math.sin(yaw)))


class SpatialPanel(QWidget):
    received = Signal(str, object)

    def __init__(self, domain_id=20, auto_connect=True):
        super().__init__()
        self.domain_id = domain_id
        self.participant = None
        self.last_scan = 0.0
        self.mapping_writer = None
        layout = QVBoxLayout(self)
        row = QHBoxLayout()
        self.connect_button = QPushButton('连接只读 DDS 数据')
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
        load_button = QPushButton('打开本机地图…')
        load_button.clicked.connect(self.load_local_map)
        for button in (self.start_mapping_button, self.stop_mapping_button, self.save_button, load_button):
            tools.addWidget(button)
        tools.addStretch()
        layout.addLayout(tools)
        self.map_operation = QLabel('保存位置为运行 GUI 的本机；保存栅格地图，不包含 Cartographer pose graph。')
        self.map_operation.setWordWrap(True)
        layout.addWidget(self.map_operation)
        self.canvas = SpatialCanvas()
        layout.addWidget(self.canvas, 1)
        self.details = QLabel('地图：等待 /map · 轨迹：等待真实 map/odom → base_link TF')
        self.details.setWordWrap(True)
        layout.addWidget(self.details)
        self.received.connect(self.update_data)
        self.timer = QTimer(self)
        self.timer.setInterval(1000)
        self.timer.timeout.connect(self.check_stale)
        self.timer.start()
        if auto_connect:
            QTimer.singleShot(0, self.connect_data)

    def connect_data(self):
        if self.participant:
            return
        try:
            self.participant = FastDdsParticipant(self.domain_id, 'robot320_readonly_spatial_gui')
            for kind in ('scan', 'map', 'pose'):
                self.participant.create_reader('/robot320/spatial_'+kind, lambda payload, k=kind: self.decode(k, payload))
            self.participant.create_reader('/robot320/mapping_status', lambda payload: self.decode('mapping_status', payload))
            self.mapping_writer = self.participant.create_writer('/robot320/mapping_control')
            self.connect_button.setEnabled(False)
            self.status.setText(f'DDS Domain {self.domain_id} · 等待车上空间数据（需同网、VPN 或已配置路由）')
        except Exception as exc:
            self.shutdown()
            self.status.setText(f'DDS 连接失败：{exc}')

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
            ready = data.get('available') is True
            self.start_mapping_button.setEnabled(ready)
            self.stop_mapping_button.setEnabled(ready)
            self.map_operation.setText(data['message'])
            return
        if kind == 'scan':
            self.canvas.scan = data
            self.last_scan = time.monotonic()
            self.status.setText(f'实时雷达 · {len(data["points"])} 点 · 坐标系 {data["frame"]} · 只读 DDS')
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

    def mapping_control(self, action):
        if not self.participant or self.mapping_writer is None or action not in ('start', 'stop'):
            return
        if action == 'start' and QMessageBox.question(
            self, '开始新建图会话',
            '开始独立雷达建图，不启动车辆控制或导航。\n如已存在地图，请先保存；停止后重启会创建新会话。继续？',
        ) != QMessageBox.StandardButton.Yes:
            return
        self.participant.write_string(self.mapping_writer, json.dumps({'action': action, 'stamp': time.time()}))
        self.map_operation.setText('已发送建图服务请求，等待车上状态确认…')

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
            self.map_operation.setText(f'本机地图预览：{path}；未上传到机器人或切换定位。')
        except (OSError, ValueError) as exc:
            QMessageBox.critical(self, '地图读取失败', str(exc))

    def check_stale(self):
        if self.last_scan and time.monotonic()-self.last_scan > 2:
            self.status.setText('雷达数据已超时 · 显示最后一帧，不代表实时数据')

    def shutdown(self):
        if self.participant:
            self.participant.close()
            self.participant = None
        self.mapping_writer = None
        self.start_mapping_button.setEnabled(False)
        self.stop_mapping_button.setEnabled(False)
