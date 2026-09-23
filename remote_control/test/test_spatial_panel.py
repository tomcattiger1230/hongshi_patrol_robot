import base64
import json
import os
import zlib

import pytest

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')
pytest.importorskip('PySide6')
from PySide6.QtWidgets import QApplication  # noqa: E402
from remote_control.spatial_panel import SpatialPanel, parse_spatial  # noqa: E402
from remote_control.map_model import MapGeometry, load_map_yaml  # noqa: E402


def test_scan_requires_finite_points_and_frame():
    assert parse_spatial('scan', '{"frame":"lidar","points":[[1,2]]}')['points'] == [[1, 2]]
    with pytest.raises(ValueError):
        parse_spatial('scan', '{"frame":"lidar","points":[[NaN,2]]}')
    with pytest.raises(ValueError):
        parse_spatial('scan', '{"points":[]}')


def test_map_decompression_is_bounded_and_checked():
    payload = {'frame': 'map', 'width': 2, 'height': 2, 'resolution': .05,
               'origin': [0, 0, 0], 'cells': base64.b64encode(zlib.compress(bytes([0, 1, 101, 50]))).decode()}
    assert len(parse_spatial('map', json.dumps(payload))['decoded_cells']) == 4
    payload['cells'] = base64.b64encode(zlib.compress(bytes(100))).decode()
    with pytest.raises(ValueError):
        parse_spatial('map', json.dumps(payload))


def test_preview_is_disconnected_by_default_and_keeps_frames_separate():
    app = QApplication.instance() or QApplication([])
    panel = SpatialPanel(auto_connect=False)
    assert panel.participant is None
    panel.update_data('pose', {'frame': 'odom', 'pose': [1, 2, 0]})
    panel.update_data('pose', {'frame': 'map', 'pose': [3, 4, 0]})
    assert list(panel.canvas.trajectory) == [(3, 4)]
    panel.update_data('scan', {'frame': 'livox_frame', 'points': [[1, 2]]})
    panel.resize(800, 600)
    panel.show()
    app.processEvents()
    panel.canvas.grab()
    assert panel.canvas.scan['frame'] != panel.canvas.trajectory_frame
    panel.shutdown()
    panel.close()


def test_received_map_can_be_saved_on_gui_host_and_reopened(tmp_path):
    app = QApplication.instance() or QApplication([])
    panel = SpatialPanel(auto_connect=False)
    with pytest.raises(ValueError):
        panel.save_snapshot(tmp_path / 'no_map.yaml')
    geometry = MapGeometry(2, 2, .05, 1.5, -2.0, .3)
    panel.update_data('map', {'frame': 'map', 'geometry': geometry,
                              'decoded_cells': bytes([0, 1, 101, 1])})
    assert panel.save_button.isEnabled()
    yaml_path, pgm_path = panel.save_snapshot(tmp_path / 'local_map.yaml')
    assert yaml_path.parent == tmp_path
    assert pgm_path.exists()
    saved = load_map_yaml(yaml_path)
    assert saved.geometry == geometry
    assert saved.data == (-1, 0, 100, 0)
    panel.show()
    app.processEvents()
    assert panel.canvas.scan is None
    assert not panel.canvas.grab().isNull()
    panel.shutdown()
    panel.close()


def test_mapping_management_uses_only_its_dedicated_writer():
    QApplication.instance() or QApplication([])
    panel = SpatialPanel(auto_connect=False)
    sent = []

    class Participant:
        def write_string(self, writer, payload):
            sent.append((writer, json.loads(payload)))

        def close(self):
            pass

    panel.participant = Participant()
    panel.mapping_writer = 'mapping-only-writer'
    panel.mapping_control('stop')
    panel.mapping_control('drive')
    assert len(sent) == 1
    assert sent[0][0] == 'mapping-only-writer'
    assert sent[0][1]['action'] == 'stop'
    panel.shutdown()
