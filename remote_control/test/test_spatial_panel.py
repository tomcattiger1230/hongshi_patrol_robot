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
    panel = SpatialPanel(auto_connect=False, transport='dds')
    sent = []

    class Participant:
        def write_string(self, writer, payload):
            sent.append((writer, json.loads(payload)))

        def close(self):
            pass

    panel.participant = Participant()
    panel.mapping_writer = 'mapping-only-writer'
    panel.mapping_control('stop')
    panel.mapping_control('initial_pose', {'pose': [1.0, -2.0, 0.5]})
    panel.mapping_control('drive')
    assert len(sent) == 2
    assert sent[0][0] == 'mapping-only-writer'
    assert sent[0][1]['action'] == 'stop'
    assert sent[0][1]['request_id']
    assert sent[1][1]['action'] == 'initial_pose'
    assert sent[1][1]['pose'] == [1.0, -2.0, 0.5]
    panel.shutdown()


def test_nuc_spatial_mode_uses_local_dds_without_ssh_tunnel():
    QApplication.instance() or QApplication([])
    panel = SpatialPanel(auto_connect=False, transport='local')
    assert panel.transport == 'local'
    assert panel.data_source == 'NUC 本机 DDS'
    assert panel.tunnel.state() == panel.tunnel.ProcessState.NotRunning
    assert 'NUC 本机' in panel.connect_button.text()
    panel.shutdown()
    panel.close()


def test_wan_stream_accepts_only_versioned_allowlisted_envelopes():
    app = QApplication.instance() or QApplication([])
    panel = SpatialPanel(auto_connect=False, transport='wan')
    panel._handle_wan_line(b'{"ready":true,"protocol":1}')
    assert not panel.connect_button.isEnabled()
    panel._handle_wan_line(b'{"kind":"scan","data":{"frame":"map","points":[[1,2]]}}')
    app.processEvents()
    assert panel.canvas.scan['points'] == [[1, 2]]
    panel._handle_wan_line(b'{"kind":"arbitrary","data":{"frame":"map"}}')
    panel._handle_wan_line(b'not-json')
    assert panel.canvas.scan['points'] == [[1, 2]]
    panel.shutdown()
    panel.close()


def test_local_follow_view_and_scan_counter_can_switch_to_full_map():
    app = QApplication.instance() or QApplication([])
    panel = SpatialPanel(auto_connect=False, transport='wan')
    geometry = MapGeometry(100, 100, .5, -25, -25, 0)
    panel.update_data('map', {'frame': 'map', 'geometry': geometry,
                              'decoded_cells': bytes([1] * 10_000)})
    panel.update_data('pose', {'frame': 'map', 'pose': [4, -3, 0]})
    panel.update_data('scan', {'frame': 'map', 'points': [[4, -3], [5, -2]]})
    panel.resize(900, 700)
    panel.show()
    app.processEvents()
    panel.canvas.grab()
    assert not panel.canvas.show_full_map
    assert panel.canvas._view[1:] == (4, -3)
    assert '实时雷达 #1' in panel.status.text()
    panel.view_button.click()
    app.processEvents()
    panel.canvas.grab()
    assert panel.canvas.show_full_map
    assert panel.canvas._view[1:] == (0, 0)
    assert panel.view_button.text() == '跟随机器人'
    panel.shutdown()
    panel.close()


def test_mapping_status_does_not_replace_pending_request_with_stale_status():
    QApplication.instance() or QApplication([])
    panel = SpatialPanel(auto_connect=False, transport='wan')
    panel.pending_mapping_request = 'new-request'
    panel.update_data('mapping_status', {
        'available': True, 'state': 'inactive', 'message': 'old inactive',
        'stamp': 10.0, 'sequence': 1,
    })
    assert panel.pending_mapping_request == 'new-request'
    assert panel.map_operation.text() != 'old inactive'
    panel.update_data('mapping_status', {
        'available': True, 'state': 'active', 'message': 'confirmed active',
        'stamp': 11.0, 'sequence': 2, 'request_id': 'new-request',
    })
    assert panel.pending_mapping_request is None
    assert panel.map_operation.text() == 'confirmed active'
    panel.update_data('mapping_status', {
        'available': True, 'state': 'inactive', 'message': 'late inactive',
        'stamp': 9.0, 'sequence': 0,
    })
    assert panel.map_operation.text() == 'confirmed active'
    panel.shutdown()
    panel.close()
