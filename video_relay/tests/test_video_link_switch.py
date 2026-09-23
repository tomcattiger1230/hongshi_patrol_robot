import pytest

from video_relay.local.web_viewer import RtspReader, create_app


@pytest.fixture
def client(monkeypatch):
    monkeypatch.setattr(RtspReader, "start", lambda self: None)
    first = RtspReader([("局域网", "rtsp://local/one"), ("公网", "rtsp://cloud/one")], 80)
    second = RtspReader([("局域网", "rtsp://local/two")], 80)
    return create_app(first, second_reader=second).test_client()


def test_video_off_affects_only_selected_channel_and_can_resume(client):
    assert client.post('/api/link', json={'channel': 1, 'mode': 'off'}).status_code == 200
    assert client.get('/api/status').get_json()['state'] == 'disabled'
    assert client.get('/snapshot.jpg').status_code == 503
    assert client.get('/api/status2').get_json()['state'] == 'connecting'
    response = client.post('/api/link', json={'channel': 1, 'mode': 'auto'})
    assert response.get_json()['link_mode'] == 'auto'
    assert set(response.get_json()['available_modes']) == {'auto', 'off', 'lan', 'cloud'}


def test_switching_to_configured_link_preserves_other_options(client):
    response = client.post('/api/link', json={'channel': 1, 'mode': 'cloud'})
    assert response.status_code == 200
    assert response.get_json()['link_mode'] == 'cloud'
    assert 'lan' in response.get_json()['available_modes']
    assert client.post('/api/link', json={'channel': 1, 'mode': 'lan'}).status_code == 200


def test_unconfigured_second_cloud_is_rejected_without_switching(client):
    response = client.post('/api/link', json={'channel': 2, 'mode': 'cloud'})
    assert response.status_code == 400
    assert client.get('/api/status2').get_json()['link_mode'] == 'auto'
    assert 'cloud' not in client.get('/api/status2').get_json()['available_modes']


@pytest.mark.parametrize('payload', [None, {}, {'channel': True, 'mode': 'off'},
                                    {'channel': 3, 'mode': 'lan'}, {'channel': 1, 'mode': []}])
def test_invalid_link_payload_is_rejected(client, payload):
    assert client.post('/api/link', json=payload).status_code == 400
