from remote_control.navigation_gui import normalized_frame_id


def test_normalized_frame_id_never_returns_an_empty_frame():
    assert normalized_frame_id("odom", "map") == "odom"
    assert normalized_frame_id("  ", " map ") == "map"
    assert normalized_frame_id("", "") == "map"
