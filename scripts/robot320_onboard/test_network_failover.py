from network_failover import FailoverState


def test_three_failures_switch_from_wifi_to_sim():
    state = FailoverState(active="wifi")
    assert state.observe(False, 3) is None
    assert state.observe(False, 3) is None
    assert state.observe(False, 3) == "sim"
    assert state.active == "sim"


def test_success_resets_failure_hysteresis():
    state = FailoverState(active="wifi")
    state.observe(False, 3)
    state.observe(False, 3)
    assert state.observe(True, 3) is None
    assert state.failures == 0
    assert state.observe(False, 3) is None


def test_three_successes_restore_wifi_after_failover():
    state = FailoverState(active="sim")
    assert state.observe(True, 3) is None
    assert state.observe(True, 3) is None
    assert state.observe(True, 3) == "wifi"
    assert state.active == "wifi"
