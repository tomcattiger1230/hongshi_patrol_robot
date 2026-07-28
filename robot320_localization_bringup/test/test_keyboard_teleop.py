import pytest

from robot320_localization_bringup.keyboard_teleop_state import (
    AckermannTeleopState,
)


def test_forward_left_respects_minimum_turning_radius():
    state = AckermannTeleopState()

    assert state.apply_key("w")
    assert state.apply_key("a")

    linear, angular = state.command()
    assert linear == pytest.approx(0.35)
    assert angular == pytest.approx(0.35 / 2.35)


def test_reverse_right_and_arrow_keys():
    state = AckermannTeleopState()

    assert state.apply_key("\x1b[B")
    assert state.apply_key("\x1b[C")

    linear, angular = state.command()
    assert linear == pytest.approx(-0.20)
    assert angular == pytest.approx(-0.20 / 2.35)


def test_steering_does_not_create_an_in_place_turn():
    state = AckermannTeleopState()

    assert state.apply_key("a")
    assert state.command() == (0.0, 0.0)

    assert state.apply_key("w")
    assert state.command()[1] > 0.0


def test_stop_and_straighten():
    state = AckermannTeleopState()
    state.apply_key("w")
    state.apply_key("a")

    assert state.apply_key("r")
    assert state.command() == (0.35, 0.0)
    assert state.apply_key(" ")
    assert state.command() == (0.0, 0.0)
    assert not state.apply_key("?")
