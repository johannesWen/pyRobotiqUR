import socket
import pytest

from pyrobotiqur import RobotiqGripper
from pyrobotiqur.enums import ObjectStatus, GripperStatus


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


class CallRecorder:
    """Simple helper to record calls to a fake function."""

    def __init__(self):
        self.calls = []

    def __call__(self, *args, **kwargs):
        self.calls.append((args, kwargs))


# ---------------------------------------------------------------------------
# get_var / set_var tests
# ---------------------------------------------------------------------------


def test_get_var_parses_valid_response(monkeypatch):
    g = RobotiqGripper("127.0.0.1")

    def fake_send_raw(cmd: str) -> str:
        # we expect to be called with "GET POS"
        assert cmd == "GET POS"
        return "POS 123"

    monkeypatch.setattr(g, "_send_raw", fake_send_raw)

    value = g.get_var("POS")
    assert value == 123


def test_get_var_raises_on_malformed_response(monkeypatch):
    g = RobotiqGripper("127.0.0.1")

    def fake_send_raw(cmd: str) -> str:
        return "ERR"  # malformed, no value

    monkeypatch.setattr(g, "_send_raw", fake_send_raw)

    with pytest.raises(ValueError):
        g.get_var("POS")


def test_set_var_succeeds_on_ack(monkeypatch):
    g = RobotiqGripper("127.0.0.1")

    sent_commands = []

    def fake_send_raw(cmd: str) -> str:
        sent_commands.append(cmd)
        return "ack"

    monkeypatch.setattr(g, "_send_raw", fake_send_raw)

    g.set_var("POS", 42)

    assert sent_commands == ["SET POS 42"]


def test_set_var_raises_on_non_ack(monkeypatch):
    g = RobotiqGripper("127.0.0.1")

    def fake_send_raw(cmd: str) -> str:
        return "nack"

    monkeypatch.setattr(g, "_send_raw", fake_send_raw)

    with pytest.raises(RuntimeError):
        g.set_var("POS", 42)


# ---------------------------------------------------------------------------
# move_percent tests
# ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "percent, expected_pos",
    [
        (0, 0),
        (50, 128),   # 50% of 255 ≈ 127.5 => 128
        (100, 255),
    ],
)
def test_move_percent_mapping(monkeypatch, percent, expected_pos):
    g = RobotiqGripper("127.0.0.1")

    recorded_args = {}

    def fake_move(position, speed, force, wait, poll_interval):
        recorded_args["position"] = position
        recorded_args["speed"] = speed
        recorded_args["force"] = force
        recorded_args["wait"] = wait
        recorded_args["poll_interval"] = poll_interval
        # pretend final position reached
        return position, ObjectStatus.AT_DEST

    monkeypatch.setattr(g, "move", fake_move)

    result_pos, result_status = g.move_percent(percent, speed=150, force=200)

    assert recorded_args["position"] == expected_pos
    assert recorded_args["speed"] == 150
    assert recorded_args["force"] == 200
    assert recorded_args["wait"] is True
    assert result_pos == expected_pos
    assert result_status == ObjectStatus.AT_DEST


def test_move_percent_clamps_range(monkeypatch):
    g = RobotiqGripper("127.0.0.1")

    recorded_positions = []

    def fake_move(position, *_, **__):
        recorded_positions.append(position)
        return position, ObjectStatus.AT_DEST

    monkeypatch.setattr(g, "move", fake_move)

    # Below 0% -> clamp to 0
    g.move_percent(-10)
    # Above 100% -> clamp to 255
    g.move_percent(150)

    assert recorded_positions[0] == 0
    assert recorded_positions[1] == 255


# ---------------------------------------------------------------------------
# move() tests (logic, not socket I/O)
# ---------------------------------------------------------------------------


def test_move_waits_for_pre_and_object_status(monkeypatch):
    g = RobotiqGripper("127.0.0.1")

    # Record set_var calls
    set_recorder = CallRecorder()
    monkeypatch.setattr(g, "set_var", set_recorder)

    # Simulate PRE (requested position) converging to target after 2 polls
    requested_positions = [0, 50, 128]  # final equals target

    def fake_get_requested_position():
        return requested_positions.pop(0)

    monkeypatch.setattr(g, "get_requested_position", fake_get_requested_position)

    # Simulate object status: moving -> moving -> at destination
    object_statuses = [
        ObjectStatus.MOVING,
        ObjectStatus.MOVING,
        ObjectStatus.AT_DEST,
    ]

    def fake_get_object_status():
        return object_statuses.pop(0)

    monkeypatch.setattr(g, "get_object_status", fake_get_object_status)

    # Final position reported by gripper
    monkeypatch.setattr(g, "get_position", lambda: 128)

    # Avoid real sleeping in tests
    monkeypatch.setattr("time.sleep", lambda *_args, **_kwargs: None)

    final_pos, obj_status = g.move(128, speed=100, force=50, wait=True)

    # Check commands sent via set_var
    # Expected: POS, SPE, FOR, GTO (order matters)
    names = [call[0][0] for call in set_recorder.calls]
    values = [call[0][1] for call in set_recorder.calls]

    assert names == ["POS", "SPE", "FOR", "GTO"]
    assert values[0] == 128
    assert values[1] == 100
    assert values[2] == 50
    assert values[3] == 1

    assert final_pos == 128
    assert obj_status == ObjectStatus.AT_DEST


def test_move_nowait_returns_immediately(monkeypatch):
    g = RobotiqGripper("127.0.0.1")

    # Record set_var calls but ensure no get_* polling is done
    set_recorder = CallRecorder()
    monkeypatch.setattr(g, "set_var", set_recorder)

    get_pre = CallRecorder()
    monkeypatch.setattr(g, "get_requested_position", get_pre)

    get_obj = CallRecorder()
    monkeypatch.setattr(g, "get_object_status", get_obj)

    position = 200
    final_pos, obj_status = g.move(position, speed=128, force=128, wait=False)

    # Still must send config commands
    names = [call[0][0] for call in set_recorder.calls]
    assert names == ["POS", "SPE", "FOR", "GTO"]

    # But no polling should happen
    assert get_pre.calls == []
    assert get_obj.calls == []

    assert final_pos == position
    assert obj_status == ObjectStatus.MOVING


def test_move_clamps_position_speed_and_force(monkeypatch):
    g = RobotiqGripper("127.0.0.1")

    set_calls = []

    def fake_set_var(name, value):
        set_calls.append((name, value))

    monkeypatch.setattr(g, "set_var", fake_set_var)
    requested_positions = [-1, 0]

    def fake_get_requested_position():
        return requested_positions.pop(0)

    monkeypatch.setattr(g, "get_requested_position", fake_get_requested_position)
    monkeypatch.setattr(g, "get_object_status", lambda: ObjectStatus.AT_DEST)
    monkeypatch.setattr(g, "get_position", lambda: 0)
    monkeypatch.setattr("time.sleep", lambda *_args, **_kwargs: None)

    g.move(-50, speed=999, force=-10, wait=True, poll_interval=0.0)

    # Values should be clamped to [0, 255]
    assert ("POS", 0) in set_calls
    assert ("SPE", 255) in set_calls
    assert ("FOR", 0) in set_calls


# ---------------------------------------------------------------------------
# open() / close() tests
# ---------------------------------------------------------------------------


def test_open_calls_move_with_zero_position(monkeypatch):
    g = RobotiqGripper("127.0.0.1")

    recorded = {}

    def fake_move(position, speed, force, wait, poll_interval):
        recorded["position"] = position
        recorded["speed"] = speed
        recorded["force"] = force
        recorded["wait"] = wait
        recorded["poll_interval"] = poll_interval
        return position, ObjectStatus.AT_DEST

    monkeypatch.setattr(g, "move", fake_move)

    g.open(speed=10, force=2)
    assert recorded["position"] == 0
    assert recorded["speed"] == 10
    assert recorded["force"] == 2


def test_close_calls_move_with_max_position(monkeypatch):
    g = RobotiqGripper("127.0.0.1")

    recorded = {}

    def fake_move(position, speed, force, wait, poll_interval):
        recorded["position"] = position
        recorded["speed"] = speed
        recorded["force"] = force
        recorded["wait"] = wait
        recorded["poll_interval"] = poll_interval
        return position, ObjectStatus.AT_DEST

    monkeypatch.setattr(g, "move", fake_move)

    g.close(speed=180, force=220)
    assert recorded["position"] == 255
    assert recorded["speed"] == 180
    assert recorded["force"] == 220


# ---------------------------------------------------------------------------
# reset() / activate() tests (logic only)
# ---------------------------------------------------------------------------


def test_reset_calls_act_and_atr_until_reset(monkeypatch):
    g = RobotiqGripper("127.0.0.1")

    set_recorder = CallRecorder()
    monkeypatch.setattr(g, "set_var", set_recorder)

    # Sequence of states: first loop -> not reset, second loop -> reset
    state_sequence = [
        {"ACT": 1, "STA": GripperStatus.ACTIVE},
        {"ACT": 0, "STA": GripperStatus.RESET},
    ]

    call_count = 0

    def fake_get_var(name: str):
        nonlocal call_count
        # After two total calls (ACT/STA or just ACT twice),
        # move to the second state (reset)
        idx = 0 if call_count < 2 else 1
        call_count += 1
        return state_sequence[idx][name]

    monkeypatch.setattr(g, "get_var", fake_get_var)

    # Avoid real sleeping
    monkeypatch.setattr("time.sleep", lambda *_args, **_kwargs: None)

    g.reset()

    # We should have tried to clear ACT and ATR at least once
    names = [c[0][0] for c in set_recorder.calls]
    assert "ACT" in names
    assert "ATR" in names


def test_reset_raises_after_timeout(monkeypatch):
    g = RobotiqGripper("127.0.0.1")

    # Always report non-reset state so loop never finishes
    monkeypatch.setattr(
        g,
        "get_var",
        lambda name: 1 if name == "ACT" else GripperStatus.ACTIVE,
    )

    # Prevent real socket usage
    monkeypatch.setattr(g, "set_var", lambda *_, **__: None)

    # Avoid sleeping to keep test fast
    monkeypatch.setattr("time.sleep", lambda *_args, **_kwargs: None)

    with pytest.raises(TimeoutError):
        g.reset(poll_interval=0.0, timeout=0.0)


def test_activate_skips_reset_if_already_active(monkeypatch):
    g = RobotiqGripper("127.0.0.1")

    # Pretend gripper is already active
    monkeypatch.setattr(
        g,
        "get_status",
        lambda: GripperStatus.ACTIVE,
    )

    # get_var should report ACT == 1 so the loop terminates quickly
    def fake_get_var(name: str):
        if name == "ACT":
            return 1
        raise AssertionError(f"Unexpected get_var({name}) in this test")

    monkeypatch.setattr(g, "get_var", fake_get_var)

    set_recorder = CallRecorder()
    monkeypatch.setattr(g, "set_var", set_recorder)

    # Avoid sleeping
    monkeypatch.setattr("time.sleep", lambda *_args, **_kwargs: None)

    g.activate(wait=True)

    # Ensure activation command was sent
    names = [c[0][0] for c in set_recorder.calls]
    values = [c[0][1] for c in set_recorder.calls]
    assert ("ACT" in names) and (1 in values)


# ---------------------------------------------------------------------------
# connect / disconnect
# ---------------------------------------------------------------------------


def test_connect_creates_socket_and_connects(monkeypatch):
    created = {}

    class DummySocket:
        def __init__(self, family, type_):
            created["family"] = family
            created["type"] = type_
            self.timeout = None
            self.connected_to = None

        def settimeout(self, t):
            self.timeout = t

        def connect(self, addr):
            self.connected_to = addr

    def fake_socket(family, type_):
        s = DummySocket(family, type_)
        created["instance"] = s
        return s

    monkeypatch.setattr(socket, "socket", fake_socket)

    g = RobotiqGripper("10.0.0.1", port=63352, timeout=1.5)

    g.connect()

    s = created["instance"]
    assert created["family"] == socket.AF_INET
    assert created["type"] == socket.SOCK_STREAM
    assert s.timeout == 1.5
    assert s.connected_to == ("10.0.0.1", 63352)
    assert g._sock is s


def test_connect_is_noop_if_already_connected(monkeypatch):
    calls = []

    def fake_socket(*args, **kwargs):
        calls.append((args, kwargs))
        raise AssertionError("socket.socket should not be called when already connected")

    monkeypatch.setattr(socket, "socket", fake_socket)

    g = RobotiqGripper("10.0.0.1")
    g._sock = object()  # pretend already connected

    g.connect()

    assert calls == []  # no new socket created


def test_disconnect_closes_socket_and_clears_attr():
    class DummySocket:
        def __init__(self):
            self.closed = False

        def close(self):
            self.closed = True

    g = RobotiqGripper("10.0.0.1")
    s = DummySocket()
    g._sock = s

    g.disconnect()

    assert s.closed is True
    assert g._sock is None


def test_disconnect_noop_when_not_connected():
    g = RobotiqGripper("10.0.0.1")
    # Should simply return without error
    g.disconnect()


def test_disconnect_clears_attr_even_if_close_raises():
    class DummySocket:
        def __init__(self):
            self.closed = False

        def close(self):
            self.closed = True
            raise RuntimeError("boom")

    g = RobotiqGripper("10.0.0.1")
    s = DummySocket()
    g._sock = s

    with pytest.raises(RuntimeError):
        g.disconnect()

    # Even though close raised, _sock must be cleared by finally:
    assert g._sock is None
    assert s.closed is True


# ---------------------------------------------------------------------------
# context manager (__enter__ / __exit__)
# ---------------------------------------------------------------------------

def test_context_manager_calls_connect_and_disconnect(monkeypatch):
    g = RobotiqGripper("10.0.0.1")

    connect_rec = CallRecorder()
    disconnect_rec = CallRecorder()

    monkeypatch.setattr(g, "connect", connect_rec)
    monkeypatch.setattr(g, "disconnect", disconnect_rec)

    with g as ctx:
        assert ctx is g

    assert len(connect_rec.calls) == 1
    assert len(disconnect_rec.calls) == 1


def test_context_manager_disconnects_even_on_exception(monkeypatch):
    g = RobotiqGripper("10.0.0.1")

    monkeypatch.setattr(g, "connect", lambda: None)
    disconnect_rec = CallRecorder()
    monkeypatch.setattr(g, "disconnect", disconnect_rec)

    with pytest.raises(RuntimeError):
        with g:
            raise RuntimeError("bang")

    assert len(disconnect_rec.calls) == 1


# ---------------------------------------------------------------------------
# _send_raw
# ---------------------------------------------------------------------------

def test_send_raw_raises_if_not_connected():
    g = RobotiqGripper("10.0.0.1")
    g._sock = None

    with pytest.raises(RuntimeError) as excinfo:
        g._send_raw("GET POS")

    assert "Socket not connected" in str(excinfo.value)


def test_send_raw_sends_trimmed_command_and_returns_stripped_response():
    g = RobotiqGripper("10.0.0.1")

    class DummySocket:
        def __init__(self):
            self.sent_data = None

        def sendall(self, data):
            self.sent_data = data

        def recv(self, bufsize):
            # ensure buffer size matches implementation
            assert bufsize == 1024
            return b"POS 123 \n"

        # these are unused in this test but present for completeness
        def settimeout(self, t):  # pragma: no cover
            pass

        def connect(self, addr):  # pragma: no cover
            pass

        def close(self):  # pragma: no cover
            pass

    s = DummySocket()
    g._sock = s

    # Note the extra spaces – they should be stripped inside _send_raw
    resp = g._send_raw("  GET POS  ")

    # Command should be trimmed and have exactly one '\n' appended
    assert s.sent_data == b"GET POS\n"
    # Response should be decoded and stripped
    assert resp == "POS 123"


def test_send_raw_raises_if_remote_closed():
    g = RobotiqGripper("10.0.0.1")

    class DummySocket:
        def __init__(self):
            self.sent_data = None

        def sendall(self, data):
            self.sent_data = data

        def recv(self, bufsize):
            return b""  # simulate remote disconnect

    s = DummySocket()
    g._sock = s

    with pytest.raises(ConnectionError):
        g._send_raw("GET POS")


def test_is_connected_property_reflects_socket_state():
    g = RobotiqGripper("10.0.0.1")
    assert g.is_connected is False

    class DummySocket:
        pass

    g._sock = DummySocket()
    assert g.is_connected is True

