"""Local mocks only: WSL must remain awake during network UAC and leave cleanly."""

from unittest.mock import Mock
import subprocess

import pytest

from scripts.setup import launcher


@pytest.mark.parametrize("failure", [None, ValueError("network refused"), KeyboardInterrupt()])
def test_session_lives_during_network_stage_and_closes_stdin_on_exit(monkeypatch, failure):
    child = Mock()
    child.poll.return_value = None
    popen = Mock(return_value=child)
    monkeypatch.setattr(subprocess, "Popen", popen)
    try:
        with launcher.wsl_session("RM65-Servo"):
            child.communicate.assert_not_called()
            if failure:
                raise failure
    except (ValueError, KeyboardInterrupt) as exc:
        assert exc is failure
    child.communicate.assert_called_once_with(timeout=10)
    child.terminate.assert_not_called()
    command = popen.call_args.args[0]
    assert command[1:] == ["--distribution", "RM65-Servo", "--exec", "cat"]
    kwargs = popen.call_args.kwargs
    assert kwargs["stdin"] == subprocess.PIPE
    assert kwargs["stdout"] == subprocess.DEVNULL
    assert kwargs["cwd"] == launcher.ROOT
    assert kwargs["env"]["WSL_UTF8"] == "1"
    assert "--terminate" not in command and "--shutdown" not in command


def test_failed_session_refuses_network_but_still_cleans_child(monkeypatch):
    child = Mock()
    child.poll.return_value = 1
    monkeypatch.setattr(subprocess, "Popen", Mock(return_value=child))
    with pytest.raises(OSError, match="未继续网络配置"):
        with launcher.wsl_session("RM65-Servo"):
            pytest.fail("Network stage must not run")
    child.communicate.assert_called_once_with(timeout=10)


@pytest.mark.parametrize("needs_kill", [False, True])
def test_stuck_cleanup_terminates_only_own_child(monkeypatch, needs_kill):
    child = Mock()
    child.poll.return_value = None
    child.communicate.side_effect = subprocess.TimeoutExpired("own cat", 10)
    if needs_kill:
        child.wait.side_effect = [subprocess.TimeoutExpired("own cat", 5), 0]
    monkeypatch.setattr(subprocess, "Popen", Mock(return_value=child))
    with launcher.wsl_session("RM65-Servo"):
        pass
    child.terminate.assert_called_once_with()
    assert child.kill.call_count == int(needs_kill)
    assert child.wait.call_count == 1 + int(needs_kill)
