import hashlib
import json
import shlex
import subprocess
import sys
from pathlib import Path
from unittest.mock import MagicMock, patch

import pytest

import teleop


@pytest.fixture(autouse=True)
def no_remote_manifest_writes(monkeypatch):
    monkeypatch.setattr(teleop, "_write_remote_manifest", lambda files: True)


def test_noninteractive_sim_requires_home_joint():
    with patch.object(teleop.sys.stdin, "isatty", return_value=False):
        with pytest.raises(ValueError, match="home-joint-deg"):
            teleop._resolve_sim_home_joint(None)


def test_run_in_container_forwards_home_joint_values():
    process = MagicMock()
    process.returncode = 0
    with (
        patch.object(teleop.sys.stdin, "isatty", return_value=False),
        patch.object(teleop.subprocess, "Popen", return_value=process) as popen,
    ):
        result = teleop.run_in_container("sim", [1.0, -2.0, 3.5, 4.0, 5.0, 6.0])

    command = popen.call_args.args[0]
    assert result == 0
    assert "--home-joint-deg 1 -2 3.5 4 5 6" in command[-1]
    assert "--ik-backend placo" in command[-1]


def test_run_in_container_forwards_official_sim_ik_backend():
    process = MagicMock()
    process.returncode = 0
    with (
        patch.object(teleop.sys.stdin, "isatty", return_value=False),
        patch.object(teleop.subprocess, "Popen", return_value=process) as popen,
    ):
        result = teleop.run_in_container(
            "sim",
            [1.0, -2.0, 30.0, 4.0, 5.0, 6.0],
            sim_ik_backend="official",
        )

    command = popen.call_args.args[0]
    assert result == 0
    assert "--ik-backend official" in command[-1]


def test_run_in_container_rejects_missing_sim_home_joint():
    with pytest.raises(ValueError, match="six calibrated"):
        teleop.run_in_container("sim", None)


def test_run_in_container_uses_safe_hardware_entrypoint():
    process = MagicMock()
    process.returncode = 0
    with (
        patch.object(teleop.sys.stdin, "isatty", return_value=False),
        patch.object(teleop.subprocess, "Popen", return_value=process) as popen,
    ):
        result = teleop.run_in_container("hardware")

    assert result == 0
    command = popen.call_args.args[0][-1]
    assert "scripts/hardware/teleop_realman_rm65_safe_hardware.py" in command
    assert f"--arm-host {teleop.ARM_HOST}" in command
    assert "--dry-run" not in command


def test_run_in_container_supports_hardware_dry_run():
    process = MagicMock()
    process.returncode = 0
    with (
        patch.object(teleop.sys.stdin, "isatty", return_value=False),
        patch.object(teleop.subprocess, "Popen", return_value=process) as popen,
    ):
        result = teleop.run_in_container("hardware-dry-run")

    assert result == 0
    command = popen.call_args.args[0][-1]
    assert "scripts/hardware/teleop_realman_rm65_safe_hardware.py" in command
    assert f"--arm-host {teleop.ARM_HOST}" in command
    assert "--dry-run" in command


def _fake_remote_manifest(monkeypatch, teleop_module, manifest):
    """把远端清单读取替换为固定返回值。"""
    monkeypatch.setattr(teleop_module, "_read_remote_manifest", lambda: manifest)


def test_sync_code_skips_upload_when_remote_is_current(monkeypatch):
    """远端清单与本地一致时，不应产生任何上传命令。"""
    uploaded = []
    monkeypatch.setattr(teleop, "TAR_PATH", "tar")
    monkeypatch.setattr(teleop, "SSH_PATH", "ssh")
    monkeypatch.setattr(teleop, "_load_manifest", lambda: {"files": {}, "signatures": {}})
    monkeypatch.setattr(teleop, "_iter_source_files", lambda: [])
    monkeypatch.setattr(teleop, "_upload_changed_files", lambda paths: uploaded.append(paths) or True)
    _fake_remote_manifest(monkeypatch, teleop, {"version": 1, "files": {}})

    assert teleop.sync_code() is True
    assert uploaded == []


def test_sync_code_uploads_only_changed_files(monkeypatch):
    """只上传内容变化的文件，并把新清单写回本地缓存。"""
    root = Path(teleop.LOCAL_PATH)
    changed = root / "pkg" / "changed.py"
    untouched = root / "pkg" / "untouched.py"

    uploaded = []
    saved = {}
    stale_digest = hashlib.sha256(b"value = 0\n").hexdigest()
    current_digest = hashlib.sha256(b"value = 1\n").hexdigest()
    monkeypatch.setattr(teleop, "SSH_PATH", "ssh")
    monkeypatch.setattr(teleop, "TAR_PATH", "tar")
    monkeypatch.setattr(teleop, "_iter_source_files", lambda: [changed, untouched])
    monkeypatch.setattr(teleop, "_file_signature", lambda path: "sig:" + path.name)
    monkeypatch.setattr(
        teleop,
        "_file_sha256",
        lambda path: stale_digest if path.name == "changed.py" else current_digest,
    )
    monkeypatch.setattr(
        teleop,
        "_load_manifest",
        lambda: {
            "files": {
                "pkg/changed.py": current_digest,
                "pkg/untouched.py": current_digest,
            },
            "signatures": {},
        },
    )
    monkeypatch.setattr(teleop, "_upload_changed_files", lambda paths: uploaded.append(paths) or True)
    monkeypatch.setattr(teleop, "_save_manifest", lambda manifest: saved.update(manifest))
    _fake_remote_manifest(
        monkeypatch,
        teleop,
        {
            "version": 1,
            "files": {
                "pkg/changed.py": current_digest,
                "pkg/untouched.py": current_digest,
            },
        },
    )

    assert teleop.sync_code() is True
    assert len(uploaded) == 1
    assert [path.name for path in uploaded[0]] == ["changed.py"]
    assert set(saved["files"]) == {"pkg/changed.py", "pkg/untouched.py"}
    assert saved["files"]["pkg/changed.py"] == stale_digest
    assert saved["files"]["pkg/untouched.py"] == current_digest


@pytest.mark.parametrize("remote", [None, {}, {"files": {}}, {"files": {"pkg/stable.py": "old"}}])
def test_sync_code_reuses_hash_cache_but_uploads_missing_or_stale_remote(monkeypatch, remote):
    """签名未变时不重新读取文件内容（增量同步的性能前提）。"""
    root = Path(teleop.LOCAL_PATH)
    stable = root / "pkg" / "stable.py"
    digest = hashlib.sha256(b"value = 1\n").hexdigest()

    monkeypatch.setattr(teleop, "SSH_PATH", "ssh")
    monkeypatch.setattr(teleop, "_iter_source_files", lambda: [stable])
    monkeypatch.setattr(teleop, "_file_signature", lambda path: "sig:stable")
    monkeypatch.setattr(teleop, "_read_remote_manifest", lambda: remote)
    monkeypatch.setattr(teleop, "_save_manifest", lambda manifest: None)

    def unexpected_hash(path):
        raise AssertionError("未变化的文件不应被重新读取")

    monkeypatch.setattr(
        teleop,
        "_load_manifest",
        lambda: {"files": {"pkg/stable.py": digest}, "signatures": {"pkg/stable.py": "sig:stable"}},
    )
    monkeypatch.setattr(teleop, "_file_sha256", unexpected_hash)

    uploaded = []
    monkeypatch.setattr(teleop, "_upload_changed_files", lambda paths: uploaded.append(paths) or True)

    assert teleop.sync_code() is True
    assert uploaded == [[stable]]


def test_sync_code_removes_locally_deleted_files(monkeypatch):
    """本地已删除的文件要通知远端删除，并在删除失败时判定同步失败。"""
    deleted = []
    monkeypatch.setattr(teleop, "SSH_PATH", "ssh")
    monkeypatch.setattr(teleop, "_iter_source_files", lambda: [])
    monkeypatch.setattr(
        teleop,
        "_load_manifest",
        lambda: {"files": {"gone.py": "abc"}, "signatures": {}},
    )
    monkeypatch.setattr(teleop, "_remove_remote_files", lambda items: deleted.append(items) or True)
    monkeypatch.setattr(teleop, "_save_manifest", lambda manifest: None)
    # 远端仍持有上次同步的 gone.py，因此本地删除后必须同步删除远端副本
    _fake_remote_manifest(monkeypatch, teleop, {"version": 1, "files": {"gone.py": "abc"}})

    assert teleop.sync_code() is True
    assert deleted == [[("gone.py", "abc")]]

    monkeypatch.setattr(teleop, "_remove_remote_files", lambda items: False)
    assert teleop.sync_code() is False


def test_every_launch_mode_syncs_code(monkeypatch):
    """确保每次运行脚本都会走同步流程（sync 在启动容器脚本之前执行）。"""
    order = []
    monkeypatch.setattr(teleop, "show_menu", lambda: "hardware-dry-run")
    monkeypatch.setattr(teleop, "sync_code", lambda: order.append("sync") or True)
    monkeypatch.setattr(teleop, "start_tunnel", lambda mode: order.append("tunnel") or None)
    monkeypatch.setattr(teleop, "cleanup", lambda: None)
    monkeypatch.setattr(
        teleop,
        "run_in_container",
        lambda mode, home_joint_deg=None, sim_ik_backend="placo": order.append("run") or 0,
    )

    with pytest.raises(SystemExit):
        teleop.run("hardware-dry-run")

    assert order == ["sync", "tunnel", "run"]


@pytest.mark.parametrize("failure", ["upload", "manifest"])
def test_sync_failure_does_not_commit_local_cache(monkeypatch, failure):
    root = Path(teleop.LOCAL_PATH)
    saved = []
    writes = []
    monkeypatch.setattr(teleop, "_iter_source_files", lambda: [root / "stable.py"])
    monkeypatch.setattr(teleop, "_file_signature", lambda path: "sig")
    monkeypatch.setattr(teleop, "_file_sha256", lambda path: "digest")
    monkeypatch.setattr(teleop, "_load_manifest", lambda: {"files": {}, "signatures": {}})
    monkeypatch.setattr(teleop, "_read_remote_manifest", lambda: None)
    monkeypatch.setattr(teleop, "_upload_changed_files", lambda paths: failure != "upload")
    monkeypatch.setattr(teleop, "_write_remote_manifest", lambda files: writes.append(files) or False)
    monkeypatch.setattr(teleop, "_save_manifest", lambda value: saved.append(value))
    assert teleop.sync_code() is False
    assert saved == []
    assert len(writes) == (0 if failure == "upload" else 1)


def test_sync_writes_manifest_after_upload(monkeypatch):
    root = Path(teleop.LOCAL_PATH)
    events = []
    monkeypatch.setattr(teleop, "_iter_source_files", lambda: [root / "stable.py"])
    monkeypatch.setattr(teleop, "_file_signature", lambda path: "sig")
    monkeypatch.setattr(teleop, "_file_sha256", lambda path: "digest")
    monkeypatch.setattr(teleop, "_load_manifest", lambda: {"files": {}, "signatures": {}})
    monkeypatch.setattr(teleop, "_read_remote_manifest", lambda: None)
    monkeypatch.setattr(teleop, "_upload_changed_files", lambda paths: events.append("upload") or True)
    monkeypatch.setattr(teleop, "_write_remote_manifest", lambda files: events.append(("remote", files)) or True)
    monkeypatch.setattr(teleop, "_save_manifest", lambda value: events.append("local"))
    assert teleop.sync_code() is True
    assert events == ["upload", ("remote", {"stable.py": "digest"}), "local"]


def test_remote_manifest_is_read_from_container(monkeypatch):
    seen = []

    def fake_run(cmd, **kwargs):
        seen.append(shlex.split(cmd[-1]))
        return subprocess.CompletedProcess(cmd, 0, '{"files": {}}', "")

    monkeypatch.setattr(teleop.subprocess, "run", fake_run)
    assert teleop._read_remote_manifest() == {"files": {}}
    assert seen[0] == ["sudo", "docker", "exec", teleop.CONTAINER, "cat", teleop.REMOTE_MANIFEST]


def test_remote_delete_script_preserves_modified_file_and_fails(tmp_path):
    target = tmp_path / "changed.py"
    target.write_text("remote edit", encoding="utf-8")
    script = teleop._REMOTE_DELETE_SCRIPT.format(workspace=str(tmp_path))
    result = subprocess.run(
        [sys.executable, "-c", script],
        input=json.dumps({"path": "changed.py", "sha256": "old"}) + "\n",
        text=True,
        capture_output=True,
    )
    assert result.returncode == 1
    assert "CHANGED changed.py" in result.stdout
    assert target.read_text(encoding="utf-8") == "remote edit"
