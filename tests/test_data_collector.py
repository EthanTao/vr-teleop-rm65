import tempfile
from pathlib import Path
import h5py
import numpy as np
import pytest
from xrobotoolkit_teleop.common.data_collector import DataCollector


@pytest.fixture
def dc_and_tmpdir():
    """Provide a fresh DataCollector backed by a temporary directory."""
    with tempfile.TemporaryDirectory() as tmpdir:
        dc = DataCollector(output_dir=tmpdir)
        yield dc, tmpdir


def test_start_episode_initializes_buffers(dc_and_tmpdir):
    dc, tmpdir = dc_and_tmpdir
    dc.start_episode()
    dc.record_step(obs={"joint": np.zeros(6)}, action={"delta": np.zeros(3)})
    dc.end_episode()
    files = list(Path(tmpdir).glob("episode_*.h5"))
    assert len(files) == 1


def test_record_only_when_active(dc_and_tmpdir):
    dc, _ = dc_and_tmpdir
    dc.record_step(obs={"joint": np.zeros(6)}, action={"delta": np.zeros(3)})
    assert len(dc._buffer_obs) == 0


def test_end_episode_without_start_is_noop(dc_and_tmpdir):
    dc, tmpdir = dc_and_tmpdir
    dc.end_episode()
    files = list(Path(tmpdir).glob("episode_*.h5"))
    assert len(files) == 0


def test_recorded_data_matches_input(dc_and_tmpdir):
    dc, tmpdir = dc_and_tmpdir
    dc.start_episode()
    for i in range(5):
        dc.record_step(
            obs={"joint": np.array([i] * 6, dtype=float)},
            action={"delta": np.array([i * 0.1] * 3, dtype=float)},
        )
    dc.end_episode()
    files = list(Path(tmpdir).glob("episode_*.h5"))
    with h5py.File(str(files[0]), "r") as f:
        assert f["obs/joint"].shape == (5, 6)
        assert f["action/delta"].shape == (5, 3)
        np.testing.assert_array_equal(f["obs/joint"][2], np.array([2.0] * 6))


def test_multiple_episodes_increment_index(dc_and_tmpdir):
    dc, tmpdir = dc_and_tmpdir
    for _ in range(3):
        dc.start_episode()
        dc.record_step(obs={"j": np.zeros(3)}, action={"d": np.zeros(2)})
        dc.end_episode()
    files = sorted(Path(tmpdir).glob("episode_*.h5"))
    assert len(files) == 3
    assert "_00" in files[0].name


def test_empty_episode_not_saved(dc_and_tmpdir):
    dc, tmpdir = dc_and_tmpdir
    dc.start_episode()
    dc.end_episode()
    files = list(Path(tmpdir).glob("episode_*.h5"))
    assert len(files) == 0


def test_start_episode_rejects_nested_episode(dc_and_tmpdir):
    dc, _ = dc_and_tmpdir
    dc.start_episode()
    with pytest.raises(RuntimeError):
        dc.start_episode()


def test_record_step_rejects_changing_schema(dc_and_tmpdir):
    dc, _ = dc_and_tmpdir
    dc.start_episode()
    dc.record_step(obs={"joint": np.zeros(2)}, action={"delta": np.zeros(1)})
    with pytest.raises(ValueError):
        dc.record_step(obs={"other": np.zeros(2)}, action={"delta": np.zeros(1)})
