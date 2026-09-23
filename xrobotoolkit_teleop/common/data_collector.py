"""数据记录器：Grip 边界管理 + HDF5 写入。"""
from __future__ import annotations

import time
from pathlib import Path

import h5py
import numpy as np


class DataCollector:
    """记录遥操作数据到 HDF5 文件。"""

    def __init__(
        self,
        output_dir: str = "logs",
    ):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self._recording = False
        self._buffer_obs: list[dict] = []
        self._buffer_act: list[dict] = []
        self._episode_idx = 0

    def start_episode(self) -> None:
        if self._recording:
            raise RuntimeError("an episode is already being recorded")
        self._recording = True
        self._buffer_obs = []
        self._buffer_act = []

    def record_step(self, obs: dict, action: dict) -> None:
        if not self._recording:
            return
        if not isinstance(obs, dict) or not isinstance(action, dict):
            raise TypeError("obs and action must be dictionaries")
        if not obs or not action:
            raise ValueError("obs and action must not be empty")
        if self._buffer_obs and set(obs) != set(self._buffer_obs[0]):
            raise ValueError("observation keys must remain constant within an episode")
        if self._buffer_act and set(action) != set(self._buffer_act[0]):
            raise ValueError("action keys must remain constant within an episode")
        # 调用方已保证传入的值不会被外部修改，不需要额外 copy
        self._buffer_obs.append({k: np.asarray(v) for k, v in obs.items()})
        self._buffer_act.append({k: np.asarray(v) for k, v in action.items()})

    def end_episode(self) -> None:
        if not self._recording:
            return
        self._recording = False
        n = len(self._buffer_obs)
        if n == 0:
            self._buffer_obs = []
            self._buffer_act = []
            return
        obs_arr = {}
        for key in self._buffer_obs[0]:
            obs_arr[key] = np.stack([o[key] for o in self._buffer_obs])
        act_arr = {}
        for key in self._buffer_act[0]:
            act_arr[key] = np.stack([a[key] for a in self._buffer_act])
        stamp = time.strftime("%Y%m%d_%H%M%S")
        filename = self.output_dir / f"episode_{stamp}_{self._episode_idx:02d}.h5"
        self._episode_idx += 1
        def _write_group(h5file, group_name, data_dict):
            grp = h5file.create_group(group_name)
            for key, arr in data_dict.items():
                grp.create_dataset(key, data=arr)

        with h5py.File(str(filename), "w") as f:
            _write_group(f, "obs", obs_arr)
            _write_group(f, "action", act_arr)
        self._buffer_obs = []
        self._buffer_act = []
        print(f"[DATA] 已保存 {n} 帧 -> {filename}")

    @property
    def is_recording(self) -> bool:
        return self._recording
