# 旧控制源码归档

2026-10-26 按仓库清理要求，将已由 MoveIt Servo 替代的实现移出工作目录。归档保存的是整理开始时的实际文件字节，包含此前未提交的修改，不用 Git HEAD 覆盖工作树。

- [source.tar.gz](source.tar.gz)：131份文件，保留原仓库相对路径。
- [manifest.json](manifest.json)：压缩包及每份文件的 SHA-256、大小和清单。

| 原路径 | 归档内容 |
| --- | --- |
| `xrobotoolkit_teleop/` | Placo、官方 IK、DLS、旧设备接口和记录实现 |
| `tests/` | 对应旧控制栈的测试 |
| `assets/` | 旧 RM65-B / RM65-official URDF 与 STL；现行模型来自锁定的 ROS 官方依赖 |
| `scripts/simulation/`、`hardware/`、`debug/` | 旧仿真、硬件、日志分析和离线模型脚本 |
| `scripts/tools/` 的部分文件 | 旧依赖安装、远程验证、Meshcat 查看和标定抓取 |
| `docker/Dockerfile`、`docker-compose.yml`、`entrypoint.sh`、`run.sh`、`install_sdk.sh` | 已替换的 Conda / Python SDK 环境 |
| `config/realman_rm65_side_mount_teleop.json` | 旧参考参数 |

查询历史实现时，先校验压缩包，再解压到独立临时目录；不要覆盖现行源码。归档保留停用状态及当时依赖，不能直接作为另一运行入口。历史文档、日志中的原路径指向本压缩包内同名成员。

```powershell
Get-FileHash docs/archive/2026-10-26-legacy-control/source.tar.gz -Algorithm SHA256
tar -tzf docs/archive/2026-10-26-legacy-control/source.tar.gz
```

本机另有 `build/archive/pre-repository-cleanup-20261026/`，保存整理前267份工作文件、Git状态、未提交差异和数据目录清单；该本机快照不随 Git 分发。本机标定 JSON、实验会话、研究材料、旧环境恢复备份和当前网络状态均保留。

当前入口和目录见[仓库结构](../../仓库结构.md)，本次操作及验证见[工作日志](../../logs/2026.10.26.md)。
