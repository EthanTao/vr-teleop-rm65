#!/bin/bash
# 远程验证脚本 —— 数据采集系统在 Docker 容器内的验证
# 使用方法: bash scripts/tools/run_validation.sh
set -e

# ~/.ssh/config 别名；该别名在 LAN 不可达时自动回退 Tailscale，实验室/宿舍通用。
# 未配置别名时可改回 "rm@192.168.0.115"（仅实验室内网可用）。
HOST="rm"
CONTAINER="xrobo-vr-teleop"
WORKSPACE="/workspace"

echo "=== 步骤 1: 同步代码到远程板 ==="
rsync -avz --delete --exclude='.git' --exclude='__pycache__' --exclude='*.pyc' \
  --exclude='.pytest_cache' --exclude='node_modules' \
  /d/HDU-EILab/vr-teleop-rm65/ $HOST:/tmp/vr-teleop-rm65/

echo "=== 步骤 2: docker cp 到容器 ==="
ssh $HOST "sudo docker cp /tmp/vr-teleop-rm65/. $CONTAINER:$WORKSPACE/"

echo "=== 步骤 3: 安装 h5py 和 pytest ==="
ssh $HOST "sudo docker exec $CONTAINER bash -c 'pip install h5py pytest 2>&1'"

echo "=== 步骤 4: 运行完整单元测试 ==="
ssh $HOST "sudo docker exec $CONTAINER bash -c 'cd $WORKSPACE && python -m pytest tests -v 2>&1'"

echo "=== 步骤 5: 仿真入口与 CLI 验证 ==="
ssh $HOST "sudo docker exec $CONTAINER bash -c 'cd $WORKSPACE && python scripts/simulation/teleop_rm65_sim.py --help 2>&1'"

echo "=== 步骤 6: 检查生成的数据文件 ==="
ssh $HOST "sudo docker exec $CONTAINER bash -c 'ls -la $WORKSPACE/logs_test/ 2>&1 || echo \"(无 XR 数据流时不会生成 HDF5)\"'"

echo "=== 验证完成 ==="
