"""在远程容器内安装 h5py + pytest 并运行验证"""
import subprocess
import sys

# ~/.ssh/config 别名；该别名在 LAN 不可达时自动回退 Tailscale，实验室/宿舍通用。
# 未配置别名时可改回 "rm@192.168.0.115"（仅实验室内网可用）。
HOST = "rm"

cmds = [
    # 步骤 1：安装 h5py + pytest
    f'ssh {HOST} "sudo docker exec xrobo-vr-teleop bash -c \'pip install h5py pytest 2>&1\'"',
    # 步骤 2：运行单元测试
    f'ssh {HOST} "sudo docker exec xrobo-vr-teleop bash -c \'cd /workspace && python -m pytest tests -v 2>&1\'"',
    # 步骤 3：入口导入与 CLI 验证。真实仿真必须显式提供已标定的安全 home joint。
    f'ssh {HOST} "sudo docker exec xrobo-vr-teleop bash -c \'cd /workspace && python scripts/simulation/teleop_rm65_sim.py --help 2>&1\'"',
    # 步骤 4：检查数据文件
    f'ssh {HOST} "sudo docker exec xrobo-vr-teleop bash -c \'ls -la /workspace/logs_test/ 2>&1 || echo \\\"(无 XR 数据流时不会生成 HDF5)\\\"\'"',
]

for i, cmd in enumerate(cmds, 1):
    print(f"\n=== 步骤 {i} ===")
    print(f"$ {cmd[:80]}...")
    result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=180)
    if result.returncode == 0:
        print(result.stdout[-500:])
    else:
        print(f"返回码: {result.returncode}")
        print(result.stderr[-500:])
        print(result.stdout[-500:])

print("\n=== 验证完成 ===")
