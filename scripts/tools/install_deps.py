"""在 Docker 容器内安装 meshcat + placo 等缺失依赖"""
import subprocess, sys

cmd = [
    # ~/.ssh/config 别名（LAN 不可达时自动回退 Tailscale）；未配置时可改回 "rm@192.168.0.115"
    "ssh", "rm",
    "sudo docker exec xrobo-vr-teleop bash -c "
    "'pip install meshcat placo tyro numpy -i https://pypi.tuna.tsinghua.edu.cn/simple 2>&1'"
]

p = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
if p.returncode == 0:
    print(p.stdout[-1000:])
else:
    print(f"RET={p.returncode}")
    print(p.stderr[-500:])
    print(p.stdout[-500:])
