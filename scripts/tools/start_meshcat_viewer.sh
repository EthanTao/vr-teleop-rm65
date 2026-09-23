#!/bin/bash
# VR Teleop - Meshcat Viewer 启动脚本（Git Bash 用）
# 使用: bash scripts/tools/start_meshcat_viewer.sh

echo "============================================"
echo " VR Teleop - Meshcat 仿真可视化"
echo " 远程容器: realman (ssh 别名 rm，自动 LAN/Tailscale 回退)"
echo "============================================"
echo

# 启动 SSH 隧道（后台）
echo "[1/3] 建立 SSH 隧道..."
ssh -N -L 18081:127.0.0.1:7000 -L 63901:127.0.0.1:63901 rm &
SSH_PID=$!
echo "  SSH 隧道 PID: $SSH_PID"

# 打开浏览器
echo "[2/3] 打开浏览器..."
sleep 2
start http://127.0.0.1:18081/static/ 2>/dev/null || \
  powershell.exe start http://127.0.0.1:18081/static/

echo "[3/3] 完成！"
echo
echo "按 Ctrl+C 关闭隧道和本脚本"
echo

# 等待用户中断
trap "kill $SSH_PID 2>/dev/null; echo '隧道已关闭'; exit 0" INT TERM
wait $SSH_PID
