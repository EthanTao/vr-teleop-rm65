@echo off
chcp 65001 >nul
title VR Teleop - Meshcat Viewer

echo ============================================
echo  VR Teleop - Meshcat 仿真可视化
echo  远程容器: realman (ssh 别名 rm，自动 LAN/Tailscale 回退)
echo ============================================
echo.

:: Step 1: Check if container is running, start sim if needed
echo [1/4] 检查远程容器仿真状态...
ssh rm "sudo docker exec xrobo-vr-teleop bash -c '/opt/conda/envs/xrobo/bin/python -c \"import meshcat; print(1)\" 2>/dev/null' || echo SIM_NEED_START" > %TEMP%\vr_check.tmp
set /p SIM_STATUS=<%TEMP%\vr_check.tmp
if "%SIM_STATUS%"=="SIM_NEED_START" (
    echo   ※ 仿真未运行，正在启动...
    start "VR Simulation" cmd /c "ssh rm "sudo docker exec xrobo-vr-teleop bash -c 'cd /workspace && PYTHONPATH=/workspace /opt/conda/envs/xrobo/bin/python scripts/simulation/teleop_dual_ur5e_placo.py'" & pause"
    echo   等待 5 秒让仿真初始化...
    timeout /t 5 /nobreak >nul
) else (
    echo   ✓ 容器正常运行
)

:: Step 2: Start SSH tunnel
echo [2/4] 建立 SSH 隧道（转发 Meshcat 7000 + XR 63901）
echo       新窗口将打开，请保持运行
echo.
start "SSH Tunnel - VR Teleop" cmd /c "ssh -N -L 18081:127.0.0.1:7000 -L 63901:127.0.0.1:63901 rm & echo. & echo 隧道已关闭 && pause"

:: Step 3: Wait and open browser
echo [3/4] 等待 3 秒后打开浏览器...
timeout /t 3 /nobreak >nul

echo [4/4] 正在打开 http://127.0.0.1:18081/static/
start http://127.0.0.1:18081/static/

echo.
echo ============================================
echo  操作说明
echo ============================================
echo  ■ 浏览器页面空白？等几秒刷新即可
echo  ■ SSH 隧道窗口要一直开着（关掉后页面断连）
echo  ■ 关闭本窗口不影响 SSH 隧道
echo  ■ 连接头显后，握持 Grip 键即可驱动机械臂
echo  ■ 如需停止仿真，关掉 VR Simulation 窗口
echo ============================================
echo.
pause
