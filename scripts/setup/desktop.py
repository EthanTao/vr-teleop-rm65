"""Native desktop front end; all control remains in the existing launch chain."""

from __future__ import annotations

import os
from pathlib import Path
import queue
import signal
import subprocess
import sys
import threading
from types import SimpleNamespace
import webbrowser

from scripts.setup import launcher, menu

ROOT = Path(__file__).resolve().parents[2]
BOARD_WORKER = """
import sys
from scripts.setup.menu import run_board_action
raise SystemExit(run_board_action(sys.argv[1],sys.argv[2],sys.argv[3],arm_ip=sys.argv[4],udp_ip=sys.argv[5]))
"""
INTERRUPT_WORKER = """
import ctypes, sys, time
k = ctypes.WinDLL('kernel32', use_last_error=True)
k.FreeConsole()
if not k.AttachConsole(int(sys.argv[1])):
    raise ctypes.WinError(ctypes.get_last_error())
try:
    k.SetConsoleCtrlHandler(None, True)
    if not k.GenerateConsoleCtrlEvent(0, 0):
        raise ctypes.WinError(ctypes.get_last_error())
    time.sleep(0.3)
finally:
    k.FreeConsole()
"""


def python_executable() -> str:
    path = Path(sys.executable)
    console = path.with_name("python.exe")
    return str(console) if path.name.lower() == "pythonw.exe" and console.is_file() else str(path)


def simulation_command(
    kind: str, *, distro: str, address: str = "", rviz: bool = False, preview: bool = False
) -> list[str]:
    if kind not in {"sim", "pico"}:
        raise ValueError("请选择无手柄或 PICO 仿真。")
    options = ["--distro", distro]
    if kind == "pico" and address:
        options += ["--listen-address", address]
    if rviz:
        options.append("--rviz")
    # GUI text is data, not CLI syntax: invalid fields must not raise SystemExit.
    parsed = SimpleNamespace(
        distro=distro, listen_address=address if kind == "pico" else None, rviz=rviz, no_viewer=False
    )
    launcher.sim_arguments("pico" if kind == "pico" else "manual", parsed)
    if preview:
        options.append("--check-only")
    return [python_executable(), "-X", "utf8", "-u", str(ROOT / "start.py"), kind, *options]


def board_command(action: str, settings: menu.BoardSettings) -> list[str]:
    if action not in {"start-readonly", "status", "input"}:
        raise ValueError("不支持的只读操作。")
    host, _ = menu.validate_location(settings.host, settings.remote_dir)
    launcher.canonical_ipv4(settings.arm_ip)
    launcher.canonical_ipv4(settings.udp_ip)
    return [
        python_executable(),
        "-X",
        "utf8",
        "-u",
        "-c",
        BOARD_WORKER,
        action,
        host,
        settings.remote_dir,
        settings.arm_ip,
        settings.udp_ip,
    ]


def execution_command(
    settings: menu.BoardSettings, validation: str, acknowledgement: str, *, preview: bool
) -> list[str]:
    host, _ = menu.validate_location(settings.host, settings.remote_dir)
    options = [
        "--host",
        host,
        "--remote-dir",
        settings.remote_dir,
        "--arm-ip",
        settings.arm_ip,
        "--udp-ip",
        settings.udp_ip,
        "--hardware-validation",
        validation,
        "--live-ack",
        acknowledgement,
    ]
    # Reuse the same explicit requirements; the child also verifies the field record.
    launcher.real_arguments(
        SimpleNamespace(
            host=host,
            remote_dir=settings.remote_dir,
            arm_ip=settings.arm_ip,
            udp_ip=settings.udp_ip,
            hardware_validation=validation,
            live_ack=acknowledgement,
            dry_run=False,
        )
    )
    if preview:
        options.append("--check-only")
    return [python_executable(), "-X", "utf8", "-u", str(ROOT / "start.py"), "real", *options]


class SessionTask:
    """One owned process console, with bounded log delivery and graceful interrupt."""

    def __init__(self, command: list[str], events: queue.Queue):
        self.events = events
        self.interrupt_requested = False
        options = {}
        if os.name == "nt":
            info = subprocess.STARTUPINFO()
            info.dwFlags |= subprocess.STARTF_USESHOWWINDOW
            info.wShowWindow = 0
            options.update(creationflags=subprocess.CREATE_NEW_CONSOLE, startupinfo=info)
        else:
            options["start_new_session"] = True
        self.process = subprocess.Popen(
            command,
            cwd=ROOT,
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            encoding="utf-8",
            errors="replace",
            bufsize=1,
            env={**os.environ, "PYTHONUTF8": "1", "PYTHONUNBUFFERED": "1"},
            **options,
        )
        threading.Thread(target=self._read, daemon=True).start()

    def _emit(self, kind: str, value) -> None:
        while True:
            try:
                self.events.put_nowait((kind, value))
                return
            except queue.Full:
                if kind == "line":
                    return
                try:
                    self.events.get_nowait()
                except queue.Empty:
                    pass

    def _read(self) -> None:
        try:
            with self.process.stdout as stream:
                for line in stream:
                    self._emit("line", line[:8192])
        finally:
            code = self.process.wait()
            if self.interrupt_requested and code in {0xC000013A, -1073741510, -signal.SIGINT}:
                code = 130
            self._emit("done", code)

    def interrupt(self) -> None:
        if self.process.poll() is not None:
            return
        self.interrupt_requested = True
        try:
            if os.name == "nt":
                result = subprocess.run(
                    [python_executable(), "-c", INTERRUPT_WORKER, str(self.process.pid)],
                    capture_output=True,
                    text=True,
                    encoding="utf-8",
                    errors="replace",
                    timeout=8,
                    creationflags=subprocess.CREATE_NO_WINDOW,
                    check=False,
                )
                if result.returncode:
                    raise OSError(result.stderr.strip() or "无法向本次会话发送中断")
            else:
                os.killpg(self.process.pid, signal.SIGINT)
            self._emit("line", "[结束请求] 已向本次会话发送中断，等待进程退出。\n")
        except (OSError, subprocess.SubprocessError) as exc:
            if self.process.poll() is None:
                self.interrupt_requested = False
                self._emit("interrupt-error", str(exc))


class DesktopApp:
    """CustomTkinter sidebar/dashboard layout adapted from its MIT examples."""

    BG = "#eef2f8"
    INK = "#162238"
    MUTED = "#6a7890"
    BLUE = "#285ce6"

    def __init__(self, root):
        import customtkinter as ctk

        self.ctk, self.root = ctk, root
        self.events = queue.Queue(maxsize=4096)
        self.task = None
        self.closing = False
        self.launch_buttons, self.nav_buttons = [], []
        root.title("RM65 Studio")
        scale = root._get_window_scaling()
        width = min(1160, int(root.winfo_screenwidth() / scale) - 60)
        height = min(820, int(root.winfo_screenheight() / scale) - 80)
        root.geometry(f"{width}x{height}")
        root.minsize(min(1000, width), min(640, height))
        root.configure(fg_color=self.BG)
        self.values = {
            name: ctk.StringVar(value=value)
            for name, value in {
                "host": menu.DEFAULT_HOST,
                "remote_dir": menu.DEFAULT_REMOTE_DIR,
                "arm_ip": "192.168.10.18",
                "udp_ip": "192.168.10.100",
                "distro": "RM65-Servo",
                "address": "",
                "validation": "",
                "ack": "",
            }.items()
        }
        self.rviz = ctk.BooleanVar(value=False)
        self.state = ctk.StringVar(value="空闲 · 尚未启动任务")
        self.current = ctk.StringVar(value="选择一个实验场景")
        self._build()
        root.protocol("WM_DELETE_WINDOW", self.close)
        root.after(100, self.poll)

    def label(self, parent, text, *, size=13, color=None, bold=False, **kwargs):
        return self.ctk.CTkLabel(
            parent,
            text=text,
            text_color=color or self.INK,
            font=self.ctk.CTkFont(family="Microsoft YaHei UI", size=size, weight="bold" if bold else "normal"),
            anchor="w",
            **kwargs,
        )

    def button(self, parent, text, command, *, primary=False, danger=False, task=False, width=112):
        color = "#b83d46" if danger else self.BLUE if primary else "#eaf0fc"
        button = self.ctk.CTkButton(
            parent,
            text=text,
            command=command,
            width=width,
            height=36,
            corner_radius=8,
            fg_color=color,
            text_color="white" if primary or danger else self.BLUE,
            hover_color="#a2303a" if danger else "#1e4fcc" if primary else "#dce6fb",
            font=self.ctk.CTkFont(family="Microsoft YaHei UI", size=12),
        )
        if task:
            self.launch_buttons.append(button)
        return button

    def _build(self):
        ctk = self.ctk
        self.root.grid_columnconfigure(1, weight=1)
        self.root.grid_rowconfigure(0, weight=1)
        side = ctk.CTkFrame(self.root, width=196, corner_radius=0, fg_color="#142239")
        side.grid(row=0, column=0, sticky="nsew")
        side.grid_rowconfigure(5, weight=1)
        self.label(side, "R / 65", size=34, bold=True, color="#8aafff").grid(
            row=0, column=0, padx=22, pady=(30, 4), sticky="w"
        )
        self.label(side, "RM65 STUDIO", size=17, bold=True, color="white").grid(row=1, column=0, padx=22, sticky="w")
        self.label(side, "机器人实验控制台", size=11, color="#91a6c5").grid(
            row=2, column=0, padx=22, pady=(3, 25), sticky="w"
        )
        nav = ctk.CTkFrame(side, fg_color="transparent")
        nav.grid(row=3, column=0, sticky="ew", padx=10)
        for index, text in enumerate(["01    仿真实验", "02    真机连接", "03    连接设置"]):
            button = ctk.CTkButton(
                nav,
                text=text,
                command=lambda i=index: self.select(i),
                width=176,
                height=42,
                anchor="w",
                fg_color="transparent",
                text_color="#c3d0e4",
                hover_color="#294878",
                corner_radius=8,
                font=ctk.CTkFont(family="Microsoft YaHei UI", size=13),
            )
            button.grid(row=index, column=0, pady=4)
            self.nav_buttons.append(button)
        self.label(side, "MOVEIT SERVO\n单一控制链", size=11, color="#91a6c5", justify="left").grid(
            row=6, column=0, sticky="w", padx=22, pady=24
        )

        main = ctk.CTkFrame(self.root, fg_color="transparent")
        main.grid(row=0, column=1, sticky="nsew", padx=20, pady=18)
        main.grid_columnconfigure(0, weight=1)
        main.grid_rowconfigure(2, weight=1)
        header = ctk.CTkFrame(main, fg_color="transparent")
        header.grid(row=0, column=0, sticky="ew", pady=(0, 12))
        header.grid_columnconfigure(0, weight=1)
        self.label(header, "实验工作台", size=24, bold=True).grid(row=0, column=0, sticky="w")
        self.label(header, "仿真、设备连接与运行记录，在一个工作台完成。", size=11, color=self.MUTED).grid(
            row=1, column=0, sticky="w"
        )
        self.stop_button = self.button(header, "结束当前会话", self.interrupt)
        self.stop_button.grid(row=0, column=1, rowspan=2, padx=(8, 0))
        self.stop_button.configure(state="disabled")
        summary = ctk.CTkFrame(main, fg_color="transparent")
        summary.grid(row=1, column=0, sticky="ew", pady=(0, 12))
        for index, (title, variable) in enumerate([("任务状态", self.state), ("当前任务", self.current)]):
            summary.grid_columnconfigure(index, weight=1)
            card = ctk.CTkFrame(summary, corner_radius=12, fg_color="white")
            card.grid(row=0, column=index, sticky="ew", padx=(0, 10 if index == 0 else 0))
            self.label(card, title, size=10, color=self.MUTED).pack(anchor="w", padx=16, pady=(9, 0))
            ctk.CTkLabel(
                card,
                textvariable=variable,
                text_color=self.INK,
                font=ctk.CTkFont(family="Microsoft YaHei UI", size=14, weight="bold"),
            ).pack(anchor="w", padx=16, pady=(0, 9))
        holder = ctk.CTkFrame(main, fg_color="transparent")
        holder.grid(row=2, column=0, sticky="nsew")
        holder.grid_columnconfigure(0, weight=1)
        holder.grid_rowconfigure(0, weight=1)
        self.pages = [ctk.CTkScrollableFrame(holder, fg_color=self.BG, corner_radius=0) for _ in range(3)]
        self.sim_page, self.real_page, self.settings_page = self.pages
        self._simulation()
        self._hardware()
        self._settings()
        self._logs(main)
        self.select(0)
        self.write("欢迎使用 RM65 Studio。选择实验场景后启动；打开窗口不会连接设备。\n")

    def _simulation(self):
        ctk = self.ctk
        for index, (kind, title, subtitle, body) in enumerate(
            [
                (
                    "sim",
                    "无手柄仿真",
                    "MANUAL SIMULATION",
                    "通过浏览器按钮或键盘控制机械臂。\n适合调试控制链，无需头显或机器人。",
                ),
                (
                    "pico",
                    "PICO 遥操作仿真",
                    "PICO SIMULATION",
                    "使用真实手柄操作本机 MuJoCo。\n按启动日志显示的地址连接头显。",
                ),
            ]
        ):
            self.sim_page.grid_columnconfigure(index, weight=1)
            card = ctk.CTkFrame(self.sim_page, fg_color="white", corner_radius=14)
            card.grid(row=0, column=index, sticky="nsew", padx=(0, 10 if index == 0 else 0), pady=2)
            self.label(card, subtitle, size=10, color=self.BLUE, bold=True).pack(anchor="w", padx=18, pady=(18, 6))
            self.label(card, title, size=19, bold=True).pack(anchor="w", padx=18)
            self.label(card, body, size=12, color=self.MUTED, justify="left", wraplength=340).pack(
                anchor="w", padx=18, pady=(8, 18)
            )
            row = ctk.CTkFrame(card, fg_color="transparent")
            row.pack(anchor="w", padx=18, pady=(0, 18))
            self.button(
                row, "启动仿真", lambda k=kind: self.start_sim(k, False), primary=True, task=True, width=108
            ).pack(side="left", padx=(0, 8))
            self.button(row, "离线预览", lambda k=kind: self.start_sim(k, True), task=True, width=104).pack(side="left")
        row = ctk.CTkFrame(self.sim_page, fg_color="transparent")
        row.grid(row=1, column=0, columnspan=2, sticky="w", pady=(12, 0))
        self.button(row, "打开无手柄控制页面 ↗", lambda: webbrowser.open("http://127.0.0.1:8766"), width=180).pack(
            side="left"
        )
        self.label(row, "服务就绪后打开；MuJoCo 窗口按 R 复位。", size=10, color=self.MUTED).pack(side="left", padx=12)

    def _hardware(self):
        ctk = self.ctk
        self.real_page.grid_columnconfigure(0, weight=1)
        card = ctk.CTkFrame(self.real_page, fg_color="white", corner_radius=14)
        card.grid(row=0, column=0, sticky="ew", pady=2)
        self.label(card, "真机连接与观察", size=17, bold=True).pack(anchor="w", padx=18, pady=(14, 4))
        self.label(card, "只读连接不要求头显在线；查询结果显示在下方日志中。", size=11, color=self.MUTED).pack(
            anchor="w", padx=18
        )
        row = ctk.CTkFrame(card, fg_color="transparent")
        row.pack(anchor="w", padx=18, pady=(12, 14))
        for text, action in [
            ("连接（只读）", "start-readonly"),
            ("查询真实反馈", "status"),
            ("检查 PICO 输入", "input"),
        ]:
            self.button(
                row,
                text,
                lambda a=action: self.start_board(a),
                primary=action == "start-readonly",
                task=True,
                width=140,
            ).pack(side="left", padx=(0, 8))
        live = ctk.CTkFrame(self.real_page, fg_color="white", corner_radius=14)
        live.grid(row=1, column=0, sticky="ew", pady=(10, 0))
        live.grid_columnconfigure(1, weight=1)
        self.label(live, "正式执行", size=15, bold=True).grid(row=0, column=0, sticky="w", padx=18, pady=(14, 8))
        self.label(live, "需要现场验收记录与本人输入确认。", size=11, color=self.MUTED).grid(
            row=0, column=1, columnspan=2, sticky="w"
        )
        self.label(live, "验收文件", size=12).grid(row=1, column=0, sticky="w", padx=18)
        ctk.CTkEntry(live, textvariable=self.values["validation"], height=34).grid(
            row=1, column=1, sticky="ew", padx=(0, 10)
        )
        self.button(live, "选择文件", self.browse_validation, width=100).grid(row=1, column=2, padx=(0, 18))
        self.label(live, "人工确认", size=12).grid(row=2, column=0, sticky="w", padx=18, pady=8)
        ctk.CTkEntry(live, textvariable=self.values["ack"], height=34).grid(
            row=2, column=1, sticky="ew", padx=(0, 10), pady=8
        )
        self.label(live, launcher.ACK, size=10, color=self.MUTED).grid(row=3, column=1, sticky="w")
        actions = ctk.CTkFrame(live, fg_color="transparent")
        actions.grid(row=4, column=1, columnspan=2, sticky="e", padx=18, pady=(10, 14))
        self.button(actions, "离线校验", lambda: self.start_real(True), task=True).pack(side="left", padx=(0, 8))
        self.button(actions, "开始执行", lambda: self.start_real(False), danger=True, task=True).pack(side="left")

    def _settings(self):
        ctk = self.ctk
        self.settings_page.grid_columnconfigure(0, weight=1)
        card = ctk.CTkFrame(self.settings_page, fg_color="white", corner_radius=14)
        card.grid(row=0, column=0, sticky="nsew", pady=2)
        self.label(card, "连接设置", size=17, bold=True).grid(
            row=0, column=0, columnspan=2, sticky="w", padx=18, pady=(14, 8)
        )
        fields = [
            ("板端地址 / SSH 别名", "host"),
            ("板端项目目录", "remote_dir"),
            ("机械臂控制器 IP", "arm_ip"),
            ("板端反馈网口 IP", "udp_ip"),
            ("WSL 环境名称", "distro"),
            ("PICO 仿真本机 IP（可留空）", "address"),
        ]
        for index, (text, name) in enumerate(fields):
            row, column = 1 + (index // 2) * 2, index % 2
            self.label(card, text, size=11, color=self.MUTED).grid(
                row=row, column=column, sticky="w", padx=18, pady=(5, 3)
            )
            ctk.CTkEntry(card, textvariable=self.values[name], height=34).grid(
                row=row + 1, column=column, sticky="ew", padx=18, pady=(0, 8)
            )
        card.grid_columnconfigure((0, 1), weight=1)
        ctk.CTkCheckBox(
            card,
            text="仿真时打开 RViz 调试窗口",
            variable=self.rviz,
            font=ctk.CTkFont(family="Microsoft YaHei UI", size=11),
        ).grid(row=7, column=0, sticky="w", padx=18, pady=(8, 16))
        self.label(card, "设置仅在本次窗口内生效。", size=10, color=self.MUTED).grid(
            row=7, column=1, sticky="e", padx=18
        )

    def _logs(self, parent):
        ctk = self.ctk
        panel = ctk.CTkFrame(parent, fg_color="white", corner_radius=12)
        panel.grid(row=3, column=0, sticky="ew", pady=(12, 0))
        panel.grid_columnconfigure(0, weight=1)
        row = ctk.CTkFrame(panel, fg_color="transparent")
        row.grid(row=0, column=0, sticky="ew", padx=14, pady=(10, 6))
        self.label(row, "运行日志", size=13, bold=True).pack(side="left")
        self.button(row, "打开日志目录", self.open_logs, width=110).pack(side="right")
        self.button(row, "清空显示", self.clear_logs, width=90).pack(side="right", padx=8)
        self.log = ctk.CTkTextbox(
            panel,
            height=118,
            fg_color="#101c30",
            text_color="#d1dff4",
            font=ctk.CTkFont(family="Consolas", size=11),
            corner_radius=8,
            wrap="word",
            state="disabled",
        )
        self.log.grid(row=1, column=0, sticky="ew", padx=12, pady=(0, 12))
        self.label(
            parent, "任务状态表示启动进程状态；实体停止仍需核对真实反馈或现场急停。", size=10, color=self.MUTED
        ).grid(row=4, column=0, sticky="w", pady=(6, 0))

    def select(self, index):
        for page in self.pages:
            page.grid_remove()
        self.pages[index].grid(row=0, column=0, sticky="nsew")
        for position, button in enumerate(self.nav_buttons):
            button.configure(
                fg_color="#294878" if position == index else "transparent",
                text_color="white" if position == index else "#c3d0e4",
            )

    def settings(self):
        return menu.BoardSettings(
            **{name: self.values[name].get().strip() for name in ("host", "remote_dir", "arm_ip", "udp_ip")}
        )

    def start_sim(self, kind, preview):
        self.launch(
            "PICO 仿真" if kind == "pico" else "无手柄仿真",
            lambda: simulation_command(
                kind,
                distro=self.values["distro"].get().strip(),
                address=self.values["address"].get().strip(),
                rviz=self.rviz.get(),
                preview=preview,
            ),
            preview=preview,
        )

    def start_board(self, action):
        title = {"start-readonly": "真机只读连接", "status": "真实反馈查询", "input": "PICO 输入查询"}[action]
        self.launch(title, lambda: board_command(action, self.settings()))

    def start_real(self, preview):
        self.launch(
            "真机正式执行",
            lambda: execution_command(
                self.settings(),
                self.values["validation"].get().strip(),
                self.values["ack"].get(),
                preview=preview,
            ),
            preview=preview,
        )
        if not preview:
            self.values["ack"].set("")

    def launch(self, title, build_command, *, preview=False):
        if self.task is not None:
            self.write("[提示] 请先结束当前任务，再启动另一个任务。\n")
            return
        try:
            command = build_command()
            self.task = SessionTask(command, self.events)
        except (OSError, ValueError) as exc:
            self.state.set("参数或启动失败")
            self.write(f"[未启动] {exc}\n")
            return
        self.current.set(title + (" · 离线预览" if preview else ""))
        self.state.set("任务运行中")
        self.write(f"\n── {self.current.get()} ──\n")
        for button in self.launch_buttons:
            button.configure(state="disabled")
        self.stop_button.configure(state="normal")

    def interrupt(self):
        if self.task is not None:
            self.state.set("正在请求结束…")
            self.stop_button.configure(state="disabled")
            threading.Thread(target=self.task.interrupt, daemon=True).start()

    def poll(self):
        for _ in range(200):
            try:
                kind, value = self.events.get_nowait()
            except queue.Empty:
                break
            if kind == "line":
                self.write(value)
            elif kind == "interrupt-error":
                self.write(f"[结束请求失败] {value}\n")
                self.state.set("结束请求失败 · 请查看日志")
                self.stop_button.configure(state="normal")
                self.closing = False
            elif kind == "done":
                self.task = None
                self.state.set(
                    "任务已结束" if value == 0 else "本次任务已中断" if value == 130 else f"任务已退出 · {value}"
                )
                self.write(f"[进程退出] 返回码 {value}\n")
                for button in self.launch_buttons:
                    button.configure(state="normal")
                self.stop_button.configure(state="disabled")
                if self.closing:
                    self.root.destroy()
                    return
        self.root.after(100, self.poll)

    def write(self, text):
        self.log.configure(state="normal")
        self.log.insert("end", text)
        if int(self.log.index("end-1c").split(".")[0]) > 2000:
            self.log.delete("1.0", "500.0")
        self.log.see("end")
        self.log.configure(state="disabled")

    def clear_logs(self):
        self.log.configure(state="normal")
        self.log.delete("1.0", "end")
        self.log.configure(state="disabled")

    def open_logs(self):
        folder = ROOT / "build/logs/startup"
        folder.mkdir(parents=True, exist_ok=True)
        if os.name == "nt":
            os.startfile(folder)
        else:
            webbrowser.open(folder.as_uri())

    def browse_validation(self):
        from tkinter import filedialog

        name = filedialog.askopenfilename(
            parent=self.root,
            title="选择现场验收文件",
            initialdir=ROOT / "ros2_ws/src/rm65_servo_teleop/config",
            filetypes=[("YAML 文件", "*.yaml *.yml"), ("所有文件", "*.*")],
        )
        if name:
            self.values["validation"].set(name)

    def close(self):
        from tkinter import messagebox

        if self.task is None:
            self.root.destroy()
        elif messagebox.askyesno(
            "结束本次任务", "结束当前会话后关闭窗口？\n实体停止仍须核对真实反馈。", parent=self.root
        ):
            self.closing = True
            self.interrupt()


def main() -> int:
    try:
        import customtkinter as ctk
    except ImportError:
        print('桌面依赖未安装，请执行：python -m pip install -e ".[desktop]"')
        return 2
    ctk.set_appearance_mode("Light")
    ctk.set_default_color_theme("blue")
    root = ctk.CTk()
    DesktopApp(root)
    root.mainloop()
    return 0
