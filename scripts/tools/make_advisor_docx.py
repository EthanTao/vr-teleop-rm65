# -*- coding: utf-8 -*-
"""一次性脚本：生成面向导师的 .docx 方案文档。"""
from docx import Document
from docx.shared import Pt, Cm, RGBColor
from docx.enum.text import WD_ALIGN_PARAGRAPH, WD_BREAK
from docx.enum.table import WD_TABLE_ALIGNMENT
from docx.oxml.ns import qn
from docx.oxml import OxmlElement
from pathlib import Path

OUT = str(Path(__file__).resolve().parents[2] / "docs" / "research" / "数据集主线方案-给导师-2026.9.24.docx")

doc = Document()

# ---------- 页面 ----------
for s in doc.sections:
    s.top_margin = Cm(2.2)
    s.bottom_margin = Cm(2.2)
    s.left_margin = Cm(2.4)
    s.right_margin = Cm(2.4)

# ---------- 字体工具 ----------
def set_run(run, ea="宋体", ascii_="Times New Roman", size=10.5, bold=None, color=None):
    run.font.name = ascii_
    run.font.size = Pt(size)
    if bold is not None:
        run.font.bold = bold
    if color is not None:
        run.font.color.rgb = color
    rPr = run._element.get_or_add_rPr()
    rFonts = rPr.find(qn("w:rFonts"))
    if rFonts is None:
        rFonts = OxmlElement("w:rFonts")
        rPr.append(rFonts)
    rFonts.set(qn("w:ascii"), ascii_)
    rFonts.set(qn("w:hAnsi"), ascii_)
    rFonts.set(qn("w:eastAsia"), ea)

# 正文默认样式
normal = doc.styles["Normal"]
normal.font.name = "Times New Roman"
normal.font.size = Pt(10.5)
normal._element.rPr.rFonts.set(qn("w:eastAsia"), "宋体")
normal.paragraph_format.line_spacing = 1.35
normal.paragraph_format.space_after = Pt(4)

HEAD_SPEC = {1: ("黑体", 16), 2: ("黑体", 13.5), 3: ("黑体", 11.5)}

def H(text, level=1):
    p = doc.add_heading("", level=level)
    ea, size = HEAD_SPEC[level]
    r = p.add_run(text)
    set_run(r, ea=ea, ascii_="Arial", size=size, bold=True)
    p.paragraph_format.space_before = Pt(14 if level == 1 else 10)
    p.paragraph_format.space_after = Pt(6)
    return p

def P(text="", bold=False, size=10.5, indent=0, align=None, color=None, ea="宋体"):
    p = doc.add_paragraph()
    if indent:
        p.paragraph_format.left_indent = Cm(indent)
    if align:
        p.alignment = align
    if text:
        r = p.add_run(text)
        set_run(r, ea=ea, size=size, bold=bold, color=color)
    return p

def BULLET(text, level=0):
    p = doc.add_paragraph(style="List Bullet")
    p.paragraph_format.left_indent = Cm(0.75 + 0.6 * level)
    p.paragraph_format.space_after = Pt(2)
    r = p.add_run(text)
    set_run(r, size=10.5)
    return p

def RICH(parts, indent=0):
    """parts: [(text, bold, color_or_None), ...]"""
    p = doc.add_paragraph()
    if indent:
        p.paragraph_format.left_indent = Cm(indent)
    for t, b, c in parts:
        r = p.add_run(t)
        set_run(r, size=10.5, bold=b, color=c)
    return p

def shade(cell, hexcolor):
    tcPr = cell._tc.get_or_add_tcPr()
    shd = OxmlElement("w:shd")
    shd.set(qn("w:val"), "clear")
    shd.set(qn("w:color"), "auto")
    shd.set(qn("w:fill"), hexcolor)
    tcPr.append(shd)

def TABLE(headers, rows, widths=None, fontsize=9.5):
    t = doc.add_table(rows=1, cols=len(headers))
    t.style = "Table Grid"
    t.alignment = WD_TABLE_ALIGNMENT.CENTER
    t.autofit = False
    tblPr = t._tbl.tblPr
    layout = OxmlElement("w:tblLayout")
    layout.set(qn("w:type"), "fixed")
    tblPr.append(layout)
    hdr = t.rows[0].cells
    for i, htext in enumerate(headers):
        hdr[i].text = ""
        r = hdr[i].paragraphs[0].add_run(htext)
        set_run(r, ea="黑体", ascii_="Arial", size=fontsize, bold=True)
        shade(hdr[i], "E8EEF7")
        hdr[i].paragraphs[0].paragraph_format.space_after = Pt(1)
    for row in rows:
        cells = t.add_row().cells
        for i, val in enumerate(row):
            cells[i].text = ""
            first = True
            for line in str(val).split("\n"):
                para = cells[i].paragraphs[0] if first else cells[i].add_paragraph()
                first = False
                para.paragraph_format.space_after = Pt(1)
                r = para.add_run(line)
                set_run(r, size=fontsize)
    if widths:
        for row in t.rows:
            for i, w in enumerate(widths):
                row.cells[i].width = Cm(w)
    doc.add_paragraph().paragraph_format.space_after = Pt(2)
    return t

def NOTE(text):
    p = doc.add_paragraph()
    p.paragraph_format.left_indent = Cm(0.4)
    p.paragraph_format.space_before = Pt(4)
    p.paragraph_format.space_after = Pt(8)
    r = p.add_run(text)
    set_run(r, ea="楷体", size=10, color=RGBColor(0x44, 0x44, 0x44))
    return p

def PAGEBREAK():
    doc.add_paragraph().add_run().add_break(WD_BREAK.PAGE)

# ============================================================
# 封面区
# ============================================================
p = doc.add_paragraph()
p.alignment = WD_ALIGN_PARAGRAPH.CENTER
r = p.add_run("VR 遥操作 RM65 项目")
set_run(r, ea="黑体", ascii_="Arial", size=20, bold=True)
p.paragraph_format.space_before = Pt(30)
p.paragraph_format.space_after = Pt(2)

p = doc.add_paragraph()
p.alignment = WD_ALIGN_PARAGRAPH.CENTER
r = p.add_run("论文路线调研与数据集主线方案")
set_run(r, ea="黑体", ascii_="Arial", size=16, bold=True)
p.paragraph_format.space_after = Pt(2)

p = doc.add_paragraph()
p.alignment = WD_ALIGN_PARAGRAPH.CENTER
r = p.add_run("—— 面向 CCF A 的可行性论证与实施计划")
set_run(r, ea="楷体", size=12, color=RGBColor(0x55, 0x55, 0x55))
p.paragraph_format.space_after = Pt(18)

TABLE(
    ["项目", "内容"],
    [
        ["文档日期", "2026-09-24"],
        ["项目仓库", "vr-teleop-rm65（PICO 头显 + XRoboToolkit → Realman RM65 六轴机械臂）"],
        ["调研范围", "2023–2025 年 VR 遥操作、演示数据采集、控制层共 5 个子方向，约 45 篇代表性工作逐条核验"],
        ["当前生效 CCF 目录", "第七版（2026-03-31 发布，2026-04-09 勘误）"],
        ["本方案目标", "以「构建并发表遥操作演示数据集」为主线，冲击 CCF A"],
    ],
    widths=[3.6, 12.4],
    fontsize=10,
)

# ============================================================
H("一、摘要", 1)

P("本项目已完成一套可运行的 VR 遥操作工程系统（仿真 + 真机两条链路，含侧装映射、阻尼最小二乘局部逆解、"
  "奇异点保护与故障锁存安全层），工程完成度较高。但经系统调研后得到一个必须正视的判断：", bold=False)
RICH([("按现状无法发表 CCF A 论文。缺的不是工程量，而是一个「别人还不知道的结论」。", True, RGBColor(0xB0, 0x30, 0x20))])
P("调研得出以下五条结论：")

TABLE(
    ["#", "结论", "依据"],
    [
        ["1", "机器人赛道本身没有 CCF A。ICRA 是 CCF B，IROS 是 CCF C，IEEE T-RO 是 B，"
              "RA-L / RSS / CoRL 均不在 CCF 目录内。",
              "第七版目录逐条核实"],
        ["2", "CCF A 的真实入口只有三条：HCI/VR（CHI、UIST、IEEE VR、TVCG）、"
              "网络（INFOCOM、MobiCom、JSAC、TMC）、形式化与软工（CAV、IEEE TSE）。",
              "第七版目录"],
        ["3", "在 CCF A 中，「数据集」路线的门比「系统」路线更宽：NeurIPS 专设 Datasets & Benchmarks track，"
              "CVPR / ICML / ICLR 均接收数据集与基准类工作。",
              "NeurIPS 官方 FAQ；RoboTwin = CVPR 2025 Highlight；RoboCasa365 = ICLR 2026"],
        ["4", "但本项目的现有数据规模与头部数据集相差 2–3 个数量级，硬拼规模不可行，"
              "必须换一条「别人没有的轴」来竞争。",
              "见 §4.1 规模对照"],
        ["5", "已核实出三条空白轴：PICO 头显、机械臂安装朝向、运动学约束事件标注。"
              "其中后两条足以支撑一个明确的科学问题。",
              "约 45 篇逐条核验 + 数据集核验报告"],
    ],
    widths=[0.9, 11.0, 4.1],
    fontsize=9.5,
)

P("本方案的核心科学问题：", bold=True)
p = doc.add_paragraph()
p.paragraph_format.left_indent = Cm(0.8)
p.paragraph_format.space_after = Pt(8)
r = p.add_run("当机器人本体不变、仅安装朝向改变时，学到的策略会失效吗？"
              "如果失效，主因是视觉（操作者看到的世界转了）还是动作空间（基座坐标系与重力方向变了）？")
set_run(r, ea="楷体", size=11.5, bold=True)

P("本方案需要导师确认三件事（详见 §6）：", bold=True)
BULLET("Datasets & Benchmarks track 的论文，本校是否认定为 CCF A 会议论文；")
BULLET("RM65 是否具备更换安装朝向的物理条件（这是本数据集唯一的核心自变量）；")
BULLET("是否可提供至少 1 张用于策略训练的 GPU，以及 2–3 人的人力配置。")

# ============================================================
H("二、项目现状", 1)

H("2.1 已完成的工作", 2)
TABLE(
    ["模块", "状态", "说明"],
    [
        ["远程 Docker 仿真环境", "已完成", "容器 xrobo-vr-teleop，Meshcat 可视化经 SSH 隧道"],
        ["侧装专用映射矩阵", "已完成并冻结", "仿真入口与真机控制器共用同一矩阵"],
        ["6-DoF 姿态跟随（仿真）", "已完成", "含死区 + 指数平滑滤波、关节约束、B 键复位"],
        ["DLS 局部逆解 + 奇异点保护", "已完成", "SVD 阻尼最小二乘、J3/J5 禁止穿零、硬区保持、有界计算预算"],
        ["真机安全层", "代码就绪", "UDP 实时反馈、速度/加速度限幅、超速与超时判定、故障锁存与使能状态机"],
        ["数据采集基础", "部分完成", "HDF5 采集器，以 Grip 按下/松开为 episode 边界"],
        ["自动化测试", "已完成", "407 项测试通过"],
    ],
    widths=[4.2, 2.6, 9.2],
)

H("2.2 项目中具有科研价值的实测发现", 2)
P("这些是别的团队没有机会观察到的现象，是后续论文选题的实际来源：")
BULLET("URDF 模型的正运动学结果与真机控制器上报的 TCP 位姿偏差达 660–1300 mm；根因已定位到末端杆件参数"
       "的型号差异（d6 = 161.2 mm  vs  SDK 默认 144 mm）以及工作坐标系安装姿态。")
BULLET("XRoboToolkit 的世界坐标系是在应用启动瞬间锚定的，因此操作者躯干走动或前倾会污染输入信号，"
       "而单纯转头不会。固定的世界锚定参考系下，这是固有行为，目前尚未采用补偿方案。")
BULLET("侧装工况下映射矩阵的方向是否符合人体直觉，代码推导与一次仿真口述观测在 X/Y 两轴上正好互换，"
       "真机观测栏至今为空 —— 即方向问题尚未实测定案。")

H("2.3 尚未完成的部分（硬阻塞）", 2)
RICH([("真机验收从未完成。", True, RGBColor(0xB0, 0x30, 0x20)),
      ("执行关节跟随指令的完整链路仅在容器内完成过 1000 周期的空跑时序验证"
       "（整链中位数 10.2 ms、最大 14.4 ms，预算 18–20 ms），真实运动下发、完整主循环、"
       "机械臂实际跟踪与物理安全尚未验证。", False, None)])
P("此外：数据采集目前仅覆盖仿真路径，无相机图像、无夹爪数据；尚无任何对照实验、被试数据或学习结果。")

# ============================================================
PAGEBREAK()
H("三、调研结论：为什么必须换赛道", 1)

H("3.1 CCF A 目录事实（第七版，2026-03-31 发布）", 2)
TABLE(
    ["去处", "等级", "备注"],
    [
        ["IEEE VR", "A 类会议", "归入「计算机图形学与多媒体」；该方向 A 类会议全套仅 4 个：ACM MM、SIGGRAPH、IEEE VR、IEEE VIS"],
        ["IEEE TVCG", "A 类期刊", "同方向。IEEE VR 主赛道论文即发表于 TVCG 专刊，一次投稿可同时获得 A 类会议与 A 类期刊"],
        ["CHI / UIST / UbiComp / CSCW", "A 类会议", "人机交互与普适计算"],
        ["CVPR / ICCV / NeurIPS / ICML / AAAI / ACL", "A 类会议", "人工智能"],
        ["ICLR", "A 类会议", "第七版新增并直接进入 A 类"],
        ["IJCAI", "B 类会议", "第七版由 A 降为 B，不建议投"],
        ["NeurIPS Datasets & Benchmarks", "A 类会议的独立 track", "数据集类工作的正式去处，认定口径见 §6"],
        ["ICRA", "B 类会议", "注：通常易被误认为 A"],
        ["IROS", "C 类会议", ""],
        ["IEEE T-RO", "B 类期刊", ""],
        ["RA-L / RSS / CoRL", "不在 CCF 目录内", "RSS 的 CORE 等级为 A*，但 CCF 不予收录"],
    ],
    widths=[4.6, 3.2, 8.2],
)

H("3.2 本方向的发表版图（约 45 篇逐条核验）", 2)
TABLE(
    ["会议", "核验到的篇数", "说明"],
    [
        ["CoRL", "约 20 篇", "第一主场（2024 = PMLR v270，2025 = PMLR v305）"],
        ["RSS", "约 12 篇", "第二主场"],
        ["ICRA", "4 篇", "含 Open X-Embodiment（ICRA 2024 最佳会议论文）"],
        ["IROS", "3 篇", "GELLO、Bunny-VisionPro、Haptic-ACT"],
        ["CVPR / ICML / ICLR", "各 1 篇", "RoboTwin（CVPR 2025 Highlight）、RoboTwin 2.0、RoboCasa365"],
        ["IEEE VR / ISMAR", "各 1 篇", "人机交互支线，问题意识与机器人社区不同"],
        ["NeurIPS 主轨 / D&B", "本轮未核验到", "属覆盖缺口，不等于没有"],
    ],
    widths=[4.2, 3.0, 8.8],
)
NOTE("关键含义：这条研究线约七成的工作发表在 CoRL 与 RSS —— 而这两个会议都不在 CCF 目录内。"
     "CVPR / NeurIPS 只在「数据集与基准」和「跨本体算法」两个口子上接收本方向工作。"
     "因此「投数据集」这一选择本身，就是把研究方向对齐到 CCF A 的少数可行通道之一。")

H("3.3 一条必须正面处理的反方证据", 2)
P("调研发现，VR 头显是否是最优的演示采集接口，在 2025 年仍未定论，且有顶会论文给出明确反对证据：")
BULLET("BRS（CoRL 2025）刻意采用非 VR 的 JoyLo 接口（Joy-Con 手柄 + 运动学同构主手），"
       "其 10 人用户研究报告该接口在成功率、耗时与奇异率上均优于 VR 手柄与 Apple Vision Pro。")
BULLET("Bidex（CoRL 2024）同样报告其方案优于 Vision Pro 与 SteamVR 基线。")
BULLET("GELLO（IROS 2024）12 人用户研究显示主从同构臂在演示采集上优于 VR 手柄。")
RICH([("影响：", True, None),
      ("论文中不能默认「VR 手感更好」，必须把接口选择当作需要论证或测量的对象，"
       "或者将其作为既定前提而不作优劣声明。", False, None)])

# ============================================================
H("四、数据集主线：定位与科学问题", 1)

H("4.1 规模对照：硬拼规模不可行", 2)
TABLE(
    ["数据集", "规模", "出处"],
    [
        ["AgiBot World", "100 万条以上轨迹 / 217 个任务", "arXiv:2503.06669"],
        ["RoboMIND", "10.7 万条轨迹 / 479 个任务 / 4 种本体，另含 5 千条失败演示", "RSS 2025，DOI 10.15607/RSS.2025.XXI.152"],
        ["DROID", "7.6 万条轨迹 / 350 小时 / 13 家机构", "RSS 2024, p120"],
        ["Open X-Embodiment", "汇聚 60 个数据集 / 22 种本体 / 100 万条以上轨迹", "ICRA 2024 最佳会议论文"],
        ["本项目现状", "HDF5 采集器已就绪，仅仿真、无相机、无夹爪、无真机采集，episode 数约为 0", "—"],
    ],
    widths=[3.8, 8.4, 3.8],
)
RICH([("差距为 2–3 个数量级，直接对标 DROID 或 RoboMIND 的路线不存在。", True, RGBColor(0xB0, 0x30, 0x20))])

H("4.2 三条已核实的空白轴", 2)
TABLE(
    ["#", "空白轴", "核实结论"],
    [
        ["1", "PICO 头显", "已核实的头部数据集分别使用 Oculus Quest 2（DROID）与 Apple Vision Pro"
                            "（Fourier ActionNet）；本轮未能核实到任何基于 PICO 的大规模数据集。"],
        ["2", "机械臂安装朝向", "现有数据集的机器人全部为规范安装（桌面或地面）。RM65 官方支持任意角度安装，"
                                "但尚无数据集把底座朝向作为受控自变量。相邻工作 Mobi-π（CoRL 2025）"
                                "研究的是移动底盘的摆放位置，并非安装朝向。"],
        ["3", "运动学约束事件标注", "调研结论：绝大多数遥操作数据集只保留成功演示，采数时的失败片段通常被丢弃，"
                                    "导致策略未见过失败、不会恢复。而本项目具备 UDP 实时遥测、"
                                    "逆解指标（奇异值比、速度缩放）与故障锁存/使能事件，"
                                    "可为每一帧自动标注运动学状态。"],
    ],
    widths=[0.9, 3.6, 11.5],
)

H("4.3 定位与科学问题", 2)
RICH([("数据集定位：", True, None),
      ("首个带安装朝向自变量与运动学约束事件标注的 VR 遥操作数据集。", False, None)])
P("数据集只是载体，决定论文能否被接收的是它所回答的问题：")
p = doc.add_paragraph()
p.paragraph_format.left_indent = Cm(0.8)
r = p.add_run("当机器人本体不变、仅安装朝向改变时，学到的策略会失效吗？"
              "主因是视觉差异，还是动作空间（基座坐标系与重力方向）差异？")
set_run(r, ea="楷体", size=11.5, bold=True)
P("该问题的优势：文献中尚无对应研究；可通过对照实验明确回答；工业动机真实"
  "（任意角度安装是现场常态）；且回答它必须依赖第 3 条空白轴的标注数据，"
  "形成他人难以复制的壁垒。")

# ============================================================
PAGEBREAK()
H("五、实施方案", 1)

H("5.1 采集规模目标（按「能回答科学问题」倒推，不对标头部数据集）", 2)
TABLE(
    ["维度", "目标", "理由"],
    [
        ["安装朝向", "3 种（水平 / 侧装 90° / 斜装 45° 或倒装）", "核心自变量，至少 3 个才能观察趋势"],
        ["任务", "6 个", "覆盖纯平移拾放、需姿态对准的插装、需绕障、接触密集、长程多步、精细对准"],
        ["每（任务 × 朝向）", "50 条成功 + 15 条含失败恢复", "便于开展数据筛选实验"],
        ["合计", "约 900 条成功 + 270 条失败恢复 ≈ 1200 条", ""],
        ["总帧数", "约 180 万帧（30 秒/条 @ 50 Hz）", "含 2 路相机"],
    ],
    widths=[3.4, 6.6, 6.0],
)
P("采集工时估算：参照已发表实测数据（UMI 手持采集 111 条/小时，SpaceMouse 35 条/小时），"
  "本项目为 VR 手柄 + 真机 + 复位，保守估计 20–40 条/小时，"
  "则 1200 条约需 30–60 小时纯操作时间。", bold=False)
RICH([("结论：数据规模在本方案下是可行的，真正的成本在调试与重置，而非操作本身。", True, None)])

H("5.2 数据模态", 2)
TABLE(
    ["数据组", "内容", "频率", "现状"],
    [
        ["本体状态", "六关节角与速度、TCP 位姿、跟踪误差、错误码", "50 Hz", "遥测已有，待落盘"],
        ["视觉", "腕部相机（随末端）+ 第三人称相机 1–2 路", "30 Hz", "待接入"],
        ["动作", "手柄增量位姿、Grip 值、实际下发关节目标、夹爪开度", "50 Hz", "部分具备（缺夹爪）"],
        ["运动学事件", "奇异值比、速度缩放比例、阻尼系数、状态标记、J3/J5 距零位、关节限位余量", "50 Hz", "变量已在逆解模块中，仅需落盘"],
        ["安全事件", "软停止 / 硬停止 / 故障锁存 / 重新使能 / 看门狗超时 / 超速判定", "事件流", "状态机已有"],
        ["元信息", "安装朝向、映射矩阵版本、缩放因子、操作者编号、时间同步信息", "每条 episode", "待建"],
    ],
    widths=[2.6, 7.4, 2.2, 3.8],
)
NOTE("优先级建议：运动学事件与安全事件的落盘成本极低（相关变量已在 damped_ik 模块中现成存在），"
     "但它是本数据集差异化的全部来源，应优先实现。")

H("5.3 下游实验（数据集论文的必要条件）", 2)
P("已核实的头部数据集工作（DROID、Open X-Embodiment、RoboMIND、AgiBot World）无一例外都训练了策略。"
  "本方案规划三个实验：")
TABLE(
    ["实验", "设计", "价值"],
    [
        ["实验一\n跨安装朝向迁移", "朝向 A 数据训练 → 在 A / B / C 测试；对照组为三朝向混合训练；"
                                  "进一步把动作表示换成安装无关表示，以分离视觉与动作空间的作用",
         "核心洞见，直接回答科学问题"],
        ["实验二\n演示质量筛选", "用运动学事件为 1200 条演示打分 → 按 100% / 50% / 33% / 25% 筛选；"
                                "绘制成功率随数据量的变化曲线",
         "对标 CUPID（CoRL 2025：使用不足 33% 的筛选数据即达到 SOTA 水平）；"
         "该实验几乎不需要额外硬件，最易产出清晰结果"],
        ["实验三\n失败恢复数据价值", "仅成功演示 vs 成功 + 失败恢复演示；测试策略在人为扰动后的恢复能力",
         "命中「多数数据集只留成功片段」这一开放问题"],
    ],
    widths=[3.0, 8.0, 5.0],
)
RICH([("评测基准的注意事项：", True, None),
      ("审计研究（arXiv:2606.04233）发现 LIBERO 既是社区最常用的仿真基准，也最容易被刷分 —— "
       "一个 0.09B 规模、不含语言编码器的探针即可达到或接近报告的最优结果，"
       "且多数论文报告的提升无法证明统计显著。因此仿真评测建议选用 RoboCasa 或 RoboTwin 2.0，"
       "主结果应以真机实验为准。", False, None)])

H("5.4 合规与交付清单", 2)
TABLE(
    ["项目", "要求", "状态"],
    [
        ["Datasheet", "按 Gebru 等提出的数据集说明书格式撰写", "待写"],
        ["Croissant 元数据", "NeurIPS D&B 的硬性要求，非加分项", "待做"],
        ["许可证", "数据建议 CC BY 4.0（或 CC BY-NC 4.0）", "待定"],
        ["托管", "HuggingFace Datasets + ModelScope 国内镜像", "待做"],
        ["长期维护计划", "D&B 明确要求", "待写"],
        ["伦理审批", "涉及人体被试（操作者），需伦理审查批件与知情同意", "需立即启动"],
        ["可复现性", "采集代码、处理脚本、训练配置、随机种子", "已有基础"],
    ],
    widths=[3.4, 9.6, 3.0],
)

# ============================================================
PAGEBREAK()
H("六、需要导师确认与支持的三件事", 1)
NOTE("以下三项不需要编写任何代码，但直接决定整份方案是否成立，建议优先处理。")

TABLE(
    ["#", "事项", "为什么关键", "建议时限"],
    [
        ["1", "NeurIPS Datasets & Benchmarks track 的论文，本校是否认定为 CCF A 会议论文？",
         "CCF 目录的原文口径是「会议论文指 Full paper 或 Regular paper」，而 D&B 是该会议下的独立 track。"
         "多数单位按 NeurIPS 计入，但也有单位仅认可主赛道。这一条决定整条路线是否成立。",
         "立即"],
        ["2", "RM65 是否具备更换安装朝向的物理条件（水平 / 侧装 90° / 斜装或倒装）？",
         "「安装朝向」是本数据集唯一的核心自变量。若物理上只能安装一种朝向，"
         "方案需整体降级为备选方案 B（见 §8）。",
         "2026 年 10 月底"],
        ["3", "能否提供至少 1 张用于策略训练的 GPU（建议显存 24 GB 以上），以及 2–3 人的人力配置？",
         "数据集论文必须包含下游策略结果，无算力即无实验，也就无法发表。",
         "2026 年 11 月底"],
    ],
    widths=[0.9, 5.4, 7.6, 2.1],
)

# ============================================================
H("七、工程前置与时间表", 1)

H("7.1 关键路径（按依赖顺序）", 2)
TABLE(
    ["#", "工作项", "依赖", "计划时间"],
    [
        ["1", "真机验收：真实执行关节跟随指令、复位、长时间运行", "—", "2026-10 ~ 12"],
        ["2", "逐轴实测定案，补全坐标系数值表", "1", "2026-10 ~ 11"],
        ["3", "伦理审批申报", "—", "立即启动，约 1–3 个月"],
        ["4", "相机接入（腕部 + 第三人称）并接入 VR 显示", "1", "2026-11 ~ 12"],
        ["5", "夹爪接入（当前采集器无夹爪数据）", "1", "2026-11"],
        ["6", "运动学事件与安全事件记录器", "1", "2026-11"],
        ["7", "异步写盘，并实测采集开启前后的控制周期时序", "6", "2026-12"],
        ["8", "真机采集链路接入（当前采集器仅挂仿真）", "4,5,6,7", "2026-12"],
        ["9", "跨安装朝向实验台：更换底座朝向、重新标定映射与工作空间、安全评估", "2, 1", "2027-01 ~ 02"],
        ["10", "端到端试点采集 50 条", "8,9", "2027-02"],
        ["11", "正式采集约 1200 条", "10", "2027-03 ~ 05"],
        ["12", "策略训练与三个实验", "11 + GPU", "2027-05 ~ 07"],
        ["13", "Datasheet、Croissant、托管上线、论文写作", "12", "2027-07 ~ 08"],
    ],
    widths=[0.9, 7.6, 2.6, 4.9],
)

H("7.2 目标会议与截稿", 2)
TABLE(
    ["优先级", "目标", "预计截稿", "备注"],
    [
        ["首选", "NeurIPS 2027 Datasets & Benchmarks", "约 2027 年 5–6 月", "数据集工作唯一的专属顶级赛道；时间较紧"],
        ["稳妥", "CVPR 2028", "约 2027 年 11 月", "接收数据集与基准类工作（RoboTwin 为 CVPR 2025 Highlight）"],
        ["稳妥", "ICLR 2028", "约 2027 年 9 月", "RoboCasa365 即走此路线"],
        ["保底\n（非 CCF A）", "RSS 2027 / 2028 Demo track", "约 2027 年 1 月 / 2028 年 1 月", "RSS 官方明确接收数据集与基准；但 RSS 不在 CCF 目录内"],
        ["副线", "IEEE VR 2028", "约 2027 年 8 月", "人因研究，见 §8"],
    ],
    widths=[2.2, 5.0, 3.6, 5.2],
)
NOTE("已错过的窗口（供参考）：CVPR 2027 全文截稿 2026-11-16；ICLR 2027 摘要截止 2026-09-18 已过"
     "且官方不允许事后增删作者；NeurIPS 2026 已关闭。")

# ============================================================
H("八、风险与备选方案", 1)
TABLE(
    ["风险", "概率", "影响", "应对"],
    [
        ["真机验收拖期", "中高", "全盘后移", "设硬性节点：2026-12-31 未跑通则时间表整体后移 3 个月，目标改 CVPR 2028"],
        ["RM65 无法更换安装朝向", "中", "核心自变量消失", "备选方案 B：自变量降级为「操作者参考系 vs 世界锚定参考系」，"
                                                     "数据集降为副产品；或改为仿真中改变基座朝向"],
        ["D&B track 不被认定为 CCF A", "中", "投稿目标变更", "备选方案 A：直接瞄准 CVPR 2028 或 ICLR 2028，方案主体不变"],
        ["无可用 GPU", "中", "无法完成下游实验", "退化为「数据集 + 运动学事件分析」，但通过 A 类的概率大幅下降，应优先争取算力"],
        ["采集到低质量数据", "中", "数据作废", "方向未实测定案前不得开始正式采集；先采 50 条试点并做一次筛选实验验证信号"],
        ["被他人抢先发表", "低", "—", "安装朝向这一维度目前无人占位；建议自 2026 年 12 月起定期复查 arXiv"],
    ],
    widths=[3.4, 1.5, 2.8, 8.3],
)

H("8.1 与 IEEE VR 副线的关系", 2)
P("IEEE VR 2028（约 2027 年 8 月截稿）的人因研究路线不必放弃，它与数据集主线共用全部基础设施"
  "（真机验收、逐轴实测、相机接入、伦理审批），且可复用采集数据中操作者的主观评分，边际成本很低。"
  "但建议先打通采集流程，再启动副线，不建议同时开工。")

# ============================================================
H("九、附录：关键证据来源", 1)
P("以下事实均已逐条核验并附出处，完整清单（约 150 个 URL）见仓库内调研报告。", size=10)
TABLE(
    ["结论", "来源"],
    [
        ["CCF 目录第七版发布与变动", "中国计算机学会官网，2026-03-31 发布（含官方 PDF 原文，与第六版逐条对照）"],
        ["IEEE VR 与 TVCG 为 CCF A；IEEE VR 主赛道论文发表于 TVCG", "IEEE VR 官方征稿页（单一投稿、统一评审、三种结果）"],
        ["IEEE VR 截稿与投稿要求", "IEEE VR 2026 / 2027 官方 Call for Papers"],
        ["ICRA = CCF B、IROS = CCF C、T-RO = B、RA-L/RSS/CoRL 不在目录", "CCF 目录第七版 PDF 逐条核对"],
        ["本方向发表版图（约 45 篇）", "PMLR（CoRL 2024 v270 / 2025 v305）、RSS 官方论文集（rss19–rss21）、"
                                       "出版方 DOI 记录、arXiv"],
        ["头部数据集规模", "RSS XXI 官方论文页（RoboMIND）、DROID 官方项目页、AgiBot World 论文、Open X-Embodiment 官方页"],
        ["BRS 使用非 VR 的 JoyLo 接口并优于 VR", "CoRL 2025 论文与官方项目页（10 人用户研究）"],
        ["LIBERO 基准可被刷分", "What Are We Actually Benchmarking in Robot Manipulation?（arXiv:2606.04233）"],
        ["CUPID 使用不足 33% 筛选数据达到 SOTA", "CoRL 2025，PMLR v305:2907–2932"],
        ["NeurIPS D&B track 的定位与硬性要求", "NeurIPS 官方 FAQ 与征稿说明"],
    ],
    widths=[6.4, 9.6],
)

P()
p = doc.add_paragraph()
p.alignment = WD_ALIGN_PARAGRAPH.RIGHT
r = p.add_run("（本方案为调研结论，具体数值与截稿日期请以各会议官方最新公告为准。）")
set_run(r, ea="楷体", size=9, color=RGBColor(0x66, 0x66, 0x66))

doc.save(OUT)
print("SAVED:", OUT)
print("paragraphs:", len(doc.paragraphs), "tables:", len(doc.tables))
