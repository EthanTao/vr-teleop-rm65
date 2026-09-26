# 子任务 1：机械臂/遥操作中的奇异点处理与规避（2023–2025 代表性论文核查）

- 核查日期：2026-09-24
- 核查规则：每条都对应一次**实际网页抓取**；标题/venue/年份从抓取到的页面正文里抄录，不用记忆、不推断。抓不到的页面按「待核实」处理或直接剔除（见 §4）。
- 抓取受限域名（已实测）：`ieeexplore.ieee.org`（HTTP 202 空页）、`dl.acm.org`（403）、`journals.sagepub.com`（fetch failed）、`doaj.org`（403）、`pubmed.ncbi.nlm.nih.gov`（203 空页）、`dblp.org`（Anubis 反爬）、`api.crossref.org` / `api.semanticscholar.org`（fetch failed）。
- 可用抓取源（已实测）：`arxiv.org`（abs 页与 advanced search 页）、`elib.dlr.de`（DLR 机构库，含 DOI/卷期/页码）、`api.openalex.org`（JSON 记录，含 DOI/venue/年份/摘要）、作者项目主页。
- 可信度分级：**A** = 抓到 arXiv abs 页 / 机构库页 / 作者页，字段逐字抄录；**B** = 正文落地页被反爬拦截，改用 OpenAlex JSON 记录（含 DOI、raw_source_name、卷期页码）交叉核实，并在条目里标注。

---

## 1. 核心 9 篇（全部为 2023–2025）

### 1. J-PARSE: Jacobian-based Projection Algorithm for Resolving Singularities Effectively in Inverse Kinematic Control of Serial Manipulators

`J-PARSE: Jacobian-based Projection Algorithm for Resolving Singularities Effectively in Inverse Kinematic Control of Serial Manipulators` | arXiv 预印本（项目页标 "In Submission"，无会议/期刊 venue） | 2025（v1: 2025-05-01，最后修订 2026-09-22） | 速度级 IK 里不再"靠近奇异就减速/停"，而是构造一个保持可操作度椭球长短轴比的安全雅可比（Safety Jacobian），把末端速度指令按奇异/非奇异方向分解、只对奇异方向按奇异值缩放，从而在奇异位形内也能稳定进出、且消除 DLS 的稳态误差——直接服务于遥操作手感与"够得到的极端位姿"。 | https://arxiv.org/abs/2505.00306

- 核实（A）：抓取 arXiv abs 页读到标题、作者（Shivani Guptasarma, Matthew Strong, Honghao Zhen, Monroe Kennedy III）、提交日期 2025-05-01、分类 cs.RO、摘要全文（含 "aspect ratio of the manipulability ellipsoid above a threshold value"、"Applications in teleoperation, servoing, and learning"）。另抓取作者项目页 https://jparse-manip.github.io/ ，页面顶部标注 "In Submission"，正文明确写 "benchmarked against the Least-Squares and Damped Least-Squares inversions of the Jacobian"、"Note how the leading method of DLS suffers from a steady state error even in non-singular poses"，并说明"奇异点是不稳定点，扰动后可自行离开"。
- 覆盖技术点：速度级 vs 位置级控制、SVD/可操作度（σmin/σmax、w=√det(JJᵀ)）、DLS 对照、遥操作手感。
- 注意：**无正式 venue**，引用时不能写成 ICRA/IROS。

### 2. Singularity-Robust Prioritized Whole-Body Tracking and Interaction Control With Smooth Task Transitions

`Singularity-Robust Prioritized Whole-Body Tracking and Interaction Control With Smooth Task Transitions` | ICRA 2024（2024 IEEE International Conference on Robotics and Automation, Yokohama） | 2024 | 用加权广义逆（weighted generalized inverse）推导出在奇异与冗余任务下都成立的层级控制律，再用一个"平滑激活矩阵"连续地塑形零空间投影器与各层任务动作，使严格任务优先级在穿越奇异、切换任务时不产生跳变——解决全身控制里"一进奇异就抖动/任务层突变"的问题（仿真在 Rollin' Justin 上验证）。 | https://elib.dlr.de/204082/

- 核实（A）：抓取 DLR 机构库页，读到作者 Wu, Xuwei / Albu-Schäffer, Alin Olimpiu / Dietrich, Alexander、年份 2024、出版处 "2024 IEEE International Conference on Robotics and Automation, ICRA 2024"、DOI `10.1109/ICRA57147.2024.10610961`、ISBN 979-835038457-4、会议日期 2024-05-13~17，以及摘要原文（"weighted generalized inverse… hierarchical control law compatible with singular and redundant tasks… smooth activation matrix… continuously shape both null-space projectors and task level control actions"）。

### 3. Hierarchical Incremental MPC for Redundant Robots: A Robust and Singularity-Free Approach

`Hierarchical Incremental MPC for Redundant Robots: A Robust and Singularity-Free Approach` | IEEE Transactions on Robotics (T-RO), vol. 40, pp. 2128–2148 | 2024 | 把多层任务写成多层约束最优控制问题并转成 QP（线性 MPC），用等式约束实现"动力学一致"的任务优先级，从公式层面让控制器**脱离运动学/算法奇异**（singularity-free），同时硬性处理关节力矩与位置约束，1 kHz 实测——解决冗余臂在做层级任务时既要精度又要抗奇异、还要满足物理约束的三难。 | https://elib.dlr.de/203423/

- 核实（A）：抓取 DLR 机构库页，读到作者 Wang, Yongchao / Liu, Yang / Leibold, Marion / Buss, Martin / Lee, Jinoh、年份 2024-02-27、期刊 IEEE Transactions on Robotics、vol. 40、pp. 2128-2148、DOI `10.1109/TRO.2024.3370049`、官方 URL `https://ieeexplore.ieee.org/document/10449433`；摘要原文含 "untethered from kinematic/algorithmic singularities"、"the constrained OCPs are cast as quadratic programming problems"、"dynamically-consistent task priority is achieved by deploying equality constraints"、"implemented under the sampling frequency of 1 kHz"。

### 4. Reactive Model Predictive Contouring Control for Robot Manipulators

`Reactive Model Predictive Contouring Control for Robot Manipulators` | IROS 2025（IEEE/RSJ International Conference on Intelligent Robots and Systems） | 2025 | 用控制屏障函数（CBF）把"奇异性"和"碰撞"写成 MPC 里的硬约束，并把参考路径用路径参数重参数化，配合雅可比线性化 + Gauss-Newton Hessian 近似，在 100 Hz 内实时求解——解决动态环境下避障/避奇异与轮廓跟踪误差互相打架的问题（比现有方法快 10 倍）。 | https://arxiv.org/abs/2508.09502

- 核实（A）：抓取 arXiv abs 页（https://arxiv.org/abs/2508.09502v1 ），读到作者 Junheon Yoon, Woo-Jeong Baek, Jaeheung Park、提交日期 2025-08-13、Comments 字段原文 "8 pages, 7 figures, 3 tables, conference paper, Accepted for publication at IEEE/RSJ International Conference on Intelligent Robots and Systems(IROS) 2025"；摘要原文含 "avoids obstacles, singularities and self-collisions in dynamic environments at 100 Hz"、"Control Barrier Functions (CBFs) are introduced to avoid collisions and singularities"。

### 5. Analytically Informed Inverse Kinematics Solution at Singularities

`Analytically Informed Inverse Kinematics Solution at Singularities` | ARK 2024（Advances in Robot Kinematics 2024，Springer Proceedings in Advanced Robotics, vol. 31） | 2024 | 指出 DLS 这类"伪逆 + 阻尼"方案在奇异位形上可能根本无解（某些末端运动不可实现），于是先解析地描述奇异运动的切向成分、据此构造一个把构型"推离奇异"的扰动量，再交给数值迭代——解决奇异点上数值 IK 发散/无解的死结（7-DOF KUKA iiwa 数值验证）。 | https://arxiv.org/abs/2412.20409

- 核实（A）：抓取 arXiv abs 页，读到作者 Andreas Mueller、提交日期 2024-12-29、Journal reference 原文 "In: Lenarcic, J., Husty, M. (eds) Advances in Robot Kinematics 2024. ARK 2024. Springer Proceedings in Advanced Robotics, vol 31. Springer, Cham"、Related DOI `10.1007/978-3-031-64057-5_29`；摘要原文含 "The damped least squares (DLS) method provides a robust solution with controllable convergence rate. However, at singularities, it may not even be possible to solve the IK problem using any PI solution…"。

### 6. Adaptive Manipulability-Based Path Planning Strategy for Industrial Robot Manipulators

`Adaptive Manipulability-Based Path Planning Strategy for Industrial Robot Manipulators` | IEEE/ASME Transactions on Mechatronics (T-Mech) | 2023 | 在 RRT* 采样时同时施加路径长度与**可操作度（manipulability）**约束，并用"约束闭环 IK"利用冗余把末端往高可操作度方向推、用可操作度指标自适应调整步长——解决工业臂（6 自由度 FANUC M-20iA）规划出的路径虽然到得了目标、却贴着构型奇异走的问题。 | https://doi.org/10.1109/TMECH.2022.3231467

- 核实（B）：DOI 落地页被 IEEE 反爬拦截（`https://doi.org/10.1109/TMECH.2022.3231467` 报 "cross-origin redirect to https://ieeexplore.ieee.org is not followed"，直连 Xplore 为 202 空页）。改用抓取到的 OpenAlex 记录核实：`https://api.openalex.org/works?filter=title.search:Adaptive%20Manipulability-Based%20Path%20Planning&select=title,doi,publication_year,primary_location,authorships,abstract_inverted_index&per-page=3` ，记录显示 title 一致、DOI `10.1109/tmech.2022.3231467`、publication_year 2023、raw_source_name "IEEE/ASME Transactions on Mechatronics"、作者 Henghua Shen / Wen-Fang Xie / Jianyu Tang / Tao Zhou（Concordia University）；摘要含 "avoid configuration singularities"、"six-degree-of-freedom FANUC-M-20iA"。**venue 与年份依据 OpenAlex/DOI 记录，非 Xplore 页面本身。**
- 覆盖技术点：可操作度度量驱动的奇异规避 + 冗余利用（本表中唯一一篇面向 6 自由度工业臂、以可操作度为主指标的工作）。

### 7. IKSel: Selecting Good Seed Joint Values for Fast Numerical Inverse Kinematics Iterations

`IKSel: Selecting Good Seed Joint Values for Fast Numerical Inverse Kinematics Iterations` | arXiv 预印本（无 venue） | 2025 | 把数值 IK 的成败归因到**初值（seed）选择**：用 KDTree 按工作空间距离找候选、按"关节空间调整量最小"排序优先尝试、失败后再挑差异最大的构型重试——解决数值 IK 卡局部极小/关节限位、成功率与耗时不稳定（也就是连续性/跳变的上游原因）。 | https://arxiv.org/abs/2503.22234

- 核实（A）：抓取 arXiv abs 页，读到作者 Xinyi Yuan, Weiwei Wan, Kensuke Harada、提交日期 2025-03-28、分类 cs.RO、无 journal ref / comments；摘要原文含 "re-attempt strategy effectively helps circumvent local minima and joint limit constraints"、"Comparison results with both traditional numerical solvers and learning-based methods"。

### 8. Efficient Learning of Fast Inverse Kinematics with Collision Avoidance

`Efficient Learning of Fast Inverse Kinematics with Collision Avoidance` | Humanoids 2023（2023 IEEE-RAS International Conference on Humanoid Robots） | 2023 | 用神经网络预测 IK 初值、再由两级雅可比求解器精修，且把"无碰撞"直接纳入学习目标（无需样本数据生成步骤、无监督架构）——解决高自由度机器人"学出来的 IK 不懂避障、且训练代价高"的落地问题（19 自由度 Agile Justin，单核 <10 ms、精度 1e-4 m / 1e-3 rad）。 | https://arxiv.org/abs/2311.05938

- 核实（A）：抓取 arXiv abs 页，读到作者 Johannes Tenhumberg, Arman Mielke, Berthold Bäuml、提交日期 2023-11-10、Comments "Presented at the 2023 IEEE-RAS International Conference on Humanoid Robots"、Journal reference "2023 IEEE-RAS International Conference on Humanoid Robots (Humanoids)"、Related DOI `10.1109/Humanoids57100.2023.10375143`。

### 9. Path Planning and Optimization for Cuspidal 6R Manipulators

`Path Planning and Optimization for Cuspidal 6R Manipulators` | arXiv 预印本（无 venue） | 2025（v1 2025-01-30，v2 2025-04-29） | 针对"每个笛卡尔点都有 IK 解、但拼成连续关节轨迹可能不可行"的 6R 机械臂（cuspidal 型）：识别 cuspidal 机器人（首次指出 ABB GoFa 及某些三平行轴机型属此类），对路径上每点求全部 IK 解并建图、以"最小关节速度"等指标做最短路，从而找到**连续、无跳变**的关节空间路径（也可显式穿奇异）。 | https://arxiv.org/abs/2501.18505

- 核实（A）：抓取 arXiv abs 页（https://arxiv.org/abs/2501.18505v2 ），读到作者 Alexander J. Elias, John T. Wen、v1 提交 2025-01-30、v2 2025-04-29、Comments "11 pages, 11 figures"；摘要原文含 "A task-space path may have a valid IK solution for each point along the path, but a continuous joint-space path may depend on the choice of the IK solution or even be infeasible."。
- 覆盖技术点：IK 解连续性、构型切换（cuspidal / elbow-flip 类问题）、seed/解分支选择——对 6 自由度臂最贴近的一篇。

---

## 2. 补充条目（已核实，非核心 9 篇；用于佐证趋势）

| 标题 | venue | 年份 | 一句话贡献 | URL |
| --- | --- | --- | --- | --- |
| Concentric Tube Robot Redundancy Resolution via Velocity/Compliance Manipulability Optimization | IEEE Robotics and Automation Letters (RA-L), 8(11):7495–7502 | 2023 | 把速度/柔顺可操作度指标做成可微性能函数，用梯度投影到轨迹跟踪零空间里做冗余分解（梯度计算比有限差分快 68%），保证"安全运动 + 执行器指令可行" | https://arxiv.org/abs/2305.06194 |
| CppFlow: Generative Inverse Kinematics for Efficient and Robust Cartesian Path Planning | ICRA 2024（DOI 10.1109/ICRA57147.2024.10611724） | 2024 | 用生成式（学习式）IK 在 GPU 上一次性产出整条候选关节轨迹、再数值精修，使笛卡尔路径规划最快提速 129×，并在传统方法失败的问题上成功 | https://api.openalex.org/works?filter=title.search:CppFlow （arXiv 版本 https://arxiv.org/abs/2309.09102 ，B 级核实） |
| Bunny-VisionPro: Real-Time Bimanual Dexterous Teleoperation for Imitation Learning | IROS 2025（DOI 10.1109/IROS60139.2025.11247017）；arXiv 版 2024 | 2024/2025 | VR 双臂灵巧遥操作系统，把碰撞与奇异规避内建进实时遥操作回路（并加低成本触觉反馈），同时保证示范数据质量 | https://arxiv.org/abs/2407.03162 |
| VR-Based Teleoperation Framework: Integration of Haptic Feedback and Singularity Management | IEEE Access（DOI 10.1109/ACCESS.2026.3673307） | **2026（超出 2023–2025 窗口）** | VR 遥操作框架里把触觉反馈与奇异点管理联合设计（作者 Seungnam Yu / Geegum Lee / Jeongmok Kim，KAERI 等） | https://api.openalex.org/works?filter=title.search:VR-Based%20Teleoperation%20Framework （B 级核实，Xplore 正文页未抓到） |

- RA-L 2023 条目核实（A+B）：抓取 arXiv abs 页读到标题/作者/日期与摘要（"projecting the gradient into the null-space of trajectory tracking"）；卷期页码与 venue 经抓取 `https://api.openalex.org/works/https://doi.org/10.1109/lra.2023.3318115` 记录核实（IEEE Robotics and Automation Letters, 8(11), 7495–7502, 2023）。
- Bunny-VisionPro 核实（A+B）：抓取 arXiv abs 页读到摘要原文 "Our system prioritizes safety by incorporating collision and singularity avoidance while maintaining real-time performance"；IROS 2025 出版信息经 OpenAlex 记录（raw_source_name "2025 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)"）核实。

---

## 3. 三个问题的结论（venue 阵地 / 是否饱和 / 新问题）

1. **主流阵地**：真正把"奇异点"当核心贡献的 2023–2025 论文，落在 ICRA、IROS、T-RO、T-Mech、RA-L、Humanoids 这条主线上（本表 9 篇中 4 篇是 ICRA/IROS、2 篇 T-RO/T-Mech、1 篇 Humanoids），再加上机器人运动学专门会议 ARK；纯 DLS 变体则明显外溢到低层级阵地（见第 2 点）。
2. **经典 DLS/伪逆这条线已经饱和**：以 OpenAlex 标题检索 `damped least squares` 限定 2023–2025 只有 17 条，且多数根本不是机器人（地球物理反演、光学设计、电机控制、测量不确定度），机器人相关的几条分别落在 IEEE Access、Int. J. Advanced Robotic Systems、ACM 会议论文集、J. Phys. Conf. Ser. 等非优先阵地；反过来说，"再提一个自适应阻尼公式"很难再进 ICRA/T-RO，**必须绑定新场景或新框架**（实时 MPC/QP、CBF 约束、学习式 IK、遥操作手感、全身控制）才成立。
3. **近三年确实出现了被当作"新问题"的点**，至少四类：(a) **learning-based IK 的连续性与安全性**——Humanoids 2023 把避障并入学习目标、ICRA 2024 的 CppFlow 用生成式 IK 服务整条笛卡尔路径，同时 2025 年的 IKSel 反过来主张"好 seed + 数值迭代"仍能赢学习式方法，说明"学习式 IK 的解跳变/安全性"已是被正面争论的议题；(b) **全身/层级控制的奇异鲁棒与任务切换平滑**——ICRA 2024 用平滑激活矩阵连续化零空间投影器、T-RO 2024 用层级 QP/MPC 做到"singularity-free"并给出层级可行性证明，重点已从"避奇异"转向"在奇异附近仍然保持优先级与平滑"；(c) **把奇异性写成一阶约束（CBF）塞进实时优化**——IROS 2025 在 100 Hz 下与避障、轮廓误差一起解；(d) **遥操作下的奇异手感**——J-PARSE 明确把"接近奇异就减速/急停，对操作者不直观"当作动机并宣称"embrace the singularity"，Bunny-VisionPro 把 collision+singularity avoidance 内置进遥操作回路，2026 年 IEEE Access 更出现了"触觉反馈 + 奇异点管理"的 VR 遥操作框架（本报告窗口外，仅作趋势佐证）。
4. **仍偏空白的角落**：6 自由度**非冗余**工业臂上"速度级 DLS 与位置级 IK 混用时的关节跳变/连续性判据"没有顶会新文，seed 选择与在线连续性判据主要由预印本和低层级期刊承载；这与本项目 RM65（6 轴、侧装、官方连续 IK + DLS 双路径）遇到的"上一周期 seed → 本周期解"连续性问题正好对应，可引用的最贴近文献是第 7、9 篇。

---

## 4. 未纳入 / 待核实（明确剔除，避免污染文献表）

| 候选 | 线索来源 | 状态 |
| --- | --- | --- |
| Enhanced Singularity Avoidance Control for Hydraulic Manipulators Using Damped Least-Squares Optimization | 搜索结果给出 Engineering Letters 33(9) PDF（engineeringletters.com） | **待核实**：未抓取该 PDF，venue 层级低，未纳入 |
| Damped Least-Squares Optimization-Based Singularity Configuration Avoidance for Manipulator | OpenAlex 记录：DOI `10.1145/3704558.3705523`，Proceedings of the 2024 2nd International Conference on Frontiers of Intelligent Manufacturing and Automation | **待核实**：ACM DL 页 403 抓不到，仅有索引记录，未纳入 |
| Inverse Kinematics of a 7-Degree-of-Freedom Robotic Arm Based on Deep Reinforcement Learning and Damped Least Squares | OpenAlex 记录：DOI `10.1109/ACCESS.2024.3521539`，IEEE Access 2024 | **待核实**：仅有索引记录 + 非优先 venue，未纳入 |
| Singularity Avoidance with Application to Online Trajectory Optimization for Serial Manipulators | arXiv:2211.02516，IFAC World Congress 2023（arXiv 页显示 "Accepted for publication at IFAC World Congress 2023"） | 年份落在 2023，但 IFAC 非优先 venue 且核心是轨迹优化中的奇异约束，未纳入核心表 |
| A Bidirectional Human-Machine Control Interface Based on Virtual Fixtures for Safe Teleoperation of Hyper-Redundant Robots | 搜索命中 IEEE Xplore 10658970；OpenAlex 显示为 IEEE Access 2024，DOI `10.1109/access.2024.3452029` | 与"遥操作安全"相关但奇异点非主线，未纳入 |

---

## 5. 对本项目（RM65 六轴侧装 + VR 遥操作）的短映射（推断，非文献结论）

- `hardware/damped_ik.py` 走的正是第 1、5 篇讨论的那条路：DLS 能保证有界但引入稳态误差、且真奇异点上仍可能"无解/推不动"；J-PARSE（第 1 篇）与 AI-IK（第 5 篇）分别给出"按方向缩速"和"解析扰动跳出奇异"两条可借鉴的替代思路。
- 真机用"上一已接受目标作 seed"的做法，其风险与第 7 篇（seed 选择、局部极小、关节限位）和第 9 篇（解分支不连续导致关节跳变）直接对应；第 9 篇的"对路径每点求全部 IK 解 + 建图求最短路"可作为离线核查 J3/J5 跳变的工具思路。
- 第 2、3 篇（层级/优先级 + 平滑激活）可用于说明"为什么奇异性最好写成约束或优先级层，而不是只在速度级做阻尼"。
