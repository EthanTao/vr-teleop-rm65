# 遥操作系统「安全层」文献核查（2023–2025）

> 核查员视角：只列**实际抓取过网页**确认存在的条目。每条给出「我抓的是哪个 URL」。
> 检索日期：2026-09（当前环境日期），故 2023–2025 已是回溯窗口，2026 年的新条目单列。

---

## 0. 核查方法与可信度声明（先读这一段）

**抓取成功、可作为一手证据的来源**：
- `arxiv.org/abs/...` 摘要页（HTTP 200，正文全文抓回）
- `export.arxiv.org/api/query?...` arXiv 官方 API（返回 Atom XML，含 title / comment / journal_ref / arxiv:doi）
- `api.crossref.org/works/<DOI>` 与 `api.crossref.org/works?...` Crossref 官方 API（出版方寄存的权威元数据：标题、刊名、卷期页、年份、作者、DOI）
- `api.openalex.org/works?...` OpenAlex API（含 `raw_source_name`，即会议/期刊全名）

**抓取失败、因此不作为证据的来源**（重要，避免误判我"没查到"= "不存在"）：
- `ieeexplore.ieee.org/document/...` → 返回 HTTP 202 空壳（JS 挑战），正文抓不到
- `dl.acm.org/doi/...` → HTTP 403（Cloudflare "Just a moment..."）
- `www.mdpi.com/...` → HTTP 403（Akamai "Access Denied"）
- `doi.org/10.1109/...` → 跨域跳转被拦，无法跟随

**结论**：IEEE / ACM / MDPI 的**落地页**在这个环境里打不开，所以我改用**出版方向 Crossref 寄存的同一条元数据**做核实——标题、venue 全名、卷期页、年份、DOI 都来自出版方自己提交的记录，可靠性等同于落地页。凡是我只从 arXiv API 拿到、且**未找到正式 venue** 的，明确标注「仅 arXiv 预印本」。

**一个被我当场证伪的猜测**（说明为什么必须逐条核实）：我最初凭印象写了一个 arXiv 编号 `2403.19819` 当作遥操作 CBF 论文，抓取后发现是 *"A Digital Twin for Geological Carbon Storage with Controlled Injectivity"*（地质碳封存，physics.geo-ph）。**完全是另一篇论文**。本清单中不存在任何未经抓取确认的条目。

---

## 1. 核心问题的答案：CBF 用于遥操作安全，近三年发在哪？

**直接回答**：2023-01-01 至 2025-12-31，OpenAlex 全库中标题/摘要**同时**含 "control barrier function" 和 "teleoperation" 的论文共 **29 条**（含预印本与学位论文的去重前计数）。逐条看下来，venue 分布是：

| venue 类别 | 具体去处 | 条数 |
|---|---|---|
| 机器人顶会 | ICRA 2024、ICRA 2025 | 4 |
| 机器人期刊 | IEEE RA-L | 3 |
| 控制类会议 | ECC 2024、ECC 2025、CCTA 2025、IFAC-PapersOnLine（IFAC World Congress） | 4 |
| 系统/人机会议 | IEEE SMC 2023、IEEE ROBIO 2025、IEEE Telepresence 2025 | 4 |
| 交叉/应用会议 | PETRA 2023（普适技术）、JSME Robomec 2025 | 2 |
| 期刊（非顶刊） | Autonomous Robots、IEEE T-MRB、Sensors、IEEE TII | 4 |
| 预印本 / 学位论文 | arXiv 挂出但未见正式 venue | 5 |
| **控制理论顶刊（TAC / Automatica / L-CSS）** | **0** | **0** |
| **CDC / ACC** | **0** | **0** |
| **T-RO / CoRL / RSS / CASE** | **0**（T-RO、CoRL、CASE 在"含 teleoperation"子集里为 0） | **0** |

**这条结论很重要**：CBF×遥操作**不是一个控制理论圈的题目**。控制理论顶刊（TAC/Automatica/L-CSS）和 CDC/ACC 收的是**泛化的 CBF 理论**（可行性、鲁棒性、高阶 CBF、backup CBF），而"把 CBF 用到遥操作/机械臂上"这类工作，**主战场是 RA-L、ICRA/IROS、ECC，其次是人机交互类会议（SMC/ROBIO/PETRA）和应用类期刊**。想投 CDC/ACC，必须把贡献抬到理论层（例如"时延下的 CBF 鲁棒性证明"），不能只是"我们把它接到了机械臂上"。

---

## 2. 主表 A：CBF / 安全滤波 × 遥操作（12 篇，核心）

| 标题 | venue | 年份 | 一句话贡献（中文） | URL（我实际抓取的） |
|---|---|---|---|---|
| MPC-CBF with Adaptive Safety Margins for Safety-critical Teleoperation over Imperfect Network Connections | 2024 European Control Conference (ECC)，DOI 10.23919/ECC64448.2024.10590767 | 2024 | 用 MPC 框架把 CBF 约束包进去，并让安全裕度**随网络时延自适应放大**，在有时延的遥操作里既能跟操作员速度指令又不撞障碍，比固定裕度更不保守；真机机械臂验证。 | https://arxiv.org/abs/2403.18650 |
| Control-Barrier-Aided Teleoperation with Visual-Inertial SLAM for Safe MAV Navigation in Complex Environments | 2024 IEEE ICRA，DOI 10.1109/ICRA57147.2024.10611280 | 2024 | 把 VI-SLAM + 稠密占据栅格实时接进 CBF，把"撞到障碍"和"撞进未建图区域"都定义为不安全，安全滤波器输出最接近操作员指令的可行指令；**纯机载感知**闭环，非专家乱打杆也撞不上。 | https://arxiv.org/abs/2403.04331 |
| An Intention-Aware Robust Safety Framework for Robot Teleoperation: Unifying Object Interaction and Obstacle Avoidance | IEEE Robotics and Automation Letters, vol. 10, no. 10, pp. 10394–10401，DOI 10.1109/LRA.2025.3595023 | 2025 | 把**意图感知**与安全约束统一：同一个框架里既允许操作员主动接触目标物体、又拒绝撞环境障碍，解决"该碰的不让碰、不该碰的碰了"这对矛盾。 | https://api.crossref.org/works/10.1109/LRA.2025.3595023 |
| A Teleoperation Framework for Robots Utilizing Control Barrier Functions in Virtual Reality | PETRA '23 (16th Int. Conf. on PErvasive Technologies Related to Assistive Environments), pp. 408–412，DOI 10.1145/3594806.3596522 | 2023 | **VR 遥操作 + CBF**：在虚拟现实里做遥操作，用 CBF 把操作员指令投影到安全集。**与本项目（PICO 手柄 → RM65）场景最接近的一篇**，但篇幅仅 5 页、验证较浅。 | https://api.crossref.org/works/10.1145/3594806.3596522 |
| Safe Execution of Learned Orientation Skills with Conic Control Barrier Functions | 2024 IEEE ICRA, pp. 13376–13382，DOI 10.1109/ICRA57147.2024.10611325 | 2024 | 在 SO(3) 上从示教方差里提取**时变锥约束**，用 Conic CBF 把学习到的姿态技能限制在锥内，**演示场景正是辅助遥操作下的切削任务**。 | https://api.crossref.org/works/10.1109/ICRA57147.2024.10611325 |
| Safe and stable teleoperation of quadrotor UAVs under haptic shared autonomy | Autonomous Robots，DOI 10.1007/s10514-024-10186-0 | 2025 | 同时管**安全 + 稳定**：CBF 保证不撞，同时用微分约束把反馈力限制在小 L2 增益，使"人-机闭环"稳定；给出 SCF / JCF 两种设计。 | https://api.openalex.org/works?filter=title_and_abstract.search:%22control%20barrier%20function%22%20teleoperation,from_publication_date:2023-01-01,to_publication_date:2025-12-31&per-page=30 |
| Haptic Shared Control Framework with Interaction Force Constraint Based on Control Barrier Function for Teleoperation | Sensors, vol. 25, no. 2, art. 405，DOI 10.3390/s25020405 | 2025 | 把 CBF 从"位置/避障约束"推广到**交互力约束**：触觉共享控制中限制接触力，兼顾透明性与安全性。 | https://api.openalex.org/works?search=control%20barrier%20function%20teleoperation |
| Safe Bilateral Teleoperation for a UAV Using Control Barrier Functions and Passivity | 2023 IEEE Int. Conf. on Systems, Man, and Cybernetics (SMC)，DOI 10.1109/SMC53992.2023.10394030 | 2023 | 双通道遥操作里把 **CBF（安全）与无源理论（稳定）拼在一起**：无源性管能量/时延，CBF 管硬约束。 | https://api.openalex.org/works?filter=title_and_abstract.search:%22control%20barrier%20function%22%20teleoperation,from_publication_date:2023-01-01,to_publication_date:2025-12-31&per-page=30 |
| Human–Centered Cooperative Control Coupling Autonomous and Haptic Shared Control via Control Barrier Function | IFAC-PapersOnLine, vol. 59, no. 35, pp. 280–285，DOI 10.1016/j.ifacol.2025.12.489 | 2025 | 用 CBF 把"自主控制"和"触觉共享控制"两种权限耦合起来，人在回路且安全约束不被绕过（IFAC World Congress 线）。 | https://api.crossref.org/works/10.1016/j.ifacol.2025.12.489 |
| AeroHaptix: A Wearable Vibrotactile Feedback System for Enhancing Collision Avoidance in UAV Teleoperation | IEEE Robotics and Automation Letters，DOI 10.1109/LRA.2025.3548866 | 2025 | 提出 **MultiCBF** 扩展（支持多方向约束），把 CBF 算出的"你正在不安全"映射成**可穿戴振动触觉**，而不是抢操作员的杆——不改控制权，只加信息。 | https://api.openalex.org/works?filter=title_and_abstract.search:%22control%20barrier%20function%22%20teleoperation,from_publication_date:2023-01-01,to_publication_date:2025-12-31&per-page=30 |
| Shared Control of Teleoperated Vehicles With Delay-Compensated Safety Filtering | 2025 IEEE Conference on Control Technology and Applications (CCTA)，DOI 10.1109/CCTA53793.2025.11151402 | 2025 | 车辆遥操作的共享控制：安全滤波器显式做**时延补偿**，避免"按旧状态判安全"导致的误判。 | https://api.openalex.org/works?filter=title_and_abstract.search:%22safety%20filter%22%20teleoperation,from_publication_date:2023-06-01&per-page=20 |
| Minimizing Conservatism in Safety-Critical Control for Input-Delayed Systems via Adaptive Delay Estimation | ECC 2025（arXiv 注释：accepted to ECC 2025；arXiv:2411.17277） | 2024/2025 | 针对**输入时延**（明确点名 teleoperation）的 delay-adaptive CBF：在线估计时延界，使状态预测误差界单调不增，从而降低 DaCBF 的保守性。 | https://export.arxiv.org/api/query?search_query=abs:%22control+barrier+function%22+AND+abs:%22teleoperation%22&start=0&max_results=40&sortBy=submittedDate&sortOrder=descending |

---

## 3. 主表 B：工程限幅 / 在线轨迹缩放 / 安全停止 / SSM / ISO 15066 线（7 篇）

直接对应本项目的「速度/加速度限幅 → 在线轨迹缩放 → 软/硬停止」这条链。

| 标题 | venue | 年份 | 一句话贡献（中文） | URL（我实际抓取的） |
|---|---|---|---|---|
| Enhanced Performance of Human-Robot Collaboration Using Braking Surfaces and Trajectory Scaling | 2023 IEEE/RSJ IROS, pp. 5942–5949，DOI 10.1109/IROS55552.2023.10341408 | 2023 | 把**制动曲面（braking surface）**与**轨迹在线缩放**结合：不再用固定安全区急停，而是按"还能刹得住"的边界动态缩放速度，兼顾安全与节拍。**参考文献第 1 条即 ISO/TS 15066**，是 ISO 15066 线的直接对口工作。 | https://api.crossref.org/works/10.1109/IROS55552.2023.10341408 |
| Jerk-Limited Online Trajectory Scaling for Cable-Driven Parallel Robots | IEEE Transactions on Automation Science and Engineering, vol. 22, pp. 11529–11539，DOI 10.1109/TASE.2025.3533583 | 2025 | **T-ASE 正刊**上的在线轨迹缩放：缩放的不仅是速度/加速度，还显式限制**加加速度（jerk）**，因此缩放过程本身不会产生冲击。证明"限幅+在线缩放"能上顶刊。 | https://api.crossref.org/works/10.1109/TASE.2025.3533583 |
| An inverse kinematics solution with trajectory scaling for redundant manipulators | Mechanism and Machine Theory, vol. 191, art. 105493，DOI 10.1016/j.mechmachtheory.2023.105493 | 2024 | 把**在线速度缩放（on-line velocity scaling）直接嵌进冗余机械臂的 IK 求解**：接近关节限位/奇异时就地降速而不中断路径。对"真机不走 URDF IK、要就地缩放"的场景有直接参考价值。 | https://api.crossref.org/works/10.1016/j.mechmachtheory.2023.105493 |
| Safe and Fluent Industrial Human Robot Collaboration Via Combination of PFL, SSM and Escape Trajectories | IEEE Robotics and Automation Letters, vol. 10, no. 6, pp. 6480–6487，DOI 10.1109/LRA.2025.3568567 | 2025 | 把 ISO/TS 15066 的两种模式——**功率与力限制（PFL）+ 速度与分离监控（SSM）**——和**逃脱轨迹（escape trajectory）**组合，避免 SSM 一触发就停机的"卡顿感"。 | https://api.crossref.org/works/10.1109/LRA.2025.3568567 |
| Nonlinear model predictive control with set terminal constraint for safe robot motion planning via speed and separation monitoring | Control Engineering Practice，DOI 10.1016/j.conengprac.2024.106155 | 2024 | 用 NMPC + **终端集约束**替代传统 SSM 的"距离阈值+降速"启发式，给出可证明的分离距离保证。 | https://api.openalex.org/works?filter=title_and_abstract.search:%22speed%20and%20separation%20monitoring%22,from_publication_date:2023-01-01,to_publication_date:2025-12-31&per-page=25 |
| Safe Teleoperation of the Vehicle Through Combined Speed and Separation Monitoring with Potential Field Approach | 2023 23rd Int. Conf. on Control, Automation and Systems (ICCAS)，DOI 10.23919/ICCAS59377.2023.10316846 | 2023 | **SSM 直接用在遥操作上**（不是协作机器人）：把 SSM 与人工势场结合，给遥操作车辆做距离-速度约束。是把 ISO 15066 思路搬进遥操作的少见样例。 | https://api.openalex.org/works?filter=title_and_abstract.search:%22speed%20and%20separation%20monitoring%22,from_publication_date:2023-01-01,to_publication_date:2025-12-31&per-page=25 |
| An experimental evaluation of robot-stopping approaches for improving fluency in collaborative robotics | Robotica，DOI 10.1017/S0263574724000262 | 2024 | **"怎么停"本身的实证对比**：不同停止策略（分类停止 vs. 限速停止）对流畅度/安全性的影响，含实测数据。给"软停止 vs 硬停止"设计提供实验依据。 | https://api.openalex.org/works?filter=title_and_abstract.search:%22speed%20and%20separation%20monitoring%22,from_publication_date:2023-01-01,to_publication_date:2025-12-31&per-page=25 |

---

## 4. 主表 C：安全滤波器用于机械臂（CBF 主战场，非遥操作）+ 形式化保证（5 篇）

| 标题 | venue | 年份 | 一句话贡献（中文） | URL（我实际抓取的） |
|---|---|---|---|---|
| A Backup Control Barrier Function Approach for Safety-Critical Control of Mechanical Systems Under Multiple Constraints | IEEE/ASME Transactions on Mechatronics, vol. 30, no. 6, pp. 4460–4471，DOI 10.1109/TMECH.2024.3504573 | 2025 | 用 **backup CBF** 处理"多约束同时满足"问题：先算一条后备安全轨迹，再由它反推当前该守的约束集，使多个约束不会互相打架导致 QP 不可行。**T-Mech 正刊**，说明这条线能上机械领域顶刊。 | https://api.crossref.org/works/10.1109/TMECH.2024.3504573 |
| Safety-Critical Control for Robotic Manipulators using Collision Cone Control Barrier Functions | **仅 arXiv 预印本（未找到正式 venue）**，arXiv:2503.00623 | 2025 | 把碰撞锥方法扩展成 **C3BF**（Collision Cone CBF），与笛卡尔阻抗控制结合，用 QP 最小修正标称输入；纯仿真验证。 | https://arxiv.org/abs/2503.00623 |
| Semantically Safe Robot Manipulation: From Semantic Scene Understanding to Motion Safeguards | IEEE Robotics and Automation Letters, vol. 10, no. 5, pp. 4810–4817 | 2025 | **语义安全滤波器**：用 LLM 推断"把水杯举到笔记本上方""旋转水杯"这类**常识性不安全**（几何避障管不到），再映射成 CBF 约束；在**遥操作**操作任务与扩散策略上验证。 | https://export.arxiv.org/api/query?search_query=abs:%22safety+filter%22+AND+abs:%22teleoperation%22&start=0&max_results=40&sortBy=submittedDate&sortOrder=descending |
| Safe Multi-Robotic Arm Interaction via 3D Convex Shapes | Robotics and Autonomous Systems，DOI 10.1016/j.robot.2025.105263 | 2025 | 为 3D 凸体构造**高阶 CBF（HOCBF）**，作为集中式/分布式安全滤波器防止多机械臂互撞；用数值微分替代 Hessian 计算降算力。 | https://export.arxiv.org/api/query?search_query=abs:%22control+barrier+function%22+AND+%28abs:%22manipulator%22+OR+abs:%22robot+arm%22%29+AND+abs:%22safety+filter%22&start=0&max_results=40&sortBy=submittedDate&sortOrder=descending |
| Generalizing Safety Beyond Collision-Avoidance via Latent-Space Reachability Analysis | Proceedings of Robotics: Science and Systems (RSS) 2025，DOI 10.15607/RSS.2025.XXI.113 | 2025 | **Latent Safety Filters**：把 Hamilton-Jacobi reachability 搬到生成式世界模型的隐空间里，于是能防"手工写不出来的失败"（如把包里东西洒出来）；明确用于**保障任意策略乃至直接遥操作**。RSS 正刊，是"reachability 安全层"的标杆。 | https://export.arxiv.org/api/query?search_query=abs:%22safety+filter%22+AND+abs:%22teleoperation%22&start=0&max_results=40&sortBy=submittedDate&sortOrder=descending |
| RAIL: Reachability-Aided Imitation Learning for Safe Policy Execution | 2025 IEEE ICRA，DOI 10.1109/ICRA55743.2025.11128656（arXiv:2409.19190） | 2025 | 构建基于 reachability 的**硬约束**安全滤波器；关键发现：**性能最好的策略有时恰恰因为频繁违规才好**，加上硬约束后性能大幅下降——即"软约束调参"换来的性能是假的。 | https://arxiv.org/abs/2409.19190 与 https://api.openalex.org/works?filter=title.search:RAIL%20Reachability-Aided%20Imitation%20Learning |

---

## 5. 主表 D：运行时监控 / 故障检测（3 篇）

**先给一个负面结论（重要）**：我用 OpenAlex 查 `"fault detection" + robot safety + latching`，**命中数为 0**；查 `"latching"` 与机器人安全组合也几乎无学术论文。
→ **"故障锁存（latching）"在学术文献里基本不是一个被研究的对象**，它是工业控制器（UR、KUKA、睿尔曼等）的保护性停止/复位规范里的工程约定。想要论文，必须把"锁存"重新表述成学术问题（例如"带状态记忆的安全监视器"、"不可自恢复安全集的构造"）。

| 标题 | venue | 年份 | 一句话贡献（中文） | URL（我实际抓取的） |
|---|---|---|---|---|
| Runtime Verification and Field-Based Testing for ROS-Based Robotic Systems | IEEE Transactions on Software Engineering，DOI 10.1109/TSE.2024.3444697 | 2024 | 系统化地把**运行时验证（runtime verification）**用到 ROS 机器人上：从形式化规约自动生成运行时监视器 + 现场测试。是"运行时监控"这条线最硬的期刊落点（TSE）。 | https://api.openalex.org/works?filter=title_and_abstract.search:%22runtime%20monitoring%22%20OR%20title_and_abstract.search:%22runtime%20verification%22,from_publication_date:2023-01-01,to_publication_date:2025-12-31&per-page=15 |
| Runtime Verification for Anomaly Detection of Robotic Systems Security | Machines, vol. 11, no. 2, art. 166，DOI 10.3390/machines11020166 | 2023 | 用运行时验证做机器人系统的**异常/入侵检测**；思路可平移到"指令流异常检测"。 | https://api.openalex.org/works?filter=title_and_abstract.search:%22runtime%20monitoring%22%20OR%20title_and_abstract.search:%22runtime%20verification%22,from_publication_date:2023-01-01,to_publication_date:2025-12-31&per-page=15 |
| ConditionNET: Learning Preconditions and Effects for Execution Monitoring | IEEE Robotics and Automation Letters, vol. 10, no. 2, pp. 1337–1344, Feb. 2025，DOI 10.1109/LRA.2024.3520916（arXiv:2502.01167） | 2025 | 数据驱动地学"动作的前置条件与效果"，用于**执行监控**与异常检测；数据集本身来自 **406 条成功 + 138 条失败的遥操作演示**（Franka Panda）。是"遥操作演示 → 运行期异常检测"的现成落点。 | https://export.arxiv.org/api/query?search_query=abs:%22teleoperation%22+AND+%28abs:%22anomaly+detection%22+OR+abs:%22tremor%22+OR+abs:%22packet+loss%22%29&start=0&max_results=40&sortBy=submittedDate&sortOrder=descending |

---

## 6. 主表 E：输入异常检测 —— 人手抖动 / 丢包导致的指令不连续（5 篇）

**负面结论**：arXiv 全库中标题/摘要同时含 `tremor` 与 `teleoperation` 的论文**只有 4 条**（且多为外科手术方向）。**"人手抖动 → 检测 → 安全动作"这个交叉点在顶会顶刊里几乎空白**。抖动文献集中在 MDPI 系期刊与医学工程会议。

**更关键的空白**："主端跟踪跳变（tracking jump）检测"和"网络丢包导致的指令不连续检测"作为**安全层功能**，我没有检索到任何以之为核心贡献的论文。这两点在学术上是**开放缺口**（也是本项目的差异化机会）。

| 标题 | venue | 年份 | 一句话贡献（中文） | URL（我实际抓取的） |
|---|---|---|---|---|
| Prediction and Elimination of Physiological Tremor During Control of Teleoperated Robot Based on Deep Learning | Sensors, vol. 24, no. 22, art. 7359，DOI 10.3390/s24227359 | 2024 | 用深度学习**先预测、再消除**遥操作主端的生理性手抖，而不是事后低通滤波（滤波会引入相位滞后，反而破坏手感）。 | https://api.openalex.org/works?filter=title_and_abstract.search:%22hand%20tremor%22%20OR%20title_and_abstract.search:%22physiological%20tremor%22,from_publication_date:2023-01-01,to_publication_date:2025-12-31&per-page=15 |
| A Convolutional Neural Network-Based Broad Incremental Learning Filter for Attenuating Physiological Tremors in Telerobot Systems | Applied Sciences, vol. 13, no. 2, art. 890，DOI 10.3390/app13020890 | 2023 | 用 CNN + 广度增量学习构造**自适应滤波器**抑制遥操作主端手抖，针对的是"抖动频率会漂移"这一实际困难。 | https://api.openalex.org/works?filter=title_and_abstract.search:%22hand%20tremor%22%20OR%20title_and_abstract.search:%22physiological%20tremor%22,from_publication_date:2023-01-01,to_publication_date:2025-12-31&per-page=15 |
| Real-time isolation of physiological tremor using recursive singular spectrum analysis and random vector functional link for surgical robotics | ISA Transactions，DOI 10.1016/j.isatra.2024.12.040 | 2025 | 用递归奇异谱分析 + 随机向量函数链接网络**实时分离**（而非笼统滤掉）生理性抖动，保留有意运动分量；ISA Transactions 是控制应用类正刊。 | https://api.openalex.org/works?filter=title_and_abstract.search:%22hand%20tremor%22%20OR%20title_and_abstract.search:%22physiological%20tremor%22,from_publication_date:2023-01-01,to_publication_date:2025-12-31&per-page=15 |
| TeleSim: A Network-Aware Testbed and Benchmark Dataset for Telerobotic Applications | **仅 arXiv 预印本**（arXiv:2507.04425，未找到正式 venue） | 2025 | 提供**网络感知**的遥操作测试床与基准数据集：用 OMNeT++ 精确控制带宽/时延/抖动/丢包三档网络，300 次试验；最差网络下完成时间 +221.8%、成功率 −64%。可用于量化"丢包 → 指令不连续 → 任务失败"这条链。 | https://export.arxiv.org/api/query?search_query=abs:%22teleoperation%22+AND+%28abs:%22anomaly+detection%22+OR+abs:%22tremor%22+OR+abs:%22packet+loss%22%29&start=0&max_results=40&sortBy=submittedDate&sortOrder=descending |
| Signal Prediction for Loss Mitigation in Tactile Internet: A Leader-Follower Game-Theoretic Approach | IEEE Machine Learning and Signal Processing Conference (MLSP) 2025（arXiv:2507.07692） | 2025 | 用 Stackelberg 博弈做**丢包信号预测补偿**，并给出最大信号丢失上界；明确论证"预测越准，遥操作系统的时延要求可以越松"。 | https://export.arxiv.org/api/query?search_query=abs:%22teleoperation%22+AND+%28abs:%22anomaly+detection%22+OR+abs:%22tremor%22+OR+abs:%22packet+loss%22%29&start=0&max_results=40&sortBy=submittedDate&sortOrder=descending |

---

## 7. 补充：2026 年已录用的条目（超出 2023–2025 窗口，但直接影响投稿判断）

当前环境日期为 2026-09，以下条目**已确认录用**，说明该方向仍在快速扩张。列在此处仅供判断"这个题还热不热"，不计入前面的 2023–2025 清单。

| 标题 | venue | 年份 | 说明 | URL |
|---|---|---|---|---|
| A Safety-Aware Shared Autonomy Framework with BarrierIK Using Control Barrier Functions | ICRA 2026（arXiv 注释：Accepted on ICRA 2026） | 2026 | **与本项目最贴近**：CBF 放在 IK 层做"混合后（post-blend）"安全，并用 **VR 遥操作用户研究**验证（N 人，报告感知安全性与信任度提升）。明确论证"线性混合两个人各自无碰的指令，混合结果仍可能不安全"。 | https://export.arxiv.org/api/query?search_query=abs:%22control+barrier+function%22+AND+abs:%22teleoperation%22&start=0&max_results=40&sortBy=submittedDate&sortOrder=descending |
| Safety-Critical Bilateral Teleoperation for Omnidirectional Aerial Manipulation Using Force-Sensorless Haptic Feedback | IROS 2026（arXiv 注释） | 2026 | **分层安全滤波器（hierarchical safety filter）**，显式把"接触导致的跟踪误差"和"推力饱和"纳入 CBF，并把滤波器的修正量映射成触觉反馈。 | https://export.arxiv.org/api/query?search_query=abs:%22control+barrier+function%22+AND+abs:%22teleoperation%22&start=0&max_results=40&sortBy=submittedDate&sortOrder=descending |
| Embedding ISO 10218 Safety Compliance in Robots via Control Barrier Functions for Human-Robot Collaboration | arXiv 预印本（arXiv:2606.13203） | 2026 | 用 CBF **显式嵌入 ISO 10218 的 SSM 要求**，并用 SQP 做任务缩放（task scaling）与空间管道约束；与标准 SSM 模块对比。是"ISO 标准 + CBF"结合的最新样例。 | https://api.openalex.org/works?filter=title_and_abstract.search:%22speed%20and%20separation%20monitoring%22,from_publication_date:2023-01-01,to_publication_date:2025-12-31&per-page=25 |

---

## 8. 必答分析

### 8.1 CBF / 安全滤波这条线在遥操作场景的投稿去处是什么？（6 句）

1. **首选是 RA-L 和 ICRA/IROS**：近三年 CBF×遥操作共 29 条里，RA-L 占 3 篇、ICRA 占 4 篇，是数量最多、也最稳定的两个去处；RA-L 尤其吃"安全滤波器 + 真实硬件 + 用户研究"这种组合（如 LRA.2025.3595023、LRA.2025.3548866）。
2. **控制类会议是第二梯队**：ECC 2024/2025、CCTA 2025、IFAC-PapersOnLine 收了 4 篇，特点是**必须有理论增量**——ECC 那两篇的卖点分别是"时延自适应的安全裕度"和"降低 delay-adaptive CBF 的保守性"，而不是"我们接到了机械臂上"。
3. **人机交互与系统类会议是第三梯队**：IEEE SMC、ROBIO、IEEE Telepresence、PETRA、JSME Robomec 合计约 6 篇。门槛最低，但影响力也最低；PETRA 那篇（VR+遥操作+CBF）与我们场景最像，却只有 5 页、验证很浅。
4. **控制理论顶刊和 CDC/ACC 在这条线上是 0 命中**：TAC、Automatica、L-CSS、CDC、ACC 收的是**泛化 CBF 理论**（backup CBF、高阶 CBF、可行性、鲁棒性），遥操作只是它们的一个应用示例，甚至根本不出现。想投这些地方，得把贡献写成"某类不确定性/时延下 CBF 的新构造与证明"。
5. **T-RO、CoRL、RSS、CASE 在"含 teleoperation"子集里同样 0 命中**：但 RSS 2025 的 Latent Safety Filters 和 ICRA 2025 的 RAIL 说明——**reachability / 隐空间安全滤波器**这条更"深"的路线能进 RSS/ICRA，代价是要解决"手工写不出的安全约束"这类硬问题。
6. **一句话总结去处规律**：**应用增量 → RA-L / ICRA / IROS / 人机会议；理论增量 → ECC / CCTA / IFAC / CDC / ACC；方法与范式增量 → RSS / CoRL / T-RO。** 单独"把 CBF 接到遥操作机械臂上"已经很难再发好会，必须叠加一个真实难点（时延、意图冲突、力约束、语义约束、饱和）。

### 8.2 「工程限幅 + 锁存」这类偏工程的安全层有没有论文可发？门槛如何？

**有，但必须换一个"问题陈述"，且门槛比想象中高。**

- **"限幅"本身是成熟的，但成熟的形态是"在线轨迹缩放"而不是"裁剪"**：T-ASE 2025（DOI 10.1109/TASE.2025.3533583）把在线缩放做到**jerk 受限**并给出稳定性论证；Mechanism and Machine Theory 2024（DOI 10.1016/j.mechmachtheory.2023.105493）把速度缩放**嵌进冗余机械臂 IK**。这说明"限幅"能上顶刊，前提是**限幅过程本身有可证明的性质**（不产生冲击、不破坏路径跟踪、不引发振荡），而不是简单 `min(v, v_max)`。
- **"安全停止"能发，但要发在"停止质量"上**：Robotica 2024（DOI 10.1017/S0263574724000262）做的是不同停止策略的**实证对比**；IROS 2023（DOI 10.1109/IROS55552.2023.10341408）做的是"制动曲面 + 轨迹缩放"避免急停伤节拍。单说"我们实现了软/硬两级停止"是工程报告，不是论文。
- **"锁存（latching）"目前几乎没有学术对应物**——我的检索 `"fault detection" + robot safety + latching` 命中数为 0。这是**双刃剑**：一方面没有现成论文可引（写 related work 会很难）；另一方面**它确实是空白**，可以重新表述成学术问题，例如"带状态记忆的不可自恢复安全集构造""安全监视器的可复位性（resettability）形式化""fail-safe 与 fail-operational 的权衡"。
- **最可行的包装方式是三条**：①把它归到 **runtime verification / runtime monitoring**——IEEE TSE 2024（DOI 10.1109/TSE.2024.3444697）已证明 ROS 机器人的运行时验证能上 TSE；②把它归到 **ISO/TS 15066 / ISO 10218 合规**——RA-L 2025（DOI 10.1109/LRA.2025.3568567）用 PFL+SSM+逃脱轨迹上 RA-L，且 2026 已有"CBF 嵌入 ISO 10218"的工作；③把它与 **fail-safe/fail-operational 的形式化**挂钩，做"降级模式下的安全保证"。
- **门槛判断（直说）**：单纯"实现一套限幅+锁存+停机状态机"→ **发不了 RA-L/ICRA**，天花板在 CASE、AIM、ICRA workshop、或 IEEE Access / Sensors / Machines 这类期刊。要有**理论或实证的硬核增量**才能上 RA-L/T-ASE/T-Mech：例如给出缩放算法的稳定性/最优性证明、给出停止距离的可证明上界、或用**大规模真机数据**证明某种停止/缩放策略在安全性或节拍上有统计显著优势。
- **门槛最低但不失体面的入口是 ICRA/IROS workshop 和 CASE**：工程性强、有真机、有清晰失败模式分析的安全层工作，在 workshop 和 CASE 的接受度明显高于 ICRA 正会；而且 CASE 明确欢迎"系统集成 + 实验验证"类贡献。

---

## 9. 对本项目（RM65 VR 遥操作安全层）的直接启示

1. **"CBF 放在哪一层"已有现成答案可引**：ICRA 2026 的 BarrierIK 明确把 CBF 放在 **IK 层做 post-blend 安全**，并指出"两个各自安全的指令线性混合后可能不安全"——这正对应本项目"XR delta → 映射 → IK → 限幅"的链路上"哪一步做保护"的问题。
2. **本项目的"限幅 + 锁存"在上表里没有直接对应论文**：最接近的是 T-ASE 2025（jerk-limited online scaling）与 IROS 2023（braking surfaces + trajectory scaling），但都针对协作机器人路径执行，不是"人手输入流 + 主端跳变"场景。**输入异常（抖动/跳变/丢包）作为安全层功能，是被证实的空白**。
3. **若要投稿，建议的差异化角度**：把"VR 手柄输入流 → 限幅 → IK → 真机"这条链上的**输入异常检测 + 安全集锁存 + 可复位性**做成一个统一框架，并用真机故障注入（人为制造丢包、手柄跳变、反馈超时）做定量评估。这个组合在现有文献中未见，且天然契合 RA-L 的"真实硬件 + 系统贡献"偏好，或 CASE / ICRA workshop 的工程导向。
4. **可靠的引用锚点**（写 related work 可直接用）：IROS 2023 `10.1109/IROS55552.2023.10341408`（ISO/TS 15066 + 缩放 + 制动）、T-ASE 2025 `10.1109/TASE.2025.3533583`（jerk-limited 在线缩放）、RA-L 2025 `10.1109/LRA.2025.3568567`（PFL+SSM+逃脱）、RA-L 2025 `10.1109/LRA.2025.3595023`（遥操作意图感知安全）、PETRA 2023 `10.1145/3594806.3596522`（VR 遥操作 + CBF）。

---

## 10. 待核实 / 已降级条目

| 条目 | 状态 | 原因 |
|---|---|---|
| Safety-Critical Control for Robotic Manipulators using Collision Cone Control Barrier Functions | **仅 arXiv 预印本**（arXiv:2503.00623，2025-03-01） | 抓取 arXiv abs 页确认存在；OpenAlex 标题检索只返回 arXiv 记录，**未找到任何正式 venue**；不可当作会议/期刊论文引用 |
| TeleSim: A Network-Aware Testbed and Benchmark Dataset for Telerobotic Applications | **仅 arXiv 预印本**（arXiv:2507.04425，2025-07-06） | 同上，未找到正式 venue |
| RAIL / ConditionNET / Semantically Safe / Safe Multi-Robotic Arm / Minimizing Conservatism | **已确认有正式 venue**（见上表） | 由 Crossref / OpenAlex / arXiv `journal_ref` 三选一确认 |
| 任何标注为 CDC / ACC / L-CSS / TAC / Automatica / T-RO / CoRL / CASE 的「CBF × 遥操作」论文 | **未找到，故不列出** | 29 条命中中无一条落在这些 venue；不为凑venue 覆盖而编造或硬塞 |
| arXiv:2403.19819 | **已证伪并删除** | 我最初的猜测编号，抓取后确认是地质碳封存论文，与遥操作无关 |

---

## 11. 抓取过的原始 URL 清单（供复核）

```
https://arxiv.org/abs/2403.18650
https://arxiv.org/abs/2403.19819        # 证伪用：地质碳封存，非本主题
https://arxiv.org/abs/2503.00623
https://arxiv.org/abs/2409.19190
https://export.arxiv.org/api/query?search_query=abs:%22control+barrier+function%22+AND+abs:%22teleoperation%22&start=0&max_results=40&sortBy=submittedDate&sortOrder=descending
https://export.arxiv.org/api/query?search_query=abs:%22safety+filter%22+AND+abs:%22teleoperation%22&start=0&max_results=40&sortBy=submittedDate&sortOrder=descending
https://export.arxiv.org/api/query?search_query=%28abs:%22velocity+limiting%22+OR+abs:%22trajectory+scaling%22+OR+abs:%22speed+and+separation+monitoring%22%29+AND+cat:cs.RO&start=0&max_results=40&sortBy=submittedDate&sortOrder=descending
https://export.arxiv.org/api/query?search_query=abs:%22teleoperation%22+AND+%28abs:%22anomaly+detection%22+OR+abs:%22tremor%22+OR+abs:%22packet+loss%22%29&start=0&max_results=40&sortBy=submittedDate&sortOrder=descending
https://export.arxiv.org/api/query?search_query=abs:%22tremor%22+AND+%28abs:%22teleoperation%22+OR+abs:%22teleoperated%22%29&start=0&max_results=30&sortBy=submittedDate&sortOrder=descending
https://export.arxiv.org/api/query?search_query=abs:%22control+barrier+function%22+AND+%28abs:%22manipulator%22+OR+abs:%22robot+arm%22%29+AND+abs:%22safety+filter%22&start=0&max_results=40&sortBy=submittedDate&sortOrder=descending
https://api.openalex.org/works?search=control%20barrier%20function%20teleoperation&per-page=50&...
https://api.openalex.org/works?filter=title_and_abstract.search:%22control%20barrier%20function%22%20teleoperation,from_publication_date:2023-01-01,to_publication_date:2025-12-31&per-page=30
https://api.openalex.org/works?filter=title_and_abstract.search:%22safety%20filter%22%20teleoperation,from_publication_date:2023-06-01&per-page=20
https://api.openalex.org/works?filter=title_and_abstract.search:%22speed%20and%20separation%20monitoring%22,from_publication_date:2023-01-01,to_publication_date:2025-12-31&per-page=25
https://api.openalex.org/works?filter=title_and_abstract.search:%22velocity%20scaling%22%20OR%20title_and_abstract.search:%22trajectory%20scaling%22,from_publication_date:2023-01-01,to_publication_date:2025-12-31&per-page=25
https://api.openalex.org/works?filter=title_and_abstract.search:%22fault%20detection%22%20robot%20safety%20AND%20title_and_abstract.search:latching&per-page=20
https://api.openalex.org/works?filter=title_and_abstract.search:%22fault%20detection%22%20manipulator,from_publication_date:2023-01-01,to_publication_date:2025-12-31&per-page=15
https://api.openalex.org/works?filter=title_and_abstract.search:%22runtime%20monitoring%22%20OR%20title_and_abstract.search:%22runtime%20verification%22,from_publication_date:2023-01-01,to_publication_date:2025-12-31&per-page=15
https://api.openalex.org/works?filter=title_and_abstract.search:%22hand%20tremor%22%20OR%20title_and_abstract.search:%22physiological%20tremor%22,from_publication_date:2023-01-01,to_publication_date:2025-12-31&per-page=15
https://api.openalex.org/works?filter=title.search:RAIL%20Reachability-Aided%20Imitation%20Learning&per-page=5
https://api.openalex.org/works?filter=title.search:Collision%20Cone%20Control%20Barrier%20Functions%20manipulators&per-page=5
https://api.crossref.org/works?query.bibliographic=control+barrier+function+teleoperation+safety&rows=40
https://api.crossref.org/works/10.1109/LRA.2025.3595023
https://api.crossref.org/works/10.1109/IROS55552.2023.10341408
https://api.crossref.org/works/10.1109/TMECH.2024.3504573
https://api.crossref.org/works/10.1109/LRA.2025.3568567
https://api.crossref.org/works/10.1109/ICRA57147.2024.10611325
https://api.crossref.org/works/10.1109/TASE.2025.3533583
https://api.crossref.org/works/10.1145/3594806.3596522
https://api.crossref.org/works/10.1016/j.ifacol.2025.12.489
https://api.crossref.org/works/10.1016/j.mechmachtheory.2023.105493

# 抓取失败（未作为证据）
https://ieeexplore.ieee.org/document/11123727      # HTTP 202 空壳
https://ieeexplore.ieee.org/document/10341408      # HTTP 202 空壳
https://dl.acm.org/doi/10.1145/3594806.3596522     # HTTP 403 Cloudflare
https://www.mdpi.com/1424-8220/25/2/405            # HTTP 403 Akamai
https://doi.org/10.1109/LRA.2025.3595023           # 跨域跳转被拦
```
