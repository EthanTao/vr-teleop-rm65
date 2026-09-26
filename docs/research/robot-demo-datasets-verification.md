# 机器人演示数据集核验报告（2025 前后）

核验方法：只用 web_search 定位，然后**实际抓取页面**确认。标注 `[OFFICIAL]` = 项目主页/厂商文档/出版商/会议 proceedings；`[arXiv元数据]` = arXiv 摘要页或其镜像（arxiv.org 在本机不可达，改用 alphaxiv 镜像，只采信其中的**摘要原文与引用元数据**，不采信其 AI 生成综述）；`待核实` = 抓不到可靠页面，不猜。

> ⚠️ alphaxiv 页面上的 “AI Overview” 是模型生成内容，本次核验中已发现它与摘要正文冲突（例：RoboMIND 2.0 摘要写 739 tasks，AI Overview 写 759 tasks）。因此凡只出现在 AI Overview 里的事实，本报告一律标为“仅二级来源，未采信”。

---

## ITEM 1 — Fourier ActionNet

| 字段 | 结论 | 来源等级 |
|---|---|---|
| 准确全名 | 官网标题 **“Fourier ActionNet Dataset”**；官网 BibTeX 标题 **“ActionNet: A dataset for dexterous bimanual manipulation”** | [OFFICIAL] |
| arXiv id | **待核实**（未找到任何 arXiv 条目；官网只给 `@article{fourier2025actionnet, ...}`，无 journal / 无 arXiv 号） | — |
| venue + year | **2025 数据集发布，非论文**；同行评审 venue 待核实 | [OFFICIAL] |
| 作者/机构 | Fourier ActionNet Team + Yao Mu（上海交通大学） | [OFFICIAL] |
| 规模 | **“over 30K teleoperated trajectories or around 140 hours of interaction data”**；桌面双臂操作，随机化桌面场景 | [OFFICIAL] |
| 采集方式 | **VR 遥操作，主控设备是 Apple Vision Pro**（不是 PICO）；第一人称视角对齐；视觉用 **OAK-D W 97** 相机；数据来自 **Fourier GR1-T1、GR1-T2、GR2** 三款人形 + **Fourier 灵巧手两种（6 DoF 与 12 DoF）** | [OFFICIAL] |
| 数据格式 | HDF5（robot state/action/timestamp）+ 每 episode 的 rgb.mp4 / depth.mkv / timestamps.json；可转 LeRobotDatasetV2 | [OFFICIAL] |
| 标注 | 用 Qwen2.5-VL-7B 抽 26 帧自动生成语言标注，再人工复核；3 万条样本均有 prompt，平均 53 词 | [OFFICIAL] |
| 开源 | **是（数据非商用许可）**。数据 **CC BY-NC-SA 4.0**；配套训练/评估工具链 **Apache 2.0** | [OFFICIAL] |
| 下载 | 项目页 https://action-net.org/ ；HF `FourierIntelligence/ActionNet`（**需申请、gated**：“You need to agree to share your contact information to access this dataset”，此条来自搜索结果摘要，HF 站点本机 DNS 不可达故未直接抓取）；另提供百度网盘镜像 | [OFFICIAL] + 一条待核实 |

**注意事项**：官方文档页在 `support-old.fftai.com`（旧域名），新域名 `support.fftai.com` 上该路径已 404 —— 引用时用旧域名或 action-net.org。

---

## ITEM 2 — BEHAVIOR Robot Suite (BRS)

| 字段 | 结论 | 来源等级 |
|---|---|---|
| 准确全名 | **“BEHAVIOR Robot Suite: Streamlining Real-World Whole-Body Manipulation for Everyday Household Activities”** | [OFFICIAL] |
| arXiv id | **2503.05652** | [OFFICIAL]（项目页 + 官方文档页均直链） |
| venue + year | **CoRL 2025**（9th Annual Conference on Robot Learning）。项目页 BibTeX：`booktitle={9th Annual Conference on Robot Learning}, year={2025}`，OpenReview id `v2KevjWScT`（OpenReview 论坛页被浏览器人机校验拦截，未直接抓取，但该 id 出自官方项目页） | [OFFICIAL] |
| 作者 | Yunfan Jiang, Ruohan Zhang, Josiah Wong, Chen Wang, Yanjie Ze, Hang Yin, Cem Gokmen, Shuran Song, Jiajun Wu, Li Fei-Fei（Stanford） | [OFFICIAL] |
| 机器人 | **Galaxea R1**（星海图），**不是 RBY1 / Rainbow Robotics**。官方文档原文：JoyLo “paired with the **Galaxea R1 robot**”；致谢感谢 Galaxea.ai 提供硬件支持。双臂轮式移动底盘 + 4-DoF 躯干，整机 21 DoF | [OFFICIAL] |
| 规模（真机任务） | **5 个真实家庭长时程任务**：put items onto shelves / clean the toilet / lay clothes out / clean house after a wild party / take trash outside。JoyLo 采集轨迹数分别为 **100、103、98、138、122**（合计 561），单条 60–210 s | [OFFICIAL] |
| 场景数 | **待核实**（任务分别发生在储藏间、卫生间、卧室、客厅/厨房、客厅+室外，但官方页未给出明确的“场景计数”） | — |
| 采集硬件 | **JoyLo**——运动学同构（kinematic-twin）主从臂 + **Nintendo Joy-Con**，带双边力反馈，**成本 < $500**。**不是 VR 系统**；论文用 10 人用户研究把 JoyLo 与 VR 手柄、Apple Vision Pro 对比，JoyLo 成功率最高、耗时最短、奇异率最低 | [OFFICIAL] |
| 开源 | **是（完全开源）**：数据 https://huggingface.co/datasets/behavior-robot-suite/data ；算法 https://github.com/behavior-robot-suite/brs-algo ；硬件/控制 https://github.com/behavior-robot-suite/brs-ctrl ；文档 https://behavior-robot-suite.github.io/docs/ | [OFFICIAL] |
| 相关但不同的东西 | **BEHAVIOR-1K**（仿真基准）＝ **1,000 个活动 / 50 个全交互场景 / 10,000+ 物体**。BRS 支撑 **NeurIPS 2025 首届 BEHAVIOR Challenge**，官方称该挑战含 **“1,200+ hours of human data collected with JoyLo”** —— 这是**另一个更大的采集**，不要与 BRS 论文的 561 条轨迹混淆 | [OFFICIAL] |

---

## ITEM 3 —(a) GR-1 / GR-2（ByteDance）：是模型，不是数据集

| | GR-1 | GR-2 |
|---|---|---|
| 准确全名 | “Unleashing Large-Scale Video Generative Pre-training for Visual Robot Manipulation” | “GR-2: A Generative Video-Language-Action Model with Web-Scale Knowledge for Robot Manipulation” |
| arXiv id | **2312.13139**（2023-12-21） | **2410.06158**（2024-10-08） |
| 性质 | **模型**：GPT 风格、语言条件的多任务视觉机器人操作模型 | **模型**：生成式 Video-Language-Action 模型 |
| 训练数据 | 大规模视频上做生成式预训练，再在机器人数据上微调。**具体视频数据集身份待核实**（抽象只写 “a large-scale video dataset”） | 摘要明确：**3800 万视频片段、超过 500 亿 token** 做预训练，再用机器人轨迹微调视频生成与动作预测 |
| 发布为数据集？ | **否**（是模型+代码仓库 bytedance/GR-1） | **否**（模型；数据集未作为独立数据发布，待核实） |
| 来源等级 | [arXiv元数据]（摘要原文） | [arXiv元数据]（摘要原文） |

> **命名碰撞（重要）**：
> 1. **Fourier GR-1 / GR-2 是人形机器人**（傅利叶智能 fftai.com），与 ByteDance 的 GR-1/GR-2 模型毫无关系。交叉证据：NVIDIA 官方博客写明 GR00T N1 被部署在 **“the Fourier GR-1 humanoid robot”** 上；傅利叶官方文档中心同时列出 GR-1 / GR-2 / GR-3 / Fourier N1 四款人形。 [OFFICIAL]
> 2. 还有第三个碰撞：**AgiBot 的 “Genie Operator-1 (GO-1)”** 是 AgiBot World 论文提出的策略模型 —— 和 DeepMind 的 Genie 世界模型同名不同物。

## ITEM 3 —(b) 2025 大型机器人数据集（仅列已用 URL 核实的）

格式：`Name | Venue+Year | scale | how collected | open-source`

1. **RoboMIND** | RSS 2025（arXiv:2412.13877） | **107k 条演示轨迹 / 4 种本体（Franka Emika Panda、UR5e、AgileX 双臂、X-Humanoid Tien Kung 双臂灵巧手人形）/ 479 任务 / 96 类物体 / 另含 5k 条真机失败演示（带失败原因）**；Isaac Sim 数字孪生环境 | 统一平台上的**人工遥操作**，标准化协议、多视角观测+本体状态+语言描述 | **yes** — https://huggingface.co/datasets/x-humanoid-robomind/RoboMIND ，项目页 https://x-humanoid-robomind.github.io/ 〔来源：[OFFICIAL] 项目页 + [OFFICIAL] RSS proceedings〕
   - ⚠️ **RoboMIND 不是 ByteDance 的**：作者单位是北京人形机器人创新中心 + 北京大学 + 北京智源（BAAI）。
2. **RoboMIND 2.0** | arXiv:2512.24653 preprint（alphaxiv 标注提交于 2026-02-27） | **310K+ 双臂操作轨迹 / 6 种本体 / 739 个任务**；含 **12K 触觉增强 episode**、**20K 移动操作轨迹**，另发布 **20K 轨迹的仿真数据集**（数字孪生） | 真机采集，具体遥操作硬件在摘要中未说明 → **待核实** | **partial/yes** — 官方项目页公告“RoboMIND V2.0 collection on ModelScope”：https://modelscope.cn/collections/X-Humanoid/RoboMIND20 〔规模数字来源：[arXiv元数据]；ModelScope 链接来源：[OFFICIAL] 项目页公告〕
   - ⚠️ 同一页面 AI Overview 写 “759 tasks / 1,000+ hours”，与摘要的 739 冲突，**不采信**。
3. **AgiBot World** | arXiv:2503.06669（v4） | **超过 100 万条轨迹 / 217 个任务 / 5 类部署场景**；摘要另称相对既有数据集有数量级提升 | 真机遥操作 + human-in-the-loop 质量校验（4000 m² 专用场地等细节仅见于 AI Overview → 未采信） | **yes** — 代码/数据 https://github.com/OpenDriveLab/AgiBot-World ，站点 https://agibot-world.com/ 〔规模来源：[arXiv元数据] 摘要原文〕
4. **NVIDIA Physical AI Dataset（GR00T 开放数据集）** | 2025-03-18 随 GR00T N1 发布（GR00T N1 论文 arXiv:2503.14734） | **15 TB 数据、超过 320,000 条轨迹，另有最多 1,000 个 OpenUSD/SimReady 资产** | **不是遥操作数据**：官方明确这是**标准化合成数据**（Omniverse/Isaac 生成），用于物理 AI 预训练/后训练 | **yes** — https://huggingface.co/collections/nvidia/physicalai-67c643edbb024053dcbcd6d8 〔来源：[OFFICIAL] NVIDIA 官方博客 + NVIDIA 开发者博客〕
   - 补充（[OFFICIAL] NVIDIA 开发者博客）：GR00T blueprint 在 11 小时内生成 **75 万+ 条合成轨迹**（≈6.5K 小时），合成+真机混合比纯真机提升 40%。
5. **DreamGen** | arXiv:2505.12705（NVIDIA、UW、KAIST、UCLA、UCSD、Caltech、NTU、UMD、UT Austin；venue **待核实**——检索结果暗示 CoRL 2025，但未从官方 proceedings 页面确认） | 核心产物是**从视频世界模型生成的合成 “neural trajectories”**；真机实验覆盖 GR1 人形、Franka、SO-100 三种本体 | **不是遥操作数据集**：用图像到视频生成模型产出逼真机器人视频，再用 IDM 或 LAPA 反推伪动作；真机遥操作数据只作为微调种子（“teleoperation data from only a single pick-and-place task in one environment”） | **partial** — 摘要写 “Code available at this https URL”，但该 URL 未能在本次会话中解析出 → 代码仓库 URL **待核实** 〔来源：[arXiv元数据] 摘要原文〕
6. **EgoScale** | arXiv:2602.16710（**2026-02**，NVIDIA 等） | **20,854 小时带动作标注的第一人称人类视频**，据称是此前工作的 20 倍以上；22-DoF 灵巧手策略 | **不是机器人遥操作数据**：在人类第一人称视频上预训练 VLA，再做 human-robot 对齐 mid-training | **待核实**（arXiv 页面未列出代码/数据仓库链接） 〔来源：[arXiv元数据] 摘要原文〕
   - ⚠️ 是 **2026** 年 2 月的论文，不是 2025。用户列表里的 “EgoScale?” 确实存在，但年份对不上。
7. **π0 / openpi**（Physical Intelligence） | arXiv:2410.24164 | **模型**（VLA flow matching，基于 PaliGemma + action expert），不是数据集。摘要只说“在来自多种灵巧机器人平台的大型多样数据集上训练”，**未给出规模数字**；“~10,000 小时 / 7 种机器人配置 / 68 个任务”仅见于 AI Overview → **未采信** | — | openpi 仓库 https://github.com/Physical-Intelligence/openpi 是**代码+权重**；**是否随仓库发布数据集：待核实** 〔来源：[arXiv元数据]〕
8. **VLA-Adapter** | arXiv:2509.09372（AAAI 收录页 https://ojs.aaai.org/index.php/AAAI/article/view/38931 ） | **模型/范式**（0.5B 轻量 VLA，Bridge Attention），**不是数据集**；训练/评测用 LIBERO、CALVIN 和真机（6-DOF Synria Alicia-D） | — | 代码 https://github.com/OpenHelix-Team/VLA-Adapter 〔来源：[arXiv元数据] 摘要原文〕
9. **Google / DeepMind** | — | **未找到任何 2025 年 Google/DeepMind 发布的大型机器人操作数据集** → **待核实**（Gemini Robotics 是模型族，不是数据集） | — | — |
10. **ByteDance** | — | **未找到任何可核实的 2025 年 ByteDance 大型操作数据集发布**（GR-1/GR-2 是模型；RoboMIND 不属 ByteDance） → **待核实** | — | — |

### “Genie” 命名碰撞（按要求显式标注）

- **DeepMind Genie 3**：官方博客 “Genie 3: A new frontier for world models”，2025-08-05，作者 Jack Parker-Holder 与 Shlomi Fruchter，定位是**生成并探索可交互世界的世界模型** —— **不是**机器人操作数据集。 [OFFICIAL] https://deepmind.google/blog/genie-3-a-new-frontier-for-world-models/
- Genie / Genie 2 / Genie 3 三者均属同一世界模型系列；本次只核验了 Genie 3 页面。
- **第二个 “Genie” 碰撞**：AgiBot World 论文的 **Genie Operator-1 (GO-1)** 是策略模型，与 DeepMind Genie 无关。

---

## 本报告使用过的全部 URL

**官方 / 出版商 / proceedings**
- https://action-net.org/ — Fourier ActionNet 官方数据集主页
- https://support-old.fftai.com/docs/Opensource/Teleoperation/dataset_CN/ — 傅利叶官方文档中心（ActionNet 数据结构）
- https://behavior-robot-suite.github.io/ — BRS 官方项目主页
- https://behavior-robot-suite.github.io/docs/sections/getting_started/overview.html — BRS 官方文档（Galaxea R1 出处）
- https://github.com/behavior-robot-suite/brs-algo — BRS 官方算法代码库
- https://roboticsproceedings.org/rss21/p152.html — RSS 2025 RoboMIND 论文页
- https://x-humanoid-robomind.github.io/ — RoboMIND 官方项目主页
- https://modelscope.cn/collections/X-Humanoid/RoboMIND20 — RoboMIND V2.0 官方发布集合
- https://github.com/OpenDriveLab/AgiBot-World — AgiBot World 官方仓库
- https://agibot-world.com/ — AgiBot World 站点
- https://developer.nvidia.com/blog/accelerate-generalist-humanoid-robot-development-with-nvidia-isaac-gr00t-n1/ — NVIDIA 开发者博客（GR00T N1 / Physical AI 数据集）
- https://blogs.nvidia.com/blog/open-physical-ai-dataset/ — NVIDIA 官方博客（15 TB / 320,000+ 轨迹）
- https://github.com/NVIDIA/Isaac-GR00T — NVIDIA Isaac-GR00T 官方仓库
- https://deepmind.google/blog/genie-3-a-new-frontier-for-world-models/ — DeepMind Genie 3 官方博客
- https://www.fftai.cn/about-medium-technical/36 — 傅利叶官网 ActionNet 新闻（重定向至 fftai.com，未直接抓取成功）
- https://ojs.aaai.org/index.php/AAAI/article/view/38931 — AAAI 收录页（VLA-Adapter）
- https://behavior.stanford.edu/ — BEHAVIOR-1K 官网（1,000 任务 / 50 场景）
- https://openreview.net/forum?id=v2KevjWScT — BRS OpenReview 条目（被浏览器校验拦截，未抓取成功）

**arXiv 元数据（经 alphaxiv 镜像抓取；arxiv.org / export.arxiv.org 在本机不可达）**
- https://www.alphaxiv.org/abs/2503.05652 — BRS（arXiv 2503.05652）
- https://www.alphaxiv.org/abs/2312.13139 — GR-1（arXiv 2312.13139）
- https://www.alphaxiv.org/abs/2410.06158 — GR-2（arXiv 2410.06158）
- https://www.alphaxiv.org/abs/2503.06669 — AgiBot World（arXiv 2503.06669）
- https://www.alphaxiv.org/abs/2505.12705 — DreamGen（arXiv 2505.12705）
- https://www.alphaxiv.org/abs/2602.16710 — EgoScale（arXiv 2602.16710）
- https://www.alphaxiv.org/abs/2512.24653 — RoboMIND 2.0（arXiv 2512.24653）
- https://www.alphaxiv.org/abs/2503.14734 — GR00T N1（arXiv 2503.14734）
- https://www.alphaxiv.org/abs/2410.24164 — π0（arXiv 2410.24164）
- https://www.alphaxiv.org/abs/2509.09372 — VLA-Adapter（arXiv 2509.09372）

**数据集卡片 / 其他**
- https://huggingface.co/datasets/behavior-robot-suite/data — BRS 数据集卡片
- https://festivus.hapticlabs.ai/data/datasets/behavior-robot-suite-data — BRS 数据集的第三方索引镜像
- https://festivus.hapticlabs.ai/data/datasets/x-humanoid-robomind-robomind — RoboMIND 数据集的第三方索引镜像
- https://huggingface.co/datasets/x-humanoid-robomind/RoboMIND — RoboMIND HF 数据集（huggingface.co 本机解析到非公网 IP，未直接抓取）
- https://huggingface.co/collections/nvidia/physicalai-67c643edbb024053dcbcd6d8 — NVIDIA Physical AI 数据集集合（同上，未直接抓取）

### 本机网络限制（影响覆盖度，需上级知悉）
- `arxiv.org`、`export.arxiv.org`、`ar5iv.labs.arxiv.org`、`raw.githubusercontent.com` **全部 fetch 失败**，arXiv 元数据只能经 alphaxiv 镜像取得。
- `huggingface.co` 被判定为“解析到非公网 IP”，**无法直接抓取任何 HF 数据集卡片** —— 因此 HF 上的 gated 状态、精确 episode 计数等未能一手确认（相关条目已标注）。
- OpenReview 论坛页被浏览器人机校验拦截。
