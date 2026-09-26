# 力/触觉反馈与双边遥操作（bilateral teleoperation）2023–2025 文献核查

> 核查方法：web_search 找候选 → web_fetch 实际抓取 arXiv abs 页 / DOI 落地页 / 出版社页 / Crossref API 记录 → 抄录标题、venue、年份。
> 已实测：`dl.acm.org`、`ieeexplore.ieee.org`、`doi.org`（IEEE 系）在本环境均被 403/202 拦截；`api.crossref.org`、`arxiv.org/abs`、`researchr.org`、机构库可正常抓取。下列每篇均给出**实际抓取成功的 URL**。
> 未抓到落地页的候选一律不收录；文中不出现任何未经抓取的标题、DOI、arXiv 编号。

---

## 一、核查通过的论文清单（主力 15 篇 + 补充 2 篇 = 17 篇，均 2023–2025）

### A. 无源理论 / 时延稳定性（经典线主力）

1. `Passive Bilateral Surgical Teleoperation With RCM and Spatial Constraints in the Presence of Time Delays` | IEEE Transactions on Robotics (T-RO), vol.41, pp.612–627 | 2025 | 把远心点约束（RCM）与空间约束塞进无源双边框架，在有时延条件下同时保住无源性与手术约束满足。
   URL: https://api.crossref.org/works/10.1109/TRO.2024.3502221

2. `Passivity-Based Control of Distributed Teleoperation With Velocity/Force Manipulability Optimization` | IEEE Transactions on Robotics (T-RO), vol.41, pp.647–665 | 2025 | 多操作者/分布式遥操作中把速度与力的可操作度优化与无源约束耦合求解，避免优化破坏无源性。
   URL: https://api.crossref.org/works/10.1109/TRO.2024.3508192

3. `Enhancing the Force Transparency of the Energy-Reflection-Based Time-Domain Passivity Approach` | IEEE Transactions on Control Systems Technology (TCST), vol.33, pp.181–188 | 2025 | 改进能量反射型 TDPA，在不放弃无源保证的前提下提升力透明度（减少 PC 动作带来的阻尼感）。
   URL: https://api.crossref.org/works?query.bibliographic=Enhancing+the+Force+Transparency+of+the+Energy-Reflection-Based+Time-Domain+Passivity+Approach&rows=3&select=DOI,title,container-title,issued,author,type,page,volume

4. `Passivity-Based Teleoperation With Variable Rotational Impedance Control` | IEEE Robotics and Automation Letters (RA-L), vol.9, no.12, pp.11658–11665 | 2024 | 对变旋转阻抗进行先验无源分析，让阻抗在任务中实时变化时仍不破坏系统无源性。
   URL: https://api.crossref.org/works?query.bibliographic=Passivity-Based+Teleoperation+with+Variable+Rotational+Impedance+Control&rows=3&select=DOI,title,container-title,issued,author,type,page,volume

5. `Actuator Capabilities Aware Limitation for TDPA Passivity Controller Action` | IEEE ICRA 2023, pp.12058–12064 | 2023 | 指出 TDPA 的无源性控制器（PC）动作可能超出执行器实际能力，给出能力感知的限幅，使理论无源在真实硬件上仍成立。
   URL: https://researchr.org/publication/PorciniFSAF23

6. `Mutual Impedance-Based Force/Velocity Transmission Improvement for Bilateral Teleoperation` | IFAC-PapersOnLine, vol.56, pp.9992–9997 (IFAC World Congress) | 2023 | 用「相互阻抗」把力与速度的传输解耦整形，改善双边遥操作的双向传输保真度。
   URL: https://api.crossref.org/works?query.bibliographic=Mutual+Impedance-Based+Force+Velocity+Transmission+Improvement+for+Bilateral+Teleoperation&rows=3&select=DOI,title,container-title,issued,type,page,volume

7. `Mapping for stable bilateral teleoperation of manipulators considering time delays` | International Journal of Systems Science, vol.55, pp.2306–2324 | 2024 | 研究操作端—从动端运动学映射（mapping）本身如何影响时延下的稳定性，把映射从「工程标定」提升为稳定性设计变量。
   URL: https://api.crossref.org/works?query.title=Mapping+for+stable+bilateral+teleoperation+of+manipulators+considering+time+delays&rows=2&select=DOI,title,container-title,issued,type,page,volume

8. `The Classification and New Trends of Shared Control Strategies in Telerobotic Systems: A Survey` | IEEE Transactions on Haptics, vol.16, no.2, pp.118–133 | 2023 | 共享控制策略的系统综述与分类，给出该分支（含触觉共享控制）的术语体系与研究趋势。
   URL: https://api.crossref.org/works?query.title=The+Classification+and+New+Trends+of+Shared+Control+Strategies+in+Telerobotic+Systems+A+Survey&rows=2&select=DOI,title,container-title,issued,type,page,volume

### B. 四通道架构 / 透明度 vs 稳定性

9. `Wave dynamics intrinsic in symmetric four-channel bilateral teleoperation: Mutual impedance-based motion control` | Mechatronics, vol.102, art.103209 | 2024 | 证明对称四通道架构内部本身含波动力学，据此设计运动控制，兼顾四通道的透明度优势与抗时延鲁棒性。**注：Mechatronics 不在目标 venue 列表内，因直接命中「四通道」技术点而保留。**
   URL: https://api.crossref.org/works?query.bibliographic=Wave+dynamics+intrinsic+in+symmetric+four-channel+bilateral+teleoperation+Mutual+impedance-based+motion+control&rows=3&select=DOI,title,container-title,issued,type,page,volume

10. `Relaxing Conservatism for Enhanced Impedance Range and Transparency in Haptic Interaction` | IEEE Transactions on Haptics, vol.17, no.1, pp.100–107 | 2024 | 放松传统无源判据的保守性，换取更大的可呈现阻抗范围与更高透明度——直接对着「透明度 vs 稳定性」这条权衡做文章。
    URL: https://api.crossref.org/works?query.bibliographic=Relaxing+Conservatism+for+Enhanced+Impedance+Range+and+Transparency+in+Haptic+Interaction&rows=3&select=DOI,title,container-title,issued,type,page,volume

### C. 模型介导遥操作 / 共享触觉控制

11. `Digital Twin-empowered Model-Mediated Teleoperation using Multimodality Data with Signed Distance Fields` | IEEE Haptics Symposium (HAPTICS), pp.353–359 | 2024 | 用符号距离场（SDF）构建数字孪生模型，把模型介导遥操作从几何刚体扩展到多模态数据驱动的环境模型。
    URL: https://api.crossref.org/works?query.bibliographic=Digital+Twin-empowered+Model-Mediated+Teleoperation+using+Multimodality+Data+with+Signed+Distance+Fields&rows=3&select=DOI,title,container-title,issued,type,page,volume

12. `Quality-of-Task Perception-Based Optimization of Time-Delayed Teleoperation Using a Hybrid Passivity-Digital Twin Control Strategy` | IEEE World Haptics Conference (WHC), pp.301–307 | 2025 | 把「任务感知质量」（QoP）当作优化目标，混合无源控制与数字孪生控制，在有延迟时按人的感知而非纯信号指标调参。
    URL: https://api.crossref.org/works?query.title=Quality-of-Task+Perception-Based+Optimization+of+Time-Delayed+Teleoperation+Using+a+Hybrid+Passivity-Digital+Twin+Control+Strategy&rows=3&select=DOI,title,container-title,issued,type,page,volume

13. `A Haptic Shared Autonomy With Partial Orientation Regulation for DoF Deficiency in Remote Side` | IEEE Transactions on Haptics, vol.16, no.1, pp.86–95 | 2023 | 针对从动端自由度不足（DoF deficiency），用触觉共享自主做部分姿态调节，把人的意图与自治的补全在力通道上融合。
    URL: https://api.crossref.org/works?query.bibliographic=A+Haptic+Shared+Autonomy+With+Partial+Orientation+Regulation+for+DoF+Deficiency+in+Remote+Side&rows=3&select=DOI,title,container-title,issued,type,page,volume

14. `Safe haptic teleoperations of admittance controlled robots with virtualization of the force feedback` | IEEE CASE 2024, pp.1574–1579 | 2024 | 导纳控制从动端 + 力反馈虚拟化，在不装力/力矩传感器的前提下给出安全触觉遥操作方案（阻抗/导纳取舍的工程解法）。**注：CASE 不在目标 venue 列表内，因直接命中「导纳控制」技术点而保留。**
    URL: https://api.crossref.org/works?query.bibliographic=Safe+haptic+teleoperations+of+admittance+controlled+robots+with+virtualization+of+the+force+feedback&rows=3&select=DOI,title,container-title,issued,type,page,volume

### D. 学习 + 无源混合 / 力反馈感知评测

15. `Sensorless Transparency Optimized Haptic Teleoperation on the da Vinci Research Kit` | IEEE Robotics and Automation Letters (RA-L), vol.9, no.2, pp.971–978 | 2024 | 在 da Vinci 研究平台上无传感器地优化透明度，把透明度从理论指标落到真实手术机器人平台的工程实现与实验。
    URL: https://api.crossref.org/works?query.bibliographic=Sensorless+Transparency+Optimized+Haptic+Teleoperation+on+the+da+Vinci+Research+Kit&rows=3&select=DOI,title,container-title,issued,type,page,volume

16. `A Pragmatic Approach to Bi-Directional Impedance Reflection Telemanipulation Control: Design and User Study` | IEEE Robotics and Automation Letters (RA-L), vol.9, no.4, pp.3617–3624 | 2024 | 提出务实的双向阻抗反射控制设计，并配用户实验评测操作者感受——「控制设计 + 人因评测」合体的典型样本。
    URL: https://api.crossref.org/works?query.title=A+Pragmatic+Approach+to+Bi-Directional+Impedance+Reflection+Telemanipulation&rows=2&select=DOI,title,container-title,issued,type,page,volume

### E. 补充（预印本 / 邻近 venue，标注清楚，不计入上表主力）

16. `Beyond Wave Variables: A Data-Driven Ensemble Approach for Enhanced Teleoperation Transparency and Stability` | arXiv preprint (eess.SY), arXiv:2512.08436 | 2025（2025-12-09 提交） | 用 LSTM/CNN-LSTM + stacking 集成模型**替换**传统波变量变换，针对四通道架构在变时延与噪声下追求与波变量相当的透明度并在无源约束下保稳定。**注：预印本，未见正式 venue，属「待核实发表去向」。**
    URL: https://arxiv.org/abs/2512.08436

17. `Assessment of Force Feedback Models in a Haptic Device Using Alignment Accuracy and Brain Activity` | IEEE EMBC 2024, pp.1–4 | 2024 | 用对齐精度 + 脑电活动评估力反馈模型质量，属于「力反馈设备与操作者感知」评测方法学。**注：EMBC 不在目标 venue 列表内，仅作感知评测线索。**
    URL: https://api.crossref.org/works?query.title=Assessment+of+Force+Feedback+Models+in+a+Haptic+Device+Using+Alignment+Accuracy+and+Brain+Activity&rows=2&select=DOI,title,container-title,issued,type,page,volume

---

## 二、未能核实 / 主动弃用的候选（透明记录）

| 候选 | 处理 | 原因 |
|---|---|---|
| EuroHaptics 2023/2024 遥操作论文 | **待核实，未收录** | 未抓到任何 EuroHaptics 论文的可确认落地页；Crossref 对 Springer LNCS 会议录检索未能定位到具体的遥操作论文。 |
| IEEE VR 2023–2025 力反馈遥操作论文 | **待核实，未收录** | 同上，未抓到可确认记录，不做推测性填写。 |
| `Development of an Open Experimental Apparatus for the Comparison of Haptic Teleoperation Control Systems` | 弃用 | Crossref 显示 venue 为 **HORA 2024**（Human-Computer Interaction, Optimization and Robotic Applications），非目标 venue。 |
| `Optimized time-domain control of passive haptic teleoperation systems for multi-DoF interaction` | 弃用 | 检索到 HAL/UniSI 机构库条目，但无法确认发表年份落在 2023–2025，故不收录。 |

---

## 三、核心结论：这条线 2023–2025 还有没有发表空间？

**结论：有，而且是稳定接收，不是残余阵地。** 依据上面实际抓取的分布：

1. **仍在稳定接收的 venue（按我抓到的集中度）**：IEEE TCST 是能量反射/时间域无源（TDPA）这条线最稳定的主阵地；T-RO 接收「无源 + 复杂系统集成」类（手术 RCM 约束、分布式/多操作者可操作度优化）；T-Haptics 集中在共享自主、透明度—阻抗范围权衡、共享控制综述；RA-L 最快且明确接收透明度优化与阻抗反射这类「设计 + 实验/用户研究」；WHC 与 IEEE Haptics Symposium 是模型介导遥操作与感知驱动优化的首选；ICRA 接收无源理论的**实现层瓶颈**论文（例如执行器能力限制 PC 动作）。IFAC-PapersOnLine（IFAC WC）继续收传输整形类工作。
2. **理论（无源/时延）类落在哪**：我这一轮针对 TAC / Automatica 的多次检索**没有命中**双边遥操作无源/时延的 2023–2025 条目（注意：这是「未命中」，不等于「已确认零篇」，因本环境无法直接检索 IEEE Xplore / ScienceDirect 全库）；实际抓到的理论落点是 **TCST、T-RO、IFAC-PapersOnLine**。从抓到的 T-RO 两篇看，门槛特征是：都带硬约束或多机/多目标扩展，纯控制器微调型理论难以进入。
3. **学习 + 无源混合类落在哪**：这一块目前**主要在预印本与偏应用会场**（arXiv eess.SY、CASE、RO-MAN 之类），我抓到的学习类代表作 `arXiv:2512.08436` 只有预印本、无正式 venue。也就是说：审稿人尚未把关的学习+无源工作很多，**顶刊正式接收的很少**——这既是机会（题新）也是风险（缺少可信基准）。
4. **是否已被认为「经典问题做完了」**：**不是，但重心搬了。** 波变量、散射变换、TDPA 这些**机理本身**已基本封闭，2023–2025 仍在发的不是「再证一遍无源」，而是三件别的事：(a) 把无源判据的**保守性**换成绩效（T-Haptics 2024 放松保守性、TCST 2025 能量反射提透明度）；(b) 让理论在**真实硬件约束**下依然成立（ICRA 2023 执行器能力限制）；(c) 换**评价指标**，从信号层无源性转到人的感知层（WHC 2025 的 QoP、RA-L 2024 的用户研究）。纯理论增量空间窄，**「无源 + 具体瓶颈 + 人的指标」三者组合**才是当前可发区间。
5. **对工程项目的直接含义**：如果目标是发论文，最现实的落点是 RA-L（快、收设计+实验）与 TCST（无源实现细节），其次是 WHC/Haptics Symposium（感知评测）；T-RO 需要系统级新问题（如力/速度可操作度、约束满足）而非控制器微调。若只做「加个能量罐让机械臂不抖」，按本轮分布判断**不足以**成为独立贡献。

---

## 四、抓取记录（实际成功的 URL 原样）

- https://api.crossref.org/works/10.1109/TRO.2024.3502221
- https://api.crossref.org/works/10.1109/TRO.2024.3508192
- https://api.crossref.org/works?query.bibliographic=Passivity-Based+Teleoperation+with+Variable+Rotational+Impedance+Control&rows=3&select=DOI,title,container-title,issued,author,type,page,volume
- https://api.crossref.org/works?query.bibliographic=Enhancing+the+Force+Transparency+of+the+Energy-Reflection-Based+Time-Domain+Passivity+Approach&rows=3&select=DOI,title,container-title,issued,author,type,page,volume
- https://researchr.org/publication/PorciniFSAF23
- https://api.crossref.org/works?query.bibliographic=Mutual+Impedance-Based+Force+Velocity+Transmission+Improvement+for+Bilateral+Teleoperation&rows=3&select=DOI,title,container-title,issued,type,page,volume
- https://api.crossref.org/works?query.title=Mapping+for+stable+bilateral+teleoperation+of+manipulators+considering+time+delays&rows=2&select=DOI,title,container-title,issued,type,page,volume
- https://api.crossref.org/works?query.bibliographic=Wave+dynamics+intrinsic+in+symmetric+four-channel+bilateral+teleoperation+Mutual+impedance-based+motion+control&rows=3&select=DOI,title,container-title,issued,type,page,volume
- https://api.crossref.org/works?query.bibliographic=Relaxing+Conservatism+for+Enhanced+Impedance+Range+and+Transparency+in+Haptic+Interaction&rows=3&select=DOI,title,container-title,issued,type,page,volume
- https://api.crossref.org/works?query.bibliographic=Digital+Twin-empowered+Model-Mediated+Teleoperation+using+Multimodality+Data+with+Signed+Distance+Fields&rows=3&select=DOI,title,container-title,issued,type,page,volume
- https://api.crossref.org/works?query.title=Quality-of-Task+Perception-Based+Optimization+of+Time-Delayed+Teleoperation+Using+a+Hybrid+Passivity-Digital+Twin+Control+Strategy&rows=3&select=DOI,title,container-title,issued,type,page,volume
- https://api.crossref.org/works?query.bibliographic=A+Haptic+Shared+Autonomy+With+Partial+Orientation+Regulation+for+DoF+Deficiency+in+Remote+Side&rows=3&select=DOI,title,container-title,issued,type,page,volume
- https://api.crossref.org/works?query.bibliographic=Safe+haptic+teleoperations+of+admittance+controlled+robots+with+virtualization+of+the+force+feedback&rows=3&select=DOI,title,container-title,issued,type,page,volume
- https://api.crossref.org/works?query.bibliographic=Sensorless+Transparency+Optimized+Haptic+Teleoperation+on+the+da+Vinci+Research+Kit&rows=3&select=DOI,title,container-title,issued,type,page,volume
- https://api.crossref.org/works?query.title=A+Pragmatic+Approach+to+Bi-Directional+Impedance+Reflection+Telemanipulation&rows=2&select=DOI,title,container-title,issued,type,page,volume
- https://arxiv.org/abs/2512.08436
- https://api.crossref.org/works?query.title=Assessment+of+Force+Feedback+Models+in+a+Haptic+Device+Using+Alignment+Accuracy+and+Brain+Activity&rows=2&select=DOI,title,container-title,issued,type,page,volume

**抓取失败的 URL（本环境拦截，非论文不存在）**：
- https://dl.acm.org/doi/10.1109/TRO.2024.3502221 （HTTP 403）
- https://dl.acm.org/doi/abs/10.1109/TRO.2024.3508192 （HTTP 403）
- https://ieeexplore.ieee.org/document/10740678 （HTTP 202，空内容）
- https://ieeexplore.ieee.org/document/10325584 （HTTP 202，空内容）
- https://doi.org/10.1109/TRO.2024.3502221 （跨域重定向到 IEEE，不跟随）
- https://pubmed.ncbi.nlm.nih.gov/37028362/ （HTTP 203，空内容）
