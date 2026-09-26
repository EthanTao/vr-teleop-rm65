# 会议投稿实务数据（可核查版）

**编制日期**：2026-09-24
**数据来源策略**：只使用官方会议网站 / 出版商页面 / IEEE、ACM 官方页面。每一条声明后面紧跟来源 URL。
**标记约定**：

- ✅ **已核实** = 我在该 URL 上亲眼读到该声明
- **待核实** = 官方页面未找到该信息（不代表"不存在"，只代表"我没能从官方来源确认"）
- 凡属我的换算（例如用官方公布的分子分母做除法），一律标注"**换算**"，不当作官方数字

> ⚠️ 关于"待核实"：本报告刻意不使用第三方聚合站（wikicfp、CCF 列表、公众号、维基、博客）补数字。一个错误 URL 比"待核实"危害更大。

---

## 0. 三个跨会议的关键结论（先看这里）

1. **IEEE VR 的主赛道论文确实走 TVCG，但同时保留独立的会议论文集赛道。** IEEE VR 2026 用**单一投稿 + 统一评审**，产出三种结果：(1) 作为 IEEE TVCG 论文接收并在 IEEE VR 2026 报告；(2) 作为 IEEE VR 2026 **会议论文**接收并在 IEEE VR 2026 报告；(3) 拒稿。会议论文进 IEEE Xplore。来源：[IEEE VR 2026 Call for Papers](https://ieeevr.org/2026/contribute/papers/)

2. **CHI 与 UIST 的"修订再投"机制不同，不要混为一谈。** CHI 有正式的 **revise & resubmit 轮次**（审稿意见返回后有约 1 个月改稿窗口）；UIST **没有正式 R&R 轮**，而是 **conditional accept + camera-ready 阶段核验**，另有极少数论文被 shepherd。两者都在 ACM Digital Library 归档。

3. **"系统集成型"论文在 HCI/VR/图形社区有明文席位，在 CV/多媒体/ML 社区基本没有明文席位。** CHI 把 "Systems, Tools, Architectures, and Infrastructure" 列为正式贡献类型；UIST 有 "Systems Contributions Need To Be Evaluated Holistically" 专节；IEEE VR 把 "System papers" 列为四类贡献之一；SIGGRAPH 明确说重视 "systems" 类贡献。反过来，CVPR/ICCV 主赛道只有 "high-quality, original research" 一句总纲，ACM MM 主赛道开篇是 "novel theoretical and algorithmic solutions"，AAAI 只有 "integrative" 一词，IJCAI 主赛道连这个词都没有。

4. **12 个会议里唯一对"真机"给出实质约束的官方条款在 IJCAI。** IJCAI-ECAI 2026 **AI and Robotics Special Track** 原文要求投稿 "go beyond pure simulation and show relevance to real or realistically embodied robots, including physical platforms and high-fidelity simulators"——硬件不是强制，但**纯仿真被明文排除**。其余 11 个会议的主赛道都没有任何真机/物理硬件要求。

5. **"录取率"是本次核查中最难拿到的一栏。** 12 个会议中只有 3 个公布了可核实的年度录取率（CVPR 2026 绝对数、NeurIPS 2025 = 24.52%、ICLR 2026 = 27.4%）；CHI/UIST/SIGGRAPH 只有官方**定性区间**（约 25%／20–25%／~20–30%）；IEEE VR、ICCV、ACM MM、AAAI、IJCAI 完全没有官方数字 → 一律标"待核实"，不用第三方数字填。

---

## 1. IEEE VR（IEEE Conference on Virtual Reality and 3D User Interfaces）

**主要来源**：[IEEE VR 2026 Call for Papers](https://ieeevr.org/2026/contribute/papers/)（唯一主源，以下未另注者均出自此页）

| 字段 | 内容 | 状态 |
|---|---|---|
| (a) 投稿截止月 | **9 月**。摘要 2025-09-05（强制）；全文 2025-09-12；修订稿 2026-01-16 | ✅ 已核实 |
| (b) 评审周期 | 投稿(2025-09-12) → 初审通知(2025-12-12) = **3.0 个月**；→ camera-ready(2026-01-30) = **4.6 个月**；两轮评审 | ✅ 已核实 |
| (c) 录取率 | **待核实**（CFP 未公布） | 待核实 |
| (d) 页数 | **正文 4–9 页**（含文字、图、表），**不含参考文献**；参考文献不超过 2 页；附录计入页数；最少 4 个整页 | ✅ 已核实 |
| (e) 盲审 | **双盲（强制）**，未匿名直接 desk rejection | ✅ 已核实 |
| (f) 真机要求 | 官方未要求物理硬件；但**要求有评估** | ✅ 已核实（见下） |
| (g) 系统论文 | **明文欢迎**，是四类贡献之一 | ✅ 已核实 |

### 关键原文引用

**TVCG / 会议赛道双轨（这是本次核查的重点）**：

> "This year there is again a SINGLE submission deadline for a unified review process for both the IEEE TVCG and the conference-only papers. The possible outcomes of this unified process are: 1. Accept as IEEE TVCG paper, with presentation at IEEE VR 2026 / 2. Accept as IEEE VR 2026 conference paper, with presentation at IEEE VR 2026 / 3. Reject"

> "**Conditionally accept as IEEE TVCG paper.** Continuing our cooperation with the IEEE Transactions on Visualization and Computer Graphics (TVCG), the top submissions will be considered for publication in a special issue of IEEE TVCG."

> "**Conditionally accept as conference paper.** These submissions will be considered for inclusion in the Proceedings of IEEE VR, archived in the IEEE Xplore Digital Library."

→ 结论：**是**，IEEE VR 主赛道论文进 TVCG 特刊；**也是**，另有一条独立的 conference-only 赛道（进 IEEE Xplore 会议论文集）。两条赛道共用同一份投稿和同一个页数上限。

**页数上限（journal-track 与 conference-track 共用）**：

> "Papers should be 4-9 pages of text, figures, and tables, but NOT including references. References should not exceed 2 pages in length. Note that the minimum length is 4 full pages of text. Appendices are included in the page count."

*注：TVCG 正式发表版的版式与页数由 TVCG camera-ready 指南决定，该指南本报告未能从可抓取的官方页面读到 → 待核实。*

**贡献类型（对"系统集成型"论文最有利的一条）**：

> "Each research paper should provide a contribution covering one or more of the following categories: **methodological, technical, applications, and systems**."
>
> "**System papers** should indicate how the developers integrated techniques and technologies to produce an effective system, and convey any lessons learned in the process."

**评估要求（对应"真机实验"）**：

> "Each paper should include an evaluation of its contributions, such as user studies, benchmarking and/or comparison with existing systems/techniques/methods."

→ 官方**没有**"必须真机/必须物理硬件"的字样；但明确要求给出评估。对 VR 遥操作这类工作，官方口径下"评估"可以是用户研究、benchmark 或与现有系统对比三者之一。

**盲审原文**：

> "IEEE VR 2026 uses a **DOUBLE-BLIND** review process. Failure to anonymize submissions will result in **DESK REJECTION**."

**投稿后须知**：被拒论文**不再**自动转入 IEEE VR Posters，作者需自行投 poster track（2026 新规）。Senior author 全员须登记审稿，严重违规可能被 desk reject。

---

## 2. IEEE TVCG（IEEE Transactions on Visualization and Computer Graphics）

**这一栏的"待核实"最多，原因写在下面。**

| 字段 | 内容 | 状态 |
|---|---|---|
| (a) 投稿截止月 | TVCG 是期刊，滚动投稿（无固定截止）。**具体官方表述待核实** | 待核实 |
| (b) 评审周期 | **待核实**（官方页面未给出投稿→首次决定/→camera-ready 的时长） | 待核实 |
| (c) 录取率 | **待核实**（IEEE 未公布 TVCG 录取率） | 待核实 |
| (d) 页数 | 期刊自身固定上限：**待核实**。作为会议 journal track 时由会议规定，见下（VIS 9+2；IEEE VR 4–9 页不含参考文献） | 部分✅ |
| (e) 盲审 | **待核实** | 待核实 |
| (f) 真机要求 | **待核实**（期刊层面无此表述） | 待核实 |
| (g) 系统论文 | **待核实**（期刊层面无贡献类型清单） | 待核实 |

### 为什么这一栏大量"待核实"（透明说明）

TVCG 的官方投稿信息在以下位置，但**均为 JS 动态渲染页面，抓取后正文为空**，我无法从中读到数字，因此不引用其内容：

- IEEE VGTC 官方期刊投稿/模板页：<https://tc.computer.org/vgtc/publications/journal/>（IEEE VIS 2026 官方页面把此 URL 指定为 TVCG 期刊投稿格式指南，说明它是对口官方页面；但我抓取后只得到标题）
- TVCG 在 IEEE CSDL 的 "Author Information" 页：<https://www.computer.org/csdl/journal/tg/write-for-us/15094>（同上，抓取后正文为空）
- TVCG 的 "Information for Authors" 在 IEEE Xplore 上是 **PDF**（`application/pdf`），抓取工具不支持 → 无法读取

### TVCG 层面**可以**核实的官方事实

- ✅ **IEEE Computer Society 通用期刊作者指南**（官方）："Page length and reference limits **vary by journal**. **All page limits include abstracts, references, and author biographies.**" 且存在 "Mandatory Overlength Page Charge (MOPC)" 与 "Double-Anonymous Review" 两个政策栏目。来源：[IEEE CS Author Guidelines](https://www.computer.org/publications/author-resources)
- ✅ **TVCG 作为会议 journal track 时，页数由会议 CFP 规定**，两个官方实例：
  - IEEE VIS 2026：**9 页正文 + 2 页（主要放参考文献）= 最多 11 页**。"Manuscripts that are submitted with non-reference content such as other text, main-paper figures, or main-paper tables beyond 9 pages may be desk rejected." 来源：[IEEE VIS 2026 Paper Submission Guidelines](https://ieeevis.org/year/2026/info/call-participation/paper-submission-guidelines/)
  - IEEE VR 2026：4–9 页不含参考文献。来源：[IEEE VR 2026 CFP](https://ieeevr.org/2026/contribute/papers/)

> 对 VR 遥操作投稿的实务含义：**走 IEEE VR → TVCG 特刊这条路，提交时按 4–9 页（不含参考文献）准备**，这是官方明文规定的提交格式；TVCG 正式发表版式另行处理。

---

## 3. ACM CHI（CHI Conference on Human Factors in Computing Systems）

**主要来源**：[CHI 2026 Papers](https://chi2026.acm.org/authors/papers/)、[CHI 2026 首页 Important Dates](https://chi2026.acm.org/)、[Guide to a Successful Submission](https://chi2026.acm.org/guide-to-a-successful-submission/)、[CHI Anonymization Policy](https://chi2026.acm.org/chi-anonymization-policy/)、[Contributions to CHI](https://chi2026.acm.org/contributions-to-chi/)、[CHI 2026 官方博客：投稿作者分析](https://chi2026.acm.org/2025/12/22/insights-into-submitting-authors-of-the-papers-track/)

| 字段 | 内容 | 状态 |
|---|---|---|
| (a) 投稿截止月 | **9 月**（每年一次）。Abstract/metadata 2025-09-04（硬截止，逾期不能提交全文）；全文 2025-09-11；视频/补充材料 2025-09-18 | ✅ 已核实 |
| (b) 评审周期 | 投稿(2025-09-11) → 审稿意见返回(2025-11-04) = **1.8 个月**；改稿窗口 11-05～12-04；→ 最终决定(2026-01-15) = **4.1 个月**；→ publication-ready(2026-02-06) = **4.9 个月** | ✅ 已核实 |
| (c) 录取率 | 无年度官方百分比。官方定性表述："typically around **75%**, get rejected"。官方公布的**投稿规模**：CHI 2026 有效投稿 **6,725 篇**（不含已撤回）、独立作者 20,896 人；CHI 2026 官方预计 "over 5,000 submissions" | 部分✅（投稿量）／年度百分比 待核实 |
| (d) 页数 | **按字数**，非页数：Short ≤ 5,000 词；Standard 5,000–12,000 词（均值 7,000–8,000）；> 12,000 词除非强论证否则 **desk reject**。**不计**参考文献、图表题注、附录 | ✅ 已核实 |
| (e) 盲审 | **双盲**（作者与审稿人互不知身份，**但 1AC 知道**）；违规（含补充材料、外部链接）desk reject | ✅ 已核实 |
| (f) 真机要求 | 无普遍硬性要求；但 artifact/technique 类贡献要求"rigorous and convincing validation" | ✅ 已核实（见下） |
| (g) 系统论文 | **是合法贡献类型**，且官方明说系统论文**不一定需要用户研究** | ✅ 已核实 |

### CHI 贡献类型（官方原文，共 8 类）

来源：[Contributions to CHI](https://chi2026.acm.org/contributions-to-chi/)

> "In all cases, a CHI Paper must make an original research contribution. Below you will find some of the types of contributions a paper can make to HCI…"

1. **Development or Refinement of Interface Artifacts or Techniques**
2. **Understanding Users**
3. **Systems, Tools, Architectures, and Infrastructure**
4. **Methodology**
5. **Theory**
6. **Innovation, Creativity, and Vision**
7. **Argument**
8. **Validation and Replication**

（页面原话是 "some of the types"，即这张表非穷尽。）

**第 3 类（系统论文）的完整判据原文**——这条直接决定"集成式遥操作/VR 系统"能不能当主贡献：

> "**Systems, Tools, Architectures, and Infrastructure** contributions address the underlying software and technology associated with a novel interactive application, interface feature, interface design, or development tool. Your paper will likely include answers to the following questions:
> - Do you show that the idea is original, of interest, and valuable to a reasonably broad subset of the CHI community?
> - Is the idea placed in context, where you detail the expected situation, tasks, and users?
> - …
> - Do you provide sufficient information for an experienced researcher to implement a similar system?
> - Can the system work within its proposed settings, or are there significant obstacles to its use?
> - Do you validate the idea? Validation could be achieved by sound argumentation, reflection of experiences, user studies, or other evidence proving its value. … (**Note that a good systems paper does not necessarily require a user study, but it does require an appropriate and reasonable validation.**)"

→ **"系统集成"在 CHI 是正式贡献类型**，且验证方式可以"不要求用户研究"。

### CHI 的 R&R 与归档（本次核查点）

**Revise and Resubmit —— 有**，来源：[CHI 2026 Papers](https://chi2026.acm.org/authors/papers/)：

> "**Revise and Resubmit:** CHI 2026 Papers will continue with a revise and resubmit phase for papers above a certain threshold, where authors can respond to reviewer feedback and improve their paper as part of our review cycle."

改稿窗口：2025-11-05 ～ 2025-12-04（约 1 个月），resubmission 2025-12-04。

审稿结论档位（[Guide to a Successful Submission](https://chi2026.acm.org/guide-to-a-successful-submission/)）："Accept with minor revisions (A), Accept with minor revisions or Revise and resubmit (ARR); Revise and resubmit (RR), Revise and resubmit or Reject (RRX), and Reject (X)"

**是否进 ACM Digital Library —— 是**：

> "Accepted manuscripts appear in the Proceedings of the ACM Conference on Human Factors in Computing Systems, which appears in the ACM Digital Library."

（CHI 2026 起全部论文走 **ACM Open** 开放获取，2026 年过渡期 APC 补贴为会员 $250 / 非会员 $350，来源同上页。）

**时间压力的重要提示（官方原文）**：

> "unlike journal papers, CHI Papers are reviewed mostly on an 'as-is' basis. Unlike typical journals, the time frame for the Revise and Resubmit process is limited, and will not allow authors to execute another extensive implementation and/or evaluation… A submission will thus be rejected if the reviewers feel that the required revisions would not be achievable within this short time frame."

→ 实务含义：**投稿时系统就必须是完整可演示的，不能指望 R&R 期间补做实验**。

---

## 4. ACM UIST（ACM Symposium on User Interface Software and Technology）

**主要来源（2026 为最新周期）**：[UIST 2026 Call for Participation](https://uist.acm.org/2026/cfp/)、[UIST 2025 Call for Participation](https://uist.acm.org/2025/cfp/)、[UIST 2025 Author Guide](https://uist.acm.org/2025/author-guide/)

> ⚠️ **重要变更（UIST 2026 起）**：UIST **从"字数上限"改成了"页数上限"**。官方原文：
>
> "**UIST 2026 will have a page limit instead of a word limit. Therefore, we ONLY accept submissions in the 2-column format.**"
>
> 来源：[UIST 2026 CFP](https://uist.acm.org/2026/cfp/)

| 字段 | 内容（以 UIST 2026 为准） | 状态 |
|---|---|---|
| (a) 投稿截止月 | **3 月底**（分层截止）：摘要 2026-03-24；论文 PDF 2026-03-31；材料 2026-04-09。（UIST 2025 为 4 月初：04-02 / 04-09 / 04-11） | ✅ 已核实 |
| (b) 评审周期 | 论文(2026-03-31) → 初审意见(2026-05-28) = **1.9 个月**；rebuttal 06-05；→ 决定(2026-06-27) = **2.9 个月**；→ 最终接收(2026-08-07) = **4.2 个月**。（2025：2.0 / 2.9 / 3.7 个月） | ✅ 已核实 |
| (c) 录取率 | 无年度官方百分比。官方定性表述："historically in the **20–25%** range" | 部分✅／具体年份百分比 待核实 |
| (d) 页数 | **页数上限**：Standard **10 页** 或 Short **5 页**；**参考文献与附录不计入页数**（但须放进提交的 PDF）；**致谢计入页数**；附录可用任意版式。rebuttal 与 conditional accept 后，可增加不超过原文 **10%** 的篇幅（standard +1 页，short +0.5 页） | ✅ 已核实 |
| (e) 盲审 | **匿名**：1AC 知道作者身份，2AC 与两位外审不知道；违规（含补充材料、外链）desk reject | ✅ 已核实 |
| (f) 真机要求 | **不要求用户研究**；但"papers must support the claims they make with evidence" | ✅ 已核实 |
| (g) 系统论文 | **有专节明文支持** | ✅ 已核实 |

### 页数上限原文（UIST 2026）

来源：[UIST 2026 CFP](https://uist.acm.org/2026/cfp/)

> "**Page limit**: UIST papers can either be standard length papers **(10 pages)** OR short papers **(5 pages)**. A standard length paper represents a substantial contribution which has been thoroughly explored. A short paper represents a smaller contribution which has been thoroughly explored: **short papers are NOT works-in-progress**. … After rebuttal and conditional acceptance, authors may add up to 10% of the length of their paper in addressing reviewers' requests (i.e., up to 1 extra page for a standard length paper, up to 0.5 extra page for a short paper)."
>
> "**References** and **appendices** are excluded from the page count, but should be included in the submitted PDF."
>
> "**Acknowledgements** count towards the page limit."

超限后果："Submissions of **more than 5 pages for short papers or 10 pages for standard papers will be desk rejected.**"

UIST 2026 同时引入 **Assisted Desk Rejection**（与 CHI 2026 同机制），四类触发条件：文献综述严重不足、方法细节严重不足、数据严重不足以支撑结论、**贡献与篇幅严重不匹配**（"a standard length paper making a very small contribution may be desk rejected, as paper length should be commensurate with the size of the contribution"）。

### 旧口径：UIST 2025 的字数上限（历史参考，已不再适用）

> "UIST papers are of variable length. Paper length must be based on the weight of the contribution. … As a guideline, consider
> - **Short Papers** being submissions of 5,000 words or less.
> - **Standard Length Papers** being submissions of between 5,000 and 12,000 words … (about 7,000-8,000 words on average).
> - **References** and **appendices** are excluded from the word count."

超限后果：">5,000 words for short papers or 12,000 words for standard papers will be desk rejected if their length is not justified."

### 系统论文的官方立场（对集成式工作最有利的原文）

来源：[UIST 2025 Author Guide](https://uist.acm.org/2025/author-guide/)

> "**Systems Contributions Need To Be Evaluated Holistically:** The UIST 2025 Papers Chairs ask authors to explicitly state their contributions, including claims and scientific evidence, and ask reviewers to perform a holistic evaluation of papers. … **Just asking for usability testing is not adequate for evaluating complex systems. Neither is the search for 'fatal flaws' or focusing on what the system does not do. A more holistic approach to evaluating systems research is required.**"

该节还引用了 Olsen (UIST 2007) 的系统评估启发式（versatility / expressive leverage / expressive match / scalability）、Greenberg & Buxton (CHI 2008)、Ledo et al. (CHI 2018) 的评估策略分类。

**用户研究不强制（原文）**：

> "**User Studies Are Not Required:** … user tests are not strictly required for UIST papers; rather, papers must support the claims they make with evidence. The form of that evidence will vary based on the claims of the paper."

### 归档与"修订再投"（本次核查点）

**是否进 ACM Digital Library —— 是**，来源：[UIST 2026 CFP](https://uist.acm.org/2026/cfp/)（UIST 2025 原文完全一致）：

> "**Publication** — Archival, i.e. accepted papers will be published in the ACM Digital Library and distributed in digital form to conference attendees"

（UIST 2026 同样全面转入 ACM Open：会员 APC $250 / 非会员 $350 的 2026 过渡补贴，来源同上页。）

**是否有 revise-and-resubmit —— 没有正式 R&R 轮**。UIST 走的是 **rebuttal（约 1 周，5,000 字符）+ conditional accept + camera-ready 核验**，另有极少数 shepherd。原文（[Author Guide](https://uist.acm.org/2025/author-guide/)）：

> "Notification of Decision: After the program committee meeting, authors will be informed if their paper was rejected or **conditionally accepted**. … Conditionally accepted papers need to go through the camera-ready process …, i.e., need to be improved based on the changes described in the rebuttal before final acceptance."
>
> "A very small number of papers may be chosen by the program committee to be **shepherded**. … an AC … will initiate a separate review process with the authors to make sure the paper meets the bar for acceptance before the camera-ready deadline."

**新颖性硬指标（UIST 特有，投稿前务必核对）**：

> "a paper has to include about **70% new material** to be considered a novel contribution over existing **archival** work." / "about **30% new material** … over existing **non-archival** work."

---

## 5. ACM Multimedia（ACM International Conference on Multimedia）

**主要来源**：[ACM MM 2026 Important Dates](https://2026.acmmm.org/site/important-dates.html)、[Call for Technical Papers](https://2026.acmmm.org/site/cfp-guidelines.html)、[Author Instructions](https://2026.acmmm.org/site/author-instructions.html)

| 字段 | 内容 | 状态 |
|---|---|---|
| (a) 投稿截止月 | **3 月底～4 月初**。摘要 2026-03-25；正文 2026-04-01；补充材料 2026-04-08 | ✅ 已核实 |
| (b) 评审周期 | 投稿(2026-04-01) → rebuttal(2026-06-04) → 通知(2026-07-07 见 CFP 页／**2026-07-09** 见 Important Dates 页，**官方两处不一致**) ≈ **3.2–3.3 个月**；→ camera-ready(2026-08-06) ≈ **4.2 个月** | ✅ 已核实（含一处官方不一致） |
| (c) 录取率 | **待核实**（官方未公布主赛道百分比；dataset track CFP 只说"expected acceptance rates will be similar to the ACM Multimedia regular paper track"） | 待核实 |
| (d) 页数 | 主赛道 **6–8 页正文 + 最多 2 页参考文献**（参考页只能放参考文献）；超页 desk reject；主文件内不允许放 appendix | ✅ 已核实 |
| (e) 盲审 | **双盲**（除 Reproducibility / Open Source Software / **Dataset** 三个赛道外，其余全部双盲）。未正确盲化 → 不经评审直接拒 | ✅ 已核实 |
| (f) 真机要求 | 官方无表述 | 待核实 |
| (g) 系统论文 | **无明文"systems paper"条款**，主赛道开篇偏向算法 | ✅ 已核实（见下） |

### 页数与盲审原文

> "Submitted papers may consist of **6 to 8 pages**. Up to **2 additional pages** may be added for references. The reference pages must only contain references. Overlength papers will be desk-rejected without review."

> "Paper submissions must conform with the '**double-blind**' review policy. … Papers without appropriate blinding will be rejected without review."

（赛道页数总表见 [Author Instructions](https://2026.acmmm.org/site/author-instructions.html)：Main / Brave New Ideas / Workshop long 均为 8+2；Dataset 为 6+2；Demo & Video 2 页。）

Dataset track 是**单盲**（"The dataset track follows a single-blind review process, with authors visible to the reviewers."），来源：[Call for Dataset Papers](https://2026.acmmm.org/site/call-datasets.html)

### "系统集成型"贡献的官方口径（对本次项目不利的一点，需正视）

主赛道 CFP 开篇：

> "ACM Multimedia 2026 calls for research papers presenting **novel theoretical and algorithmic solutions** to address problems across multimedia and related application fields."

官方**没有** "systems paper" 这类贡献类型条款。对系统类工作最接近的官方出口是主题清单里的 **Multimedia Applications**：

> "Papers under this topic area should push the envelope of how multimedia can be used to improve the user experience in a rich and meaningful manner for a specific application domain. We solicit papers that **design, implement, and evaluate applications** that employ multimedia data in surprising new ways or in application scenarios that user experience remains challenging based on today's state-of-the-art, such as **immersive telepresence**, distance education, and metaverse."

另有 **Embodied and Immersive Multimedia**、**New Forms of Media Content**（含 VR/AR/MR/XR/360°/haptics）两个主题可对齐。

**必须注意的范围政策**：

> "Our focus lies in promoting research that is inherently multimedia or multimodal in nature. While papers that involve unimedia/unimodal processing will not necessarily be rejected, papers that make multimedia/multimodal research contributions will be preferred… **Out-of-scope submissions may be desk-rejected.**"

→ 纯机器人遥操作（无多模态/多媒体贡献）投 ACM MM 有被判定 out-of-scope 的风险。

**其它硬规则**：每位作者投稿上限 10 篇（Technical + BNI + Dataset 合并计数），超出按 submission ID 顺序 desk reject；投稿即自动承担审稿义务，不审或审稿质量不足可致 desk reject；**rebuttal 期间不得放外链**。

---

## 6. CVPR（IEEE/CVF Conference on Computer Vision and Pattern Recognition）

**主要来源**：[CVPR 2026 CFP](https://cvpr.thecvf.com/Conferences/2026/CallForPapers)、[CVPR 2026 Author Guidelines](https://cvpr.thecvf.com/Conferences/2026/AuthorGuidelines)、[CVPR 2026 Dates](https://cvpr.thecvf.com/Conferences/2026/Dates)、[CVPR 2026 Reviewer Guidelines](https://cvpr.thecvf.com/Conferences/2026/ReviewerGuidelines)、[CVPR 2026 官方新闻稿](https://cvpr.thecvf.com/Conferences/2026/News/Technical_Program)

| 字段 | 内容 | 状态 |
|---|---|---|
| (a) 投稿截止月 | **11 月上中旬**。摘要 2025-11-07 AOE；全文 2025-11-13 AOE；补充材料 2025-11-20 AOE。官方："Date is fixed, no extension will be given" | ✅ 已核实 |
| (b) 评审周期 | 投稿(2025-11-13) → 审稿意见返回(2026-01-22) = **2.3 个月**；rebuttal 01-22～01-29（限一页 PDF）；→ 最终决定(2026-02-20) = **3.3 个月**；camera-ready 截止日官方未公布 | 部分✅／camera-ready 待核实 |
| (c) 录取率 | 官方公布**绝对数**：投稿 16,092 篇（较 2025 增 24%），接收 **4,089 篇**，"about one-quarter were accepted"；程序主席原话录取率 "consistently in the **low-to-mid 20% range**"。**换算** 4,089/16,092 ≈ 25.4%（此百分比是我们的除法，非官方写出） | ✅ 已核实（绝对数）／百分比为换算 |
| (d) 页数 | **正文 8 页**（含图与表）+ **仅参考文献**可加页不限；超限 desk reject | ✅ 已核实 |
| (e) 盲审 | **双盲** | ✅ 已核实 |
| (f) 真机要求 | 官方 CFP 与 Author Guidelines **无任何**真机/物理硬件实验要求 | 待核实（= 官方无此要求） |
| (g) 系统论文 | 主赛道无 systems 条款；官方兜底出口是 **Findings Track** 与 **Demo Track** | ✅ 已核实 |

### 页数与盲审原文

> "Papers are limited to **eight pages, including figures and tables**, in the CVPR style. Additional pages containing only cited references are allowed."

> "CVPR reviewing is **double blind**, in that authors do not know the names of the area chairs or reviewers … and the area chairs/reviewers cannot, beyond a reasonable doubt, infer the names of the authors."

### "系统集成型"贡献的官方口径

主赛道只有一句总纲："Papers in the main technical program must describe **high-quality, original research**." → **关于 systems paper 的明文政策：待核实**。

但有两个官方兜底出口（均已核实）：

- **Findings Track（CVPR 2026 新设）**：官方定位是接纳 "technically sound papers with solid experimental validation, **even if their novelty is more incremental**"。来源：[Author Guidelines](https://cvpr.thecvf.com/Conferences/2026/AuthorGuidelines)
- **Demo Track**：官方原话 "Demos can be as complicated as a **robotics demonstration** or as simple as an app running on a phone or laptop"，且 "demos that do not correspond to any publication are encouraged"（不要求已有论文）。来源：[CVPR 2026 Call for Demos](https://cvpr.thecvf.com/Conferences/2026/CallForDemos)

评审指南的缓和条款（对系统类工作略有利）：每篇被接收论文应 "be technically sound and make a contribution to the field"，且 "the fact that a proposed method does not exceed the state-of-the-art accuracy on an existing benchmark dataset is not grounds for rejection by itself"。

**2026 新增限制**：投稿、补充材料、rebuttal 中**不得包含外部链接**（arXiv / 公开数据集 / 非贡献性 GitHub 除外）。

**下一周期（已公布）**：CVPR 2027 注册 2026-11-10、全文 2026-11-16、补充 2026-11-23、评审返回 2027-01-25、决定 2027-02-25 AOE。来源：[CVPR 2027 CFP](https://cvpr.thecvf.com/Conferences/2027/CallForPapers)。页数/盲审政策页面 `/Conferences/2027/AuthorGuidelines` 目前 404 → 待核实。

---

## 7. ICCV（IEEE/CVF International Conference on Computer Vision）

**主要来源**：[ICCV 2025 CFP](https://iccv.thecvf.com/Conferences/2025/CallForPapers)、[ICCV 2025 Author Guidelines](https://iccv.thecvf.com/Conferences/2025/AuthorGuidelines)、[ICCV 2025 Dates](https://iccv.thecvf.com/Conferences/2025/Dates)、[ICCV 2025 Changes](https://iccv.thecvf.com/Conferences/2025/Changes)

| 字段 | 内容 | 状态 |
|---|---|---|
| (a) 投稿截止月 | **3 月上旬**（奇数年）。Registration 2025-03-03 11:59pm HST；正文+补充材料 2025-03-07 11:59pm HST。官方："Date is fixed, no extension will be given"。2025 起**正文与补充材料同一截止日** | ✅ 已核实 |
| (b) 评审周期 | 投稿(2025-03-07) → 审稿意见返回(2025-05-09) = **2.1 个月**；rebuttal 05-10～05-16；→ 最终决定 **2025-06-25**（Dates 页与 Reviewer Guidelines）**或 2025-06-26**（CFP 页）—— **官方两处不一致**，按 06-25 计 = **3.6 个月**；camera-ready 截止日官方未公布 | 部分✅／camera-ready 待核实 |
| (c) 录取率 | **待核实**（官方无 statistics 页，CFP/Dates/Changes/Reviewer Guidelines 均未公布投稿量或录取率） | 待核实 |
| (d) 页数 | **正文 8 页**（含图与表）+ 仅参考文献可加页；超限 **reject without review** | ✅ 已核实 |
| (e) 盲审 | **双盲** | ✅ 已核实 |
| (f) 真机要求 | 官方无表述 | 待核实 |
| (g) 系统论文 | 主赛道无 systems 条款；兜底出口 **Demo Track** | ✅ 已核实 |

### 页数与盲审原文

> "Papers are limited to **eight pages, including figures and tables**, in the ICCV style. Additional pages containing only cited references are allowed."

> "ICCV reviewing is **double blind**…"

**ICCV 2025 变更点**：正文与补充材料截止日合并，官方理由是防止作者"把关键新实验塞进补充材料"（原文：*"some submissions squeezed in new critical experiments into the supplementary material"*）；论文决定做出后，评审人姓名对同篇论文的其他评审人可见。来源：[ICCV 2025 Changes](https://iccv.thecvf.com/Conferences/2025/Changes)

**"系统集成型"贡献**：与 CVPR 一样，主赛道只有 "high-quality, original research" 总纲 → systems 明文政策 **待核实**。主题清单含 "Vision applications and systems"、"Computer vision for robotics"。CVPR 2026 官方把 Findings Track 称为 "following successful pilots in **ICCV**"，说明 ICCV 是这类"增量新颖性但技术扎实"论文的试验场。Demo track 同样接受 robotics demo 且不要求已有论文（[ICCV 2025 Call for Demos](https://iccv.thecvf.com/Conferences/2025/CallForDemos)）。

**ICCV 2027**：仅会期地点已公布（2027-10-02～08，香港，来源 [CVF 官网](https://www.thecvf.com/)）。投稿截止/页数/盲审：待核实（`/Conferences/2027/CallForPapers` 返回 404）。

---

## 8. NeurIPS（Conference on Neural Information Processing Systems）

**主要来源**：[NeurIPS 2026 CFP](https://neurips.cc/Conferences/2026/CallForPapers)、[NeurIPS 2026 Dates](https://neurips.cc/Conferences/2026/Dates)、[NeurIPS 2026 Main Track Handbook](https://neurips.cc/Conferences/2026/MainTrackHandbook)、[NeurIPS 2026 Reviewer Guidelines](https://neurips.cc/Conferences/2026/ReviewerGuidelines)、[NeurIPS 2025 CFP](https://neurips.cc/Conferences/2025/CallForPapers)、[NeurIPS 2025 Dates](https://neurips.cc/Conferences/2025/Dates)

| 字段 | 内容 | 状态 |
|---|---|---|
| (a) 投稿截止月 | **5 月上旬**。2026：摘要 2026-05-04 AOE；全文（含全部补充材料）2026-05-06 AOE。2025：摘要 05-11、全文 05-15、补充 05-22。**摘要截止先于全文，强制** | ✅ 已核实 |
| (b) 评审周期 | 2026：投稿(05-06) → 意见返回(2026-07-22) = **2.5 个月**；→ 通知 **2026-09-24** AOE = **4.6 个月**。2025：→ 通知 2025-09-18 = 4.1 个月；→ camera-ready 2025-10-23 = 5.3 个月。2026 camera-ready 日期官方未公布 | 部分✅／2026 camera-ready 待核实 |
| (c) 录取率 | **NeurIPS 2025 主赛道 = 24.52%**（官方博客：21,575 篇有效投稿，接收 5,290 篇）。NeurIPS 2026：待核实（决定日即今日） | ✅ 2025／待核实 2026 |
| (d) 页数 | 正文 **9 页**（含全部图与表）；camera-ready **+1 页 = 10 页**。**不计**参考文献、附录、强制 checklist；"no page limit for the technical appendices"。50 MB 上限；**仅支持 LaTeX**（Word 模板已停用）；格式违规 desk reject | ✅ 已核实 |
| (e) 盲审 | **双盲**（含补充材料与代码），违规 desk reject。**允许 arXiv**："The existence of non-anonymous preprints … will not result in rejection" | ✅ 已核实 |
| (f) 真机要求 | 官方无要求 | 待核实（= 官方无此要求） |
| (g) 系统论文 | **最强的官方支持条款之一**（见下） | ✅ 已核实 |

### 页数原文

> "**9 content pages** … including all figures and tables"；camera-ready "+1 page"；"no page limit for the technical appendices"

### "系统集成型"贡献的官方口径（对本次项目最有利的一段）

来源：[NeurIPS 2026 Reviewer Guidelines](https://neurips.cc/Conferences/2026/ReviewerGuidelines)

> "**originality does not necessarily require introducing an entirely new method.** Rather, a work that provides novel insights by evaluating existing methods, or demonstrates improved efficiency, fairness, etc. is also equally valuable."

Use-Inspired 贡献类型：

> "**Originality need not mean wholly novel methods.** It could mean a novel combination of existing methods…"

（附加条件：作者必须"analyze *why* certain methodological choices are effective or ineffective"。）

**NeurIPS 2026 新设 5 种作者自选贡献类型**（提交后不可更改）：General / Theory / Use-Inspired / Concept & Feasibility / Negative Results。来源：[Reviewer Guidelines](https://neurips.cc/Conferences/2026/ReviewerGuidelines)、[NeurIPS 博客 2026-04-16](https://blog.neurips.cc/2026/04/16/a-choice-of-contribution-types-at-neurips-2026/)

**系统/机器人是明文的 in-scope 主题**：2026 CFP 列出 "Robotics, embodied systems, and engineering" 与 "SysML Infrastructure"；2025 CFP 列出 "Infrastructure (e.g., libraries, improved implementation and scalability, distributed solutions)"。来源：[2026 CFP](https://neurips.cc/Conferences/2026/CallForPapers)、[2025 CFP](https://neurips.cc/Conferences/2025/CallForPapers)

### 作者回应（rebuttal）——注意与 ICLR 的区别

2026 分三阶段：作者单独初答 → 作者+评审+AC（07-27～08-03）→ 评审+AC（08-03～08-10）。每条 review 回复上限 **10,000 字符**；**不允许上传文件**；"**Do not use links in any part of the response**"（仅在被评审要求时可给 AC 一个代码链接）。**不允许修改论文**。来源：[Main Track Handbook](https://neurips.cc/Conferences/2026/MainTrackHandbook)

---

## 9. ICLR（International Conference on Learning Representations）

**主要来源**：[ICLR 2027 CFP](https://iclr.cc/Conferences/2027/CallForPapers)、[ICLR 2027 Author Guidelines](https://iclr.cc/Conferences/2027/AuthorGuidelines)、[ICLR 2027 Dates](https://iclr.cc/Conferences/2027/Dates)、[ICLR 2027 Reviewer Guidelines](https://iclr.cc/Conferences/2027/ReviewerGuidelines)、[ICLR 2026 Author Guide](https://iclr.cc/Conferences/2026/AuthorGuide)

> ⏰ **时间敏感**：ICLR 2027 全文截止 **2026-09-25 23:59 AOE（明天）**；摘要截止 2026-09-18 已过，且"no authors can be added or removed after the abstract submission deadline"。若未在 09-18 前登记摘要，本轮 ICLR 2027 **不可达**。

| 字段 | 内容 | 状态 |
|---|---|---|
| (a) 投稿截止月 | **9 月下旬**。2026：摘要 2025-09-19、全文 2025-09-24。2027：摘要 2026-09-18、全文 2026-09-25。**摘要截止强制**；补充材料与正文同一截止 | ✅ 已核实 |
| (b) 评审周期 | 2026：投稿(2025-09-24) → 意见(2025-11-11) = **1.6 个月**；→ 决定(2026-01-25) = **4.0 个月**。2027：投稿(2026-09-25) → 意见(2026-11-05) = **1.4 个月**；→ 决定(2026-12-16) = **2.7 个月**。camera-ready "mid-February 2027"，具体日期待核实 | ✅（除 camera-ready 确切日期） |
| (c) 录取率 | **ICLR 2026 = 27.4%**（官方博客：19,525 篇有效投稿；779 篇 desk reject、5,042 篇撤回；13,763 篇进入决定；接收 5,355 / 拒 8,408）。ICLR 2027：待核实 | ✅ 2026／待核实 2027 |
| (d) 页数 | 投稿 **9 页正文** → discussion/rebuttal 与 camera-ready 阶段 **10 页**；严格执行，超限 desk reject。参考文献"unlimited additional pages"；附录不限（但评审无义务读）。**不计**：ethics statement（≤1 页）、reproducibility statement、acknowledgements、2027 新增的 AI use statement | ✅ 已核实 |
| (e) 盲审 | **双盲（强制）**；正文或补充材料泄露身份 → desk reject。**明确允许 arXiv**："Submission of the paper to archival repositories such as arXiv is allowed during the review period." | ✅ 已核实 |
| (f) 真机要求 | 官方无要求（robotics 只是 in-scope 应用领域） | 待核实（= 官方无此要求） |
| (g) 系统论文 | **无 ICLR 式"无需新颖性"明文条款**（此表述待核实）；但有官方缓和语 | 部分✅ |

### 页数原文

> "9 pages main text at submission → 10 pages during discussion/rebuttal and camera-ready；strictly enforced, desk-reject otherwise"；参考文献 "unlimited additional pages"

### "系统集成型"贡献的官方口径

**注意**：官方**没有**一句"ICLR 不要求新颖性"的话 → 该表述**待核实**。可用的官方缓和语（来源 [ICLR 2027 Reviewer Guidelines](https://iclr.cc/Conferences/2027/ReviewerGuidelines)）：

> "**Be open-minded**: not all work needs to compete on an established leaderboard, or work in an established theoretical framework—instead, ask yourself whether the results are likely to be interesting or surprising to a broad audience."

> "a lack of state-of-the-art results does not by itself constitute grounds for rejection. Submissions bring value to the ICLR community when they convincingly demonstrate new, relevant, impactful knowledge."

Significance 明确包含对 "**for practitioners**" 的价值；"infrastructure, software libraries, hardware, etc." 是 in-scope 主题。ICLR **没有** NeurIPS 那样的贡献类型分类体系。

### ⚠️ ICLR 2027 新增资格门槛（可能直接卡掉投稿）

- 任何作者合作论文 **不得超过 20 篇**；
- **在指定会议列表中没有任何已接收论文的作者，最多只能投 1 篇**；
- 超额部分由系统**随机 desk reject**；
- 合格会议列表包含 **ICRA / IROS / RSS / CoRL**，以及 ICLR/NeurIPS/ICML/UAI/AISTATS/JMLR/TMLR、COLM、CVPR/ICCV/ECCV/PAMI/3DV、ACL 系列、AAAI/IJCAI/JAIR、KDD、COLT。只算主会议论文，不算 workshop/demo/position paper。资格以摘要截止时为准。

来源：[ICLR 2027 Author Guidelines](https://iclr.cc/Conferences/2027/AuthorGuidelines)、[ICLR 博客 2026-09-02](https://blog.iclr.cc/2026/09/02/submission-policies-for-iclr-2027/)

### 作者回应（rebuttal）——与 NeurIPS 的关键结构差异

**ICLR 允许在公开讨论期修改论文 PDF**（系统会生成 pdfdiff）。2027：公开讨论 11-05～11-18；评审+AC 私下讨论 11-19～12-16。代码分享有 3 条官方渠道。来源：[ICLR 2027 Author Guidelines](https://iclr.cc/Conferences/2027/AuthorGuidelines)

**归档的不对称风险**：ICLR 投稿即归档且不可删除——截止后撤回的论文仍公开可见，且"**Withdrawn papers will be de-anonymized immediately**"。

---

## 10. AAAI（AAAI Conference on Artificial Intelligence）

**主要来源**：[AAAI-27 Main Technical Track Call](https://aaai.org/conference/aaai/aaai-27/main-technical-track-call/)、[AAAI-26 Main Technical Track Call](https://aaai.org/conference/aaai/aaai-26/main-technical-track-call/)、[AAAI-27 Submission Instructions](https://aaai.org/conference/aaai/aaai-27/submission-instructions/)、[AAAI-26 Review Process Update](https://aaai.org/conference/aaai/aaai-26/review-process-update/)、[AAAI-26 Author FAQ (Rebuttal)](https://aaai.org/conference/aaai/aaai-26/aaai-26-faq-for-authors-rebuttal/)、[AAAI-27 Demonstrations Call](https://aaai.org/conference/aaai/aaai-27/demonstration-call/)

| 字段 | 内容 | 状态 |
|---|---|---|
| (a) 投稿截止月 | **7 月下旬（摘要）／7 月底–8 月初（全文）**，AoE。AAAI-27：摘要 2026-07-21、全文 2026-07-28、补充 2026-07-31。AAAI-26：摘要 2025-07-25、全文 2025-08-01、补充 2025-08-04 | ✅ 已核实 |
| (b) 评审周期 | AAAI-27：全文(2026-07-28) → Phase-1 拒稿通知(2026-09-24) = **1.9 个月**；作者反馈 10-19～10-25；→ 最终通知(2026-11-30) = **4.1 个月**；→ camera-ready(2026-12-14) = **4.6 个月**。AAAI-26：3.3 / 3.6 个月 | ✅ 已核实 |
| (c) 录取率 | **待核实**（官方站点无任何百分比）。官方**投稿量**数据可查：AAAI-26 主赛道 "almost 29,000 submissions"、约 23,000 篇进入评审（约为 AAAI-25 的两倍） | 待核实（率）／✅（量） |
| (d) 页数 | AAAI-27：主 PDF **最多 9 页，其中第 8–9 页只能放参考文献 → 正文最多 7 页**；ethics statement 必须挤进这 7 页内。AAAI-26：7 页正文 + 参考文献 + 可复现性 checklist（checklist 不计入） | ✅ 已核实 |
| (e) 盲审 | 主赛道 **双盲（强制）**。**Demonstrations Program 例外：作者自选单盲或双盲** | ✅ 已核实 |
| (f) 真机要求 | 主赛道官方无表述。Demo Program 要求可运行实物（含硬件原型），且明确允许 "backup mode (e.g., using system videos and/or simulations on a laptop)" | 主赛道待核实／Demo ✅ |
| (g) 系统论文 | 主赛道承认 **"integrative"** 为合法贡献类型，但仍保留新颖性门槛；系统类工作的官方主场是 **Demonstrations Program** | ✅ 已核实 |

### 页数原文

> AAAI-27 主赛道：main PDF 最多 **9 页**，其中 **pages 8–9 reserved exclusively for references**；overlength 直接拒。
> （"camera-ready 加一页"这条规则：**待核实** —— CFP 与 submission instructions 均未写明，author kit 页面无法抓取。）

### 评审机制原文（"two-phase" 的含义，容易误解）

> "Two-phase reviewing for the Main Track: **Phase 1**: Two reviews supplemented by one AI-generated, non-decisional review. **Phase 2**: Additional reviews for papers not rejected in Phase 1."

→ 是**两阶段评审**，不是"夏/冬两轮投稿窗口"。

### "系统集成型"贡献的官方口径

主赛道把 **"integrative (connecting ideas and methods across disparate subfields of AI)"** 列为可接受的贡献类型，但保留门槛："preferred to papers that advance the state of the art, but only incrementally, or only within a narrow sub-area of AI"。

**最适合集成式系统的是 Demonstrations Program**，其 topics-of-interest 逐字包含：**"Systems that integrate different AI technologies"**、**"Robots"**、**"Virtual and augmented reality systems"**、**"Human-AI interaction (including Human-robot interaction)"**；门槛更低："must present new ideas, which are not already established in mainstream products or services… the demonstration need not run solely at the conference"。

另：**AI for Social Impact** 特别赛道按"技术与社会问题契合度"评审，官方原文 "**rather than simply rewarding technical novelty**"。

AAAI-26 主赛道作者可在投稿时**选择加入 demo session**。

**Demonstrations Program 的关键数字**：页数 2 页正文 + 1 页参考文献；AAAI-27 demo 截止 **2026-09-18**。来源：[AAAI-27 Demonstrations Call](https://aaai.org/conference/aaai/aaai-27/demonstration-call/)

### Rebuttal 与补充材料

> AAAI-26：rebuttal **上限 2,500 字符**，所有 review 共用一份，**不允许放 URL、不允许给新结果、不允许更新补充材料**。

补充材料分三类（technical supplement / multimedia / code and data），截止在全文截止后 **3 天**，且 "reviewers are not required to review this material"。

---

## 11. IJCAI（International Joint Conference on Artificial Intelligence）

**主要来源**：[IJCAI-ECAI 2026 Main Track CFP](https://2026.ijcai.org/ijcai-ecai-2026-call-for-papers-main-track/)、[IJCAI-ECAI 2026 AI and Robotics Track CFP](https://2026.ijcai.org/ijcai-ecai-2026-call-for-papers-ai-and-robotics/)、[IJCAI 2025 Main Track CFP](https://2025.ijcai.org/call-for-papers-main-track/)

| 字段 | 内容 | 状态 |
|---|---|---|
| (a) 投稿截止月 | **1 月中旬**，AoE；摘要比全文早约一周（强制）。IJCAI-ECAI 2026：摘要 2026-01-12、全文+作者信息 2026-01-19。IJCAI 2025：摘要 2025-01-16、全文 2025-01-23 | ✅ 已核实 |
| (b) 评审周期 | 2026：全文(2026-01-19) → Phase-1 summary reject(2026-03-04) = **1.5 个月**；作者回应 04-07～04-10；→ 通知(2026-04-29) = **3.3 个月**。2025：1.4 / 3.2 个月。**主赛道 camera-ready 截止日官方未公布** | 部分✅／camera-ready 待核实 |
| (c) 录取率 | **待核实**（官方各页均无百分比；2026 闭幕 recap 只写"分享了统计"，未印数字） | 待核实 |
| (d) 页数 | 主赛道 2025 与 2026 **一致：共 9 页 = 7 页正文 + 2 页参考文献**；ethics statement 可放任一部分；超页 **rejected without review** | ✅ 已核实 |
| (e) 盲审 | 主赛道 **双盲（强制）**，"Submissions that violate anonymity will be rejected without further review"；容忍预印本。**Demos 赛道为单盲**；**Survey 赛道不匿名**；AI and Robotics 赛道双盲 | ✅ 已核实 |
| (f) 真机要求 | 主赛道无表述。**AI and Robotics 特别赛道有全部 12 个会议中最强的官方硬件语言**（见下） | 主赛道待核实／AI&Robotics ✅ |
| (g) 系统论文 | 主赛道**无 systems 类别**；系统类主场是 **Demonstrations Track** | ✅ 已核实 |

### 页数原文

> 主赛道 2025 与 2026 一致："9 pages total = **7 body + 2 references**"；overlength ⇒ rejected without review。
> IJCAI 2025 另有 "up to two additional pages may be purchased at an additional cost per page"（**付费加页**）——**这句话在 IJCAI-ECAI 2026 主赛道 CFP 中不存在**，因此 2026 是否仍可付费加页：**待核实**。（该句仍出现在 2026 Survey Track CFP 中。）

其它赛道页数（2026）：AI and Robotics 7+2；Demos **3 页正文 + 2 页参考文献 = 5 页**；Survey 7+2 且**不允许补充材料**。

### 真机/物理平台要求（本次核查中唯一有明文的）

**IJCAI-ECAI 2026 AI and Robotics Special Track**，来源：[官方 CFP](https://2026.ijcai.org/ijcai-ecai-2026-call-for-papers-ai-and-robotics/)

> "Submissions should **go beyond pure simulation** and show relevance to real or realistically embodied robots, **including physical platforms and high-fidelity simulators**"

→ 硬件不是**强制**，但**纯仿真被明确排除**。这是 12 个会议里唯一对"真机"给出实质约束的官方条款。

### "系统集成型"贡献的官方口径

**主赛道**：鼓励 "papers on novel AI research problems, **AI techniques for novel application domains**, and papers that cross discipline boundaries within AI"；评选标准是 "novelty and originality of ideas, correctness, clarity, the significance of results, potential impact and quality of the presentation" → **没有 systems-paper 类别，也没有 systems 赛道语言**。

**系统类工作的官方主场是 Demonstrations Track**：官方原文 "The track seeks to bridge theoretical advancements and practical applications in AI"；"Demos should clearly demonstrate the applicability of the chosen AI techniques and **highlight the practical considerations for their deployment**"；论文需描述 "the demonstrated system, its application domain, problem scenario, technology, AI techniques, innovations, and interactive elements"；主题含 "**Robotics and Embodied AI**"；评审看 "originality, innovation, applicability, and potential impact"。

**Survey Track** 要求 "a structured synthesis"、至少一位公认专家作者、"should not be a simple juxtaposition of summaries of papers or groups of papers"，且每位作者限投 1 篇 survey。

### Rebuttal 与补充材料

- 主赛道作者回应窗口 2026-04-07～04-10，"concise and… not intended to create a dialogue"；Phase-1 summary reject 后**无任何回应机会**。
- **AI and Robotics 赛道完全没有 rebuttal**：官方原文 "Differing from the main track, there will be no phase 1 notification and author rebuttal."
- **Demos："There is no rebuttal phase."**
- 补充材料：≤ **50MB**，PDF 或 ZIP，需匿名，两个 CME 条目（"Technical Appendix"、"Resubmission File"），截止与全文相同；"Reviewers may choose to consult the technical appendix at their discretion." 过去 12 个月在他处被拒的论文**必须**提供 resubmission 信息。可复现性指南与 checklist 强制。Survey 赛道**不接受任何补充材料**。

---

## 12. ACM SIGGRAPH / SIGGRAPH Asia（Technical Papers track）

**主要来源**：[SIGGRAPH 2026 Technical Papers](https://s2026.siggraph.org/program/technical-papers/)、[SIGGRAPH 2026 Technical Papers FAQ](https://s2026.siggraph.org/technical-papers-submissions-faq/)、[SIGGRAPH Asia 2026 Technical Papers](https://asia.siggraph.org/2026/submissions/technical-papers/)、[SIGGRAPH Asia 2025 Technical Papers](https://asia.siggraph.org/2025/submissions/technical-papers/)、[SIGGRAPH Asia 2026 Anonymity Policy](https://asia.siggraph.org/2026/submissions/technical-papers/anonymity-policy/)

| 字段 | 内容 | 状态 |
|---|---|---|
| (a) 投稿截止月 | **SIGGRAPH（北美）：1 月中下旬**（Stage1 表单 2026-01-15；Stage2 论文 2026-01-22；上传/COI 2026-01-23）。**SIGGRAPH Asia：5 月中下旬**（2026：表 05-05、论文 05-12、上传 05-13；2025：05-16 / 05-23 / 05-24） | ✅ 已核实 |
| (b) 评审周期 | SIGGRAPH 2026：投稿(2026-01-22) → 意见返回作者(2026-03-05) = **1.4 个月**；rebuttal 截止 2026-03-12；官方："The SIGGRAPH decision process is only about **two months** long"。之后进入 conditional accept → 修订 → camera-ready 二轮核验 | ✅ 已核实（camera-ready 确切日 待核实） |
| (c) 录取率 | 官方定性："the acceptance rate of SIGGRAPH papers has remained stable in the range of **~20-30%**"，且明确 "There is no quota"。年度确切百分比：待核实 | 部分✅ |
| (d) 页数 | **Journal-only 赛道：无最大/最小页数限制。Dual-track（可进会议论文集）：正文不超过 7 页**（不含参考文献与纯图页）；参考文献不限页；**最多 2 页纯图页**，置于参考文献之后；附录必须作为补充材料，不能放进正文。会议论文 camera-ready：**最多 10 页 + 参考文献** | ✅ 已核实 |
| (e) 盲审 | **双盲（强制）**。SIGGRAPH NA："The SIGGRAPH review process is double-blind"、"No reviewers of any paper know the authors' identities"。SIGGRAPH Asia："strives to maintain a fair and equitable, **doubly anonymous** review process" | ✅ 已核实 |
| (f) 真机要求 | 官方无要求（robotics 是列出的领域之一） | 待核实（= 官方无此要求） |
| (g) 系统论文 | **明文重视 systems 类贡献** | ✅ 已核实 |

### SIGGRAPH 的双赛道结构（官方原文）

来源：[SIGGRAPH 2026 Technical Papers](https://s2026.siggraph.org/program/technical-papers/)

> "At SIGGRAPH 2026, the Technical Papers program will have two integrated paper tracks: **Journal (ACM Transactions on Graphics)** and **Conference**."

| | Journal-Only | Dual-Track |
|---|---|---|
| 完整性期望 | "Ideas are extensively tried and tested, and the paper is complete and comprehensive." | "The work can be less complete (e.g., have shorter validation and fewer comparisons)." |
| 页数 | "No maximum (or minimum) page length." | "no longer than **7 pages** excluding references and figures-only pages … at most **two figures-only pages**" |
| 发表处 | ACM TOG | 会议论文集（若按 Conference 接收）或 ACM TOG（若按 Journal 接收） |

**评审宽严差异（官方原文）**：

> "When deciding which track a dual-track submission is suitable for, reviewers will be instructed to consider how compelling the evidence supporting the advances presented in the paper is. In particular, they will be instructed to be **less demanding about formal completeness, validation, and experimental evaluation when it comes to accepting papers to the Conference track**, thereby allowing publication of riskier/earlier-stage work."

→ 对集成式系统论文，**Dual-track 是比 Journal-only 更容易的入口**。

### "系统集成型"贡献的官方口径（明文支持）

来源：[SIGGRAPH 2026 Technical Papers FAQ](https://s2026.siggraph.org/technical-papers-submissions-faq/)

> "We value a diverse set of contributions such as AI or machine learning techniques, algorithms, perceptual studies, **systems**, benchmarks, data sets, and applications. … Last but not least, we encourage creative and original submissions that define their own new area: **The scientific excellence of the ideas is the predominant acceptance criterion.**"

**SIGGRAPH Asia 页数**：SIGGRAPH Asia 2026 技术论文页面的正文因页面导航过长未能完整抓取 → **SIGGRAPH Asia 的页数上限：待核实**（但同一 Technical Papers 项目、同一 TOG/Conference 双赛道框架，页数口径可参考北美站的官方表述）。

**其它已核实要点**：
- Rebuttal 限 **1,000 词纯文本**，不得含图片/视频/外链；评审不得要求新实验；禁止在 rebuttal 中塞入未经要求的新结果。
- 2026 起 ACM 全面 Open Access，**所有全文与短文均需缴纳 APC**，除非通讯作者所属机构在 ACM Open 名单内并使用机构邮箱。
- 允许 arXiv 预印本，但**只可存档、不可宣传**（不得在推特等公开推广），官方给出的容忍理由是 "The SIGGRAPH decision process is only about two months long"。

---

## 汇总表

| Venue | CCF class | Deadline month | Review months | Acceptance rate | Pages | Blind | Real robot needed |
|---|---|---|---|---|---|---|---|
| IEEE VR | ? | 9 月 | 3.0 (→初审) / 4.6 (→camera-ready) | 待核实 | 4–9 页不含参考文献（参考文献 ≤2 页） | 双盲（强制） | 无明文要求；须有评估 |
| IEEE TVCG | ? | 滚动（期刊）；会议 journal track 随会议 | 待核实 | 待核实 | 待核实（VIS 用 9+2；IEEE VR 用 4–9 不含参考文献） | 待核实 | 待核实 |
| ACM CHI | ? | 9 月 | 1.8 (→审稿意见) / 4.1 (→决定) / 4.9 (→publication-ready) | 无年度官方百分比（官方定性：约 75% 被拒；CHI 2026 有效投稿 6,725 篇） | 按字数：Short ≤5k，Standard 5k–12k（不含参考文献/题注/附录） | 双盲（1AC 例外） | 无要求；系统类论文可不做用户研究 |
| ACM UIST | ? | 3 月底（2026）／4 月初（2025） | 1.9 (→初审) / 2.9 (→决定) / 4.2 (→最终接收) | 无年度官方百分比（官方定性：20–25%） | **Standard 10 页 / Short 5 页**（不含参考文献与附录；致谢计入） | 匿名（1AC 例外） | 无要求；用户研究不强制 |
| ACM Multimedia | ? | 3 月底–4 月初 | 3.2–3.3 (→通知) / 4.2 (→camera-ready) | 待核实 | 6–8 页 + ≤2 页参考文献 | 双盲（Dataset 赛道单盲） | 无表述 → 待核实 |
| CVPR | ? | 11 月上中旬 | 2.3 (→审稿意见) / 3.3 (→决定) / camera-ready 待核实 | 2026：16,092 投 / 4,089 收；官方"about one-quarter"、PC 称 "low-to-mid 20%"（换算 ≈25.4%） | 正文 8 页含图表 + 参考文献不限 | 双盲 | 官方无此要求 |
| ICCV | ? | 3 月上旬（奇数年） | 2.1 (→审稿意见) / 3.6 (→决定) / camera-ready 待核实 | 待核实 | 正文 8 页含图表 + 参考文献不限 | 双盲 | 官方无此要求 |
| NeurIPS | ? | 5 月上旬 | 2.5 (→审稿意见) / 4.6 (→通知，2026) | **2025：24.52%**；2026 待核实 | 正文 9 页（camera-ready 10 页）+ 参考文献/附录不限 | 双盲 | 官方无此要求 |
| ICLR | ? | 9 月下旬 | 1.4–1.6 (→审稿意见) / 2.7–4.0 (→决定) | **2026：27.4%**；2027 待核实 | 9 页正文 → 10 页（讨论期与 camera-ready）；参考文献不限 | 双盲 | 官方无此要求 |
| AAAI | ? | 7 月下旬（摘要）／7 月底–8 月初（全文） | 1.9 (→Phase-1 拒稿) / 4.1 (→最终通知，2027) / 4.6 (→camera-ready) | 待核实（官方投稿量：AAAI-26 主赛道近 29,000 篇） | 9 页含参考文献，其中正文最多 7 页 | 双盲（主赛道强制；Demo 自选） | 主赛道无表述；Demo 允许视频/仿真兜底 |
| IJCAI | ? | 1 月中旬（摘要早一周） | 1.5 (→summary reject) / 3.3 (→通知，2026) / camera-ready 待核实 | 待核实 | 9 页 = 7 正文 + 2 参考文献 | 双盲（主赛道；Demos 单盲，Survey 不匿名） | 主赛道无表述；**AI & Robotics 赛道明文"须超越纯仿真"** |
| SIGGRAPH / SIGGRAPH Asia | ? | NA：1 月；Asia：5 月 | ≈2.0 (→决定；官方称"about two months")；camera-ready 待核实 | 无年度官方百分比（官方定性：~20–30%，无配额） | Dual-track 正文 7 页 + 参考文献不限 + ≤2 页纯图页；Journal-only 无页数限制；会议版 camera-ready ≤10 页 + 参考文献 | 双盲（强制） | 官方无此要求 |

---

## 附：本报告中所有"待核实"清单（按会议）

| 会议 | 待核实字段 |
|---|---|
| IEEE VR | 录取率；TVCG 正式发表版页数 |
| IEEE TVCG | 投稿截止表述；评审周期；录取率；期刊自身页数上限；盲审政策；真机要求；系统论文政策 |
| ACM CHI | 确切年度录取率百分比 |
| ACM UIST | 确切年度录取率百分比 |
| ACM Multimedia | 录取率；真机要求 |
| CVPR | camera-ready 截止日；真机要求；"systems paper" 明文政策 |
| ICCV | 录取率；camera-ready 截止日；真机要求；"systems paper" 明文政策；ICCV 2027 投稿/页数/盲审 |
| NeurIPS | 2026 camera-ready 截止日；2026 录取率；真机要求；2026 Evaluations & Datasets 赛道盲审政策 |
| ICLR | 2027 录取率；2027 camera-ready 确切日期；真机要求；"ICLR 不要求新颖性"的明文条款 |
| AAAI | 录取率；camera-ready 加一页规则；AAAI-27 camera-ready（已核实为 2026-12-14，AAAI-26 为 2025-11-16）；主赛道真机要求；AAAI-26 Emerging Trends in AI 赛道 CFP |
| IJCAI | 录取率；主赛道 camera-ready 截止日；主赛道真机要求；2026 是否仍可付费加页；`ijcai.org/authors_kit` 页面内容；IJCAI 历史盲审政策 |
| SIGGRAPH / SIGGRAPH Asia | 年度确切录取率百分比；SIGGRAPH camera-ready 确切截止日；SIGGRAPH Asia 页数上限 |

## 附：官方来源页面自相矛盾之处（如实记录，未做取舍）

1. **ACM MM 2026 通知日**：[CFP 页](https://2026.acmmm.org/site/cfp-guidelines.html) 写 July 7th, 2026；[Important Dates 页](https://2026.acmmm.org/site/important-dates.html) 写 09-July。两处均为官方。
2. **ICCV 2025 最终决定日**：Dates 页与 Reviewer Guidelines 写 2025-06-25；CFP 页写 June 26th, 2025。
3. **ICLR 2026 决定日**：Dates 页与 CFP 写 2026-01-25；Author Guide 写 "Jan 22"。
4. **ICLR 2027 全文截止**：Dates/CFP/Author Guidelines 三处一致写 Sep 25, 2026；Reviewer Guide FAQ 另写 "our full paper deadline is September 16"。且 CFP 中该句以 "or" 结尾，属页面残缺。
