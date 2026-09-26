# AAAI / IJCAI Submission-Practicality Report

**Compiled:** 2026-09-24
**Sourcing rule applied:** official venue domains only (`aaai.org`, `ijcai.org`, `2025.ijcai.org`, `2026.ijcai.org`). Every claim is either **VERIFIED** with the URL it was read on, or exactly **待核实**.
**Retrieval note:** `aaai.org` and the IJCAI conference sites run WordPress and expose a REST API; most full page text below was read via `.../wp-json/wp/v2/...` routes on those same official domains (plain HTML fetches of the rendered pages were truncated to the page title). The `wp-json` URLs are the exact URLs fetched; the canonical human-readable page URL is given alongside.

---

## 0. Which cycle is "most recent"

| Venue | Most recent cycle covered | Next cycle covered |
|---|---|---|
| AAAI | **AAAI-26** (40th), Singapore, January 20–27, 2026 | **AAAI-27** (41st), Montréal, Canada, February 16–23, 2027 — fully announced |
| IJCAI | **IJCAI 2025** (34th), Montréal, Aug 16–22, 2025 (+ Guangzhou satellite Aug 28–31, 2025) | **IJCAI-ECAI 2026** (35th), Bremen, Germany, Aug 15–21, 2026 — fully announced, already held |

- AAAI-26 dates/venue: VERIFIED — https://aaai.org/wp-json/wp/v2/conference/92480 (page: https://aaai.org/conference/aaai/aaai-26/main-technical-track-call/)
- AAAI-27 dates/venue: VERIFIED — https://aaai.org/wp-json/wp/v2/conference/95937 (page: https://aaai.org/conference/aaai/aaai-27/main-technical-track-call/)
- IJCAI 2025 dates/venue: VERIFIED — https://2025.ijcai.org/wp-json/wp/v2/pages?slug=call-for-papers-main-track (page: https://2025.ijcai.org/call-for-papers-main-track/)
- IJCAI-ECAI 2026 dates/venue: VERIFIED — https://2026.ijcai.org/wp-json/wp/v2/pages/2117 (page: https://2026.ijcai.org/ijcai-ecai-2026-call-for-papers-main-track/)

---

## AAAI

### (a) Deadline month(s) / deadline pattern

**Pattern: one annual submission window per track, in late July / early August (abstracts ~1 week before full papers; supplementary ~3 days after full papers). All deadlines are "anywhere on earth" (UTC-12).**

AAAI-27 Main Technical Track — **VERIFIED** (https://aaai.org/wp-json/wp/v2/conference/95937):
- June 17, 2026 — OpenReview opens for author registration
- June 30, 2026 — OpenReview opens for paper submission
- **July 21, 2026 — Abstracts due**
- **July 28, 2026 — Full papers due**
- July 31, 2026 — Supplementary material and code due

AAAI-26 Main Technical Track — **VERIFIED** (https://aaai.org/wp-json/wp/v2/conference/92480 and https://aaai.org/wp-json/wp/v2/conference/92486):
- **July 25, 2025 — Abstracts due**
- **August 1, 2025 — Full papers due**
- August 4, 2025 — Supplementary material and code due

AAAI-27 AI Alignment Special Track runs on a **separate, ~3 weeks later** window — **VERIFIED** (https://aaai.org/wp-json/wp/v2/conference/96332): abstracts August 14, 2026; full papers August 21, 2026; supplementary August 24, 2026.

**Two-phase (summer/winter) submission model: NO.** AAAI's "two-phase" means **two-phase reviewing of a single submission**, not two submission windows. Official wording — **VERIFIED** (https://aaai.org/wp-json/wp/v2/conference/95937):
> "Two-phase reviewing for the Main Track: Phase 1: Two reviews supplemented by one AI-generated, non-decisional review. Phase 2: Additional reviews for papers not rejected in Phase 1. Author response after Phase 2, only for papers not rejected in Phase 1."

I found **no** official statement of a second (winter/spring) submission window for the AAAI main track.

### (b) Review cycle length

AAAI-27 (**VERIFIED**, https://aaai.org/wp-json/wp/v2/conference/95937):
- Full paper July 28, 2026 → Phase 1 rejection notification **September 24, 2026** ≈ **1.9 months**
- Author feedback window **October 19–25, 2026**
- Final acceptance/rejection notification **November 30, 2026** ≈ **4.1 months** after submission
- Camera-ready **December 14, 2026** ≈ **4.6 months** after submission
- Conference February 16–23, 2027

AAAI-26 (**VERIFIED**, https://aaai.org/wp-json/wp/v2/conference/92480 and https://aaai.org/wp-json/wp/v2/conference/92486):
- Full paper August 1, 2025 → Phase 1 rejection notification **September 15, 2025** ≈ **1.5 months**
- Author feedback window **October 7–13, 2025**
- Final notification **November 8, 2025** ≈ **3.3 months** after submission
- Camera-ready **November 16, 2025** (struck through and moved from November 13, 2025) ≈ **3.5 months** after submission
- Conference January 20–27, 2026

⚠️ **Caveat — two AAAI-26 date versions coexist on aaai.org.** Older AAAI-26 pages still show Phase-1 notification Sept 8, 2025; feedback Oct 2–8, 2025; final notification Nov 3, 2025; camera-ready Nov 13, 2025 (https://aaai.org/wp-json/wp/v2/conference/93550 — page: https://aaai.org/conference/aaai/aaai-26/aaai-26-faq-for-authors-rebuttal/ ; also https://aaai.org/wp-json/wp/v2/conference/93026). The newer values (Sept 15 / Oct 7–13 / Nov 8 / Nov 16) appear on both the AAAI-26 Main Technical Track CFP and the AAAI-26 Submission Instructions, so I treat the newer set as authoritative. AAAI-26 also publicly stated the schedule slipped — **VERIFIED** (https://aaai.org/wp-json/wp/v2/conference/93026): *"the review process is running slightly behind schedule."*

### (c) Acceptance rate

**待核实** — no acceptance rate is published on any official AAAI page I could fetch. I searched the AAAI site search API for "acceptance rate" and "statistics", listed all AAAI news posts matching "AAAI-26" (only one non-relevant post), and checked the AAAI-26 Main Technical Track CFP, Submission Instructions, Review Process Update and FAQ pages. No official percentage was found. The AAAI-26 opening-ceremony slide deck (https://aaai.org/wp-content/uploads/2026/01/AAAI-26-Opening-Ceremony.pdf) is an official aaai.org PDF but the fetch tool cannot read `application/pdf`, so I could not verify its contents.

**Official submission-volume statistics that ARE published** (these are submission counts, not an acceptance rate) — **VERIFIED** (https://aaai.org/wp-json/wp/v2/conference/93026, page: https://aaai.org/conference/aaai/aaai-26/review-process-update/):
- "AAAI-26 received almost **29,000** submissions to the Main Technical Track. After removing papers that were not fully compliant with submission policies (...), we still have roughly **23,000** papers under review – nearly twice the number of papers reviewed by AAAI-25!"
- "AAAI-26 saw a much higher-than-expected number of submissions from **China**—almost **20,000** of the ~29,000 total"
- "**28,000+** Program Committee members, Senior Program Committee members, and Area Chairs" recruited
- "**75,000+ unique submitting authors**"

### (d) Page limit

AAAI-27 main submission — **VERIFIED** (https://aaai.org/wp-json/wp/v2/conference/96119, page: https://aaai.org/conference/aaai/aaai-27/submission-instructions/):
> "The main submission PDF can have up to **9 pages**, with pages **8–9** reserved *exclusively* for references. That is, the main paper can have up to **7 pages of non-references content**. Acknowledgements should be omitted from papers submitted for review, and any ethical statements or similar considerations, if included, must be included in the 7 content pages."

Same rule restated in the AAAI-27 CFP — **VERIFIED** (https://aaai.org/wp-json/wp/v2/conference/95937): *"Submissions are limited to 7 pages of main content, with a maximum total length of 9 pages. Any pages beyond page 7 are reserved exclusively for references."*

AAAI-26 main submission — **VERIFIED** (https://aaai.org/wp-json/wp/v2/conference/92486): *"Submissions may consist of up to **7 pages of technical content plus additional pages solely for references and the reproducibility checklist**"*; and (https://aaai.org/wp-json/wp/v2/conference/92480): *"Submissions may consist of up to 7 pages of technical content plus additional pages solely for references."* For AAAI-26 the reproducibility checklist, placed in the PDF after the references, *"will not count toward the page limit."*

AAAI-27 AI Alignment Special Track — **VERIFIED** (https://aaai.org/wp-json/wp/v2/conference/96332): *"Submissions are limited to 7 pages of main content, with a maximum total length of 9 pages. Any pages beyond page 7 are reserved exclusively for references."*

AAAI-27 Demonstrations Program — **VERIFIED** (https://aaai.org/wp-json/wp/v2/conference/96216): *"A two-page short paper describing their system, plus one page of references only."*

**"Camera-ready gets one extra page" rule: 待核实.** Neither the AAAI-26 nor the AAAI-27 Submission Instructions, nor either Main Technical Track CFP, states any page allowance increase for camera-ready. I could not fetch the AAAI author-kit pages (`https://aaai.org/authorkit27/` and `https://aaai.org/authorkit26-1/` both failed to fetch), which is where such a rule would most likely live, so I cannot rule it out.

### (e) Double-blind / single-blind / optional

AAAI-26 and AAAI-27 **Main Technical Track: double-blind (mandatory anonymisation)** — **VERIFIED**:
- AAAI-27 (https://aaai.org/wp-json/wp/v2/conference/96119): *"AAAI submissions are anonymous and must conform to the instructions (detailed below) for **double-blind** review. The authors must remove all author and affiliation information from their submission for review..."*
- AAAI-26 (https://aaai.org/wp-json/wp/v2/conference/92486): same sentence, verbatim.

AAAI-27 **Demonstrations Program: author's choice of single-blind OR double-blind** — **VERIFIED** (https://aaai.org/wp-json/wp/v2/conference/96216):
> "In accordance with past years of AAAI, submissions may be either single-blind (i.e., reviewers are blind, authors have names on submission) or double-blind (i.e., authors and reviewers are blind). Submissions in either format are appropriate for the demos track."

### (f) Real-robot / physical hardware required?

**待核实** for the AAAI main technical track — the AAAI-26 and AAAI-27 main-track CFPs contain no requirement, expectation, or statement about physical hardware or real-robot experiments. (Robotics appears only as a scope keyword: *"The conference scope includes machine learning, natural language processing, computer vision, data mining, multiagent systems, knowledge representation, human-in-the-loop AI, search, planning, reasoning, **robotics and perception**, and ethics"* — https://aaai.org/wp-json/wp/v2/conference/95937.)

The AAAI-27 **Demonstrations Program** does expect a working physical/demo artefact — **VERIFIED** (https://aaai.org/wp-json/wp/v2/conference/96216): *"presenting software and hardware systems and research prototypes of such systems, including their capabilities and workings. All accepted demos will be allocated time for live demonstrations."* It also explicitly anticipates hardware constraints: *"It is expected that submitted demonstrations have a 'backup' mode (e.g., using system videos and/or simulations on a laptop) that is not contingent on a special arrangement."*

### (g) Tolerance for "system integration" style contributions

**VERIFIED — AAAI explicitly lists "integrative" as an accepted contribution type** (https://aaai.org/wp-json/wp/v2/conference/95937, page: https://aaai.org/conference/aaai/aaai-27/main-technical-track-call/):
> "The contributions may be theoretical, methodological, algorithmic, empirical, **integrative (connecting ideas and methods across disparate subfields of AI)**, or critical (e.g., principled analyses and arguments that draw attention to problematic choice of goals, assumptions, or approaches)."

But the same CFP sets a novelty bar that disfavours pure engineering — **VERIFIED** (same URL):
> "All submissions will be evaluated and scored for the significance and novelty of the contributions (research problems or questions addressed, methods, experiments, analyses)..."
> "Solid technical papers that explore new territory or point out new directions for research, introduce new problems, address research questions, or introduce methods that are of interest beyond a single sub-area of AI are preferred to papers that advance the state of the art, but only incrementally, or only within a narrow sub-area of AI."

**VERIFIED — AAAI-27 Demonstrations Program is the explicit "system" venue** (https://aaai.org/wp-json/wp/v2/conference/96216):
> "The AAAI-27 Demonstrations Program is intended to foster discussion and exchange of ideas among researchers and practitioners from academia and industry by **presenting software and hardware systems and research prototypes of such systems**, including their capabilities and workings."

Its topics-of-interest list — **VERIFIED** (same URL) — includes verbatim: **"Systems that integrate different AI technologies"**, **"Robots"**, **"Virtual and augmented reality systems"**, **"Human-AI interaction (including Human-robot interaction)"**, **"Simulation environments for AI agents and multi-agent systems"**, **"Cognitive systems"**. Its novelty bar is lower than the main track — **VERIFIED** (same URL): *"Proposed demonstrations must present new ideas, which are not already established in mainstream products or services. However, the demonstration need not run solely at the conference; it is acceptable to present systems that have been showcased elsewhere in the recent past."*

**VERIFIED — AAAI-26 main-track submissions can opt in to a demo session** (https://aaai.org/wp-json/wp/v2/conference/92486):
> "During paper submission, authors will be able to express interest in giving a public demo of systems described in the paper. Based on these expressions of interest and on reviews, papers will be selected to participate in a demonstration session, which will happen in addition to technical talks and posters."

**VERIFIED — the AI for Social Impact special track explicitly de-emphasises technical novelty** (https://aaai.org/wp-json/wp/v2/conference/92480):
> "Submissions to this track will be reviewed according to a rubric that **emphasizes the fit between the techniques used and a problem of social importance, rather than simply rewarding technical novelty**. In particular, reviewers will assess the significance of the addressed problem; the paper's engagement with previous literature on the application problem...; both novelty of and justification for the proposed AI-based approach; quality of evaluation; facilitation of follow-up work; and overall scope and promise for social impact."

AAAI-26 special tracks: AI for Social Impact and AI Alignment (https://aaai.org/wp-json/wp/v2/conference/92480). AAAI-27 special tracks: AI for Social Impact and AI Alignment (https://aaai.org/wp-json/wp/v2/conference/95937). AAAI-26 also ran an "Emerging Trends in AI (ETA) Track" (https://aaai.org/conference/aaai/aaai-26/eta-call/) — I did not fetch its CFP, so its content is **待核实**.

### (h) Author response / rebuttal policy and supplementary material policy

Author response (AAAI-27) — **VERIFIED** (https://aaai.org/wp-json/wp/v2/conference/95937): window **October 19–25, 2026**, open **only for papers not rejected in Phase 1**; *"Author response after Phase 2, only for papers not rejected in Phase 1."*

Author response (AAAI-26) — **VERIFIED** (https://aaai.org/wp-json/wp/v2/conference/92480): window **October 7–13, 2025**.

AAAI-26 rebuttal mechanics — **VERIFIED** (https://aaai.org/wp-json/wp/v2/conference/93550, page: https://aaai.org/conference/aaai/aaai-26/aaai-26-faq-for-authors-rebuttal/):
- *"Yes, you must respond to all reviews with a single rebuttal of **2500 characters maximum**."*
- *"Authors can provide any text (markdown formatting is okay) that fits within the length limit. The response must **not include any URLs** — responses detected to contain URLs may be withheld."*
- *"Authors should **not provide new results** during the rebuttal, even if the reviewers asked for them... New, rushed experiments cannot be properly validated by the reviewers."*
- *"According to AAAI policy, **supplementary materials cannot be updated** at this stage under any circumstances."*
- Authors may also address the AI-generated review: *"Authors can choose to address points raised in any and all of the reviews, including all human-written reviews and the AI-generated reviews."*
- *"We endeavor to provide 5 human reviews and 1 AI review to all papers... The vast majority of papers (98.98%) had 3 human reviews or more when author responses opened."*

Supplementary material (AAAI-27) — **VERIFIED** (https://aaai.org/wp-json/wp/v2/conference/95937):
> "Authors may submit supplementary material, but please note that **reviewers are not required to review this material**. Any material critical to the evaluation of the paper should be included in the main body of the paper. Three kinds of supplementary material may be submitted alongside all papers: (1) technical supplement; (2) multimedia archive; and (3) code and data archive. **The supplementary material deadline is 3 days after the paper submission deadline.**"

AAAI-26 equivalent wording, same three kinds and same 3-day offset — **VERIFIED** (https://aaai.org/wp-json/wp/v2/conference/92480).

Reproducibility checklist — **VERIFIED**: AAAI-26 (https://aaai.org/wp-json/wp/v2/conference/92486): *"Authors must complete a reproducibility checklist at the time of paper submission... Please include your answers in your PDF after the references... This will not count toward the page limit."* AAAI-27 (https://aaai.org/wp-json/wp/v2/conference/96119): the checklist *"must be uploaded separately from the main paper in the designated field of the submission form"*, and *"Claims that such materials 'will be released after acceptance or publication' must not be treated as evidence of reproducibility."*

---

## IJCAI

### (a) Deadline month(s) / deadline pattern

**Pattern: one annual submission window in mid-January; a mandatory abstract deadline ~1 week before the full-paper deadline; the full-paper deadline also carries author information, appendix and resubmission information.**

IJCAI-ECAI 2026 Main Track — **VERIFIED** (https://2026.ijcai.org/wp-json/wp/v2/pages/2117), all times 23:59 AoE (UTC-12):
- **January 12, 2026 — Abstract submission deadline**
- **January 19, 2026 — Author information and Full paper submission deadline (including Appendix and resubmission information)**
- January 26, 2026 — Non-primary paper payment deadline (Primary Paper Initiative, USD 100 submission fee)

IJCAI 2025 Main Track — **VERIFIED** (https://2025.ijcai.org/wp-json/wp/v2/pages?slug=call-for-papers-main-track):
- **January 16, 2025 — Abstract submission deadline**
- **January 23, 2025 — Author information and Full paper submission deadline (including Appendix and resubmission information)**

Mandatory-abstract rule — **VERIFIED** (https://2026.ijcai.org/wp-json/wp/v2/pages/2117): *"The paper title, author names, contact details, and a brief abstract must be submitted electronically through the IJCAI-ECAI 2026 paper submission site... by the abstract submission deadline... submissions with 'placeholder' abstracts will be removed without consideration."*

Other IJCAI-ECAI 2026 tracks run on their own windows — **VERIFIED**: Demonstrations Track submission **February 16, 2026** (https://2026.ijcai.org/wp-json/wp/v2/pages/2228); Survey Track submission **January 19, 2026** (https://2026.ijcai.org/wp-json/wp/v2/pages/2234). The main CFP lists five special tracks (Human-centered AI, AI for Social Good, AI4Tech, AI and Health, AI and Robotics) plus the Survey Track, *"their deadlines will be the same as those listed above"* — **VERIFIED** (https://2026.ijcai.org/wp-json/wp/v2/pages/2117).

**Two-phase (summer/winter) submission model: NO.** As with AAAI, IJCAI's "two-phase" is **two-phase reviewing** of one submission — **VERIFIED** (https://2026.ijcai.org/wp-json/wp/v2/pages/2117):
> "Two-phase reviewing: Every paper will receive two reviews in Phase 1. Papers that advance beyond Phase 1 will receive additional reviews in Phase 2, during which authors will have the opportunity to submit a response."

and (same URL): *"In Phase 1, every paper will be reviewed by two PC members. If a paper receives two good-quality reviews that are not sufficiently positive in Phase 1, it will be rejected without any opportunity to submit an author response... authors acknowledge that they are aware of the possibility of receiving a **summary rejection** notification."*

### (b) Review cycle length

IJCAI-ECAI 2026 Main Track (**VERIFIED**, https://2026.ijcai.org/wp-json/wp/v2/pages/2117):
- Full paper January 19, 2026 → **Summary reject notification March 4, 2026** ≈ **1.5 months**
- **Author response period April 7–10, 2026**
- **Paper notification April 29, 2026** ≈ **3.3 months** after submission
- Main-track camera-ready deadline: **待核实** — not published on the main-track CFP.
- Conference August 15–21, 2026 (Bremen)

IJCAI 2025 Main Track (**VERIFIED**, https://2025.ijcai.org/wp-json/wp/v2/pages?slug=call-for-papers-main-track):
- Full paper January 23, 2025 → **Summary reject notification March 7, 2025** ≈ **1.4 months**
- **Author response period April 6–9, 2025**
- **Paper notification April 28, 2025** ≈ **3.2 months** after submission
- Main-track camera-ready deadline: **待核实** — not published.
- Conference August 16–22, 2025 (Montréal); satellite event Guangzhou August 28–31, 2025

Camera-ready dates that *are* published, for other IJCAI-ECAI 2026 tracks (**VERIFIED**): Demonstrations **May 15, 2026** (https://2026.ijcai.org/wp-json/wp/v2/pages/2228); Survey Track **June 10, 2026** (https://2026.ijcai.org/wp-json/wp/v2/pages/2234).

### (c) Acceptance rate

**待核实** — no IJCAI acceptance rate is published on any official IJCAI page I could fetch. I checked the IJCAI 2025 main-track CFP, the IJCAI-ECAI 2026 main-track CFP, the IJCAI-ECAI 2026 Press/Daily-Recap page, the IJCAI WordPress search endpoints for "statistics"/"accept", and `ijcai.org` (home, past conferences, all-proceedings nav). The IJCAI-ECAI 2026 closing-session recap confirms statistics *were* presented but does not publish the numbers — **VERIFIED** (https://2026.ijcai.org/wp-json/wp/v2/pages/2906): *"Programme Chair Diego Calvanese shared final conference statistics, presenting detailed demographic breakdowns of total attendance, topic distributions, and international paper acceptances."* The linked Media Brief is a PDF (https://2026.ijcai.org/wp-content/uploads/2026/06/IJCAI2026_Media_Brief_Updated.pdf) and the fetch tool cannot read `application/pdf`, so it could not be checked.

### (d) Page limit

IJCAI-ECAI 2026 Main Track — **VERIFIED** (https://2026.ijcai.org/wp-json/wp/v2/pages/2117):
> "**Page limit**: Papers must be no longer than **9 pages in total: 7 pages for the body of the paper and 2 pages for references**; an optional ethics statement can be placed either in the body of the paper or in the reference pages."

and: *"Overlength papers will be rejected without review."*

IJCAI 2025 Main Track — **VERIFIED** (https://2025.ijcai.org/wp-json/wp/v2/pages?slug=call-for-papers-main-track): identical 7 + 2 split, with one extra rule:
> "Acknowledgments and the contribution statement can be included in the two reference pages for accepted papers. Moreover, **for accepted papers, up to two additional pages may be purchased at an additional cost per page.**"

⚠️ That purchasable-extra-pages sentence appears in the **IJCAI 2025** CFP but **not** in the IJCAI-ECAI 2026 main-track CFP text I fetched. For IJCAI-ECAI 2026 main track the purchasable-page rule is **待核实**. It *is* explicitly present for the IJCAI-ECAI 2026 **Survey Track** — **VERIFIED** (https://2026.ijcai.org/wp-json/wp/v2/pages/2234): *"For accepted papers, up to two additional pages may be purchased at an additional cost per page."*

Other IJCAI-ECAI 2026 page limits (**VERIFIED**):
- AI and Robotics Special Track: 9 pages total, 7 body + 2 references (https://2026.ijcai.org/wp-json/wp/v2/pages/2211)
- Demonstrations Track: **5 pages total — 3 pages main text (incl. figures/tables) + up to 2 pages references** (https://2026.ijcai.org/wp-json/wp/v2/pages/2228)
- Survey Track: 9 pages total, 7 main text + 2 references; *"No supplementary material is allowed."* (https://2026.ijcai.org/wp-json/wp/v2/pages/2234)

**IJCAI author kit:** the CFP points to **https://www.ijcai.org/authors_kit** as the location of the LaTeX styles / Word template — **VERIFIED as the officially designated URL** (https://2026.ijcai.org/wp-json/wp/v2/pages/2117). I did not successfully fetch that page, so its own contents are **待核实**; the page limits above come from the official CFPs.

### (e) Double-blind / single-blind / optional

IJCAI 2025 and IJCAI-ECAI 2026 **Main Track: double-blind (mandatory anonymisation)** — **VERIFIED**:
- IJCAI-ECAI 2026 (https://2026.ijcai.org/wp-json/wp/v2/pages/2117): *"From the authors' perspective, reviewing for IJCAI-ECAI 2026 is **double-blind**. As an author, you are responsible for anonymizing your submission... **Submissions that violate anonymity will be rejected without further review.**"*
- IJCAI 2025 (https://2025.ijcai.org/wp-json/wp/v2/pages?slug=call-for-papers-main-track): same policy, same wording.

Preprints are tolerated — **VERIFIED** (https://2026.ijcai.org/wp-json/wp/v2/pages/2117): *"The existence of non-anonymous preprints (on arXiv, social media, websites, etc.) and prior publication in non-archival venues will not result in rejection."*

**Other IJCAI-ECAI 2026 tracks are NOT double-blind** — this is the clearest official evidence that IJCAI uses both policies across its tracks:
- Demonstrations Track — **VERIFIED** (https://2026.ijcai.org/wp-json/wp/v2/pages/2228): *"The reviewing is **single-blind**, submissions do not have to be anonymised."*
- Survey Track — **VERIFIED** (https://2026.ijcai.org/wp-json/wp/v2/pages/2234): *"Because at least one of the authors must be a recognized expert on the topic, submissions to the survey track are **not anonymous** and should include the names and affiliations of all authors."*
- AI and Robotics Special Track — **VERIFIED** (https://2026.ijcai.org/wp-json/wp/v2/pages/2211): *"Papers are expected to satisfy the highest scientific standards as submissions to the main track of IJCAI-ECAI 2026 and **satisfy double-blind review conditions**."*

**IJCAI's *historical* blind policy across past editions: 待核实.** I could not verify from an official source whether IJCAI's main track previously used single-blind; the archived CFP pages for older editions were not reachable through the routes I tried, and I did not want to cite a non-official summary.

### (f) Real-robot / physical hardware required or expected?

**Main Track: 待核实** — the IJCAI-ECAI 2026 and IJCAI 2025 main-track CFPs make no statement about physical hardware or real-robot experiments.

**AI and Robotics Special Track: VERIFIED, and this is the most directly relevant official statement.** https://2026.ijcai.org/wp-json/wp/v2/pages/2211:
> "The Special Track on AI and Robotics invites original research papers that make a clear AI contribution and demonstrate a substantive connection to embodied robotic systems, across perception, reasoning, learning, and control. **Submissions should go beyond pure simulation and show relevance to real or realistically embodied robots, including physical platforms and high-fidelity simulators.**"

and: *"We particularly value strong evidence of **generalizability, transferability, and robust operation in realistic, open-world settings**."* The track also names *"Safety, trustworthiness, generalizability, and evaluation: Verification and validation methods for AI-powered robotic systems; theoretical and algorithmic guarantees on safety, robustness, and out-of-distribution generalization."*

**Interpretation (not a quote):** real hardware is **not strictly mandatory** for that track — high-fidelity simulation is explicitly allowed — but a pure-simulation paper is explicitly excluded by the "go beyond pure simulation" clause.

### (g) Tolerance for "system integration" style contributions

**Main Track — VERIFIED** (https://2026.ijcai.org/wp-json/wp/v2/pages/2117):
> "Submissions to the **main track** of IJCAI-ECAI 2026 should report **significant, original, and previously unpublished results** on any aspect of AI. Papers on **novel AI research problems, AI techniques for novel application domains, and papers that cross discipline boundaries within AI are especially encouraged.**"

Selection criteria — **VERIFIED** (same URL): *"Selection criteria include the **novelty and originality of ideas**, correctness, clarity, the significance of results, potential impact and quality of the presentation."* The main-track CFP contains **no** "systems paper" category or explicit systems-track language.

**Demonstrations Track — VERIFIED, the explicit systems venue** (https://2026.ijcai.org/wp-json/wp/v2/pages/2228):
> "The track seeks to **bridge theoretical advancements and practical applications in AI** by facilitating the exchange of knowledge and ideas... **Demos should clearly demonstrate the applicability of the chosen AI techniques and highlight the practical considerations for their deployment.**"

Submission must contain — **VERIFIED** (same URL): *"A paper (PDF) describing the demonstrated system, its application domain, problem scenario, technology, AI techniques, innovations, and interactive elements."* Its topic list includes verbatim **"AI Systems & Interaction: Human-AI Interaction, Agentic and Multi-Agent Systems, **Robotics and Embodied AI**, Recommender and Decision-Support Systems, Speech and Conversational Interfaces."** The review rubric — **VERIFIED** (same URL): *"The program committee will review submissions based on **originality, innovation, applicability, and potential impact**. There is no rebuttal phase."* Accepted demos are *"published in the conference proceedings and considered for the prestigious Innovation Award"*.

**Survey Track — VERIFIED** (https://2026.ijcai.org/wp-json/wp/v2/pages/2234): *"The IJCAI-ECAI 2026 Survey Track enables the AI community to learn from AI researchers who are experts on a topic by allowing them to give a survey talk that presents a structured synthesis of that topic."* Requirements: *"The survey should have a well-defined scope and be understandable for most of the AI community... It should **not be a simple juxtaposition of summaries of papers or groups of papers**."* Author eligibility: *"The list of authors should contain at least one author who has contributed significantly to the AI literature and one author (potentially the same person) who is an expert on the topic of the submission."* Submission limit: *"Each author is limited to no more than **one** submission to the IJCAI-ECAI 2026 Survey Track."*

### (h) Author response / rebuttal policy and supplementary material policy

Author response — **VERIFIED**:
- IJCAI-ECAI 2026 (https://2026.ijcai.org/wp-json/wp/v2/pages/2117): *"**Author response period: April 7-10, 2026**"*; *"The full paper review process will allow authors to respond to the reviews. Author responses should be **concise and are not intended to create a dialogue** between reviewers and authors. Author responses will be visible to SAC, AC, SPC, and PCs."* No response is possible for Phase-1 summary rejects.
- IJCAI 2025 (https://2025.ijcai.org/wp-json/wp/v2/pages?slug=call-for-papers-main-track): author response **April 6–9, 2025**, same wording.
- **AI and Robotics Special Track: no rebuttal at all** — **VERIFIED** (https://2026.ijcai.org/wp-json/wp/v2/pages/2211): *"Differing from the main track, **there will be no phase 1 notification and author rebuttal**."*
- **Demonstrations Track: no rebuttal** — **VERIFIED** (https://2026.ijcai.org/wp-json/wp/v2/pages/2228): *"There is no rebuttal phase."*

Supplementary material, IJCAI-ECAI 2026 Main Track — **VERIFIED** (https://2026.ijcai.org/wp-json/wp/v2/pages/2117):
> "Authors may submit **up to 50MB of supplementary material**, such as appendices, proofs, derivations, data, or source code; all supplementary material must be in **PDF or ZIP format**... Like submissions, supplementary material must be anonymized. There are two entries for supplementary files in the paper management system: one is 'Technical Appendix', and the other one is 'Resubmission File'... **Reviewers may choose to consult the technical appendix at their discretion.**"

and — **VERIFIED** (same URL): *"Supplementary material (e.g., appendices, data, source code, resubmission information) must be submitted by the **full paper submission deadline**."*

Survey Track rejects supplementary material entirely — **VERIFIED** (https://2026.ijcai.org/wp-json/wp/v2/pages/2234): *"the IJCAI-ECAI 2026 survey track will not accept or review any supplementary material."*

Resubmission information is mandatory if the paper was rejected elsewhere in the prior 12 months — **VERIFIED** (https://2026.ijcai.org/wp-json/wp/v2/pages/2117): *"Authors must declare whether their paper has been rejected from another peer-reviewed conference in the last 12 months before the submission. They will be requested to upload the latest rejected version (anonymized if necessary) with the reviewer's comments... To avoid bias, the resubmission information will only be made available to reviewers after they submit their reviews. The program committee reserves the right to reject papers that fail to report resubmission information."*

Reproducibility guidelines and checklist are mandatory at submission time — **VERIFIED** (https://2026.ijcai.org/wp-json/wp/v2/pages/2117): *"Authors must follow the **reproducibility** guidelines (available here) and checklist at the time of paper submission."* (guidelines link: https://2026.ijcai.org/reproducibility/ — I did not fetch that page, so its contents are **待核实**).

---

## Summary table

| Field | AAAI-27 (main track) | AAAI-26 (main track) | IJCAI-ECAI 2026 (main track) | IJCAI 2025 (main track) |
|---|---|---|---|---|
| Abstract deadline | Jul 21, 2026 | Jul 25, 2025 | Jan 12, 2026 | Jan 16, 2025 |
| Full paper deadline | Jul 28, 2026 | Aug 1, 2025 | Jan 19, 2026 | Jan 23, 2025 |
| Supplementary deadline | Jul 31, 2026 | Aug 4, 2025 | same as full paper | same as full paper |
| Phase-1 / summary reject | Sep 24, 2026 | Sep 15, 2025 | Mar 4, 2026 | Mar 7, 2025 |
| Author response | Oct 19–25, 2026 | Oct 7–13, 2025 | Apr 7–10, 2026 | Apr 6–9, 2025 |
| Final notification | Nov 30, 2026 | Nov 8, 2025 | Apr 29, 2026 | Apr 28, 2025 |
| Camera-ready | Dec 14, 2026 | Nov 16, 2025 | 待核实 | 待核实 |
| Submission → final decision | ≈4.1 mo | ≈3.3 mo | ≈3.3 mo | ≈3.2 mo |
| Page limit | 7 content + 2 refs (9 max) | 7 content + refs + checklist | 7 body + 2 refs | 7 body + 2 refs |
| Blind | Double-blind | Double-blind | Double-blind | Double-blind |
| Acceptance rate | 待核实 | 待核实 | 待核实 | 待核实 |

---

## Exact URLs successfully fetched

**aaai.org**
1. https://aaai.org/wp-json/wp/v2/pages?search=AAAI-26%20Main%20Technical%20Track&per_page=5 (AAAI awards page — used only to discover REST routes)
2. https://aaai.org/wp-json/wp/v2/search?search=AAAI-26%20Submission%20Instructions&per_page=20&_fields=id,title,url,subtype
3. https://aaai.org/wp-json/wp/v2/pages?search=technical%20track%20call&_fields=id,link,title,slug&per_page=30
5. https://aaai.org/wp-json/wp/v2/pages?search=submission%20instructions&_fields=id,link,title,slug&per_page=30
6. https://aaai.org/wp-json/wp/v2/types
7. https://aaai.org/wp-json/wp/v2/conference/92480?_fields=link,content — AAAI-26 Main Technical Track: Call for Papers
8. https://aaai.org/wp-json/wp/v2/conference/92486?_fields=link,content — AAAI-26 Submission Instructions
9. https://aaai.org/wp-json/wp/v2/conference/95937?_fields=link,content — AAAI-27 Main Technical Track Call
10. https://aaai.org/wp-json/wp/v2/conference/96119?_fields=content — AAAI-27 Submission Instructions
11. https://aaai.org/wp-json/wp/v2/conference/93550?_fields=content — AAAI-26 FAQ for Authors (Rebuttal)
12. https://aaai.org/wp-json/wp/v2/conference/93026 — AAAI-26 Review Process Update (scale statistics)
13. https://aaai.org/wp-json/wp/v2/conference/96216?_fields=content — AAAI-27 Demonstrations Program CFP
14. https://aaai.org/wp-json/wp/v2/conference/96332?_fields=content — AAAI-27 AI Alignment Special Track CFP
15. https://aaai.org/wp-json/wp/v2/search?search=acceptance+rate&per_page=20&_fields=id,title,url,subtype
16. https://aaai.org/wp-json/wp/v2/search?search=Author%20Kit&per_page=20&_fields=id,title,url,subtype
17. https://aaai.org/wp-json/wp/v2/posts?search=AAAI-26&_fields=link,title,date&per_page=40
18. https://aaai.org/wp-json/wp/v2/posts?search=acceptance&_fields=link,title,date&per_page=30
19. https://aaai.org/conference/aaai/aaai-26/review-process-update/ (rendered — truncated to title, superseded by #12)

**ijcai.org / conference sites**
20. https://www.ijcai.org/ (home)
21. https://www.ijcai.org/past_conferences
22. https://2025.ijcai.org/wp-json/wp/v2/pages?slug=call-for-papers-main-track&_fields=link,content — IJCAI 2025 Main Track CFP
23. https://2025.ijcai.org/wp-json/wp/v2/pages?search=statistics&_fields=id,link,title&per_page=20
24. https://2025.ijcai.org/wp-json/wp/v2/pages?search=glance&_fields=id,link,title&per_page=10
25. https://2025.ijcai.org/wp-json/wp/v2/pages/1563?_fields=link,content (Montréal at-a-glance — iframe only)
26. https://2025.ijcai.org/wp-json/wp/v2/posts?search=accept&_fields=link,title,date&per_page=40
27. https://2025.ijcai.org/wp-content/ataglance25montreal/index.html (JS app shell — no usable text)
28. https://2026.ijcai.org/wp-json/wp/v2/pages?search=call%20for%20papers&_fields=id,link,title,slug&per_page=30
29. https://2026.ijcai.org/wp-json/wp/v2/pages/2117?_fields=link,content — IJCAI-ECAI 2026 Main Track CFP
30. https://2026.ijcai.org/wp-json/wp/v2/pages/2211?_fields=link,content — AI and Robotics Special Track CFP
31. https://2026.ijcai.org/wp-json/wp/v2/pages/2228?_fields=link,content — Demonstrations Track CFP
32. https://2026.ijcai.org/wp-json/wp/v2/pages/2234?_fields=link,content — Survey Track CFP
33. https://2026.ijcai.org/wp-json/wp/v2/pages/2906?_fields=link,content — IJCAI-ECAI 2026 Press / Daily Recap
34. https://2026.ijcai.org/wp-json/wp/v2/posts?search=accept&_fields=link,title,date&per_page=40

**Fetched but unusable (no content retrieved)**
- https://aaai.org/wp-content/uploads/2026/01/AAAI-26-Opening-Ceremony.pdf — rejected: unsupported content type `application/pdf`
- https://aaai.org/authorkit27/ , https://aaai.org/authorkit26-1/ — fetch failures
- https://aaai.org/conference/aaai/aaai-27/main-technical-track-call/ , https://aaai.org/conference/aaai/aaai-26/main-technical-track-call/ , https://aaai.org/conference/aaai/aaai-26/submission-instructions/ — rendered HTML truncated to page title
- https://2026.ijcai.org/ , https://2025.ijcai.org/call-for-papers-main-track/ (rendered) — truncated
- https://2026.ijcai.org/wp-content/uploads/2026/06/IJCAI2026_Media_Brief_Updated.pdf — not attempted (PDF unsupported)
