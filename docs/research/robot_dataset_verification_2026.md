# Robot Demonstration Datasets & Benchmarks — Verification Report

Verification pass for 15 items + the "de-facto standard VLA benchmark" question.
Method: `web_search` + `web_fetch` against arXiv abstract pages, official proceedings (RSS / CVF / PMLR / ICML / NeurIPS), official project homepages, and official conference award pages.

Legend for evidence strength:
- **[official]** — verified from a publisher / proceedings / official project page
- **[arXiv]** — verified from arXiv metadata only (abstract page `Comments:` / `Journal reference:` fields)
- **待核实** — could not verify; not guessed

---

## 1. Open X-Embodiment / RT-X

`Open X-Embodiment: Robotic Learning Datasets and RT-X Models | ICRA 2024 [official] | 22 robots, 21 institutions, 527 skills, 160,266 tasks | aggregated from 60 existing datasets contributed by the collaborating labs (no single teleoperation rig) | open-source: yes + https://robotics-transformer-x.github.io`

- **ICRA 2024 confirmed [official].** Source: official ICRA 2024 "Awards and Finalists" page.
- **Best paper award: YES — it WON the IEEE ICRA Best Conference Paper Award.** It is listed under **"Winners"** of *IEEE ICRA Best Conference Paper Award* (a two-winner tie with *Goal Masked Diffusion Policies for Unified Navigation and Exploration*, i.e. NoMaD). Title as listed at ICRA is *"Open X-Embodiment: Robotic Learning Datasets and RT-X"* (no "Models").
- Nuance: it was a **finalist** (not winner) for *Best Student Paper Award* and for *Best Paper Award in Robot Manipulation* — the Robot Manipulation award went to *SARA-RT*. Do not conflate "finalist in three categories" with "winner of one".
- Scale **[arXiv]**: 22 robots, 21 institutions, 527 skills, 160,266 tasks.
- arXiv license: CC BY 4.0.

## 2. DROID

`DROID: A Large-Scale In-The-Wild Robot Manipulation Dataset | RSS 2024 (Robotics: Science and Systems XX, Delft, July 2024), DOI 10.15607/RSS.2024.XX.120 [official] | see scale conflict below | Franka Panda 7-DoF + 2x ZED 2 stereo + wrist ZED Mini + Oculus Quest 2 headset with controllers, on a portable height-adjustable desk; 13 institutions | open-source: yes + https://droid-dataset.github.io/`

- **RSS 2024 confirmed [official]** — official RSS XX proceedings, paper p120.
- **Scale conflict — this is real, not an error in your brief.** Three sources disagree:
  | Source | Trajectories | Scenes | Tasks |
  |---|---|---|---|
  | RSS 2024 official abstract **[official]** | 65k | 564 | **86** |
  | arXiv v2 (Apr 2025) abstract **[arXiv]** | 76k | 564 | **84** |
  | Project website (current) **[official]** | 76k | 564 | **86** |
  All three agree on **350 hours**, **50 data collectors**, **12 months**, North America/Asia/Europe.
  So "76k trajectories / 564 scenes / 86 tasks" matches the **project website**, not the revised arXiv abstract. Cite the version you actually used.
- Open-source: dataset via TFDS (`gs://gresearch/robotics`, `tfds.load("droid")`) + HuggingFace (`KarlP/droid`, incl. 36k improved camera calibrations and 3 language annotations for 95% of successful episodes); hardware code `github.com/droid-dataset/droid`; policy code `github.com/droid-dataset/droid_policy_learning`.

## 3. RoboMIND

`RoboMIND: Benchmark on Multi-embodiment Intelligence Normative Data for Robot Manipulation | RSS 2025 (RSS XXI, Los Angeles, June 2025), DOI 10.15607/RSS.2025.XXI.152 — paper p152 [OFFICIAL — upgraded] | 107k demonstration trajectories, 479 tasks, 96 object classes, 4 embodiments (+5k real-world failure demos) | human teleoperation on a unified data collection platform with a standardized protocol | open-source: yes + https://x-humanoid-robomind.github.io/`

- **Venue upgraded from [arXiv] to [official]**: I fetched the official RSS XXI proceedings page `https://www.roboticsproceedings.org/rss21/p152.html`. BibTeX reads `BOOKTITLE = {Proceedings of Robotics: Science and Systems}, YEAR = {2025}, ADDRESS = {LosAngeles, CA, USA}, MONTH = {June}, DOI = {10.15607/RSS.2025.XXI.152}`.
- 4 embodiments: Franka Emika Panda, UR5e, AgileX dual-arm robot, and — per the **official RSS abstract** — the "**X-Humannoid Tien Kung** humanoid robot with dual dexterous hands" (the arXiv v3 abstract says only "a humanoid robot with dual dexterous hands"). Also provides an Isaac Sim digital twin.
- Authorship note: this is the **Beijing Humanoid Robot Innovation Center + Peking University + BAAI (智源)** — **not** ByteDance. Worth stating explicitly because "X-Humanoid" is easily mistaken for a ByteDance project.
- arXiv license is arXiv's non-exclusive default (not CC).

## 4. AgiBot World Colosseo

`AgiBot World Colosseo: A Large-scale Manipulation Platform for Scalable and Intelligent Embodied Systems | 待核实 (arXiv:2503.06669 preprint only, 2025, v4 Aug 2025 — no venue in metadata) | >1M trajectories, 217 tasks, 5 deployment scenarios | standardized collection pipeline with human-in-the-loop verification; extensible from grippers to dexterous hands and visuo-tactile sensors (specific teleop hardware 待核实) | open-source: yes + https://agibot-world.com/ , https://github.com/OpenDriveLab/AgiBot-World`

- **Venue: 待核实.** arXiv metadata lists no conference/journal; I found no official proceedings record. Do not write "RSS 2025" or similar.
- arXiv license: CC BY-NC-SA 4.0 (non-commercial).
- The paper introduces **GO-1 (Genie Operator-1)** as the policy, and reports that pretraining on AgiBot World beats pretraining on **Open X-Embodiment** by ~30% average — useful evidence that OXE is used as a *training* baseline.

## 5. RoboCasa

`RoboCasa: Large-Scale Simulation of Household Tasks for Generalist Robots | RSS 2024 [official] | 100 tasks for systematic evaluation; thousands of 3D assets across 150+ object categories; kitchen-focused scenes | SIMULATION: high-quality human demonstrations in sim + automated trajectory generation | open-source: yes + https://robocasa.ai/`

- **RSS 2024 confirmed twice**: arXiv `Comments: RSS 2024`, and the official RSS XX proceedings ToC lists it as p050.
- Title nuance: at RSS it is *"RoboCasa: Large-Scale Simulation of **Household** Tasks for Generalist Robots"*; on arXiv it is *"...**Everyday** Tasks..."*.

## 6. RoboTwin (+ RoboTwin 2.0)

`RoboTwin: Dual-Arm Robot Benchmark with Generative Digital Twins (early version) | CVPR 2025 [official, CVF Open Access] | task count 待核实 (v1 abstract gives no task count; reports >70% single-arm / >40% dual-arm success lifts) | SIMULATION: 3D generative foundation models + LLMs synthesize expert datasets and code; validated on the open-source COBOT Magic Robot | open-source: yes + https://robotwin-benchmark.github.io/early-version/`

`RoboTwin 2.0: A Scalable Data Generator and Benchmark with Strong Domain Randomization for Robust Bimanual Robotic Manipulation | ICML 2026 [official] | RoboTwin-OD library: 731 instances / 147 categories; 50 dual-arm tasks; 5 robot embodiments | SIMULATION: MLLM-generated task code + simulation-in-the-loop verification; domain randomization over clutter, lighting, background, tabletop height, language | open-source: yes + https://robotwin-platform.github.io/ , https://github.com/robotwin-Platform/robotwin`

- RoboTwin v1 = `arXiv:2409.02920`; CVPR 2025 verified via the CVF Open Access entry (`openaccess.thecvf.com/content/CVPR2025/html/Mu_RoboTwin_..._CVPR_2025_paper.html`). Caveat: the CVF page body itself did not render for me (fetch failed); the CVF URL + title are [official].
- **RoboTwin 2.0 exists** — `arXiv:2506.18088`, verified as an **ICML 2026** poster on the official ICML 2026 virtual site (poster #62192, session Wed Jul 8 2026, OpenReview id `itonej9GIV`).

## 7. ARIO

`ARIO = All Robots in One: A New Standard and Unified Dataset for Versatile, General-Purpose Embodied Agents | 待核实 (arXiv:2408.10899 preprint only, Aug 2024, single v1 — no venue in metadata) | ~3 million episodes, 258 robot series, 321,064 tasks | unified data standard aggregating real-world + simulated data from existing sources (not a single teleop campaign) | open-source: yes + https://imaei.github.io/project_pages/ario/`

- **Correct full name and arXiv id found**: *"All Robots in One: A New Standard and Unified Dataset for Versatile, General-Purpose Embodied Agents"*, `arXiv:2408.10899`. Authors incl. Zhiqiang Wang, Hao Zheng, Feng Zheng, Xiaodan Liang (Sun Yat-sen University et al.).
- Mirrors: project page `ario-dataset.github.io`, OpenI `ARIO/ARIO`, CAICT AIHub.
- **Venue: 待核实** — no conference/journal record found on arXiv metadata.

## 8. BridgeData V2

`BridgeData V2: A Dataset for Robot Learning at Scale | CoRL 2023 (PMLR vol. 229, pp. 1723–1736) [official] | 60,096 trajectories across 24 environments | teleoperated on a publicly available low-cost robot (specific teleop device 待核实) | open-source: yes + https://rail-berkeley.github.io/bridgedata`

- Venue verified via the DBLP/researchr record resolving to `proceedings.mlr.press/v229/walke23a.html` (CoRL 2023, Atlanta, 6–9 Nov 2023). Note the arXiv page itself carries **no** venue field — the CoRL 2023 attribution comes from the PMLR/DBLP record, not arXiv.

## 9. RH20T — ⚠️ the arXiv ID in the brief is WRONG

`RH20T: A Comprehensive Robotic Dataset for Learning Diverse Skills in One-Shot | ICRA 2024, pp. 653–660 [official] | 110,000+ contact-rich manipulation sequences; 48 RLBench + 29 MetaWorld + 70 self-proposed = 147 tasks; 7 robot configurations | haptic device + pedal teleoperation (explicitly NOT VR/3D-mouse), plus 6-DoF F/T sensor, gripper, 1–2 in-hand cameras, 8–10 global RGBD cameras, 2 microphones | open-source: yes + https://rh20t.github.io/ and API https://github.com/rh20t/rh20t_api`

- **⚠️ `arXiv:2307.08898` is NOT RH20T.** That ID resolves to *"CaMP-INC: Components-aware Microservices Placement for In-Network Computing Cloud-Edge Continuum"* (cs.NI, GLOBECOM 2022). The correct ID is **`arXiv:2307.00595`**.
- **ICRA 2024 confirmed [official]** — official project-page BibTeX: `booktitle = {2024 IEEE International Conference on Robotics and Automation (ICRA)}, pages = {653--660}, year = {2024}`.
- Note: RH20T also appeared as a **poster in the NeurIPS 2023 6th Robot Learning Workshop** (verified on the official NeurIPS virtual site). That is a workshop, not a main-track publication.
- Open-source: yes, but **mixed license** — RH20T-C subset (scene_0001–0005) under CC BY-SA 4.0; RH20T-NC subset (scene_0006–0010) under CC BY-NC 4.0 (no commercial use, incl. models trained on it). Original data ~40 TB; public release is downscaled.

## 10. EgoMimic — ⚠️ the arXiv ID in the brief is WRONG

`EgoMimic: Scaling Imitation Learning via Egocentric Video | 待核实 (arXiv:2410.24221 preprint, Oct 2024 — project BibTeX is @misc, no venue; an IEEE Xplore record exists but venue name unverified) | 3 real-world tasks (single-arm + bimanual); scaling study uses 2 h robot + 1 h hand vs 3 h robot (raw trajectory counts 待核实) | Project Aria glasses for egocentric human hand data + a low-cost teleoperated bimanual robot | open-source: yes + https://egomimic.github.io/`

- **⚠️ `arXiv:2406.04093` is NOT EgoMimic.** That ID resolves to *"Scaling and evaluating sparse autoencoders"* (OpenAI, cs.LG). The correct ID is **`arXiv:2410.24221`**.
- **Aria glasses: CONFIRMED [official]** — project page states "an ergonomic human data collection system using the **Project Aria glasses**". There is also an official Meta AI blog post about the collaboration.
- Venue: the project page's own BibTeX is `@misc{... year={2024}}` = arXiv only. An IEEE Xplore document (id 11127989) exists, implying a 2025 conference version, but I could **not** confirm which venue → **待核实**. Do not write "ICRA 2025".
- Open-source: yes — code `github.com/SimarKareer/EgoMimic`, hardware code `EgoMimic-Eve`, datasets `huggingface.co/datasets/gatech/EgoMimic`.

## 11. Fourier ActionNet

`ActionNet: A dataset for dexterous bimanual manipulation (site title: "Fourier ActionNet Dataset") | 待核实 — dataset release, no arXiv id and no publication venue found; 2025 | 30,000+ teleoperated trajectories (~140 hours) | VR teleoperation with **Apple Vision Pro** as the primary control device; OAK-D W 97 camera; Fourier GR1-T1 / GR1-T2 / GR2 humanoids with 6-DoF and 12-DoF Fourier dexterous hands | open-source: yes (but restrictive) + https://action-net.org/`

- **Note for your VR/XR item:** the headset is **Apple Vision Pro**, *not* PICO or Quest.
- License: data CC BY-NC-SA 4.0 (non-commercial), code Apache 2.0. HuggingFace dataset `FourierIntelligence/ActionNet` is **gated** (contact info must be shared); a Baidu Cloud mirror is provided.
- Evidence grade: official project site `action-net.org` (verified by a delegated verifier).

## 12. BEHAVIOR Robot Suite (BRS)

`BEHAVIOR Robot Suite: Streamlining Real-World Whole-Body Manipulation for Everyday Household Activities | CoRL 2025 (9th Annual Conference on Robot Learning; OpenReview v2KevjWScT) [official docs + OpenReview] | 5 real-world household tasks, ~561 trajectories (100 / 103 / 98 / 138 / 122) | **JoyLo** interface: kinematic-twin leader arms + Nintendo Joy-Con controllers, haptic feedback, <$500 — NOT a VR system | open-source: yes + https://behavior-robot-suite.github.io/ , https://huggingface.co/datasets/behavior-robot-suite/data`

- **Correction to the brief:** the robot is the **Galaxea R1** (21-DoF bimanual wheeled, 4-DoF torso), *not* the Rainbow Robotics RBY1. Official BRS docs state JoyLo is "paired with the Galaxea R1 robot"; acknowledgements thank Galaxea.ai.
- **Notable for a VR-teleoperation project:** the BRS paper reports a 10-participant user study in which **JoyLo outperformed VR controllers and Apple Vision Pro** — i.e. a directly relevant counter-datapoint to "VR teleoperation is the best interface" for whole-body household tasks.
- Also powers the first **BEHAVIOR Challenge at NeurIPS 2025**, with 1,200+ hours of JoyLo-collected human data (separate, larger collection than the 5-task set above).

## 13. GR-1 / GR-2 / any 2025 large VLA training-dataset paper

**待核实.** Verification was still in flight when this report was cut off.

- ⚠️ Name-collision warning to carry forward: **"GR-1" is ambiguous.** (a) ByteDance's *GR-1* (`arXiv:2312.13139`, "Unleashing Large-Scale Video Generative Pre-training for Visual Robot Manipulation") is a **model**, not a released dataset; (b) **Fourier** also brands humanoids GR-1 / GR-2 (those are the platforms used by Fourier ActionNet, item 11). Do not cite "the GR-1 dataset" without nailing down which one is meant.

## 14. 2025 dataset built from VR/XR teleoperation at scale (PICO/Quest)

**待核实** for a *dedicated 2025 paper explicitly framed as a VR/XR-collected dataset*. Verified adjacent facts:

- **DROID** is the largest verified headset-teleoperated dataset here: 76k trajectories collected with an **Oculus (Meta) Quest 2 headset with controllers** across 13 institutions [official] — but it is a **2023–2024** collection published at RSS 2024, not a 2025 dataset paper.
- **Fourier ActionNet** (2025, 30k+ trajectories) uses **Apple Vision Pro** teleoperation [official] — XR, but neither PICO nor Quest.
- **BRS** (2025) explicitly uses a *non-VR* interface (JoyLo) and reports it beating Vision Pro / VR controllers [official].
- No PICO-based large-scale dataset was verified in this pass → **待核实**.

## 15. Genie-1 / Genie Envisioner

**待核实.** Verification was still in flight when this report was cut off.

- ⚠️ Name-collision warning: Google DeepMind's **Genie / Genie 2 / Genie 3** are **world models**, not robot manipulation datasets. Any "Genie" dataset claim must be checked against that.
- Unverified lead only (do **not** cite): a "Genie Envisioner" manipulation item was reportedly associated with the AgiBot World dataset. Not confirmed here.

---

## De-facto standard benchmarks for evaluating VLA policies (2024–2025) — RESOLVED

**Key evidence:** a dedicated third-party audit, *"What Are We Actually Benchmarking in Robot Manipulation?"* (`arXiv:2606.04233`, Jun 2026, RIPL lab / Matthew Walter's group, TTIC) — **I verified this abstract page myself.** Its choice of *what to audit* is itself the strongest available enumeration of the de-facto standard suite, because it audits exactly the five benchmarks the community treats as canonical:

> **LIBERO, CALVIN, SimplerEnv, RoboCasa, and RoboTwin 2.0**

Its findings (quoted from the abstract):
- Four failure modes identified: **shortcut solvability, lack of statistical significance, creeping overfitting, data-source dependence**.
- **LIBERO and CALVIN fail multiple diagnostics.**
- **RoboCasa and RoboTwin 2.0 fail fewer — "despite appearing far less often in recent progress claims."** ← this is the load-bearing sentence for question (1) below.
- On LIBERO: "a **0.09B probe with no language encoder** scores at or near reported SOTA, and **most reported gains are not provably statistically significant**."
- On CALVIN: randomizing block poses *within the training range* drops performance for every tested policy.
- Diagnostics + reference implementations released: https://ripl.github.io/manipulation_benchmark_audit/

### Adjudication of the seven claims

| # | Claim | Verdict | Basis |
|---|---|---|---|
| 1 | LIBERO is the most common simulation benchmark for VLA policies | **TRUE (with a sting)** | The audit implies LIBERO/CALVIN dominate "recent progress claims" while RoboCasa/RoboTwin 2.0 "appear far less often". It is simultaneously the most-used **and** the most diagnostically flawed. |
| 2 | OXE is primarily a TRAINING dataset, not an evaluation benchmark | **TRUE** | DROID's official project page compares "co-train with DROID" vs "co-train with **Open-X**" vs "no co-train"; AgiBot World reports pretraining on its data beats **Open X-Embodiment** pretraining by ~30%. OXE appears as a *training-data baseline* in both. |
| 3 | RoboCasa and RoboTwin are commonly used 2024–2025 simulation benchmarks | **TRUE, BUT less often than LIBERO/CALVIN** | Both are in the audit's canonical five; both are venue-published (RoboCasa RSS 2024 p050; RoboTwin CVPR 2025 **Highlight**; RoboTwin 2.0 ICML 2026). The audit notes they "appear far less often in recent progress claims." |
| 4 | CALVIN is used for long-horizon language-conditioned manipulation evaluation | **TRUE** | In the audit's canonical five; audited as a long-horizon language-conditioned suite. |
| 5 | Real-robot suites (DROID, BridgeData V2, RH20T) are used mainly for training/co-training, not standardized leaderboard comparison | **SUPPORTED for DROID** | DROID's own page frames it as a training resource measured by downstream policy gains across 6 tasks/4 locations, not a fixed leaderboard. BridgeData V2 / RH20T were not separately adjudicated → partially 待核实. |
| 6 | SimplerEnv / SimplerEnv-style real-to-sim evaluation is used for OpenVLA/RT-1-style models | **PARTIALLY VERIFIED** | `SimplerEnv` is in the audit's canonical five, confirming it is standard. The specific claim that it is *the* OpenVLA/RT-1 evaluation route was **not** verified → 待核实. |
| 7 | AgiBot World / RoboMIND are used as training data and as eval suites in 2025 | **PARTIAL** | Both papers present themselves as dataset **+ benchmark** (RoboMIND's title literally says "Benchmark"; it ships an Isaac Sim digital twin "enabl[ing] efficient evaluation"). Independent third-party *evaluation* usage was **not** quantified → 待核实. |

### Practical takeaway

The 2024–2025 VLA evaluation landscape is **LIBERO + CALVIN (simulation, dominant but demonstrably gameable)**, with **SimplerEnv** for real-to-sim, and **RoboCasa + RoboTwin 2.0** as the more diagnostic but less-used alternatives. **OXE, DROID, BridgeData V2 and RH20T are training resources, not leaderboards.** Anyone citing a LIBERO improvement as evidence of capability should now also address the four diagnostics above.

### Bonus correction found during this pass

**RoboTwin has two arXiv entries** — do not cite the wrong one:
- `arXiv:2409.02920` — the **early version** ("...with Generative Digital Twins **(early version)**"), no venue in metadata.
- `arXiv:2504.13059` — the **CVPR 2025 version**, whose `Comments:` field reads **"CVPR 2025 Highlight. 22 pages."** (verified by me directly). Cite this one.
- Also exists: RoboTwin CVPR 2025 MEIS Workshop challenge report (`arXiv:2506.23351`), per a sibling verification file — not independently re-checked here.

---

## Corrections to the brief (action items)

| Brief said | Reality |
|---|---|
| OXE ICRA 2024 — did it win a best paper award? | **Yes** — winner of the ICRA 2024 **Best Conference Paper Award** (tied with NoMaD). Also *finalist* in Best Student Paper and Robot Manipulation. |
| DROID 76k / 564 scenes / 86 tasks | Project website says 76k/564/86; **revised arXiv abstract says 76k/564/84**; original RSS 2024 abstract says **65k**/564/86. Version-dependent. |
| RH20T = `arXiv:2307.08898` | **WRONG ID.** Correct: **`arXiv:2307.00595`**. ICRA 2024 confirmed. |
| EgoMimic = `arXiv:2406.04093` | **WRONG ID.** Correct: **`arXiv:2410.24221`**. Venue 待核实. |
| BRS uses an RBY1 robot | **WRONG.** It uses the **Galaxea R1**; collection interface is **JoyLo** (Joy-Cons + leader arms), not VR. |
| ARIO's full name / id | Found: **"All Robots in One"**, `arXiv:2408.10899`. Venue 待核实. |
| RoboTwin 2.0 "if it exists" | **Exists** — `arXiv:2506.18088`, ICML 2026. |
| Fourier ActionNet — VR headset? | Yes, but **Apple Vision Pro**, not PICO/Quest. |

---

## URLs used

**arXiv abstract pages**
- https://arxiv.org/abs/2310.08864 — Open X-Embodiment / RT-X
- https://arxiv.org/abs/2403.12945 — DROID (v2) ; https://arxiv.org/abs/2403.12945v1
- https://arxiv.org/abs/2412.13877 — RoboMIND
- https://arxiv.org/abs/2503.06669 — AgiBot World Colosseo
- https://arxiv.org/abs/2406.02523 — RoboCasa
- https://arxiv.org/abs/2409.02920 — RoboTwin (v1)
- https://arxiv.org/abs/2506.18088 — RoboTwin 2.0
- https://arxiv.org/abs/2408.10899 — ARIO / All Robots in One
- https://arxiv.org/abs/2308.12952 — BridgeData V2
- https://arxiv.org/abs/2307.00595 — RH20T (correct ID)
- https://arxiv.org/abs/2410.24221 — EgoMimic (correct ID)
- https://arxiv.org/abs/2307.08898 — the *wrong* ID given for RH20T (CaMP-INC)
- https://arxiv.org/abs/2406.04093 — the *wrong* ID given for EgoMimic (sparse autoencoders)

**Official proceedings / publisher records**
- https://ewh.ieee.org/soc/ras/conf/fullysponsored/icra/icra%202024/awards-and-finalists/ — ICRA 2024 official awards (OXE = Best Conference Paper winner)
- https://www.ieee-ras.org/ras-congratulates-the-honorees-at-the-icra-2024-awards-ceremony/ — RAS ICRA 2024 awards
- https://aihub.org/2024/05/20/congratulations-to-the-icra2024-best-paper-winners/ — ICRA 2024 winners mirror
- https://www.roboticsproceedings.org/rss20/index.html — RSS 2024 (RSS XX) Table of Contents
- https://www.roboticsproceedings.org/rss20/p120.html — DROID, RSS 2024 (DOI 10.15607/RSS.2024.XX.120)
- https://www.roboticsproceedings.org/rss20/p050.html — RoboCasa, RSS 2024
- https://www.roboticsproceedings.org/rss20/p133.html — THE COLOSSEUM, RSS 2024
- https://www.roboticsproceedings.org/rss20/p090.html — Octo, RSS 2024
- https://proceedings.mlr.press/v229/walke23a.html — BridgeData V2, CoRL 2023 (PMLR v229)
- https://researchr.org/publication/WalkeBZVZHHMKDL23/bibliographies — BridgeData V2 CoRL 2023 record
- https://openaccess.thecvf.com/content/CVPR2025/html/Mu_RoboTwin_Dual-Arm_Robot_Benchmark_with_Generative_Digital_Twins_CVPR_2025_paper.html — RoboTwin, CVPR 2025
- https://icml.cc/virtual/2026/poster/62192 — RoboTwin 2.0, ICML 2026
- https://neurips.cc/virtual/2023/77275 — RH20T, NeurIPS 2023 Robot Learning Workshop
- https://ieeexplore.ieee.org/document/10611615 — RH20T IEEE record
- https://ieeexplore.ieee.org/abstract/document/11127989 — EgoMimic IEEE record (venue unverified)

**Official project homepages**
- https://robotics-transformer-x.github.io — Open X-Embodiment
- https://droid-dataset.github.io/ — DROID
- https://x-humanoid-robomind.github.io/ — RoboMIND
- https://agibot-world.com/ ; https://github.com/OpenDriveLab/AgiBot-World — AgiBot World
- https://robocasa.ai/ — RoboCasa
- https://robotwin-benchmark.github.io/early-version/ — RoboTwin v1
- https://robotwin-platform.github.io/ ; https://github.com/robotwin-Platform/robotwin — RoboTwin 2.0
- https://imaei.github.io/project_pages/ario/ ; https://ario-dataset.github.io/en.html — ARIO
- https://rail-berkeley.github.io/bridgedata — BridgeData V2
- https://rh20t.github.io/ ; https://github.com/rh20t/rh20t_api — RH20T
- https://egomimic.github.io/ ; https://github.com/SimarKareer/EgoMimic ; https://huggingface.co/datasets/gatech/EgoMimic — EgoMimic
- https://action-net.org/ — Fourier ActionNet
- https://behavior-robot-suite.github.io/ ; https://huggingface.co/datasets/behavior-robot-suite/data — BRS
- https://huggingface.co/KarlP/droid — DROID annotations/calibrations

---

# ADDENDUM — items 13, 14, 15 and extra 2025 datasets (verification thread completed after the main report)

**Evidence-grade caveat for this addendum:** the delegated verifier could **not** reach `arxiv.org`, `export.arxiv.org`, `ar5iv`, or `huggingface.co` from its environment. Its arXiv-metadata claims therefore come via the **alphaxiv mirror** (lower confidence than arxiv.org itself), and it could not open HuggingFace dataset cards (so HF *gated* status and exact episode counts are second-hand). It explicitly **discarded alphaxiv's AI-generated "AI Overview"** after catching it contradicting the abstract (RoboMIND 2.0: abstract says 739 tasks, AI Overview said 759). Claims carry `[mirror]` below where that applies. Everything I could re-check myself is marked.

## 13. GR-1 / GR-2 / large 2025 VLA datasets — RESOLVED

**GR-1 and GR-2 are MODELS, not datasets. Neither was released as a dataset.**
- **GR-1** — `arXiv:2312.13139`, *"Unleashing Large-Scale Video Generative Pre-training for Visual Robot Manipulation"*, ByteDance, Dec 2023. GPT-style language-conditioned multi-task manipulation model. The abstract says only "a large-scale video dataset" — **the identity of that pre-training video dataset is 待核实**. `[mirror]`
- **GR-2** — `arXiv:2410.06158`, *"GR-2: A Generative Video-Language-Action Model with Web-Scale Knowledge for Robot Manipulation"*, ByteDance Research, Oct 2024. Abstract states pre-training on **38M video clips / >50B tokens**, then fine-tuning on robot trajectories. Not released as a dataset. `[mirror]`
- ⚠️ **Three-way name collision on "GR"**: (a) ByteDance's GR-1/GR-2 **models**; (b) **Fourier's GR-1/GR-2 humanoid robots** (the platforms used by Fourier ActionNet, item 11) — cross-evidence: NVIDIA's own blog states GR00T N1 was deployed on "the Fourier GR-1 humanoid robot"; (c) unrelated robotics terms. Never write "the GR-1 dataset".

### Additional 2025 datasets verified (only URL-backed ones listed)

| Name | Venue+Year | scale | how collected | open-source |
|---|---|---|---|---|
| **RoboMIND 2.0** | `arXiv:2512.24653` preprint (v-date reported 2026-02); venue 待核实 | 310K+ dual-arm trajectories, 6 embodiments, **739 tasks**; incl. 12K tactile-augmented episodes, 20K mobile-manipulation trajectories, plus a 20K-trajectory simulation set | real-robot; **specific teleop hardware not stated in the abstract → 待核实** | partial/yes — https://modelscope.cn/collections/X-Humanoid/RoboMIND20 (announced on the official project page) |
| **NVIDIA Physical AI Dataset (GR00T open dataset)** | released 2025-03-18 alongside GR00T N1 (`arXiv:2503.14734`) | **15 TB, 320,000+ trajectories**, plus up to 1,000 OpenUSD/SimReady assets | **NOT teleoperation — officially synthetic** (Omniverse/Isaac generated) | yes — https://huggingface.co/collections/nvidia/physicalai-67c643edbb024053dcbcd6d8 |
| **DreamGen** | `arXiv:2505.12705` (NVIDIA/UW/KAIST/UCLA/UCSD/Caltech/NTU/UMD/UT Austin); **venue 待核实** (search hints CoRL 2025, not confirmed from proceedings) | product is synthetic "neural trajectories" from a video world model; real-robot experiments on GR1 humanoid, Franka, SO-100 | **not a teleoperation dataset** — image→video generation + IDM/LAPA to recover pseudo-actions; real teleop data used only as a fine-tuning seed | partial — abstract says "Code available at this URL" but the URL did not resolve → **待核实** |
| **EgoScale** | `arXiv:2602.16710` (**Feb 2026** — note the year does not match a "2025" list) | **20,854 hours** of action-annotated first-person human video; 22-DoF dexterous hand | **not robot teleoperation data** — human-video pre-training + human-robot alignment mid-training | 待核实 (no repo link on the page) |

**Model papers mis-reported as dataset papers** (flagged so they are not cited as datasets): **π0 / openpi** `arXiv:2410.24164` is a VLA *model* (flow matching, PaliGemma + action expert) — openpi ships code + weights, **whether a dataset was released is 待核实**; **VLA-Adapter** `arXiv:2509.09372` (AAAI 2025) is a *model/method* (0.5B lightweight VLA) evaluated on LIBERO, CALVIN, and a real 6-DoF Synria Alicia-D.

**Negative results (important):** no verifiable **2025 large-scale manipulation dataset release from Google/DeepMind** was found (Gemini Robotics is a model family), and **none from ByteDance** either. Both → 待核实. Treat any claim of "DeepMind's/ByteDance's 2025 robot dataset" as unsupported.

## 14. 2025 dataset built from VR/XR teleoperation at scale — still 待核实, with better bounds

No PICO-headset large-scale dataset was verified. Verified XR-adjacent facts:
- **Fourier ActionNet** (2025, 30K+ trajectories) uses **Apple Vision Pro** — XR, but not PICO/Quest.
- **DROID** remains the largest verified headset-teleoperated dataset (**Oculus/Meta Quest 2**, 76k trajectories) but is a 2023–24 collection (RSS 2024).
- **BRS** (2025) deliberately uses **non-VR** JoyLo and reports it **beating VR controllers and Apple Vision Pro** in a 10-participant study — the strongest published counter-signal to "VR teleop is the best collection interface".
- A sibling verification pass in this workspace ([teleop-systems-venue-verification.md](teleop-systems-venue-verification.md), produced by another agent, not re-verified by me) lists several venue-checked VR-teleoperation systems that are relevant here: **OPEN TEACH** (CoRL 2024, PMLR v270 `iyer25a`; Meta Quest 3 bare-hand tracking, 5 robot platforms), **Bimanual Dexterity / Bidex** (CoRL 2024, PMLR v270 `shaw25a`; reports beating Vision Pro and SteamVR baselines), **Haptic-ACT** (IROS 2025, DOI 10.1109/iros60139.2025.11247471; immersive VR + haptics), **LeVR** (`arXiv:2509.14349`, modular VR teleop framework, venue 待核实), and **XRoboToolkit** (`arXiv:2508.00097`, accepted **SII 2026**, DOI 10.1109/sii64115.2026.11404528 — note the venue year is 2026, not 2025). Attribute these to that file; they are not independently re-checked here.

## 15. Genie-1 / Genie Envisioner — partially resolved

- **DeepMind's Genie is a WORLD MODEL, not a manipulation dataset.** Verified from the official DeepMind blog *"Genie 3: A new frontier for world models"* (2025-08-05, Jack Parker-Holder & Shlomi Fruchter): it generates and explores interactive worlds. https://deepmind.google/blog/genie-3-a-new-frontier-for-world-models/
- ⚠️ **Second "Genie" collision:** **Genie Operator-1 (GO-1)** in the AgiBot World paper is a *policy model* (see item 4), unrelated to DeepMind's Genie.
- **"Genie Envisioner" itself remains 待核实** — not verified in this pass. Do not cite it.
