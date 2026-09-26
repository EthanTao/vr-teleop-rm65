# Robot Teleoperation / Demonstration-Collection Systems — Venue, Hardware, Open-Source Verification

Verification date: researched live via arXiv abstract pages, PMLR proceedings, RSS online proceedings, OpenAlex API, GitHub READMEs, and project homepages.

**Evidence-level legend**
- **[P]** = verified from an official proceedings/publisher page (PMLR, roboticsproceedings.org, IEEE DOI record via OpenAlex, GitHub README venue tag)
- **[A]** = inferred only from the arXiv `Comments` field
- **待核实** = could not verify; do not treat as fact

---

## Part 1 — The 20 named systems

`Name | Venue+Year | core contribution | hardware form factor | open-source`

1. **ALOHA** | **RSS 2023** (Robotics: Science and Systems XIX, paper p016; DOI 10.15607/rss.2023.xix.016) **[P]** — arXiv:2304.13705 has an *empty* Comments field, so the venue is **not** visible on the arXiv page; it was confirmed from the RSS XIX online proceedings and the DOI record. | Low-cost bimanual teleoperation rig + Action Chunking with Transformers (ACT), enabling fine manipulation (battery slotting, cable ties) from ~10 min of demos. | **leader-follower arms** (two 6-DoF arms; operator physically backdrives the follower-aligned leader) | **yes** — https://github.com/Physical-Intelligence/aloha + https://tonyzhaozh.github.io/aloha/

2. **Mobile ALOHA** | **CoRL 2024** (PMLR v270, `fu25b`) **[P]** — note the PMLR title is "...Learning Bimanual Mobile Manipulation *using* Low-Cost Whole-Body Teleoperation", while arXiv 2401.02117 says "*with*". | Adds a mobile base and a whole-body teleoperation interface to ALOHA; co-training with static ALOHA data raises mobile-manipulation success by up to 90%. | **leader-follower arms + mobile base** (two leader arms backdriven by the operator, plus a waist/base teleop interface — "whole-body" here means arms + base, not a mocap suit) | **yes** — https://github.com/MarkFzp/mobile-aloha + https://mobile-aloha.github.io/

3. **ALOHA 2 / ALOHA Unleashed** | **ALOHA 2: arXiv:2405.02292 preprint, 2024 — no published venue found (待核实)**. **ALOHA Unleashed: CoRL 2024** (PMLR v270, `zhao25b`) **[P]**. | *(ALOHA 2)* enhanced low-cost bimanual teleoperation hardware. *(ALOHA Unleashed)* a "simple recipe" of large-scale data collection on ALOHA 2 + Diffusion Policy for deformable/contact-rich bimanual tasks. | **leader-follower arms** (ALOHA 2 = improved ALOHA bimanual leader-follower rig) | **partial** — ALOHA 2 hardware/project page https://aloha-2.github.io/ ; ALOHA Unleashed code release 待核实 (project page https://aloha-unleashed.github.io/)

4. **UMI (Universal Manipulation Interface)** | **RSS 2024** (Robotics: Science and Systems XX, paper p045; DOI 10.15607/rss.2024.xx.045) **[P]** — confirmed on the official RSS XX table of contents, not just the arXiv comment. | Hand-held-gripper data collection with latency matching and a relative-trajectory action representation, yielding hardware-agnostic policies deployable zero-shot across robot platforms. | **handheld gripper** (GoPro-in-gripper, no robot needed during collection) | **yes** — https://umi-gripper.github.io (hardware + software open-sourced)

5. **DexCap** | **RSS 2024** (Robotics: Science and Systems XX, paper p043; DOI 10.15607/rss.2024.xx.043) **[P]** — official RSS XX TOC **and** the GitHub README tagged "[RSS 2024]". (Widely mis-cited as CoRL 2024.) | Portable hand-motion-capture data collection plus DexIL (IK + point-cloud imitation) for dexterous hands, with optional human-in-the-loop correction. | **mocap glove / wearable hand mocap** (SLAM + electromagnetic field tracking of wrist and fingers, with 3D environment capture) — *not* a VR headset | **yes** — https://github.com/j96w/DexCap + https://dex-cap.github.io

6. **Open-TeleVision** | **CoRL 2024** (PMLR v270:2729–2749, `cheng25b`) **[P]** | Immersive teleoperation with active stereoscopic visual feedback plus arm/hand mirroring; validated by collecting data and training IL policies on 4 long-horizon tasks across 2 humanoid robots. | **VR headset + hand tracking** (stereo active vision streamed to a headset; operator arm/hand motion mirrored onto the robot) | **yes** — https://github.com/OpenTeleVision/TeleVision + https://robot-tv.github.io/

7. **Bunny-VisionPro** | **IROS 2025** (DOI 10.1109/iros60139.2025.11247017) **[P]** | Real-time bimanual dexterous teleoperation with VR plus custom low-cost haptic devices, with collision and singularity avoidance; teleoperation quality improves downstream imitation learning. | **VR headset + hand tracking + custom low-cost haptic device** (wearable haptic feedback on the operator) | **yes** — https://github.com/Dingry/BunnyVisionPro + https://dingry.github.io/projects/bunny_visionpro.html

8. **AnyTeleop** | **RSS 2023** (Robotics: Science and Systems XIX, paper p015) **[P]** + arXiv Comments field says "Robotics: Science and Systems 2023". (Not CoRL 2023.) | A unified vision-based teleoperation system supporting multiple arms, hands, realities (sim/real) and camera configurations within one system. | **vision-based hand tracking** (camera/webcam-driven hand pose; no dedicated hardware required, VR optional) | **yes** — https://yzqin.github.io/anyteleop/

9. **TeleAvatar** | **待核实 — no matching academic publication found.** OpenAlex title search, OpenAlex full-text search, and multiple web searches returned no arXiv paper, conference paper, or DOI under the name "TeleAvatar". The only hit is a **commercial product page** ("TeleAvatar Teleoperation Body and TeleDroid System", fairplus.cn), which is not a citable research venue. No arXiv id can be supplied. | **待核实** | 待核实 | 待核实

10. **HATO** — name **confirmed correct**. HATO = "**h**ands-**a**rms **t**ele**o**peration" (and "dove" in Japanese). | **ICRA 2025** (DOI 10.1109/icra55743.2025.11128180) **[P]** — *not* CoRL 2024. | A low-cost hands-arms teleoperation system plus repurposed touch-sensing prosthetic hands, used to learn long-horizon high-precision bimanual visuotactile skills. | **leader-follower arms with multifingered leader hands** (two low-cost leader arms, built on off-the-shelf electronics and UR5e/GELLO-style leaders; the robot side uses two PSYONIC Ability prosthetic hands with touch sensing) | **yes** — https://github.com/toruowo/hato + https://toruowo.github.io/hato/

11. **AirExo / AirExo-2** | **AirExo: ICRA 2024** (DOI 10.1109/icra57147.2024.10610799) **[P]** — GitHub README explicitly tagged "[ICRA 2024]". **The arXiv id you supplied, 2405.02198, is wrong**: that id is *"The Cambridge RoboMaster: An Agile Multi-Robot Research Platform"* (unrelated). The correct AirExo id is **arXiv:2309.14975** ("Low-Cost Exoskeletons for Learning Whole-Arm Manipulation in the Wild"). **AirExo-2: CoRL 2025** (PMLR v305, `fang25a`) **[P]**, arXiv:2503.03081; also appeared as an ICLR 2025 workshop paper. | *(AirExo)* low-cost dual-arm exoskeletons for collecting whole-arm manipulation data "in the wild". *(AirExo-2)* scales that up and improves generalization of the resulting imitation-learning policies. | **exoskeleton** (wearable low-cost dual-arm exoskeleton) | AirExo: **yes** — https://github.com/AirExo/collector + https://airexo.github.io ; AirExo-2 code release **待核实**

12. **GELLO** | **IROS 2024** (DOI 10.1109/iros58592.2024.10801581; publisher record reads "2024 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)") **[P]** — **so the "ICRA 2024" claim is NOT correct.** Also **the arXiv id you supplied, 2305.17080, is wrong**: that id is *"Expand, Rerank, and Retrieve: Query Reranking for Open-Domain Question Answering"* (ACL 2023 Findings, an NLP paper). The correct GELLO id is **arXiv:2309.13037**. | A general framework for building low-cost, intuitive teleoperation systems: a kinematically-equivalent scaled replica of the target arm; a 12-person user study shows it beats VR controllers and 3D spacemice for demonstration collection. | **leader-follower arms** (GELLO is a scaled, kinematically equivalent replica of the target arm, 3D-printed with off-the-shelf motors, parts < $300) | **yes** — software https://github.com/wuphilipp/gello_software , hardware https://github.com/wuphilipp/gello_mechanical , project https://wuphilipp.github.io/gello_site/

13. **ACE** | **CoRL 2024** (PMLR v270, `yang25d`) **[P]** — GitHub README tagged "#-corl-2024-". Note the PMLR title is "ACE: A Cross-platform and visual-Exoskeletons System for Low-Cost Dexterous Teleoperation" (arXiv 2408.11805 capitalizes it "A Cross-Platform Visual-Exoskeletons System"). | A single cross-platform visual-exoskeleton teleoperation system that generalizes to humanoid hands, arm-hands, arm-grippers and quadruped-grippers. | **visual-exoskeleton** (hand-facing camera for 3D hand pose + exoskeleton on a portable base, capturing finger and wrist poses) | **yes** — https://github.com/ACETeleop/ACETeleop + https://ace-teleop.github.io/

14. **DexUMI** | **CoRL 2025** (PMLR v305, `xu25b`) **[P]**. Correct id = **arXiv:2505.21864** ("DexUMI: Using Human Hand as the Universal Manipulation Interface for Dexterous Manipulation") — *not* a 2505.XXXXX placeholder, and its arXiv Comments field carries no venue, so the CoRL 2025 landing was only visible via PMLR v305. | Uses the human hand as the manipulation interface: a wearable exoskeleton bridges the kinematics gap and video inpainting swaps the human hand for a high-fidelity robot hand, achieving 86% average success across two robot-hand platforms. | **hand exoskeleton** (wearable, with direct haptic feedback during collection) | **yes** — https://github.com/real-stanford/DexUMI

15. **XRoboToolkit** | **arXiv:2508.00097 (2025)**, accepted at **The 2026 IEEE/SICE International Symposium on System Integration (SII 2026)** — DOI 10.1109/sii64115.2026.11404528 **[P]**; the arXiv Comments field independently states "accepted at The 2026 IEEE/SICE International Symposium on System Integration" **[A]**. Note the venue year is **2026**, not 2025. | An OpenXR-based cross-platform XR teleoperation framework (low-latency stereoscopic feedback, optimization-based IK, head/controller/hand/auxiliary-tracker support) explicitly aimed at large-scale VLA demonstration data; VLA models were trained on its data. | **VR/MR headset + controllers + hand tracking + auxiliary IMU motion trackers** | **yes** — https://xr-robotics.github.io/ (this is the SDK used in your own `vr-teleop-rm65` project)

16. **TWIST** | **CoRL 2025** (PMLR v305, `ze25a`) **[P]** — GitHub README tagged "[CoRL 2025]". arXiv:2505.02833. | A teleoperated whole-body imitation system: human motion is retargeted to a humanoid and tracked by RL, producing whole-body manipulation skills. | **mocap suit / full-body motion capture** (human whole-body motion → humanoid; optionally with VR-style operator interface) | **yes** — https://github.com/YanjieZe/TWIST

17. **HumanPlus** | **CoRL 2024** (PMLR v270, `fu25a`) **[P]** — GitHub README tagged "[CoRL 2024]". arXiv:2406.10454. | Humanoid shadowing followed by imitation from human data, letting a humanoid learn locomotion and manipulation skills from human motion without teleoperating hardware on the robot. | **mocap suit / single-RGB-camera human motion capture** (human shadowing → humanoid; the operator's body motion drives the humanoid) | **yes** — https://github.com/MarkFzp/humanplus + https://humanoid-shadowing.github.io/

18. **OpenWBT** | **Exists. No dedicated paper and no arXiv id — 待核实 for any venue.** OpenAlex title search and full-text search return nothing for "OpenWBT". It is an open-source **repository** (Galbot × Tsinghua) whose technical basis is a different paper: *"Unleashing Humanoid Reaching Potential via Real-world-Ready Skill Space" (R2S2)*, **arXiv:2505.10918 (2025), venue 待核实**. | Open-source whole-body teleoperation of Unitree G1/H1 humanoids with Apple Vision Pro, supporting both real hardware and MuJoCo simulation; lower body via a pair of joystick controllers, upper body via IK from VR-captured hand poses. | **VR headset (Apple Vision Pro) + two handheld controllers/joysticks** for lower body; upper body from captured hand poses | **yes** — https://github.com/mrhouse-sweet/OpenWBT (Apache-2.0)

19. **Fuzzy-Dex** | **待核实 — appears not to exist under that name. The id you supplied, arXiv:2409.19745, is wrong**: that id is *"PEAR: Position-Embedding-Agnostic Attention Re-weighting Enhances Retrieval-Augmented Generation with Zero Inference Overhead"* (an NLP/RAG paper). No arXiv or OpenAlex record titled "Fuzzy-Dex" was found. | 待核实 | 待核实 | 待核实

20. **"RoboTeleop" and IEEE VR / ISMAR VR-teleoperation papers** | **"RoboTeleop" itself: 待核实** — no verifiable IEEE VR / ISMAR paper with that name was found. Two **verifiable** IEEE VR / ISMAR papers on VR teleoperation for robot arms were found instead: <br>• **"Intuitive Visual Feedback in Virtual Reality for the Teleoperation of Robots"** | **IEEE VR 2025** (DOI 10.1109/vr59515.2025.00036) **[P]** | studies how visual-feedback design in VR affects robot-arm teleoperation quality. | **VR headset** | 待核实 <br>• **"MIRAGE: Multimodal Intention Recognition and Admittance-Guided Enhancement in VR-based Multi-object Teleoperation"** | **ISMAR 2025** (DOI 10.1109/ismar67309.2025.00060) **[P]**; arXiv:2509.01996 | multimodal intention recognition + admittance control to make VR multi-object teleoperation smoother. | **VR headset** | 待核实

---

## Part 2 — Additional papers found on the same theme (not in your list)

### CoRL 2024 (PMLR v270) **[P]**

| Title | Venue+Year | Contribution | Hardware | Open-source |
|---|---|---|---|---|
| OmniH2O: Universal and Dexterous Human-to-Humanoid Whole-Body Teleoperation and Learning | CoRL 2024 (PMLR 270, `he25b`); arXiv:2406.08858 | Human-to-humanoid whole-body teleoperation plus a sim-to-real RL pipeline for dexterous whole-body skills. | mocap suit / human motion capture → humanoid | yes — https://github.com/LeCAR-Lab/human2humanoid |

### CoRL 2025 (PMLR v305) **[P]**

| Title | Venue+Year | Contribution | Hardware | Open-source |
|---|---|---|---|---|
| AirExo-2: Scaling up Generalizable Robotic Imitation Learning with Low-Cost Exoskeletons (see item 11) | CoRL 2025 (PMLR 305, `fang25a`); arXiv:2503.03081 | Scales exoskeleton-based whole-arm data collection and improves policy generalization. | exoskeleton | 待核实 |
| DexUMI (see item 14) | CoRL 2025 (PMLR 305, `xu25b`); arXiv:2505.21864 | Human hand as the manipulation interface via wearable exoskeleton + robot-hand inpainting. | hand exoskeleton | yes — https://github.com/real-stanford/DexUMI |
| TWIST (see item 16) | CoRL 2025 (PMLR 305, `ze25a`); arXiv:2505.02833 | Teleoperated whole-body humanoid imitation. | mocap suit | yes — https://github.com/YanjieZe/TWIST |

### RSS 2024 (rss20) **[P]**

| Title | Venue+Year | Contribution | Hardware | Open-source |
|---|---|---|---|---|
| Tilde: Teleoperation for Dexterous In-Hand Manipulation Learning with a DeltaHand | RSS 2024 (p128) | A teleoperation system purpose-built for learning dexterous in-hand manipulation with a DeltaHand end-effector. | leader-follower / custom teleoperation device for a dexterous hand | 待核实 |
| DROID: A Large-Scale In-The-Wild Robot Manipulation Dataset | RSS 2024 (p120) | 76k trajectories / 350h of diverse real-world manipulation data collected with a standardized teleoperation setup across 13 institutions — the canonical large-scale demo-collection resource for VLA training. | leader-follower arms (Franka + Robotiq gripper with a standardized teleop rig, incl. VR controller option) | yes — https://droid-dataset.github.io/ |
| Conformalized Teleoperation: Confidently Mapping Human Inputs to High-Dimensional Robot Actions | RSS 2024 (p008) | Uses conformal prediction to safely map low-dimensional human teleoperation inputs to high-dimensional robot actions. | generic teleoperation input (VR/joystick class) | 待核实 |

### IROS 2024 **[P]**
- **GELLO** (see item 12) — the named system in your list, corrected from ICRA 2024 to IROS 2024.

### IROS 2025 **[P]**
| Title | Venue+Year | Contribution | Hardware | Open-source |
|---|---|---|---|---|
| Haptic-ACT: Bridging Human Intuition with Compliant Robotic Manipulation via Immersive VR | IROS 2025 (DOI 10.1109/iros60139.2025.11247471); arXiv:2409.11925 | Combines immersive VR teleoperation with haptic feedback and action chunking for compliant manipulation. | VR headset + haptic device | 待核实 |
| Bunny-VisionPro (see item 7) | IROS 2025 (DOI 10.1109/iros60139.2025.11247017); arXiv:2407.03162 | VR bimanual dexterous teleoperation with low-cost haptics. | VR headset + haptics | yes — https://github.com/Dingry/BunnyVisionPro |

### SII **[P]**
- **"Improving Low-Cost Teleoperation: Augmenting GELLO with Force"** | **SII 2025** (DOI 10.1109/sii59315.2025.10871050) | Adds force feedback to the GELLO leader-arm paradigm. | leader-follower arms | 待核实
- **XRoboToolkit** (see item 15) | **SII 2026** (DOI 10.1109/sii64115.2026.11404528)

### arXiv-only preprints (2025) on the same theme — no venue verified
| Title | Venue+Year | Contribution | Hardware | Open-source |
|---|---|---|---|---|
| TWIST2: Scalable, Portable, and Holistic Humanoid Data Collection System | arXiv:2511.02832 (2025 preprint; venue 待核实) | Portable, holistic humanoid whole-body data-collection system extending TWIST. | mocap suit / portable capture rig | 待核实 |
| LeVR: A Modular VR Teleoperation Framework for Imitation Learning in Dexterous Manipulation | arXiv:2509.14349 (2025 preprint; venue 待核实) | Modular VR teleoperation framework aimed at dexterous-manipulation imitation learning. | VR headset + hand tracking | yes — https://github.com/LeVR-Teleop/LeVR |
| End-to-End Dexterous Arm-Hand VLA Policies via Shared Autonomy: VR Teleoperation Augmented by Autonomous Hand VLA Policy for Efficient Data Collection | arXiv:2511.00139 (2025 preprint; venue 待核实) | Shared-autonomy VR teleoperation where a hand VLA assists the operator to make arm-hand demonstration collection more efficient. | VR headset (arm) + autonomous hand policy | 待核实 |

### Part 2b — Extended conference sweep (delegated scan, all venue-verified against official proceedings pages)

**CoRL 2024 (PMLR v270) [P]**
| Title | Venue+Year | Contribution | Hardware | Open-source |
|---|---|---|---|---|
| OPEN TEACH: A Versatile Teleoperation System for Robotic Manipulation | CoRL 2024 (PMLR 270, `iyer25a`); arXiv:2403.07870 | VR-headset teleop (Meta Quest 3, bare-hand gestures) controlling 5 robot platforms, data validated on 10 manipulation tasks. | VR headset (Quest 3) + bare-hand tracking, no gloves/controllers | yes — https://github.com/aadhithya14/Open-Teach |
| Bimanual Dexterity for Complex Tasks | CoRL 2024 (PMLR 270, `shaw25a`); arXiv:2411.13677 | 50+ DoF low-cost bimanual dexterous teleop (Bidex) beating Vision Pro and SteamVR baselines on data quality and speed. | motion-capture gloves + low-cost leader ("teacher") arms | yes — https://bidex-teleop.github.io/ |
| ManiWAV: Learning Robot Manipulation from In-the-Wild Audio-Visual Data | CoRL 2024 (PMLR 270, `liu25c`); arXiv:2406.19464 | Handheld in-the-wild data-collection device enabling audio-visual policy learning. | handheld gripper with wrist camera + contact microphone | yes — https://github.com/real-stanford/maniwav |

**CoRL 2025 (PMLR v305) [P]**
| Title | Venue+Year | Contribution | Hardware | Open-source |
|---|---|---|---|---|
| TypeTele: Releasing Dexterity in Teleoperation by Dexterous Manipulation Types | CoRL 2025 (PMLR 305, `lin25d`); arXiv:2507.01857 | MLLM-retrieved "manipulation type" library so dexterous hands are not limited to human-hand retargeting. | dexterous-hand teleop via hand-pose capture/retargeting (capture device 待核实) | yes — https://github.com/iSEE-Laboratory/TypeTele |
| CLONE: Closed-Loop Whole-Body Humanoid Teleoperation for Long-Horizon Tasks | CoRL 2025 (PMLR 305, `li25h`); arXiv:2506.08931 | MoE whole-body control + LiDAR-odometry closed-loop correction (12 cm drift over 8.9 m) for long-horizon humanoid data collection. | whole-body operator motion → humanoid (capture device 待核实) | 待核实 |
| Vision in Action: Learning Active Perception from Human Demonstrations | CoRL 2025 (PMLR 305, `xiong25a`); arXiv:2506.15666 | VR interface creating a shared observation space (with an intermediate 3D scene to cut VR sickness) to learn active perception on a 6-DoF neck. | VR headset teleoperating a bimanual robot with 6-DoF neck | 待核实 |
| BEHAVIOR Robot Suite: Streamlining Real-World Whole-Body Manipulation for Everyday Household Activities | CoRL 2025 (PMLR 305, `jiang25b`); arXiv:2503.05652 | Bimanual wheeled robot with 4-DoF torso + cost-effective whole-body teleop interface for household data collection. | low-cost whole-body leader-follower teleop interface (bimanual + torso) | yes — https://behavior-robot-suite.github.io/ |
| FastUMI: A Scalable and Hardware-Independent Universal Manipulation Interface with Dataset | CoRL 2025 (PMLR 305, `zhaxizhuoma25a`); arXiv:2409.19499 | Mechanical re-engineering of UMI + off-the-shelf tracker replacing VIO; releases 15k+ demos over 24 tasks. | handheld gripper (redesigned UMI) + commercial off-the-shelf tracker | yes — https://fastumi.com/ |
| IRIS: An Immersive Robot Interaction System | CoRL 2025 (PMLR 305, `jiang25c`); arXiv:2502.03297 | Robot/simulator-agnostic XR immersive interaction with multi-user data collection across sim and real. | XR headset immersive interaction (rigid + deformable objects, live point clouds) | 待核实 |
| exUMI: Extensible Robot Teaching System with Action-aware Task-agnostic Tactile Representation | CoRL 2025 (PMLR 305, `xu25e`); arXiv:2509.14688 | Handheld UMI-style device adding AR-MoCap + rotary-encoder proprioception and modular visuo-tactile sensing. | handheld gripper, visuo-tactile, AR mocap | yes — https://github.com/silicx/exUMI |
| KineDex: Learning Tactile-Informed Visuomotor Policies via Kinesthetic Teaching for Dexterous Manipulation | CoRL 2025 (PMLR 305, `zhang25l`); arXiv:2505.01974 | Hand-over-hand kinesthetic teaching replaces teleoperation (2× faster data collection, ~100% vs <50% success), with a tactile-augmented policy. | kinesthetic teaching (operator physically moves the dexterous hand) — no teleop device | 待核实 |

**RSS 2024 [P]**
- **Tilde** (already listed above; arXiv:2405.18804, rss20/p128) — kinematic-twin leader device (TeleHand) giving 1:1 joint control of a soft DeltaHand. Open-source 待核实.

**RSS 2025 (rss21) [P]**
| Title | Venue+Year | Contribution | Hardware | Open-source |
|---|---|---|---|---|
| HOMIE: Humanoid Loco-Manipulation with Isomorphic Exoskeleton Cockpit | RSS 2025 (`rss21/p070`); arXiv:2502.13013 | ~$500 open-source teleop "cockpit" (isomorphic exoskeleton arms + Hall-sensor gloves + pedal) for a humanoid loco-manipulation data flywheel. | exoskeleton arms + Hall-sensor gloves (15+ DoF) + foot pedal | yes — https://github.com/InternRobotics/OpenHomie |
| DOGlove: Dexterous Manipulation with a Low-Cost Open-Source Haptic Force Feedback Glove | RSS 2025 (`rss21/p104`); arXiv:2502.07730 | <$600 21-DoF mocap glove with 5-DoF force and 5-DoF fingertip haptic feedback for contact-rich dexterous teleop. | mocap glove with cable-driven force feedback + LRA fingertip haptics | yes — https://do-glove.github.io/ |
| DexWild: Dexterous Human Interactions for In-the-Wild Robot Policies | RSS 2025 (`rss21/p075`); arXiv:2505.07813 | Low-cost mobile wearable capture system for 9,290 in-the-wild human demos across 93 environments, with human-robot co-training. | wearable palm-camera hand-pose capture system (embodiment-agnostic) | yes — https://github.com/dexwild/dexwild |

**ICRA 2024/2025 and IROS 2024/2025 — INCOMPLETE.** The dedicated ICRA/IROS proceedings sweep did not finish before this report was closed, so the ICRA/IROS sections above contain only what was verified directly: **GELLO (IROS 2024)**, **Bunny-VisionPro (IROS 2025)**, **Haptic-ACT (IROS 2025)**, **HATO (ICRA 2025)**, **AirExo (ICRA 2024)**, **XRoboToolkit (SII 2026)** and **"Improving Low-Cost Teleoperation: Augmenting GELLO with Force" (SII 2025)**. A complete ICRA/IROS enumeration is **待核实**. Note that IEEE Xplore HTML blocks automated fetches, so ICRA/IROS venue checks must go through publisher DOI records or the official conference program.

### Notes on the conference scan
- **CoRL 2024 = PMLR v270**, **CoRL 2025 = PMLR v305** — both are fully published and are the authoritative sources; the CoRL 2025 volume is *not* OpenReview-only.
- **RSS 2024 = roboticsproceedings.org/rss20**, **RSS 2025 = rss21**.
- The ICRA 2024/2025 and IROS 2024/2025 proceedings are IEEE Xplore-only and were verified through publisher DOI records (OpenAlex mirrors IEEE's `raw_source_name`), since IEEE Xplore HTML blocks automated fetches.
- **OpenAlex is the workhorse for venue disambiguation** (it exposes the publisher's container name and DOI), and it caught four substantive errors in the supplied id list.

---

## Part 3 — Corrections to the premises in the request

| Your premise | Verified reality | Confidence |
|---|---|---|
| AirExo = arXiv 2405.02198 | 2405.02198 is "The Cambridge RoboMaster". AirExo = **arXiv 2309.14975**, published at **ICRA 2024** | **[P]** OpenAlex DOI + GitHub README tag |
| GELLO is ICRA 2024 | GELLO is **IROS 2024** (DOI 10.1109/iros58592.2024.10801581) | **[P]** OpenAlex publisher container name |
| GELLO = arXiv 2305.17080 | 2305.17080 is an NLP paper (EAR, ACL 2023 Findings). GELLO = **arXiv 2309.13037** | **[P]** arXiv abstract page + OpenAlex |
| Fuzzy-Dex = arXiv 2409.19745 | 2409.19745 is "PEAR" (an NLP/RAG paper). No "Fuzzy-Dex" record exists | **[P]** OpenAlex + arXiv |
| DexCap (implied CoRL) | DexCap is **RSS 2024** | **[P]** RSS XX TOC p043 |
| HATO (implied distinct from ICRA) | HATO is **ICRA 2025**, and the HATO name is correct | **[P]** OpenAlex DOI |
| XRoboToolkit venue = 2025 | Accepted at **SII 2026** (arXiv is 2025) | **[A]**+**[P]** |
| TWIST is a 2025 preprint | **CoRL 2025** (PMLR v305) | **[P]** PMLR + GitHub tag |
| DexUMI id unknown | **arXiv:2505.21864**, and it reached **CoRL 2025** | **[P]** PMLR v305 |
| TeleAvatar is a 2025 paper | No academic record found; only a commercial product page | 待核实 |

---

## Part 4 — Every URL used

**arXiv abstract pages**
- https://arxiv.org/abs/2304.13705 (ALOHA)
- https://arxiv.org/abs/2401.02117 (Mobile ALOHA)
- https://arxiv.org/abs/2410.13126 (ALOHA Unleashed)
- https://arxiv.org/abs/2402.10329 (UMI)
- https://arxiv.org/abs/2403.07788 (DexCap)
- https://arxiv.org/abs/2407.01512 (Open-TeleVision)
- https://arxiv.org/abs/2407.03162 (Bunny-VisionPro)
- https://arxiv.org/abs/2307.04577 (AnyTeleop)
- https://arxiv.org/abs/2404.16823 (HATO)
- https://arxiv.org/abs/2405.02198 (Cambridge RoboMaster — the wrong id)
- https://arxiv.org/abs/2305.17080 (EAR — the wrong id)
- https://arxiv.org/abs/2408.11805 (ACE)
- https://arxiv.org/abs/2505.21864 (DexUMI)
- https://arxiv.org/abs/2508.00097 (XRoboToolkit)
- https://arxiv.org/abs/2309.14975 (AirExo)
- https://arxiv.org/abs/2309.13037 (GELLO)
- https://arxiv.org/abs/2503.03081 (AirExo-2)
- https://arxiv.org/abs/2409.11925 (Haptic-ACT)
- https://arxiv.org/abs/2509.14349 (LeVR)
- https://arxiv.org/abs/2505.10918 (R2S2)
- https://arxiv.org/abs/2511.00139 (Shared-autonomy VR teleop VLA)
- https://arxiv.org/abs/2511.02832 (TWIST2)
- https://arxiv.org/abs/2406.08858 (OmniH2O)
- https://arxiv.org/abs/2405.02292 (ALOHA 2)
- https://arxiv.org/abs/2505.02833 (TWIST)
- https://arxiv.org/abs/2406.10454 (HumanPlus)
- http://arxiv.org/pdf/2309.13037v2 (GELLO PDF)
- https://export.arxiv.org/pdf/2503.03081v1 (AirExo-2 PDF)

**Official proceedings (PMLR / RSS)**
- https://proceedings.mlr.press/v270/ (CoRL 2024 index)
- https://proceedings.mlr.press/v270/fu25b.html (Mobile ALOHA)
- https://proceedings.mlr.press/v270/zhao25b.html (ALOHA Unleashed)
- https://proceedings.mlr.press/v270/cheng25b.html (Open-TeleVision)
- https://proceedings.mlr.press/v270/fu25a.html (HumanPlus)
- https://proceedings.mlr.press/v270/yang25d.html (ACE)
- https://proceedings.mlr.press/v270/he25b.html (OmniH2O)
- https://proceedings.mlr.press/v305/ze25a.html (TWIST — CoRL 2025)
- https://proceedings.mlr.press/v305/fang25a.html (AirExo-2 — CoRL 2025)
- https://proceedings.mlr.press/v305/xu25b.html (DexUMI — CoRL 2025)
- https://proceedings.mlr.press/v270/iyer25a.html (OPEN TEACH — CoRL 2024)
- https://proceedings.mlr.press/v270/shaw25a.html (Bimanual Dexterity for Complex Tasks / Bidex — CoRL 2024)
- https://proceedings.mlr.press/v270/liu25c.html (ManiWAV — CoRL 2024)
- https://proceedings.mlr.press/v305/lin25d.html (TypeTele — CoRL 2025)
- https://proceedings.mlr.press/v305/li25h.html (CLONE — CoRL 2025)
- https://proceedings.mlr.press/v305/xiong25a.html (Vision in Action — CoRL 2025)
- https://proceedings.mlr.press/v305/jiang25b.html (BEHAVIOR Robot Suite — CoRL 2025)
- https://proceedings.mlr.press/v305/zhaxizhuoma25a.html (FastUMI — CoRL 2025)
- https://proceedings.mlr.press/v305/jiang25c.html (IRIS — CoRL 2025)
- https://proceedings.mlr.press/v305/xu25e.html (exUMI — CoRL 2025)
- https://proceedings.mlr.press/v305/zhang25l.html (KineDex — CoRL 2025)
- https://proceedings.mlr.press/v305/shi25a.html (CoRL 2025)
- https://proceedings.mlr.press/v305/ (CoRL 2025 index)
- https://roboticsproceedings.org/rss21/index.html (RSS 2025 TOC)
- https://roboticsproceedings.org/rss20/p128.html (Tilde — RSS 2024)
- https://roboticsproceedings.org/rss21/p070.html (HOMIE — RSS 2025)
- https://roboticsproceedings.org/rss21/p075.html (DexWild — RSS 2025)
- https://roboticsproceedings.org/rss21/p104.html (DOGlove — RSS 2025)
- https://api2.openreview.net/notes/search?term=teleoperation&group=robot-learning.org/CoRL/2025/Conference (used for discovery only, not as venue of record)
- https://openreview.net/forum?id=Yce2jeILGt (Open-TeleVision, CoRL 2024)
- https://github.com/aadhithya14/Open-Teach , https://bidex-teleop.github.io/ , https://github.com/real-stanford/maniwav , https://github.com/iSEE-Laboratory/TypeTele , https://behavior-robot-suite.github.io/ , https://fastumi.com/ , https://airexo.tech/airexo2/ , https://github.com/InternRobotics/OpenHomie , https://do-glove.github.io/ , https://github.com/dexwild/dexwild , https://dexwild.github.io/ , https://github.com/silicx/exUMI
- https://arxiv.org/abs/2505.07813 , https://arxiv.org/html/2405.18804v1 , https://ar5iv.labs.arxiv.org/html/2403.07870 , https://ar5iv.labs.arxiv.org/html/2411.13677 , https://ar5iv.labs.arxiv.org/html/2406.08858 , https://ar5iv.labs.arxiv.org/html/2507.01857 , https://ar5iv.labs.arxiv.org/html/2506.08931 , https://ar5iv.labs.arxiv.org/html/2506.15666 , https://ar5iv.labs.arxiv.org/html/2509.14688 , https://ar5iv.labs.arxiv.org/html/2505.01974 , https://ar5iv.labs.arxiv.org/html/2409.19499 , https://ar5iv.labs.arxiv.org/html/2503.03081 , https://ar5iv.labs.arxiv.org/html/2502.03297 , https://ar5iv.labs.arxiv.org/html/2503.05652 , https://ar5iv.labs.arxiv.org/html/2502.13013
- https://pmlr.com.cn/v270/cheng25b.html (PMLR mirror)
- https://pmlr.com.cn/v305/fang25a.html (PMLR mirror)
- https://pmlr.com.cn/v270/yang25d.html (PMLR mirror)
- https://www.roboticsproceedings.org/rss20/index.html (RSS 2024 TOC: DexCap p043, UMI p045, Tilde p128, DROID p120, Conformalized Teleoperation p008)
- https://www.roboticsproceedings.org/rss19/p016.pdf (ALOHA — RSS 2023)
- https://roboticsproceedings.org/rss19/p015.pdf (AnyTeleop — RSS 2023)
- https://rss2023.github.io/rss2023-website/program/papers/015/ (AnyTeleop, RSS 2023 program)

**Publisher DOI records / bibliographic APIs**
- https://api.openalex.org/works?filter=title.search:AirExo... (AirExo ICRA 2024; AirExo-2)
- https://api.openalex.org/works?filter=title.search:GELLO%20teleoperation... (GELLO IROS 2024; GELLO+Force SII 2025)
- https://api.openalex.org/works?filter=title.search:Universal%20Manipulation%20Interface... (UMI RSS 2024)
- https://api.openalex.org/works?filter=title.search:DexCap%20Mocap%20Data%20Collection... (DexCap RSS 2024)
- https://api.openalex.org/works?filter=title.search:Bunny-VisionPro... (IROS 2025)
- https://api.openalex.org/works?filter=title.search:Learning%20Visuotactile%20Skills... (HATO ICRA 2025)
- https://api.openalex.org/works?filter=title.search:Learning%20Fine-Grained%20Bimanual%20Manipulation... (ALOHA RSS 2023)
- https://api.openalex.org/works?filter=title.search:XRoboToolkit... (SII 2026)
- https://api.openalex.org/works?filter=title.search:DexUMI... (arXiv-only record)
- https://api.openalex.org/works?filter=title.search:Haptic-ACT... (IROS 2025)
- https://api.openalex.org/works?filter=title.search:MIRAGE... (ISMAR 2025)
- https://api.openalex.org/works?filter=title.search:Intuitive%20Visual%20Feedback%20in%20Virtual%20Reality... (IEEE VR 2025)
- https://api.openalex.org/works?filter=title.search:End-to-End%20Dexterous%20Arm-Hand%20VLA%20Policies... (arXiv 2511.00139)
- https://api.openalex.org/works?filter=title.search:TWIST2... (arXiv 2511.02832)
- https://api.openalex.org/works?filter=title.search:ALOHA%202%20Enhanced%20Low-Cost%20Hardware... (arXiv 2405.02292)
- https://api.openalex.org/works?filter=title.search:OpenWBT... (no record)
- https://api.openalex.org/works?search=TeleAvatar (no relevant record)
- https://ieeexplore.ieee.org/document/10801581 (GELLO — IROS 2024)
- https://ieeexplore.ieee.org/document/11128180 (HATO — ICRA 2025)
- https://ieeexplore.ieee.org/document/11247471 (Haptic-ACT — IROS 2025)
- https://ieeexplore.ieee.org/document/10937439 (Intuitive Visual Feedback in VR — IEEE VR 2025)
- https://ieeexplore.ieee.org/document/11377888 (Low-Cost VR Teleoperation w/ Diffusion Policy — venue 待核实)
- https://www.computer.org/csdl/proceedings-article/ismar/2025/876100a496/2byAG3fqv16 (MIRAGE — ISMAR 2025)

**Project pages, code repositories, homepages**
- https://tonyzhaozh.github.io/aloha/ and https://github.com/Physical-Intelligence/aloha
- https://mobile-aloha.github.io/ and https://github.com/MarkFzp/mobile-aloha
- https://aloha-2.github.io/
- https://aloha-unleashed.github.io/
- https://umi-gripper.github.io
- https://dex-cap.github.io and https://github.com/j96w/DexCap
- https://robot-tv.github.io/ and https://github.com/OpenTeleVision/TeleVision
- https://dingry.github.io/projects/bunny_visionpro.html and https://github.com/Dingry/BunnyVisionPro
- https://yzqin.github.io/anyteleop/
- https://toruowo.github.io/hato/ and https://github.com/toruowo/hato
- https://github.com/AirExo/collector and https://airexo.github.io/
- https://wuphilipp.github.io/gello_site/ , https://github.com/wuphilipp/gello_software , https://github.com/wuphilipp/gello_mechanical
- https://ace-teleop.github.io/ and https://github.com/ACETeleop/ACETeleop
- https://github.com/real-stanford/DexUMI
- https://xr-robotics.github.io/
- https://github.com/YanjieZe/TWIST
- https://github.com/MarkFzp/humanplus and https://humanoid-shadowing.github.io/
- https://github.com/mrhouse-sweet/OpenWBT and https://raw.githubusercontent.com/mrhouse-sweet/OpenWBT/main/README_zh.md and https://zzk273.github.io/R2S2/
- https://github.com/LeCAR-Lab/human2humanoid (OmniH2O)
- https://droid-dataset.github.io/ (DROID)
- https://github.com/LeVR-Teleop/LeVR (LeVR)
- https://fairplus.cn/en/products/100121/p000172/ (TeleAvatar commercial product — NOT a paper)
- https://mlanthology.org/iclrw/2025/fang2025iclrw-airexo2/ (AirExo-2 ICLR 2025 workshop)
- https://ui.adsabs.harvard.edu/abs/2025arXiv250800097Z/abstract (XRoboToolkit ADS record)
