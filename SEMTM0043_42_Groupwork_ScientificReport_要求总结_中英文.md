# SEMTM0043/42 Groupwork Scientific Report：核心要求总结 / Core Requirements Summary

> 本文件用于整理和核对小组科学报告（Assessment 2）的官方要求，不是可直接提交的报告正文。中文为概括与提炼，英文对应官方规范与模板原文要点。
>
> This document summarises the official requirements for the Assessment 2 Groupwork Scientific Report. It is not a submission draft. Both languages paraphrase and consolidate the source specification and report template.

---

## 1. 来源与适用范围 / Sources and Scope

本总结依据本地课程规范与模板文件整理，未从网络下载额外材料：

This summary is based on the following local course specification and template files; no external materials were downloaded:

- 评估规范 PDF / Assessment specification PDF：`/Users/lanqiuying/Robotics_Msc/robotics_science_systems/AssessmentSpecification_SEMTM0043.pdf`（pp. 2–3, 6–8）
- 小组报告模板 PDF / Groupwork report template PDF：`/Users/lanqiuying/Robotics_Msc/robotics_science_systems/SEMTM0043_42_Groupwork_ScientificReport.pdf`（pp. 1–4）
- LaTeX 正文模板 / LaTeX main template：`/Users/lanqiuying/Robotics_Msc/robotics_science_systems/SEMTM0043_42_Groupwork_ScientificReport/main.tex`
- LaTeX 样式文件 / LaTeX style file：`/Users/lanqiuying/Robotics_Msc/robotics_science_systems/SEMTM0043_42_Groupwork_ScientificReport/cw2026-paper.sty`
- 模板说明文件 / Template README：`/Users/lanqiuying/Robotics_Msc/robotics_science_systems/SEMTM0043_42_Groupwork_ScientificReport/README.md`

---

## 2. 最重要的硬性要求 / Essential Requirements

| 项目 / Item | 中文说明 | English Summary |
|---|---|---|
| 作业性质与权重 / Weight & Basis | Assessment 2（Phase 2），小组科学报告，占总评 **70%**。 | Assessment 2 (Phase 2): Group-basis scientific report, weighted **70%** of the final unit mark. |
| 截止时间 / Deadline | **2026 年 12 月 3 日（周四）13:00**（与 Assessment 1 个人报告同一截止时间）。 | **1pm Thursday 3rd December 2026** (same deadline as Assessment 1). |
| 小组规模与匿名要求 / Group Size & Anonymity | 2–4 名学生。报告标题下仅填写全组每位成员的 **7 位 Bristol 学号**（保留前导零）；**严禁在正文、图表或 PDF 元数据中出现姓名**。 | Groups of 2, 3, or 4 students. List only **7-digit Bristol student IDs** (retaining any leading zeros); **do not include names** in visible material or PDF metadata. |
| AI 使用政策 / AI-Use Policy | University of Bristol **Category 4 (Integral)**：允许、鼓励并支持使用及批判性反思生成式 AI 工具（如另有要求须遵守学术诚信与引用规范）。 | University of Bristol **Category 4 (Integral)**: permitted, encouraged, and supported to use and critically reflect on Generative AI tools. |
| 核心定位 / Core Purpose | 在选定的项目主题下，同时在**真实机器人**与**数字孪生仿真**上开展完整实验；不仅要对比两者表现，更要基于证据论证**哪些行为最关键需要建模**，并剖析**仿真与现实差异为何存在**。 | Evaluate the full Digital Twin Robotic System on a chosen task across **both the real robot and simulation**; build an evidence-supported argument on which aspects are most important to model and why discrepancies remain. |
| 必须提交的三项材料 / 3 Mandatory Deliverables | 1. **科学报告（Scientific Report）**：最多 **3 页 A4**；<br>2. **完整代码**：真实机器人（StampC3）与数字孪生仿真（Processing）代码，以及数据分析与绘图代码；<br>3. **原始数值数据（Original Data）**：实验采集的原始数值记录（如 CSV），仅有处理后的图表不能满足要求。<br>**⚠️ 缺上述 (1)(2)(3) 中任意一项，直接按未提交计 0 分。** | 1. **Scientific report**: **3 A4 pages maximum**;<br>2. **Code**: developed for both the real robot (StampC3) and digital twin simulation (Processing), plus analysis/plotting scripts;<br>3. **Original numerical data**: raw collected measurements (processed plots alone are insufficient).<br>**⚠️ Omission of any of (1), (2), or (3) results in an awarded mark of 0 (non-submission).** |
| 篇幅与排版 / Length & Layout | 使用官方 LaTeX 模板（A4、10 pt、双栏），不可通过缩小字体或页边距塞内容。模板不自动截断超页内容，需自行检查页数。注：模板第 4 页写明 *References* 与 *Evidence and sources* 不计入 3 页限额，而 `README.md` 建议整份成稿（含参考文献与证据说明）尽量控制在 3 页内。 | Use the supplied A4, 10 pt, two-column LaTeX template; do not shrink font size or margins. Note: template p. 4 excludes *References* and *Evidence and sources* from the 3-page limit, while `README.md` advises keeping the entire paper within 3 pages. |

---

## 3. 报告各章节与图表要求 / Section-by-Section & Figure Requirements

报告正文共 5 个主章节，外加末尾的 *Evidence and sources* 与 *References*；模板内置 **Figure 1–4（必选/核心）**、**Table 1（必选）** 和 **Figure 5（可选第 3 张结果图）**。

### 3.1 Section 1: Introduction（引言） + Figure 1

- **背景与上下文**：用 1–2 个简短段落说明选择的机器人任务、研究的行为、数字孪生中需要重点考察的方面以及该任务为何重要。**不需要写机器人发展史或文献综述**，也不要罗列未采用过的尝试；仅在支持论点或简化表达时引用外部来源。
- **Figure 1（实验装置/场景图）**：插入一张带标注的实验台/场地照片或示意图，清晰标出**机器人、起始位姿/配置、目标点及关键环境细节**；图注需解释画面内容、读者应注意的特征，以及图中所有标签和符号的定义。
- **研究问题（Investigation Question）**：本节结尾必须提出一个**具体、可用实测数据回答的研究问题**，说明实验将比较或确立什么；适当时给出基于推理的预测结果。

> **English summary**: 1–2 short paragraphs on the task, behaviour, and Digital Twin relevance (no history/literature review). **Figure 1**: annotated setup photo or diagram showing robot, starting configuration, target, and labels. End with a **specific, measurable investigation question** (and optional justified prediction).

### 3.2 Section 2: Implementation（系统实现） + Figure 2

- **Figure 2（系统逻辑图）**：用**状态转移图（state-transition diagram）、流程图（flow-chart）或伪代码（pseudocode）**展示所评估的控制/任务逻辑，并用图注说明阅读所需的符号约定。
- **关键设计选择与参数**：说明影响系统行为的关键设计决策（如阈值、控制增益、时间间隔、恢复行为等），给出具体数值与单位，并解释选型依据。
- **区分原有功能与自主修改**：明确划分课程提供的模板代码与小组自行编写/修改的功能，仅解释逻辑，不要把完整代码清单贴进正文。

> **English summary**: Explain the tested system via **Figure 2** (state-transition diagram, flow-chart, or pseudocode) and justify consequential choices (thresholds, gains, timings with values/units). Clearly distinguish supplied functionality from team modifications without listing full source code.

### 3.3 Section 3: Experimental method（实验方法） + Table 1

1. **Variables and conditions（变量与条件） + Table 1**：
   - 描述回答研究问题所需的测试，交代机器人配置、测试环境与起始条件。
   - **Table 1** 必须替换为实际使用的三类变量：
     - **Independent（自变量）**：刻意改变或对比的量（写出测试值、范围或类别及单位）；
     - **Dependent（因变量）**：观测或测量的响应量（写出物理量、单位及测量来源/仪器）；
     - **Controlled（控制变量）**：为保证对比公平而保持一致的条件（写出具体设置与检查维持方式）。
   - 说明尽管有控制但仍可能波动的因素（如**电池电量/电压状态**），并交代如何监控或降低其影响；若包含观测性对比，需区分“比较条件”与“主动控制变量”。
2. **Metrics（性能指标）**：
   - 说明如何将原始测量转化为性能指标（例如：记录的位置是因变量，终点距目标的距离是指标）。
   - 给出每个指标的**计算公式、单位、物理含义及所有符号定义**；选择能直接回答问题并揭示权衡（trade-offs）的指标，并讨论所选指标的局限性。
   - 明确定义**成功、失败及中间状态**，并说明失败或不完整试次如何纳入分析。
3. **Procedure and analysis（实验步骤与分析流程）**：
   - 描述单次试验从初始化到停止的完整过程：哪些状态被重置、如何采集数据、采样周期（sampling interval）是多少。
   - 明确给出**每种条件的重复次数（repeats）和试次执行顺序（trial order）**，详细程度需足以让另一个小组复现实验。

> **English summary**: Detail robot configuration, environment, and **Table 1** (Independent, Dependent, Controlled variables with values/units/instruments), plus monitored nuisance variables (e.g., battery state). Define **Metrics** with equations, symbol definitions, units, limitations, and failure handling. Specify the exact trial **Procedure** (initialisation, resets, capture method, sampling interval, repeats, and trial order) for reproducibility.

### 3.4 Section 4: Results（实验结果） + Figure 3, Figure 4, [可选 Figure 5]

- **图表配置与定位**：
  - **Figure 3（主实验结果图）**：回答研究问题的核心实测/对比结果；
  - **Figure 4（补充证据图）**：展示第二个指标、重复性、或者模型预测与实测的另一种对比视角；
  - **Figure 5（可选第 3 图）**：仅在能增强或限定结论（如测试第二种工况、暴露解释的局限性）时保留；若无额外信息量，应删除并释放版面。
- **定量呈现与离散度**：
  - 必须用**具体数值和单位**量化差异、趋势或误差，严禁只用“better / accurate”等模糊定性词。
  - 报告重复试次的**平均值与离散度**，图注必须明确误差棒或阴影带代表什么（例如试次间标准差 SD，或均值的不确定度）；当重复次数较少时，建议在图中同时画出每次独立试次的观测点。
  - 若出现失败、不完整试次或意外观测，需如实报告发生次数及对应条件。
- **实测与仿真的严谨对比**：
  - 必须清晰区分**真实机器人测量**与**仿真预测**（两张图若使用不同试次或数据子集，须在实验方法中交代）。
  - 在结果部分客观指出**哪些地方吻合良好、哪些地方仍存在偏差**（包括负结果或尚无定论的结果）；**把机制解释、更广泛的启示和改进建议留到第 5 节讨论**。

> **English summary**: Present **Figure 3** (principal result), **Figure 4** (complementary evidence), and optional **Figure 5** (delete if redundant). Quantify differences with units; report spread across repeats (explicitly define error bars/bands; show individual points for small $n$); report failures/incomplete trials; clearly separate real-robot data from simulation predictions; reserve mechanistic explanations for Section 5.

### 3.5 Section 5: Discussion and conclusion（讨论与结论）

1. **Interpretation（结果解释）**：
   - 说明结果如何回应引言中的研究问题（若曾做过预测，明确指出数据支持、反驳还是未能定论）。
   - 将解释与具体观测及第 2 节的实现选择联系起来；严格区分**由测量直接支持的结论**与**尚属假设的物理机制**，考虑可改变结论的替代解释，避免仅因某机制能拟合单张图就将其当成绝对真理。
   - 说明指标所揭示的实际工程权衡（trade-offs，如提升某方面精度却牺牲速度或稳定性），结合任务尺度评估在当前测试条件下支持何种决策。
2. **Limitations and next experiment（局限性与下一步实验）**：
   - 指出**最影响结论可信度的关键局限**，具体说明其后果（增大波动、引入比较偏差或限制结论适用范围），并在相关处区分**测量不确定度（measurement uncertainty）**与**试次间波动（trial-to-trial variation）**（具体剖析胜过泛泛列举误差来源）。
   - 明确指出当前研究**尚未确立的内容**（例如仅测试单一地面或单一初始姿态，结论不可随意跨条件外推）。
   - 提出**一项具体的下一步实验**：说明打算改变或测量什么、对比什么对象、以及出现何种结果会使你们重新考虑当前结论，且需与前述局限性直接挂钩。
3. **Conclusion（结论）**：
   - 用**一个简短段落**直接回答研究问题：给出最有力的定量发现、说明对机器人行为或数字孪生模型的启示，并交代主要限制条件或尚待解决的问题（此处不引入新结果）。

> **English summary**: **Interpretation**: tie results back to the question and implementation, separate evidence from hypotheses, weigh alternative explanations and trade-offs. **Limitations and next experiment**: identify the primary limitation (separating measurement uncertainty from trial-to-trial variation), state unresolved scope, and propose one concrete follow-up test with falsification criteria. **Conclusion**: one short paragraph directly answering the question with key numbers, model implications, and scope limits.

### 3.6 Evidence and sources & References（证据清单与参考文献）

- **Evidence and sources**：列出随报告上传的真实文件名/路径，指明阅卷人应查看哪些文件以核对**原始数值测量数据**、**数据采集代码**、**机器人/仿真实现代码（含具体仿真配置）**以及**分析与绘图脚本**。
- **References**：采用紧凑一致的格式引用文中使用到的外部方法、公式来源或软件库，删除所有模板指导文字，严禁虚构参考文献或保留示例引用。

---

## 4. 评分标准与高分要点 / Marking Criteria & Distinction Keys

Assessment 2 总分由三个维度加权构成：**技术方法与实验设计（40%）+ 分析与批判性评估（40%）+ 科学表达（20%）**。以下提炼来自 `AssessmentSpecification_SEMTM0043.pdf`（p. 7）：

| 成绩档次 / Grade Band | 技术方法与实验（40%） / Technical Approach & Method | 分析与批判性评估（40%） / Analysis & Critical Evaluation | 科学表达（20%） / Scientific Communication |
|---|---|---|---|
| **Pass (50–59%)** | 问题聚焦可回答；在可比条件下收集两端数据；说明主要测量、控制与重复次数，交代模型校准及所用数据。 | 定量分析基本正确；得出有证据支持的结论并联系数字孪生用途；识别出一个主要不确定性或局限并说明影响。 | 三页结构清晰；图表可读、标签单位齐全、实测与仿真可区分；交待代码/资料来源。 |
| **Merit (60–69%)** | 实现恰当、设计理由充分；控制重要影响因素并合理重复；处理主要测量局限，**明确分离标定数据与评估数据（separates calibration data from evaluation data）**；步骤清晰可复现。 | 定量对比并分析波动/不确定度的意义；将一致与偏差关联到具体模型假设或子系统效应；考虑替代解释，**区分已证实推断与未检验假设**。 | 逻辑连贯，图注信息量充足，文字精炼，读者能分清观测、预测与解释。 |
| **Distinction (70–79%)** | 展现出色的实验判断力；控制变量、重复测试与**参考测量（reference measurements）**能有效规避误导性对比；**若涉及参数拟合，将标定与独立评估严格分离**；解释关键设计权衡。 | 构建有说服力的证据链，评估差异在当前任务下是否重要；批判性审视方法的信息量、不确定度与代表性；**清楚划分已确立结论与未解决问题**。 | 论证精准经济；图文协同高效传达对比、不确定性与局限；关键主张可追溯至细节与出处。 |
| **Exceptional Distinction (80–100%)** | **选择极具信息量的条件/测量来区分不同影响因素**；妥善处理关键测量或对比误差；在相关变量变化下检验鲁棒性；**模型拟合与预测性能评估证据严格分离**。 | 形成深刻洞察，为模型取舍或系统使用提供依据；**解决一项关键不确定性或揭示关键限制/权衡**；有力权衡竞争性解释，论证结论适用边界及所需补充证据。 | 表达极致清晰凝练；核心问题、决定性证据、推理链与局限一目了然；术语精确、溯源透明。 |

### 高分核心抓手（70–100% 必看）

1. **标定数据与评估数据严格分离**：不能拿同一组运行数据既拟合/标定数字孪生参数，又用来宣称“校准后预测误差降低”。
2. **引入外部参考测量（Reference Measurements）**：仅比较“机器人编码器里程计”与“仿真里程计”可能产生误导（尤其闭环下里程计会强制逼近目标，却掩盖打滑或轮径误差），必须有独立外部参考真值（如坐标纸地面实测）。
3. **区分两类误差来源**：明确区分**单次测量仪器不确定度**（如坐标纸读数精度 $\pm 1\text{ mm}$）与**多次重复试次之间的波动**（Trial-to-trial SD）。
4. **控制与分离影响因素**：实验条件需能解耦不同误差源，避免把多种物理原因混为一谈而无法下结论。

---

## 5. 个人贡献折算机制 / Equity Share Mechanism

Assessment 2 先按小组整体提交质量给出 **Group Mark**，再根据贯穿项目的 **4 次贡献度记录（Equity Share）** 对每位成员的个人分数进行比例调整（`AssessmentSpecification_SEMTM0043.pdf` p. 8）：

- **4 次记录来源**：
  1. **2 次 Peer-Assessment（同伴互评）**：分别在 **Week 6** 与 **Week 11** 保密提交对自身及队友的贡献百分比（未提交默认均分）。
  2. **2 次 Supervision Meetings（导师督导会议）**：分别在 **Weeks 7–8（Meeting 1）** 与 **Weeks 9–10（Meeting 2）** 现场核定各方一致同意的贡献百分比（缺席且队友未代为确认则记为 `0%`）。
- **统计规则**：所有成员份额之和必须为 `100%`（如 3 人理想均分为 `33.33%`）。综合 4 次记录的平均值与理想均分之差，用来上下微调 Group Mark 得到个人成绩。
- **答辩触发（Viva）**：若组内个人分数极差超过 **10%**、组员对份额无法达成一致、或有成员因此挂科，将举行 **Viva（口试答辩）** 决定最终成绩。

---

## 6. 针对 Path Reproduction 小组项目的落地映射 / Mapping to Our Project

结合本仓库的 [`Path_Reproduction_Experiment_Design.md`](Path_Reproduction_Experiment_Design.md)，现行实验方案天然对应高分要求：

| 评分高分要求 / Criterion | Path Reproduction 实验设计对应模块 | 报告中的写法要点 |
|---|---|---|
| **区分影响因素**（80+ 档） | **单动作实验（E1–E4）**：直行、原地旋转、$R=200\text{ mm}$ 圆弧、倒车 | 说明为何不能只测终点或只跑整条路径：单动作从标准起点 `(0,0,0°)` 出发，才能解耦左右电机不对称、有效轮距 $B_{eff}$、有效轮径 $r_{eff}$ 与打滑。 |
| **标定与独立评估严格分离**（60–100 档） | **标定集**：E0 预实验 + 单动作 E1–E3<br>**评估集**：E5 组合路径（S1–S7）与 E6 新参数路径 | 在 Method 与 Results 中明确声明：数字孪生参数仅由 E0/单动作标定并冻结，组合路径 E5/E6 属于**未参与调参的独立预测测试**。 |
| **外部参考测量防误导**（70+ 档） | **三套位姿来源**：`simulation`（仿真）、`real_odometry`（机载编码器）、`external_pose`（2×A3 坐标纸前后标记点换算） | 解释单独对比编码器与仿真只能反映“电机—编码器模型偏差”，只有对照 `external_pose` 才能分离出打滑与几何参数真实误差。 |
| **误差拆解与累积分析** | E5 段间 3000 ms 停顿快速打点 + 第 7.3 节局部坐标变换 | 对比**每段局部误差**与**段末累积误差**，定量证明前段旋转朝向误差如何被后续直行段放大。 |

---

## 7. 提交前核对清单 / Pre-submission Checklist

- [ ] **匿名性检查**：标题下仅保留所有成员 7 位 Bristol 学号（含前导零），正文、图表及 PDF 属性元数据中无任何个人姓名。
- [ ] **三件套齐全**：同时准备好 ① 报告 PDF、② StampC3 与 Processing 完整代码 + 分析绘图脚本、③ 原始数值测量数据（CSV 等）。
- [ ] **篇幅与排版合规**：使用官方 A4 / 10 pt 双栏模板，未缩小字号或边距，删除所有绿色/灰色占位指导文字与多余空框；确认无超页或排版溢出。
- [ ] **Introduction**：背景控制在 1–2 段；Figure 1 清晰标注起点、目标、坐标系与符号；结尾有明确可测量的研究问题。
- [ ] **Implementation**：Figure 2 逻辑图/伪代码清晰，说明关键参数（带数值与单位），明确区分课程原代码与小组修改部分。
- [ ] **Experimental method**：Table 1 三类变量完整且带数值/单位/仪器；交代电池等波动因素监控方式；指标公式符号清晰、定义成功/失败处理；实验步骤与重复次数足够他人复现。
- [ ] **Results**：图 3、图 4（及必要时的图 5）轴标与单位清楚，图注交代条件、试次数、误差棒含义；实测与仿真明确区分；用数值+单位表述结果，不把机制猜测写进 Results。
- [ ] **Discussion & conclusion**：区分已测支持结论与未验证假设，探讨替代解释与权衡；区分测量不确定度与试次波动，提出可证伪的下一步实验；Conclusion 一段话直接定量回答研究问题。
- [ ] **Evidence and sources & References**：准确列出所有原始数据、采集代码、仿真配置与绘图脚本文件名；所有外部来源均已规范引用。
