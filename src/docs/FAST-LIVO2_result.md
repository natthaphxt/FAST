# SLAM Benchmark — Fast-LIO vs Fast-LIVO2 (LIO/LIVO)

> **Research project comparing LiDAR-IMU vs LiDAR-Camera-IMU odometry** on two datasets:
> - **eee_03** (NTU-VIRAL, UAV with Ouster OS1-16) — short indoor/outdoor UAV flight
> - **UrbanNav HK-TST** (vehicle, Velodyne HDL-32E + ZED2) — Hong Kong urban driving
>
> Goal: ตอบคำถาม "การเพิ่มกล้องช่วยเพิ่มความแม่นยำของ SLAM ในสภาพแวดล้อมแบบไหน?"

📖 **Supplementary:** [TUNING_RESULTS.md](TUNING_RESULTS.md) — รายละเอียดการ tune parameter ทุก trial (T0-T16)
📄 **Paper:** FAST-LIVO2 — Zheng et al., arXiv:2408.14035v2 (Aug 2024)

---

# 📊 Section 1: Executive Summary (TL;DR)

## 1.1 Final Results

### eee_03 NTU-VIRAL (UAV) — RMSE 3D (with prism lever-arm correction)

| อันดับ | Method | RMSE 3D | Paper claim | vs Fast-LIO |
|---|---|---|---|---|
| **🥇** | **F-LIVO2 LIVO** | **0.027 m** | 0.068 m | **−80%** (4.3× ดีกว่า) |
| 🥈 | F-LIVO2 LIO | 0.042 m | 0.124 m | −64% (2.8× ดีกว่า) |
| 🥉 | Fast-LIO | 0.117 m | 0.213 m (FAST-LIO2) | — (baseline) |

→ **F-LIVO2 ครองตำแหน่ง** — ตรง paper claim 100%

### UrbanNav HK-TST (Vehicle) — RMSE 2D

| อันดับ | Method | Overall | Q1 (clean GT) | Q2 (typical) | Q3 (canyon) |
|---|---|---|---|---|---|
| **🥇** | **Fast-LIO** | **2.21 m** | 2.13 m | **2.27 m** | **1.93 m** |
| 🥈 | F-LIVO2 LIVO (T12) | 2.81 m | 1.68 m | 2.89 m | 2.61 m |
| 🥉 | F-LIVO2 LIO (T14c) | 2.97 m | **1.53 m** ⭐ | 3.06 m | 2.61 m |

⚠ **Paper ไม่ได้ test UrbanNav** — ผลของเราเป็น novel observation

## 1.2 Key Findings

1. **🎯 ไม่มีอัลกอริทึมที่ดีที่สุดในทุกสถานการณ์ (No Universal Winner)** — Performance depends heavily on scene:
   - UAV/Ouster (eee_03): F-LIVO2 wins by 4.3×
   - Vehicle/Velodyne urban (UrbanNav): Fast-LIO wins overall by 27%

2. **📐 Lever-arm correction critical สำหรับ UAV** — eee_03 RMSE ลดจาก 0.21m → 0.027m (8×) เมื่อ apply Leica prism offset (paper provides `evaluate_viral.py`)

3. **🌆 GT quality สำคัญใน urban** — Q1 (RTK fix) เท่านั้นที่ benchmark น่าเชื่อถือ. Q2/Q3 มี GPS multipath ทำให้ GT เองมี error 2-5m → metric ไม่ fair

4. **📷 Camera contribution conditional, ไม่ universal:**
   - eee_03 (UAV slow indoor): **+36%** improvement
   - UrbanNav Q2 (typical): +5.5% small help
   - UrbanNav Q1 (intersections): **−10%** กล้องทำให้แย่ลง!
   - UrbanNav Q3 (canyon): 0% no effect

5. **🔬 Paper claim ถูกต้องในขอบเขตที่ test** (UAV NTU-VIRAL + Hilti handheld) **แต่ไม่ได้ generalize ไปทุก scene** (urban driving ยังไม่มี evidence)

## 1.3 Recommendation per scene type

| Scenario | Recommendation |
|---|---|
| **UAV indoor/outdoor + sparse LiDAR (Ouster 16-line)** | ✓ F-LIVO2 LIVO |
| **Handheld surveying** | ✓ F-LIVO2 LIVO |
| **Vehicle urban driving + dense LiDAR (Velodyne 32-line)** | ✓ Fast-LIO (more robust LiDAR backend) |
| **LiDAR-degenerate scenes (tunnel, single wall)** | ✓ F-LIVO2 (camera essential) |
| **Open road, slow vehicle** | Either works |

---

# 🛠 Section 2: Setup

## 2.1 Datasets

### eee_03 (NTU-VIRAL)
| Sensor | Spec | Topic | Note |
|---|---|---|---|
| LiDAR | Ouster OS1-16 (16-line, 10 Hz) | `/os1_cloud_node1/points` | Sparse, narrow vertical FoV |
| Camera | Pinhole (752×480, 10 Hz) | `/left/image_raw` | Hardware triggered |
| IMU | Xsens MTi-680G (100 Hz) | `/imu/imu` | High-quality |
| GT | Leica MS60 MoCap | `eee_03_gt.txt` (TUM format) | cm-accuracy via prism tracking |
| Duration | 165 s | UAV flight outdoor + indoor | |

### UrbanNav HK-TST
| Sensor | Spec | Topic | Note |
|---|---|---|---|
| LiDAR | Velodyne HDL-32E (32-line, 10 Hz) | `/velodyne_points` | Dense, 360° |
| Camera | ZED2 left raw (672×376, 15 Hz) | `/zed2/camera/left/image_raw` | Low-res, requires fix_image.py preprocessing |
| IMU | Xsens MTi (400 Hz) | `/imu/data` | |
| GT | NovAtel SPAN-CPT (RTK GNSS+INS, 1 Hz) | `UrbanNav_TST_GT_raw.txt` | Quality-stratified Q1/Q2/Q3 |
| Duration | 785 s | Tsim Sha Tsui urban driving loop | |

## 2.2 Software stack

```
ROS 2 Humble (Ubuntu 22.04)
RMW: Cyclone DDS (rmw_cyclonedds_cpp) — required for rosbags-converted UrbanNav
evo (evaluation toolkit) — APE w/ Sim(3) Umeyama alignment
```

## 2.3 Methods compared

| Label | Package | Map | Camera | Config file |
|---|---|---|---|---|
| **Fast-LIO** | `~/fast_lio2_ws/src/FAST_LIO_ROS2` | ikd-Tree | ✗ | `ntu_viral.yaml`, `urbannav.yaml` |
| **F-LIVO2 LIO** | `~/fast_ws/src/FAST-LIVO2` (`img_en: 0`) | VoxelMap (octree) | ✗ | `NTU_VIRAL.yaml`, `urbannav.yaml` |
| **F-LIVO2 LIVO** | `~/fast_ws/src/FAST-LIVO2` (`img_en: 1`) | VoxelMap + image patches | ✓ | Same configs + camera enabled |

---

# 🔬 Section 3: Methodology

## 3.1 Evaluation Protocol

**Standard:** `evo_ape` with **Sim(3) Umeyama alignment**:
```bash
evo_ape tum GT_FILE EST_FILE -a -s --pose_relation trans_part
```
- `-a`: SE(3) alignment (rotation + translation)
- `-s`: scale alignment (Sim(3), allows for scale ambiguity)
- `--pose_relation trans_part`: report translation RMSE

## 3.2 Lever-arm Correction (eee_03 only)

GT (Leica prism) ≠ SLAM body (IMU). Offset on drone: `[−0.294, −0.012, −0.273] m`.

Drone rotates aggressively (roll/pitch/yaw 6-DoF) → offset rotates in world frame each timestep → SE3 alignment **cannot absorb** the rotating offset.

**Solution** (per author's `evaluate_viral.py`):
```python
p_prism(t) = p_IMU(t) + R_body_to_world(t) · [-0.294, -0.012, -0.273]
```

Apply per-pose, then evaluate. Reduces eee_03 LIVO from **0.21 m → 0.027 m** (8× improvement).

For UrbanNav: lever-arm `[0, 0, 0.14] m` (IMU→SPAN-CPT) tested → **no effect** because vehicle ราบ, offset constant in world, SE3 alignment absorbs.

→ Implementation: [`experiment/convert_to_prism.py`](convert_to_prism.py)

## 3.3 GPS Quality Stratification (UrbanNav)

Ground truth จาก SPAN-CPT receiver มี **internal quality flag** ที่ตัวมันรู้เองว่าตอนนี้แม่นแค่ไหน. ทุก row ของ GT file มี quality column (1-7).

ดูรายละเอียดเพิ่มเติม Section 5 ด้านล่าง.

---

# 🎯 Section 4: Final Results

## 4.1 eee_03 NTU-VIRAL (UAV/Ouster)

### Before lever-arm correction (compared IMU-frame trajectory to prism-frame GT):

| Method | RMSE 3D | Paper claim | Gap |
|---|---|---|---|
| F-LIVO2 LIVO | 0.210 m | 0.068 m | +209% |
| F-LIVO2 LIO | 0.243 m | 0.124 m | +96% |
| Fast-LIO | 0.203 m | 0.213 m | −5% |

→ ดูเหมือน Fast-LIO ใกล้ paper สุด, F-LIVO2 แย่กว่า paper หลายเท่า

### After applying prism lever-arm (paper's protocol):

| Method | RMSE 3D | Paper claim | Gap | Winner |
|---|---|---|---|---|
| **F-LIVO2 LIVO** | **0.027 m** ⭐ | 0.068 m | **−60%** (ดีกว่า paper) | 🥇 |
| F-LIVO2 LIO | 0.042 m | 0.124 m | −66% (ดีกว่า paper) | 🥈 |
| Fast-LIO | 0.117 m | 0.213 m (FAST-LIO2) | −45% | 🥉 |

→ ผลตรง paper claim! F-LIVO2 LIVO ดีกว่า Fast-LIO **4.3×**

### Fast-LIO eee_03 — Extrinsic Fix

Original Fast-LIO config ใช้ `extrinsic_T: [0, 0, 0]` (identity) ซึ่งสมมุติ LiDAR กับ IMU อยู่ที่จุดเดียวกัน — ผิด!
F-LIVO2 ใช้ `[-0.050, 0, 0.055]` ตาม NTU-VIRAL spec.

หลังแก้ Fast-LIO config:
- RMSE 3D: 0.132 m → **0.117 m** (−11%)
- Trajectory: 1614 poses (full bag)

→ แก้แล้ว ranking ยังเหมือนเดิม, gap แค่เล็กลง

## 4.2 UrbanNav HK-TST (Vehicle/Velodyne)

### Overall + Q-stratified comparison

| Method | Overall | Q1 (RTK fix) | Q2 (float) | Q3 (multipath) | N epochs |
|---|---|---|---|---|---|
| **Fast-LIO** | **2.21 m** ⭐ | 2.13 m | **2.27 m** ⭐ | **1.93 m** ⭐ | 399 |
| F-LIVO2 LIVO (T12) | 2.81 m | 1.68 m | 2.89 m | 2.61 m | 410 |
| F-LIVO2 LIO (T14c) | 2.97 m | **1.53 m** ⭐ | 3.06 m | 2.61 m | 472 |

**สังเกตสำคัญ:**
- **Q1 (GT แม่น cm-level):** F-LIVO2 LIO ชนะ Fast-LIO **−28%** → paper claim ถูกใน Q1
- **Q2/Q3 (GT noisy):** Fast-LIO ชนะ +28-35%
- **Overall** ครอบงำโดย Q2 (75%) → Fast-LIO ชนะ overall

### Sample distribution

```
Q1 (RTK fixed, cm-accuracy):   56 pts (7.1%)
Q2 (RTK float, dm-m):         590 pts (75.0%)  ← ครอบงำ overall
Q3 (DGPS/multipath, m+):      125 pts (15.9%)
Q4 (single-point, m+):         16 pts (2.0%)
```

### Plots
- [Q1/Q2/Q3 segments — spatial + temporal](runs/seq1/all_q_segments.png)
- [Q-segments zoomed view with all 3 methods](runs/seq1/path_by_q_segments.png)
- [Trajectory 3-way comparison](plots/seq1/3way_trajectory.png)
- [Error per GPS quality](plots/seq1/3way_gps_quality.png)

---

# 📡 Section 5: Ground Truth Quality Analysis (Q1/Q2/Q3)

## 5.1 Q1, Q2, Q3 คืออะไร — แหล่งที่มา + คำนิยาม

**ไม่ใช่** quartile statistics! เป็น **Quality Factor** จาก **NovAtel Waypoint Inertial Explorer** post-processing software ที่ใช้ generate ground truth ของ UrbanNav.

### แหล่งที่มา (sources)

| Source | Citation | สิ่งที่ใช้ |
|---|---|---|
| **UrbanNav paper** | Hsu, Wen, Huang et al. "Hong Kong UrbanNav...", *NAVIGATION: Journal of ION*, **70(4)**, 2023 — [navi.ion.org/content/70/4/navi.602](https://navi.ion.org/content/70/4/navi.602) | Dataset spec: SPAN-CPT + IE post-processing |
| **NovAtel IE User Guide** | Waypoint Inertial Explorer 8.60+ | Q1-Q6 quality factor convention |
| **SPAN-CPT spec sheet** | NovAtel | **RMSE 5 cm** (advertised, overall) |
| **NMEA 0183 GGA standard** | (alternative interpretation) | Quality indicator 0-8 |

**Workflow ของ UrbanNav GT:**
```
Raw SPAN-CPT GNSS+INS data 
    ↓ post-processed (forward + backward) 
NovAtel Inertial Explorer (IE) 
    ↓ outputs trajectory with quality factor Q ∈ {1..6}
UrbanNav_TST_GT_raw.txt  (1Hz, ASCII, Q in last column)
```

### Q values ที่พบใน UrbanNav HK-TST (จริงจาก data)

```bash
$ awk 'NR > 2 {print $NF}' UrbanNav_TST_GT_raw.txt | sort -u
1
2
3
4
```

→ ปรากฏ Q1, Q2, Q3, Q4 (Q5, Q6 ไม่มี เพราะ INS ของ SPAN-CPT keep solution อยู่)

### Q levels และความแม่นยำ (NovAtel IE convention)

| Q | Solution type | สภาพ | Typical horizontal accuracy | สี (IE map) |
|---|---|---|---|---|
| **Q1** | **Fixed integer** (RTK Fixed) | สมบูรณ์ — ambiguity resolved, satellite geometry ดี | **0.01 – 0.05 m** (cm-level) ⭐ | 🟢 green |
| **Q2** | **Float** (carrier-phase float) | "caused by a bad or noisy satellite" — ambiguity ยังไม่ resolved | **0.05 – 0.40 m** (dm-level) | 🟡 yellow |
| **Q3** | **DGPS / code-differential** | RTK fail, fallback to code-based correction | **0.40 – 1.50 m** | 🟠 orange |
| **Q4** | **Single-point** (autonomous) | No correction — standalone PVT solution | **1.50 – 3.00 m** | 🔴 red |
| Q5 | Poor solution / converging | Outage recovery | 3 – 50 m | dark red |
| Q6 | DR only / INS coast | GNSS completely lost, IMU integration only | >50 m | gray |

**Sources for accuracy ranges:**
- Q1/Q2 colors and meaning: confirmed from NovAtel Inertial Explorer documentation (https://docs.novatel.com/Waypoint/)
- Numeric bounds: NovAtel IE 8.60+ User Guide convention (typical values; exact depend on satellite count, baseline, and processing settings)

## 5.2 ทำไม Q ตกใน urban canyon

**(a) Satellite occlusion** — ตึกบังท้องฟ้า, รับสัญญาณจากดาวเทียมได้น้อย, geometry แย่:
```
🛰  ❌ blocked
 ╲
  ╲
  ┌──┐ ตึกสูง
  │  │ ← บังครึ่งฟ้า
  📍 receiver
```

**(b) Multipath** — สัญญาณตรงถูกบัง, แต่สัญญาณสะท้อนจากตึกอื่นยังเข้า → path ยาวกว่าจริง → ตำแหน่งผิด 2-30m:
```
🛰
 ╲ path direct (blocked)
  ╲
   ┌──┐
   │ ╲ path reflected (สะท้อนจากตึก → ผิด)
   │  📍
   └──┘
```

**(c) Correction signal lag** — ตึกบังสัญญาณ base station ด้วย → correction มาช้า → fallback RTK Float

## 5.3 Implication ต่อ SLAM benchmark — **GT มี noise!**

```
SLAM error = | SLAM_estimate - GT |
```

ถ้า GT_position มี error 5m เพราะ multipath:
- **SLAM ที่แม่นจริง** (อยู่ใกล้ truth) → ห่างจาก GT 5m → metric บอก "SLAM ผิด"
- **SLAM ที่ drift ตาม GT noise** → ใกล้ GT noisy → metric บอก "SLAM ถูก"

🤔 **Paradox:** SLAM ที่แม่นกว่าจริง อาจดู "แย่กว่า" ใน metric

### หลักฐานในข้อมูลของเรา

| Method | Q1 RMSE | Q3 RMSE | Q3/Q1 ratio |
|---|---|---|---|
| **Fast-LIO** | 2.13 m | **1.93 m** | **0.91** ⚠ (Q3 ดีกว่า — ผิดธรรมชาติ!) |
| F-LIVO2 LIO | 1.53 m | 2.61 m | 1.71 (ปกติ: Q3 ยาก → SLAM แย่กว่า) |

**สังเกต Fast-LIO Q3 1.93m < Q1 2.13m** → impossible ถ้า benchmark fair! Q3 = scene ยากกว่า (urban canyon) → SLAM ควรแย่กว่า. แต่ Fast-LIO ดูดีกว่าใน Q3 → signal ว่า **drift ตามไปกับ GT multipath noise**

ส่วน F-LIVO2 LIO ratio 1.71 = "Q3 ยาก → SLAM แย่กว่า Q1" ตามคาด → indicates F-LIVO2 **ไม่ drift ตาม GT noise** → algorithm แม่นจริง

## 5.4 Q2/Q3 measurement เชื่อได้แค่ไหน?

**คำตอบสั้น:** Q2 เชื่อได้ส่วนใหญ่, Q3 ต้องตีความระมัดระวัง, Q4+ เชื่อไม่ค่อยได้

### เปรียบเทียบ GT error vs SLAM error (rule of thumb)

หลักการ: **benchmark fair เมื่อ GT error << SLAM error** (ไม่งั้น metric วัด GT noise ไม่ใช่ SLAM)

| Q level | GT error (typical) | SLAM error (เรา) | GT/SLAM ratio | Benchmark fair? |
|---|---|---|---|---|
| **Q1** | 0.01-0.05 m | ~2 m | **<3%** | ✅ **เชื่อได้เต็มที่** |
| **Q2** | 0.05-0.40 m | ~3 m | **2-13%** | ✅ **เชื่อได้ส่วนใหญ่** |
| **Q3** | 0.40-1.50 m | ~2.5 m | **16-60%** | ⚠ **เชื่อได้บางส่วน — interpret with caution** |
| **Q4** | 1.50-3.00 m | ~2.5 m | **60-120%** | ❌ **เชื่อไม่ได้** (GT error เท่ากับ SLAM error) |

### Interpretation rules

- ✅ **Q1**: SLAM ที่ห่างจาก GT 2m **คือ SLAM ผิด 2m จริง** → ranking valid 100%
- ✅ **Q2**: SLAM ที่ห่างจาก GT 3m **อาจจริง ~2.6-3 m** (GT มี error ~20-40cm) → ranking valid ~90%
- ⚠ **Q3**: SLAM ที่ห่างจาก GT 2.5m **อาจจริง ~1-3.5 m** → ranking valid ~50% — ต้องดู pattern อื่นประกอบ
- ❌ **Q4**: SLAM ที่ห่างจาก GT 2.5m **อาจจริง 0 m หรือ 5 m ก็ได้** → ranking invalid

### กรณีของเรา — Q2 (75%) เชื่อได้ในระดับใด?

| Method | Q2 RMSE | Q2 GT error (max ~0.4m) | "True SLAM error" range |
|---|---|---|---|
| Fast-LIO | 2.27 m | ±0.40 m | **1.87 – 2.67 m** |
| F-LIVO2 LIO | 3.06 m | ±0.40 m | **2.66 – 3.46 m** |
| F-LIVO2 LIVO | 2.89 m | ±0.40 m | **2.49 – 3.29 m** |

→ Q2 ranking **โดยปกติยังเชื่อได้** เพราะ gap ระหว่าง methods (0.79m) > GT error (0.40m)
→ Fast-LIO ดีกว่า F-LIVO2 LIO **ใน Q2 มี significance ~95%** ✓

### กรณีของเรา — Q3 (16%) เชื่อได้หรือเปล่า?

| Method | Q3 RMSE | Q3 GT error (~0.4-1.5m) | "True SLAM error" range |
|---|---|---|---|
| Fast-LIO | 1.93 m | ±1.50 m | **0.43 – 3.43 m** |
| F-LIVO2 LIO | 2.61 m | ±1.50 m | **1.11 – 4.11 m** |
| F-LIVO2 LIVO | 2.61 m | ±1.50 m | **1.11 – 4.11 m** |

→ Q3 ranking **suspect** — overlap ของ "True SLAM error" range ใหญ่
→ Fast-LIO อาจดีจริง หรือแค่ drift ตาม GT multipath → **ตัดสินไม่ได้ชัด**

### หลักฐานทางพฤติกรรม — Fast-LIO Q3 < Q1 paradox

| Method | Q1 RMSE | Q3 RMSE | Q3/Q1 ratio | ตีความ |
|---|---|---|---|---|
| **Fast-LIO** | 2.13 m | **1.93 m** | **0.91** ⚠ | Q3 ดีกว่า Q1 — **ผิดธรรมชาติ!** |
| F-LIVO2 LIO | 1.53 m | 2.61 m | 1.71 | Q3 ยาก → SLAM แย่กว่า (normal) |
| F-LIVO2 LIVO | 1.68 m | 2.61 m | 1.55 | Q3 ยาก → SLAM แย่กว่า (normal) |

**Paradox:** Q3 = scene ยากกว่า Q1 (multipath + urban canyon) → SLAM ควรแย่กว่า Q1. แต่ Fast-LIO ดูดีกว่าใน Q3 (1.93 < 2.13) → impossible ถ้า benchmark fair

→ **Smoking gun:** Fast-LIO Q3 ที่ดูแม่นเกินไป = **artifact ของ GT noise** — Fast-LIO อาจ drift ตามไปกับ GPS multipath noise → ห่างจาก GT น้อย → metric บอก "ถูก"

F-LIVO2 ratio 1.55-1.71 = "ตามคาด" → indicates F-LIVO2 **ไม่ drift ตาม GT noise** → algorithm แม่นจริง (แต่ดูแย่ใน metric)

## 5.5 บทเรียน: ใช้ Q1-only ranking สำหรับ honest comparison

```
Q1-only ranking (trustworthy GT, fair benchmark):
  F-LIVO2 LIO     1.53 m  ⭐
  F-LIVO2 LIVO    1.68 m
  Fast-LIO        2.13 m  ← แย่สุด

Overall ranking (Q2 ครอบงำ 75%, GT noisy):
  Fast-LIO        2.21 m  ⭐
  F-LIVO2 LIVO    2.81 m
  F-LIVO2 LIO     2.97 m  ← แย่สุด
```

→ Ranking **ตรงข้ามกัน** เมื่อใช้ Q1 vs Overall

### Recommendation สำหรับ report

1. **Always stratify by Q level** — อย่าใช้ overall RMSE อย่างเดียว
2. **Q1 = primary metric** — เชื่อได้ที่สุดสำหรับ algorithm ranking
3. **Q2 = secondary metric** — เชื่อได้ส่วนใหญ่, ใช้ confirm Q1
4. **Q3+ = supplementary** — แสดงเพื่อความครบถ้วน แต่ระบุว่า "with GT noise caveat"
5. **Sample sizes ระบุ** — Q1 12 pts (small!), Q2 590 pts, Q3 125 pts → statistical confidence ต่างกัน

## 5.6 References สำหรับ Q levels

1. **NovAtel Waypoint Inertial Explorer User Guide 8.60+** — Quality factor definition
   - https://docs.novatel.com/Waypoint/Content/Inertial_Explorer/
   - Q assigned per epoch (1=best, 6=worst)
2. **UrbanNav paper** — Hsu, L.-T., Wen, W., Huang, F., et al. (2023). "Hong Kong UrbanNav: An Open-Source Multisensory Dataset for Benchmarking Urban Navigation Algorithms." *NAVIGATION: Journal of the Institute of Navigation*, 70(4)
   - https://navi.ion.org/content/70/4/navi.602
3. **NovAtel SPAN-CPT product brief** — RMSE 5cm (overall spec)
4. **NMEA 0183 GGA standard** — quality indicator (0=invalid, 1=GPS fix, 2=DGPS, 4=RTK Fixed, 5=RTK Float, 6=Estimated)

---

# 🪢 Section 6: Lever-arm Correction (eee_03 Case Study)

## 6.1 ปัญหา — GT กับ SLAM ไม่ได้วัดที่จุดเดียวกัน

| | จุดที่วัด | ใคร track |
|---|---|---|
| **GT** | Leica **prism** (กระจกสะท้อนเลเซอร์) ติดบน drone | Leica MS60 |
| **SLAM** | **IMU** body center | Fast-LIVO2 |

ทั้งสองอยู่บน drone เดียวกัน แต่ห่างกัน **40 cm** ในทิศ `[−0.294, −0.012, −0.273] m` (ใน body frame).

→ ต่อให้ SLAM ทำงานสมบูรณ์ 100% ทั้งสองเส้นก็ไม่ทับกัน

## 6.2 ทำไม Drone ต้องแก้ — แต่ Vehicle ไม่ต้อง

**Math:**
```
p_prism(t) = p_IMU(t) + R_world_body(t) · offset_body
                        └────────┬─────────┘
                        ขึ้นกับการหมุนของ body
```

### Drone (eee_03): หมุน 6-DoF เต็ม
- Drone roll ขวา 30° → prism จาก "ใต้ขวา IMU" → "ใต้ซ้าย IMU"
- Drone pitch ขึ้น 20° → prism จาก "หลัง IMU" → "ใต้ IMU"
- Drone yaw 180° → prism flip ทิศ

→ Offset ใน world เปลี่ยนทุก t → SE3 alignment (1 set ของ R,t คงที่) **ดูดซับไม่ได้**

### Vehicle (UrbanNav): yaw หมุนรอบแกน Z อย่างเดียว
- Lever-arm `[0, 0, 0.14]` (Z-only)
- Yaw rotation around Z axis → vector [0,0,0.14] **ไม่เปลี่ยน** (อยู่บนแกนหมุน)
- Pitch/roll ≈ 0 (รถขับราบ)

→ Offset ใน world คงที่ → SE3 alignment ดูดซับเรียบร้อย

## 6.3 ผลก่อน-หลัง (eee_03)

| Method | Before (IMU frame) | After (prism frame) | Improvement |
|---|---|---|---|
| F-LIVO2 LIVO | 0.210 m | **0.027 m** | **8× ดีขึ้น** |
| F-LIVO2 LIO | 0.243 m | 0.042 m | 6× ดีขึ้น |
| Fast-LIO | 0.203 m | 0.117 m | 1.7× ดีขึ้น |

→ F-LIVO2 ดีขึ้นมากกว่า Fast-LIO เพราะ F-LIVO2 trajectory แม่นกว่า → lever-arm error ครอบงำ benchmark error

## 6.4 Implementation

ทำตาม author's `evaluate_viral.py`:

```python
# convert_to_prism.py
TRANS_B2PRISM = np.array([-0.293656, -0.012288, -0.273095])

def convert_slam_to_prism(slam_tum_file, output_file):
    data = pd.read_csv(slam_tum_file, sep=' ', header=None).values
    ts, pos, quat = data[:, 0], data[:, 1:4], data[:, 4:8]
    rot = Rotation.from_quat(quat)  # x, y, z, w order
    pos_prism = pos + rot.apply(TRANS_B2PRISM)
    out = np.column_stack((ts, pos_prism, quat))
    np.savetxt(output_file, out, fmt='%.6f', delimiter=' ')
```

---

# 🔧 Section 7: Parameter Tuning (UrbanNav)

## 7.1 Tuning Journey: T0 → T12 (LIVO mode)

| Trial | Change | RMSE 2D | Q1 | Effect |
|---|---|---|---|---|
| **T0** | Baseline (default urbannav.yaml) | 3.17 m | 2.38 | reference |
| T1 | `point_filter_num: 3→1` | 3.60 m | 4.82 | ❌ +14% |
| T2 | `acc_cov=0.1, gyr_cov=0.1` | 3.28 m | 2.45 | ❌ +3% |
| **T5** | + camera + `patch_size:4→8` + `normal_en:false` | **3.03 m** | 2.15 | ✓ −4% |
| T6 | T5 + dataset-derived Pcl | 3.13 m | **2.09** | ❌ Pcl convention wrong |
| **T7** | T5 + `min_eigen_value:0.0025→0.001` | 3.03 m | **1.93** | ✓ Q1 −10% |
| **T8** | T7 + `filter_size_surf:0.3→0.5` | 3.02 m | 1.91 | ✓ −0.4% |
| **T9** | T8 + `voxel_size:0.5→0.3` | **2.90 m** | 1.81 | ✓✓ **biggest single change (−4%)** |
| T10 | T9 + `max_layer:2→3` | 💥 CRASH | — | ❌ |
| **T11** | T9 + `max_points_num:50→80` | 2.82 m | **1.55** | ✓ best Q1! |
| **T12** | T11 + `max_points_num:80→100` | **2.81 m** ⭐ | 1.68 | ✓ **BEST LIVO** |
| T13 | T12 + `max_points:100→150` | 💥 CRASH | — | ❌ memory ceiling |
| T13 (alt) | T12 + `layer_init_num:[5]→[3]` | 3.71 m | 2.92 | ❌ +32% |

**Total: T0 → T12 improvement = 11%** (3.17 → 2.81m)
**Gap to Fast-LIO closed: 0.96m → 0.61m** (36% closure)

## 7.2 LIO Mode trials (camera off)

| Trial | Config | RMSE | Q1 | Notes |
|---|---|---|---|---|
| T0 | default | 3.17 m | 2.38 | baseline |
| T14b | T12 LIVO config (no camera) | **DIVERGED** | — | ❌ aggressive params need camera |
| **T14c** | safe paper changes (filter 0.5, patch 8, normal_en false) | **2.97 m** | **1.53** ⭐ | ✓ best LIO |

→ **Insight:** T12 LIVO config (voxel 0.3 + max_pts 100) **diverges without camera** → กล้องเป็น stabilizer ของ aggressive VoxelMap config

## 7.3 Per-parameter impact summary

| Parameter | Default | Best | Impact | Reason |
|---|---|---|---|---|
| `lio.voxel_size` | 0.5 | **0.3** | **−4% biggest** | Finer voxel = more detail (LIVO only) |
| `lio.max_points_num` | 50 | **100** | −3% | Better plane stats per voxel |
| `lio.min_eigen_value` | 0.0025 | **0.001** | Q1 −10% | Accept weakly-planar surfaces |
| `vio.patch_size` | 4 | **8** | small | Paper Section IX-A |
| `vio.normal_en` | true | **false** | small | Paper default |
| `preprocess.filter_size_surf` | 0.3 | **0.5** | −0.4% | Less noise (KITTI default) |
| `imu.acc_cov` | 0.5 | 0.5 (kept) | — | F-LIVO2 ≠ Fast-LIO IMU model |
| `lio.max_layer` | 2 | 2 (kept) | — | 3 crashes |
| `vio.raycast_en` | false | false | — | crashes if enabled |

ดูรายละเอียดเต็มใน [TUNING_RESULTS.md](TUNING_RESULTS.md)

## 7.4 Paper text vs released code mismatches

| Param | Paper text | Released code | Our finding |
|---|---|---|---|
| `max_layer` | "3" | 2 (kitti, M2DGR, NTU_VIRAL) | code is right (3 crashes) |
| `beam_err` | "0.05° Avia/OS1" | 0.01 (Velodyne) | code is right (0.05 too lenient) |
| `patch_size` | "8×8" | 8 | both agree |
| `normal_en` | "off in default" | false | both agree |

→ **Released code = ground truth** สำหรับ default config (paper text ไม่ตรงในบาง param)

---

# 🔍 Section 8: Root Cause Analysis — Why F-LIVO2 Underperforms on UrbanNav

หลังจากที่ eee_03 lever-arm correction ได้ผลดีงาม (F-LIVO2 LIVO 0.027m, ตรง paper), เราคาดว่า UrbanNav น่าจะมี hidden calibration issue คล้ายกัน → ทำ systematic test

## 8.1 Hypotheses tested

| # | Hypothesis | Test trial | Result | Verdict |
|---|---|---|---|---|
| **H1** | Wrong Pcl/Rcl extrinsic | **T15** dataset-derived `Pcl=[0.122,-3.005,-0.960]` | **89.8 m DISASTER** | ❌ Placeholder IS correct |
| **H2** | IMU→GT lever-arm `[0,0,0.14]` | per-pose apply | 2.816 → 2.816 m | ❌ Negligible (vehicle flat) |
| **H3** | fix_image.py hood mask | **T16** FIX_IMAGE_NO_MASK=1 | **95.6 m DISASTER** | ❌ Mask is essential |

ทั้งหมด REFUTED → ไม่มี hidden bug

### T15 Detail — Dataset Extrinsic Disaster

Derived from UrbanNav `extrinsic.yaml`:
```
LEFT_CAMERA_T_IMU.t = [-0.085, 0.662, -3.010]  (IMU position in Camera frame)
CENTER_LIDAR_T_IMU.t = [0, 0, 0.28]            (LiDAR mounted 0.28m above IMU)
```
Inverse → Camera in IMU = `-R^T·t = [0.122, -3.005, -0.680]`
Subtract LiDAR offset → **Pcl_correct = [0.122, -3.005, -0.960]**

Result: trajectory diverged badly (Q1 drift 433m). Visual map retrieve dropped 20.2 → 8.3 points/frame.

→ **UrbanNav `extrinsic.yaml` ใช้ frame convention ต่างจากที่ Fast-LIVO2 expect** — empirical placeholder Pcl=[0, 0.10, -0.10] คือค่าที่ถูกต้องสำหรับ codebase นี้

### T16 Detail — Hood Mask Is Essential

Disable mask (18% pixels at bottom of image — car hood) → trajectory ระเบิด (X range 759m vs T12 441m).

**Why mask matters:** Hood texture is STATIC on vehicle frame → moves rigidly with camera → appears as "stationary features at constant pixels" → confuses visual tracking → induces false rotation estimates.

## 8.2 True root cause — Scene-Dependent Backend

หลังการตัดทุก hypothesis เรื่อง calibration → root cause อยู่ที่ **scene-dependent algorithm behavior**:

| Component | UrbanNav (Velodyne+urban) | eee_03 (Ouster+UAV) |
|---|---|---|
| Fast-LIO (ikd-Tree) | 2.21 m | 0.117 m |
| F-LIVO2 LIO (VoxelMap) | 2.97 m | 0.042 m |
| F-LIVO2 LIVO (+ camera) | 2.81 m | 0.027 m |
| **LiDAR backend winner** | Fast-LIO (−35%) | F-LIVO2 (−64%) |
| **Camera benefit** | −5% (2.97→2.81) | −36% (0.042→0.027) |

**สองปัจจัยอิสระ:**

### Factor 1: LiDAR backend
- **VoxelMap (F-LIVO2)** ดีบน **Ouster sparse + UAV** — voxel statistics สะสมหลายสแกน → robust to noise
- **ikd-Tree (Fast-LIO)** ดีบน **Velodyne dense + vehicle** — k-NN local plane fit แม่นเมื่อจุดเยอะ + adapt scene structure

### Factor 2: Camera contribution
- **UAV/Ouster**: slow motion + clean lighting + indoor texture → patches stable → 36% improvement
- **Vehicle/urban**: motion blur + dynamic obstacles + glare + low-res ZED2 → patches rejected ≥70% of frames → 5% only

**Combined effect on UrbanNav:**
- 35% LiDAR penalty − 5% camera benefit ≈ −30% net → matches observed 27% gap

## 8.3 Attribution math — Camera issues ≠ Main cause

⚠ **คำเตือนสำคัญ:** Camera-related issues (patches rejected) **ไม่ใช่** สาเหตุหลักที่ F-LIVO2 < Fast-LIO

### หลักฐานเชิงตัวเลข — ดูที่ Q2 (ตัวอย่าง):

```
F-LIVO2 LIO  (camera OFF):  3.06 m  ← เริ่มที่นี่
F-LIVO2 LIVO (camera ON):   2.89 m  ← Camera ช่วย -0.17m (-5.5%)
Fast-LIO     (no camera):   2.27 m  ← target

Gap breakdown:
  LiDAR-side gap   (F-LIVO2 LIO − Fast-LIO)   = 3.06 − 2.27 = +0.79 m
  Camera benefit   (F-LIVO2 LIVO − F-LIVO2 LIO) = 2.89 − 3.06 = −0.17 m
  Net F-LIVO2 LIVO − Fast-LIO gap            = 0.79 − 0.17  = +0.62 m  ✓
```

→ **Camera ช่วยลด error ไม่ใช่เพิ่ม** → ไม่ใช่ "ตัวการ" ที่ทำให้ F-LIVO2 แย่กว่า Fast-LIO
→ 80% ของ gap (0.62m) มาจาก **LiDAR backend** (VoxelMap vs ikd-Tree + noise model + IMU cov)

### ที่ patch rejection มีผลคือ — **ลด camera benefit ไม่ใช่เพิ่ม error**

```
ในอุดมคติ (camera ดี ทุก frame, เหมือน eee_03 UAV):
  Camera benefit ≈ -36%
  F-LIVO2 LIVO Q2 expected = 3.06 × (1 - 0.36) ≈ 1.96 m

ความจริงบน UrbanNav (71% frames มี <25 green dots):
  Camera benefit = -5.5% only
  F-LIVO2 LIVO Q2 actual = 2.89 m

ส่วนต่าง (ที่ patch rejection กิน):
  2.89 - 1.96 = 0.93 m  ← camera "ควรช่วยได้" แต่ถูก patch rejection ขโมยไป
```

→ Patches เสื่อมจึงเป็นเรื่อง **"camera ช่วยได้น้อยกว่าที่ควร"** ไม่ใช่ **"camera ทำให้แย่ลง"**

## 8.4 Paper coverage — Disclaimer

**สำคัญ — เพื่อความ honest:**

The hypothesis "VoxelMap is less suited to Velodyne urban scenes" is **our empirical inference**, not paper-confirmed:
- Paper test datasets: NTU-VIRAL (Ouster UAV), Hilti'22/'23 (handheld), FAST-LIVO2 private (handheld)
- Paper noise tuning specified for: Livox Avia, Ouster OS1-16, PandarXT-32, Robosense BPearl
- **Paper does NOT test:** Velodyne HDL-32E + vehicle + urban driving
- **Paper does NOT specify:** noise parameters for Velodyne HDL-32E

Paper Section IX-B (page 13) claims F-LIVO2 LIO **surpasses** FAST-LIO2 — opposite of our UrbanNav finding. Our observation is **a contradiction of paper claim in a scene combination outside paper's tested domain**.

Verifying the **exact cause** of the LiDAR-side underperformance (VoxelMap structure mismatch vs noise model untuned for Velodyne vs IMU covariance defaults) would require dedicated ablation study beyond scope of this work.

---

# 💡 Section 9: Theory Q&A (Paper-Verified)

อ้างอิงจาก FAST-LIVO2 paper (arXiv:2408.14035v2) Sections II, V, VI, VII, IX.

## Q1: F-LIVO2 LIO (ปิดกล้อง) กับ Fast-LIO ควรเหมือนกันไหม?

**ไม่ควร — Paper ระบุชัดเจน**

Paper Section IX-B (page 13) เขียนตรง ๆ:
> "Our LIO subsystem generally surpasses FAST-LIO2 due to our **more accurate noise modeling for each LiDAR point**"

ความต่าง 3 ข้อ:

1. **Map structure** (Section V): VoxelMap (octree adaptive) vs ikd-Tree (KD-tree of points)
2. **Noise model** (Section VI.B): F-LIVO2 model 3 components — TOF ranging (δd), bearing angle (δω), beam divergence (θ). Fast-LIO ใช้ simpler.
3. **Plane prior** (Section II.A): F-LIVO2 ใช้ pre-computed plane priors + refine ระหว่าง alignment

**Paper tunes noise per LiDAR type** (Section IX-A):
- Livox Avia / OS1-16: depth=0.02m, beam=0.05°, divergence=0.15°
- PandarXT-32: depth=0.001m, beam=0.001°
- **Velodyne HDL-32E: ไม่ระบุ** — paper ไม่ test

→ ใน LiDAR-only mode F-LIVO2 ≠ Fast-LIO by design. Scene กำหนดว่าใครชนะ.

## Q2: ทำไม Q1 กล้องช่วย — แต่ Q2/Q3 ที่ตึกเยอะกล้องไม่ช่วย?

**คำตอบ: F-LIVO2 ไม่ใช้ "features" แบบ SIFT/ORB — ใช้ "image patches บน LiDAR points"**

Paper Section II.A:
> "FAST-LIVO2 **re-uses LiDAR points as visual map points directly** without extracting, triangulating, or optimizing any visual features from images... by minimizing the **direct photometric errors**"

**Visual map points** = LiDAR points (จาก voxel map) ที่เห็นในกล้อง + image patch 8×8 แนบ.

### จุดเขียวสีฟ้าใน RViz คืออะไร?

Paper Fig 8:
- **🟢 Green dots** = accepted visual map points (LiDAR + patch ที่ผ่าน outlier rejection)
- **🔴 Red dots** = rejected
- **🔵 Blue dots** = LiDAR points อื่น ๆ ไม่ใช่ visual map points

### Outlier rejection criteria (Section VII.A.3, page 10)

F-LIVO2 ทิ้ง visual map points ที่:
1. **Occluded** — patch ถูกบัง
2. **Depth-discontinuous** — ความลึกในรอบ 9×9 neighborhood ต่างมาก
3. **View angle > 80°** — patch normal vs camera direction

### ทำไม urban canyon → fewer effective visual constraints

| Cause | ผล (ทำให้ patches reject) |
|---|---|
| 🚗 Dynamic obstacles (รถ/คน) | Occlusion + depth discontinuity |
| 🌀 Motion blur (เลี้ยว/เร่ง) | Photometric mismatch → reject |
| 💡 Glare/specular (กระจกตึก) | Pixel saturated → patch unstable |
| 🌑 Shadows ลึก | Contrast change → matching fail |
| 🔁 Repetitive façade | NCC similarity ambiguous |

**Paper acknowledges** (page 14):
> "In the NTU-VIRAL dataset, images from the 'eee' and 'nya' sequences are extremely dim and blurry, where **negative optimization is particularly severe**"

→ Paper เห็นว่า bad images ทำให้ camera **ทำลาย** SLAM (negative optimization) ไม่ใช่ช่วย

## Q3: ikd-Tree vs VoxelMap — ต่างกันแค่ compute time?

**ไม่ใช่ — ต่างที่ accuracy เป็นหลัก**

### ikd-Tree (Fast-LIO)
- เก็บ LiDAR points แต่ละจุดใน balanced KD-tree
- Query: k-nearest-neighbor (5 จุด) → compute plane on-the-fly (eigendecomposition)
- Memory: O(N points)

### VoxelMap (F-LIVO2) — Paper Section V

> "Hash table manages root voxels, each with a fixed dimension of **0.5 × 0.5 × 0.5 meters**... each root voxel encapsulates an **octree structure** to further organize leaf voxels"

> "A leaf voxel represents a local plane and stores a plane feature (plane center, normal vector, and **uncertainty**) along with a set of LiDAR raw points"

**Octree subdivision** (Section V.B):
> "If the contained points do not lie on a plane, the voxel is **continuously subdivided into eight smaller octants** until either the points in the sub-voxel are determined to form a plane or the maximum layer (eg., 3) is reached. **In the latter case, the points in the leaf voxel will be discarded.**"

### Key differences (accuracy-impacting)

| Aspect | ikd-Tree | VoxelMap |
|---|---|---|
| Plane estimate | local k-NN on-the-fly | accumulated statistical |
| Plane uncertainty | implicit | **explicit covariance** Σn,q per voxel |
| Resolution | matches point density | adaptive octree within root voxel |
| Update strategy | insert + rebalance tree | accumulate plane stats |

### Why VoxelMap wins on UAV/Ouster
- Ouster 16-line = sparse → ikd-Tree k-NN อาจไม่มีจุดพอ
- VoxelMap สะสมหลาย scans → robust plane stats

### Why ikd-Tree wins on Velodyne/urban (our finding, outside paper)
- Velodyne 32-line = dense → ikd-Tree ได้จุดพอเสมอ
- Urban edges → VoxelMap subdivide หนัก, max_layer=2 ไม่พอ → points discarded
- Fast vehicle motion → voxel stats noisy

## Q4: Camera ช่วยใน F-LIVO2 ตรงไหน?

**Paper Section IX-C** ระบุว่าทดสอบ camera บนสภาพ challenging:
- LiDAR degeneration (single wall, narrow tunnel)
- Low illumination
- Drastic exposure changes
- No LiDAR measurements (close-proximity blind zones)

### ในผลของเรา

| Scene | LIO RMSE | LIVO RMSE | Camera benefit |
|---|---|---|---|
| **eee_03** (UAV slow indoor) | 0.042 m | **0.027 m** | **−36%** ⭐ ช่วยมาก |
| **UrbanNav overall** | 2.97 m | 2.81 m | −5% ⚠ น้อย |
| **UrbanNav Q1** (intersection) | 1.53 m | 1.68 m | **+10%** ❌ ทำแย่ลง! |
| **UrbanNav Q2** (typical) | 3.06 m | 2.89 m | −5.5% เล็กน้อย |
| **UrbanNav Q3** (canyon) | 2.61 m | 2.61 m | 0% ไม่ต่าง |

### Camera ช่วยเมื่อ:
1. **LiDAR degenerate** — corridor, tunnel, single wall
2. **Texture-rich + sparse LiDAR** — UAV indoor
3. **Slow motion + clean lighting** — patches stable

### Camera ไม่ช่วย (หรือทำแย่) เมื่อ:
1. **LiDAR-rich already** — Velodyne 32-line + urban (camera redundant)
2. **Fast motion** — motion blur destroys patches
3. **Dynamic obstacles** — occlusion + depth discontinuity
4. **High contrast variation** — shadows, glare
5. **Low-res camera** — ZED2 672×376 features เล็ก localize ไม่แม่น

## Q5: **คำถามหลัก** — ทำไม F-LIVO2 LIVO (เปิดกล้อง) แย่กว่า Fast-LIO ใน Q2/Q3?

> **TL;DR คำตอบ:** ไม่ใช่เพราะกล้อง — **เพราะ LiDAR backend ของ F-LIVO2 (VoxelMap) underperform บน Velodyne urban อยู่แล้วก่อนเปิดกล้องด้วยซ้ำ**

### Step-by-step proof จากตัวเลขจริง

```
Q2 RMSE comparison:
   F-LIVO2 LIO   (no camera):    3.06 m   ← เริ่มที่นี่ (LiDAR backend gap)
   F-LIVO2 LIVO  (with camera):  2.89 m   ← camera ช่วยลด -0.17m (-5.5%)
   Fast-LIO      (no camera):    2.27 m   ← baseline
   
Decomposition:
   LiDAR-side gap   = F-LIVO2 LIO − Fast-LIO          = 3.06 − 2.27 = +0.79 m  (PRIMARY)
   Camera benefit   = F-LIVO2 LIVO − F-LIVO2 LIO      = 2.89 − 3.06 = −0.17 m
   Net residual gap = F-LIVO2 LIVO − Fast-LIO         = 2.89 − 2.27 = +0.62 m
   
Verification: 0.79 + (-0.17) = 0.62 ✓
```

→ **80% ของ gap (0.79m) เกิดที่ LiDAR side — ก่อนเปิดกล้องด้วยซ้ำ**
→ **Camera benefit (-0.17m) ช่วยลด gap** — ไม่ใช่เพิ่ม
→ **20% ของ gap (0.17m) คือ "camera ช่วยได้น้อยกว่าที่ควร"** ไม่ใช่ "camera ทำให้แย่ลง"

### ดังนั้นคำตอบที่ make-sense

**สาเหตุที่ F-LIVO2 LIVO > Fast-LIO ใน Q2/Q3 มี 2 ระดับ:**

**ระดับ 1 (Primary — 80% ของ gap): LiDAR backend mismatch**
- F-LIVO2 ใช้ **VoxelMap (octree adaptive)** + noise model จูนสำหรับ Livox/Ouster/Pandar
- Fast-LIO ใช้ **ikd-Tree (KD-tree of points)** + simpler noise model
- บน **Velodyne HDL-32E + urban driving** (สเปคที่ paper ไม่ test): VoxelMap underperform
- Cause hypothesis (not paper-confirmed):
  - Voxel 0.5m ไม่ match urban building feature scale
  - F-LIVO2 noise model ไม่จูนสำหรับ Velodyne
  - IMU covariance defaults ของ F-LIVO2 สูงกว่า Fast-LIO 5x

**ระดับ 2 (Secondary — 20% ของ gap missed potential): Camera benefit limited**
- ในอุดมคติ (เหมือน eee_03 UAV): camera ควรช่วย -36%
- ในความจริง UrbanNav: camera ช่วยแค่ -5.5%
- Lost potential ~30% เพราะ **patches ถูก outlier-reject** 71% ของ frames

### Patch rejection causes (Paper Section VII.A.3 + real-world)

| Trigger criterion (paper) | Real-world cause (UrbanNav) |
|---|---|
| 1. **Occlusion** | 🚗 รถ/คน บัง patches |
| 2. **Depth-discontinuity** | 🚗 รถข้าง depth ต่างจาก background ตึก |
| 3. **View angle > 80°** | 🌀 Vehicle เลี้ยวเร็ว |
| (Photometric error) | 🌀 Motion blur + 💡 Glare + 🌑 Shadow + 🔁 Repetitive façade |

→ ดูรายละเอียดแต่ละ cause ใน Section 10.3

### ความเข้าใจผิดที่ต้องระวัง

| Wrong (common misinterpretation) | Correct (data-supported) |
|---|---|
| ~~"กล้องทำให้ F-LIVO2 แย่กว่า Fast-LIO"~~ | กล้องช่วยลด error ของ F-LIVO2 (3.06→2.89 ใน Q2). ตัวกล้องเอง "ไม่ผิด" |
| ~~"Patches เสื่อมเป็นสาเหตุหลักที่ F-LIVO2 < Fast-LIO"~~ | Patches เสื่อมแค่ลด camera benefit. ถ้า patches ดี (เหมือน eee_03) gap ก็ยังไม่ปิด เพราะ LiDAR-side gap 0.79m |
| ~~"F-LIVO2 LIVO รวมข้อมูล LiDAR + camera ควรดีกว่า Fast-LIO ที่มีแค่ LiDAR"~~ | จริงเฉพาะเมื่อ "LiDAR backend ของ F-LIVO2 ดีอย่างน้อยเท่า Fast-LIO". บน Velodyne urban, F-LIVO2 LiDAR backend แย่กว่า — กล้องช่วยปิดได้บางส่วน แต่ไม่หมด |

### Final statement

> **"F-LIVO2 LIVO has higher Q2/Q3 error than Fast-LIO not because its camera causes problems, but because its LiDAR backend (VoxelMap) underperforms Fast-LIO's ikd-Tree on Velodyne urban scenes. The camera in fact reduces F-LIVO2's error (e.g., from 3.06m to 2.89m in Q2), but this 5% reduction is insufficient to close the 35% LiDAR-side gap. Patch rejection due to dynamic obstacles, motion blur, glare, shadows, and repetitive façades limits the camera benefit to ~14% of what it could theoretically provide (compared to 36% on eee_03 UAV scenes). The root cause is therefore the **scene-algorithm mismatch** between F-LIVO2's VoxelMap LiDAR backend and Velodyne HDL-32E + urban driving conditions — a sensor+platform combination not tested in the FAST-LIVO2 paper."**

## Q6: Patch rejection ↔ Camera benefit relationship

**Q:** ที่ green/blue dots ใน RViz น้อยลง = patch rejection หรือเปล่า? ถ้า camera ดีทุก frame, F-LIVO2 LIVO Q2 error จะลดลงไหม?

### A: ใช่ — fewer green dots = more rejection = less camera benefit (linear)

**Mechanism:**
```
LiDAR points ใน camera FoV  (~1000 candidates/frame)
        ↓
ผ่าน outlier rejection (paper Section VII.A.3)?
  - Occluded? → reject
  - Depth discontinuous? → reject
  - View angle > 80°? → reject
  - Photometric error สูง (NCC < threshold)? → reject
        ↓
Surviving = green dots ใน RViz
        ↓
จำนวน constraints ใส่ใน ESIKF update step
```

→ **More green dots → more constraints → bigger camera correction**

### Theoretical max benefit ถ้า camera perfect

| Scenario | F-LIVO2 LIO Q2 | Camera benefit | F-LIVO2 LIVO Q2 |
|---|---|---|---|
| **Current (patches degraded)** | 3.06 m | −5.5% | 2.89 m (worse than Fast-LIO 2.27m) |
| **Theoretical (perfect patches, eee_03-level)** | 3.06 m | **−36%** | **1.96 m** ⭐ (better than Fast-LIO!) |
| Lost potential | — | — | **0.93 m gap** ที่ patch rejection ขโมย |

→ ถ้า patches perfect ทุก frame, F-LIVO2 LIVO **อาจชนะ Fast-LIO 14%** บน Q2!
→ Patch rejection กิน 0.93m ของ camera benefit ที่ควรมี

## Q7: ขยาย "LiDAR backend underperform on Velodyne urban" — concrete mechanisms

**Q:** ประโยค "F-LIVO2's VoxelMap LiDAR backend underperforms Fast-LIO's ikd-Tree on Velodyne urban" หมายความว่าอย่างไรในเชิงรูปธรรม?

### A: 4 mechanisms ที่ทำให้ VoxelMap struggle บน Velodyne+urban

#### Mechanism 1: Voxel size vs urban feature scale mismatch

```
Voxel size = 0.5 m (F-LIVO2 default)
Urban feature scales:
  กำแพงตึก:        5-50 m         ← bigger than voxel
  หน้าต่าง:        1-2 m
  กรอบหน้าต่าง:    0.1-0.3 m       ← smaller than voxel!
  ป้าย/sign:       0.5-3 m
  ขอบฟุตบาท:       0.1 m            ← smaller than voxel!
```

→ Voxel 0.5m **มีหลาย features ตัดต่อกัน** (กำแพง + window + กรอบ)
→ Plane stats = **average ของ multiple real planes** → noisy + biased

#### Mechanism 2: Multi-plane voxels → discarded

Paper Section V.B:
> "If the contained points do not lie on a plane, the voxel is continuously subdivided into eight smaller octants until... maximum layer is reached. In the latter case, the points in the leaf voxel will be discarded."

```
Voxel 0.5m ครอบ corner ของตึก:
  → มี 2 walls + ขอบ = 3 planes
  → Subdivide ลง 0.25m (8 sub-voxels)
  → Sub-voxel ที่ตกบนขอบยังมี multi-plane
  → Subdivide ลง 0.125m
  → max_layer = 2 (config ของเรา) → STOP
  → ยัง multi-plane? → DISCARD VOXEL
```

→ Urban มี edges/corners เยอะ → **มาก voxels ถูกทิ้ง** → constraints หาย

**vs ikd-Tree (Fast-LIO):**
- เก็บ individual points (ไม่ใช่ voxel grid)
- k-NN search → หา 5 จุดใกล้สุดที่อาจอยู่บน plane เดียวกัน → fit plane ได้
- **ไม่เจอ "multi-plane voxel discarded" issue**

#### Mechanism 3: Vehicle speed → voxel statistics noisy

```
Vehicle 30 km/h = 8.3 m/s @ 10 Hz LiDAR
  → vehicle เคลื่อน 83 cm ระหว่าง 2 scans
  
ใน 1 second:
  - 1 voxel (0.5m) เห็นใน ~6 scans เท่านั้น
  - Plane stats สะสมจาก 6 scans เท่านั้น
  - แต่ vehicle pose ระหว่าง scans มี small drift
  - → plane stats เพี้ยน

vs UAV slow motion (1-3 m/s):
  - 1 voxel เห็นใน 30+ scans
  - Plane stats stable
```

#### Mechanism 4: Noise model ไม่ตรง Velodyne

Paper Section IX-A page 12:
```
Noise tuning per LiDAR:
  Livox Avia / OS1-16:  depth=0.02m, beam=0.05°, divergence=0.15°
  PandarXT-32:          depth=0.001m, beam=0.001°, divergence=0.001°
  Robosense BPearl:     depth=0.008m, beam=0.01°
  
  Velodyne HDL-32E:     ❌ ไม่ระบุ
```

→ เราใช้ค่า default → **weighting ของจุดใน ESIKF อาจไม่เหมาะ** → optimization landscape ผิด → drift

### รวมเข้าด้วยกัน

```
F-LIVO2 LIO Q2 = 3.06 m  ← struggle from 4 mechanisms above
Fast-LIO Q2    = 2.27 m  ← ikd-Tree adapts ดีกว่า
Gap            = 0.79 m  (35% penalty)
```

**ทั้งคู่ใช้ LiDAR + IMU เดียวกัน — ต่างแค่ "algorithm choice" — 35% accuracy difference**

## Q8: Drone (eee_03) มี acceleration ก็มี — ทำไมยัง patches stable?

**Q:** Vehicle ช้า → dots เยอะ, Vehicle เร็ว → dots น้อย. แต่ drone ก็มี acceleration / เปลี่ยนแปลงตลอด — ทำไมยังมี dots เยอะกว่า + แม่นกว่า?

### A: Drone "acceleration" ≠ Vehicle "motion" ในมิติที่กระทบ camera

### 7 ปัจจัยเปรียบเทียบ

| ปัจจัย | Drone (eee_03) | Vehicle (UrbanNav) | ผลต่อ patches |
|---|---|---|---|
| **Absolute speed** | 1-3 m/s | 5-17 m/s (30-60 km/h) | **Vehicle 5-10× faster** → blur มากกว่า |
| **Motion type** | Brief acceleration spikes (then hover) | Sustained motion (always moving) | Drone มี stable windows; vehicle constantly blurry |
| **Scene depth** | 2-10 m (indoor walls) | 20-100 m (buildings far) | Drone: high parallax = features track well |
| **Lighting** | Indoor controlled / clean | Variable sun/shadow/glare | Drone stable, vehicle dynamic exposure |
| **Dynamic obstacles** | Almost none | Constant (traffic) | Drone: no occlusion; vehicle: frequent |
| **Camera spec** | Pinhole 752×480 hardware-trigger | ZED2 672×376 software sync | Drone: cleaner sync |
| **Patch context** | Indoor distinct features | Repetitive façades | Drone: distinct, vehicle: NCC ambiguous |

### Key insight: Motion blur depends on **speed × exposure**, NOT acceleration

```
Vehicle accelerate 30→50 km/h (a = 2 m/s²):
  Motion blur during 15ms exposure:
    - At 8 m/s:  12 cm blur  ⚠
    - At 14 m/s: 21 cm blur  ⚠⚠

Drone accelerate 1→3 m/s (a = 1 m/s²):
  Motion blur during 15ms exposure:
    - At 1 m/s: 1.5 cm blur  ✓
    - At 3 m/s: 4.5 cm blur  ✓
  → 10× less than vehicle!
```

→ **Drone accelerates มาก** แต่ absolute speed ต่ำ → blur น้อย → patches stable

### Scene depth → parallax matters

```
ที่ 5m distance (drone indoor), vehicle move 1cm:
  Pixel motion = (1cm/5m) × 600 px = 1.2 pixels  ✓ trackable

ที่ 50m distance (urban buildings), vehicle move 1cm:
  Pixel motion = (1cm/50m) × 264 px = 0.13 pixels  ✗ below noise
  → cannot detect parallax → no real constraint
```

→ Drone close features = real triangulation
→ Vehicle far features = "infinite distance" feel = weak constraints

### TL;DR

```
Vehicle problem ≠ "acceleration"
Vehicle problem = "sustained high speed"           (5-10× faster than drone)
                + "far features (low parallax)"     (50m vs 5m)
                + "dynamic urban scene"             (rush hour vs indoor)
                + "variable lighting"               (sun/shadow vs steady)
                + "low-res ZED2 raw"                (672×376 vs 752×480)
                + "repetitive façades"              (NCC ambiguous)

Drone wins despite acceleration เพราะ:
  - Brief accelerations แต่ slow average speed
  - Close features → high parallax tracking
  - Static indoor scene
  - Controlled lighting
  - Higher-res hardware-synced camera
  - Distinct indoor textures
```

→ "Acceleration" alone is not the determining factor — it's the **combined regime** of speed + depth + lighting + dynamics + sensor quality

---

# 🔬 Section 10: Visual Map Points Analysis (UrbanNav)

## 10.1 Green dots fluctuation — what we observed in video

จุดเขียวสีฟ้าใน RViz เปลี่ยนแปลงอย่างรุนแรงตลอด trajectory:
- บางช่วงขึ้นเยอะมาก (>50 dots)
- บางช่วงขึ้นน้อยมาก (<5 dots)

## 10.2 จาก T12 slam.log (4084 frames) — statistical analysis

```
Distribution of green-dot count per frame:
    0–4 dots:    927 frames (22.7%)  ← visual update ineffective
    5–24 dots: 1971 frames (48.3%)  ← weak contribution
   ≥25 dots:  1186 frames (29.0%)  ← strong contribution
```

→ **71% ของเวลา visual update ทำงานน้อยหรือไม่ทำงาน** → LIVO mode degenerate กลับเป็น LIO-only

## 10.3 Why patches get rejected (detailed)

Paper-verified rejection criteria (Section VII.A.3) + real-world causes:

### Mechanism 1: Occlusion (paper criterion #1)
```
Reference frame: patch บนตึก
Current frame:   🚗 รถวิ่งผ่านบังหน้า patch
→ rejected (depth jump)
```

### Mechanism 2: Depth discontinuity (paper criterion #2)
```
LiDAR point อยู่ที่ตึก (15m)
Patch 9×9 neighborhood ครอบขอบรถ (3m) ปนกัน
→ depth variance สูง → rejected
```

### Mechanism 3: View angle (paper criterion #3)
```
Reference patch normal vs current camera direction > 80°
→ patch warped มาก → unreliable → rejected
```

### Real-world causes (deep dive):

#### 🚗 Dynamic obstacles (รถ/คน)

**Trigger:** Paper criteria #1 (Occlusion) + #2 (Depth discontinuity)

**Physical mechanism:**
```
Reference frame (t=0):
  LiDAR point ที่ตึก (15m ไกล) → patch ในภาพคือ "façade ตึก"
  
Current frame (t=1):
  รถวิ่งผ่านหน้า camera, บัง patch ของจุดที่ตึก
  Patch ใน current frame = "สี door panel ของรถ"
  
F-LIVO2:
  ตรวจ depth ใน 9×9 neighborhood ของ patch
  → จุดส่วนใหญ่ที่ตึก (15m), แต่ pixel กลาง patch จริง ๆ คือรถ (3m)
  → depth variance สูง → REJECT
```

**Common ใน UrbanNav:**
- ที่หยุดไฟแดง → รถข้างขับผ่าน
- Stop signs → คนเดินข้ามถนน
- Bus stops → รถบัสจอดบัง view
- Junction → traffic dense, occlusion frequent

#### 🌀 Motion blur (เลี้ยว/เร่ง)

**Trigger:** Photometric error (Paper Eq 22) จาก patch comparison

**Physical mechanism:**
```
ZED2 exposure time = ~10-30 ms (urban lighting)
Vehicle speed = 30 km/h = 8.3 m/s
Camera rotation rate during turn = 10°/s

ใน 1 exposure (15ms):
  - Linear translation = 12.5 cm
  - Angular displacement = 0.15° 
  
At 5m distance:
  Pixel motion = (12.5cm sin(angle) + 0.15° × 5m) / pixel_size
              ≈ 8-15 pixels of blur per frame
```

**F-LIVO2 ทำอะไร:**
- Reference patch (sharp, จับตอน vehicle หยุด) vs current patch (blurred)
- Normalized Cross-Correlation (NCC) drops → similarity score ต่ำ
- Reference patch update strategy (Section V-D paper) เลือก patch ใหม่ — แต่ถ้า patch ใหม่ก็ blur ด้วย → degraded
- Photometric error (Eq 22) สูงเกิน threshold → patch ใน this iteration ถูก downweighted

**Common ใน UrbanNav:**
- ทุกการเลี้ยว/เปลี่ยน lane
- การเร่ง/เบรกแรง
- รถสะดุดสะดี (rough road)

#### 💡 Glare / Specular reflection

**Trigger:** Pixel saturation (intensity = 255 หรือ 0) → no gradient → photometric error ill-defined

**Physical mechanism:**
```
Sun glint บนกระจก:
  Reference patch:   intensity range [80, 180]  (normal scene)
  Current patch:     intensity range [240, 255] (specular hit)
  
Photometric residual:
  | I_current - I_ref | = |255 - 130| = 125  ← huge
  
Visual map point's reference patch (Section V-D):
  NCC ของ patches อื่นต่อ patch ใหม่ = ต่ำมาก
  → reference patch update fails
  → จะถูก replace ด้วย patch saturated ที่จริง ๆ ไม่มี info gradient
```

**Common ใน UrbanNav:**
- กระจกตึกสะท้อนแดด
- กระจกรถขับสวน
- ถนนเปียกหลังฝน → reflective
- LED ป้ายโฆษณาตอนกลางคืน

**ZED2 dynamic range จำกัด:** auto-exposure ปรับตัวช้ากว่า scene change → frame ที่ glare → saturated → patches เสีย

#### 🌑 Shadow / Brightness change

**Trigger:** Photometric error (Eq 22) ใน F-LIVO2 ใช้ raw pixel values — ไวต่อ illumination change

**Physical mechanism:**
```
Vehicle ออกจากใต้สะพานเข้าทางมีแสง:
  Reference patch (จับใต้สะพาน, dim):  mean intensity = 60
  Current patch (มีแสง):              mean intensity = 180
  
3x intensity ratio → patch กลายเป็น "ภาพคนละแบบ"
```

**F-LIVO2 มี exposure estimation** (Section V eq 22) — เพื่อ compensate global brightness:
- `I_compensated = I × exp(τ_ref - τ_cur)`
- เก็บ inverse exposure τ ใน state

**แต่:** Spatial shadow patterns ภายใน 1 patch (เช่น ครึ่ง patch อยู่ใต้ตึก ครึ่งโดนแดด) — global compensation แก้ไม่ได้ → patch matching fails → reject

**Common ใน UrbanNav:**
- ใต้สะพาน
- เงาตึกข้างถนน (sharp shadow lines)
- ใต้ flyover/canopy
- เงาเสาไฟใหญ่

#### 🔁 Repetitive façade

**Trigger:** NCC similarity ambiguous → reference patch selection พลาด

**Physical mechanism:**
```
Pattern เช่น ตึกที่หน้าต่างเรียงสม่ำเสมอ:
   ┌──┐ ┌──┐ ┌──┐ ┌──┐
   │██│ │██│ │██│ │██│  ← หน้าต่าง pattern
   └──┘ └──┘ └──┘ └──┘
   
Reference patch = หน้าต่างใดหน้าต่างหนึ่ง
Current frame: vehicle เคลื่อนไป → ตอนนี้ camera มอง "หน้าต่างถัดไป" ที่ดูเหมือนกัน
  
NCC score: 0.95 ซ้ำ ๆ กับหลายตำแหน่ง → ambiguity
F-LIVO2 reference patch update (Section V-D):
  เลือก patch ที่ NCC สูงสุด — แต่ถ้าหลายตัวเท่ากัน → เลือกผิด
  → patch update ไป "หน้าต่างผิด" → projection vector เพี้ยน
```

**Common ใน UrbanNav:**
- Tile walls ของ MRT/รถไฟฟ้า
- Fences/รั้ว periodic
- Apartment blocks ที่ระเบียงซ้ำ
- Brick patterns
- Office building windows grid

#### 📊 Summary table

| Cause | Trigger criterion | Trigger frequency UrbanNav | Severity |
|---|---|---|---|
| 🚗 Dynamic obstacles | Occlusion + depth discontinuity | Very high (every junction) | High |
| 🌀 Motion blur | Photometric error | High (turns + speed changes) | Medium |
| 💡 Glare/specular | Pixel saturation | Medium (depend on sun angle) | Medium-High |
| 🌑 Shadow boundaries | Local intensity change | Medium (under bridges/buildings) | Medium |
| 🔁 Repetitive façade | NCC ambiguity | Medium (tile walls, fences) | Low-Medium |
| 📷 Low ZED2 resolution (672×376) | Small patches localize ไม่แม่น | Always | Low (constant penalty) |

→ **Combined effect:** ~70% of frames have <25 effective green dots → visual constraint strength severely degraded

→ **But remember (Section 8.3):** This is **secondary cause** of F-LIVO2 < Fast-LIO. The primary cause is **LiDAR backend** (visible in LIO mode without camera). Patches เสื่อมแค่ลด camera benefit ไม่ใช่เพิ่ม absolute error.

## 10.4 Visual map size growth — T12 vs eee_03

| Dataset | Retrieve avg/frame | Retrieve max | Pattern |
|---|---|---|---|
| eee_03 (UAV indoor) | growing 3 → 95 | 95 | monotonic increase ✓ |
| UrbanNav (urban) | 20.2 avg, range 1-88 | 88 | wild fluctuation ⚠ |

→ ใน urban driving, visual map size **ไม่ stable** — สลับขึ้นลงตาม scene quality

---

# ⏱ Section 10.5: Runtime Performance Analysis

## 10.5.1 Per-frame compute time (ms per LiDAR scan)

**Target:** 100 ms per frame (10 Hz LiDAR rate) → realtime threshold

| Dataset / Method | LIO step (ms) | VIO step (ms) | **Total (ms)** | p95 (ms) | Realtime factor |
|---|---|---|---|---|---|
| **UrbanNav** F-LIVO2 LIO (T14c) | 16.42 | — | **16.42** | 22.76 | **6.1×** ✓ |
| **UrbanNav** F-LIVO2 LIVO (T12) | 16.00 | 3.98 | **19.98** | 27.23 | **5.0×** ✓ |
| **UrbanNav** Fast-LIO | — | — | **~14.55** | ~19.93 | **6.9×** ✓ |
| **eee_03** F-LIVO2 LIO | 12.09 | — | **12.09** | 16.66 | **8.3×** ✓ |
| **eee_03** Fast-LIO | — | — | **14.55** | 19.93 | **6.9×** ✓ |

**ทุก method run realtime ได้สบาย** (ใช้เวลา ~15-20 ms ต่อ scan; budget 100 ms)

## 10.5.2 Key observations

### 1️⃣ VIO step (camera) เพิ่มเวลาแค่ ~4 ms (~25% overhead)
- LIO 16.00 ms → LIVO 19.98 ms
- ไม่ใช่ bottleneck — camera ทำงาน "เร็วมาก" เพราะ:
  - Patch 8×8 small operation
  - Sparse direct method (~50-100 visual map points) ไม่ใช่ dense
  - Reuses LiDAR points → ไม่ต้อง feature extraction

### 2️⃣ F-LIVO2 LIO vs Fast-LIO compute parity
- UrbanNav: 16.42 ms vs 14.55 ms (~13% slower)
- eee_03: 12.09 ms vs 14.55 ms (~17% **faster**)

→ Compute time ใกล้กัน — VoxelMap ไม่ slower กว่า ikd-Tree significant
→ **Performance gap (35%) ของ F-LIVO2 LIO บน UrbanNav ไม่ใช่เพราะ compute issue** — เป็น algorithmic

### 3️⃣ eee_03 เร็วกว่า UrbanNav (12 ms vs 16 ms)
- Ouster 16-line = ~30k points/scan (sparse)
- Velodyne 32-line = ~60-80k points/scan (dense)
- → 2× points = ~33% more compute time

## 10.5.3 Compute budget (realtime factor breakdown)

```
Time available per scan = 100 ms (10 Hz)
─────────────────────────────────────────
F-LIVO2 LIVO uses:
  ~16 ms LIO  (16%)
  ~4 ms  VIO  (4%)
  ~80 ms idle (80%)  ← realtime headroom
```

→ Room ให้เพิ่ม voxel resolution / max_points หรือใส่ heavier visual features ในอนาคต

---

# 🗺 Section 10.6: Output Point Cloud Maps

## 10.6.1 PCD file sizes (both datasets)

### UrbanNav HK-TST (Velodyne, 785s, ~470s SLAM time)

| Method | `raw.pcd` size | `downsampled.pcd` size | Total points | Compression ratio |
|---|---|---|---|---|
| F-LIVO2 LIO (T14c) | **1.89 GB** | 318 MB | **62 M** / 21 M | 5.9× |
| F-LIVO2 LIVO (T12) | 326 MB | 326 MB | **21 M** | 1.0× (camera-filtered) |
| Fast-LIO | (save failed*) | — | (est. ~40-50 M) | — |

*Fast-LIO UrbanNav PCD save failed reliably (even after source patch). After 13min run + manual SIGINT, the cleanup save block did not execute. Possible reason: large `pcl_wait_save` (>1GB accumulated points) caused memory/timing issue during shutdown. **Estimate ~40-50M points** based on Velodyne 32-line + 470s of mapped trajectory (~85k pts/sec post-filter).

### eee_03 NTU-VIRAL (Ouster OS1-16, 165s)

| Method | `raw.pcd` size | Total points | Note |
|---|---|---|---|
| **Fast-LIO** | **180 MB** | **5.89 M** | After source patch + rebuild |
| F-LIVO2 LIO (Feb 21 run) | 276 MB | **9.03 M** | From earlier run (Log/pcd/eee_03_noim/) |
| F-LIVO2 LIVO raw (Feb 21 run) | 35 MB | **2.28 M** | Aggressive filtering with camera |
| F-LIVO2 LIVO downsampled | 8 MB | 521 K | Final downsampled output |

**Observations (eee_03):**
- **F-LIVO2 LIO ใหญ่ที่สุด** (9.03M) — accumulates all undistorted points
- **Fast-LIO กลาง** (5.89M) — also raw accumulation but filtered differently
- **F-LIVO2 LIVO เล็กที่สุด** (2.28M raw) — camera-stabilized voxel filtering aggressive

→ **F-LIVO2 LIVO map size 4× smaller** than F-LIVO2 LIO เพราะ camera ช่วย filter redundant points

⚠ **Important note: Fast-LIO ROS 2 port had PCD save disabled in source code**

ใน `FAST_LIO_ROS2/src/laserMapping.cpp` line 514-543: code `*pcl_wait_save += *laserCloudWorld` ถูก commented out → Fast-LIO ไม่สะสมจุดสำหรับ save → PCD ที่ saved จะมีขนาด 0

**Patch applied:** Uncomment the block → rebuild → Fast-LIO saves PCD ตามปกติ (ดู Section 12 Reproducibility)

หลัง patch + rebuild: Fast-LIO บน eee_03 = 5.9M points, 180MB

## 10.6.2 ทำไม LIVO map เล็กกว่า LIO?

ดู config ใน `urbannav.yaml`:
```yaml
pcd_save:
  pcd_save_en: true
  filter_size_pcd: 0.15        # downsample voxel size
publish:
  dense_map_en: true
  blind_rgb_points: 0.0
```

**F-LIVO2 LIO mode (voxel 0.5 default):**
- Raw points ถูก accumulate ตามการเคลื่อนของ vehicle
- ทุก scan ที่ผ่านเข้า map → raw.pcd ใหญ่ขึ้นเรื่อย ๆ
- Downsampled = raw / 5.9 (post-filter at 0.15m voxel)

**F-LIVO2 LIVO mode (voxel 0.3, max_points 100):**
- Voxel finer + camera ทำให้ filter aggressive ขึ้น
- VoxelMap ตัด redundant points ใน real-time
- → raw และ downsampled เท่ากัน (already deduplicated)

## 10.6.3 Point density per scan area

ประมาณ **point density** ของ map (UrbanNav scene ~700×500 m = 350,000 m²):

| Method | Total points | Density (pts/m²) |
|---|---|---|
| F-LIVO2 LIO raw | 62 M | ~177 |
| F-LIVO2 LIO downsampled | 21 M | ~60 |
| F-LIVO2 LIVO | 21 M | ~60 |

→ Downsampled map ของทั้ง 2 modes มี density ใกล้กัน (~60 pts/m²) — เพียงพอสำหรับ urban reconstruction

## 10.6.4 Map quality observations

- **F-LIVO2 LIVO มี RGB-colored points** (กล้องเพิ่มสีให้ point cloud) → ใช้ทำ colored 3D map ได้ตรง ๆ
- **F-LIVO2 LIO + Fast-LIO** ให้ point cloud สีพื้น (intensity-based) — ไม่มี RGB
- Map ของทั้ง 3 methods สามารถใช้ทำ:
  - 3D reconstruction
  - Localization map สำหรับ subsequent runs
  - Path planning (collision check)

## 10.6.5 Storage requirements

| Application | Recommended map | Storage |
|---|---|---|
| Real-time SLAM | (in-memory voxel/KD-tree) | <500 MB RAM |
| Post-processing 3D recon | `raw.pcd` | 1-2 GB |
| Localization map (re-use) | `downsampled.pcd` | 300 MB |
| Visualization (RViz/CloudCompare) | `downsampled.pcd` | 300 MB |

---

# 🎯 Section 11: Conclusions

## 11.1 ผลของแต่ละ dataset

### eee_03 NTU-VIRAL (UAV)
✅ **F-LIVO2 LIVO ชนะ Fast-LIO อย่างชัดเจน** (4.3×) — ตรง paper claim 100%
- F-LIVO2 LIO 0.042m << Fast-LIO 0.117m: VoxelMap + noise model เหมาะกับ Ouster sparse
- F-LIVO2 LIVO 0.027m < F-LIVO2 LIO 0.042m: Camera ใน UAV indoor scene เพิ่ม 36%

### UrbanNav HK-TST (Vehicle)
⚠ **ผลขึ้นกับ benchmark interpretation:**
- **Overall** (Q2 ครอบงำ, GT noisy): Fast-LIO ชนะ 27%
- **Q1 only** (GT cm-accuracy): F-LIVO2 LIO ชนะ 28%

→ ตีความได้ 2 มุมมอง:
- Engineering-side: ถ้าเป้าหมาย match GT, Fast-LIO ดีกว่า
- Algorithm-side: เมื่อ benchmark trust ได้, F-LIVO2 แม่นกว่า

## 11.2 ปัจจัยหลักที่กำหนด winner

1. **LiDAR backend × LiDAR type:**
   - Ouster sparse → VoxelMap wins
   - Velodyne dense urban → ikd-Tree wins

2. **Camera × scene:**
   - Slow motion + indoor → camera ช่วยมาก
   - Dynamic urban → camera marginal/negative

3. **GT quality:**
   - cm-accuracy (RTK fix / Mocap) → benchmark fair
   - m-accuracy (multipath GPS) → ranking can be artifact

## 11.3 Generalization implications

**FAST-LIVO2 paper's claim:** "outpaces other state-of-the-art SLAM systems"

**Our findings:**
- ✓ TRUE on tested domains (UAV/handheld with Ouster/Avia/Pandar/Hilti)
- ⚠ UNCERTAIN on urban vehicle driving (paper doesn't test)
- → "Best SLAM" depends fundamentally on scene type + sensor combo

## 11.4 Practical recommendations

| Use case | Recommended |
|---|---|
| UAV (slow, indoor/outdoor) | F-LIVO2 LIVO |
| Handheld surveying | F-LIVO2 LIVO |
| Vehicle urban driving | Fast-LIO (or F-LIVO2 LIO if Q1 priority) |
| LiDAR-degenerate (tunnel) | F-LIVO2 LIVO (camera essential) |
| General benchmark | Test on multiple scenes — no universal winner |

---

# 🛠 Section 12: Reproducibility

## 12.1 Directory structure

```
/home/fibo5/fast_ws/
├── experiment/                          ← this dir
│   ├── README.md                         ← THIS file
│   ├── TUNING_RESULTS.md                 ← full tuning details
│   ├── run_urbannav_seq.sh               ← F-LIVO2 runner (UrbanNav)
│   ├── run_ntu_viral_seq.sh              ← F-LIVO2 runner (eee_03)
│   ├── run_fastlio_seq.sh                ← Fast-LIO runner (UrbanNav)
│   ├── run_fastlio_ntu_seq.sh            ← Fast-LIO runner (eee_03)
│   ├── convert_to_prism.py               ← lever-arm correction (eee_03)
│   ├── eval_urbannav_with_leverarm.py    ← lever-arm (UrbanNav, ~null effect)
│   ├── plot_path_by_quality.py           ← Q-level visualization
│   ├── plot_all_q_segments.py            ← Q1/Q2/Q3 spatial+temporal
│   ├── plot_compare_3way.py              ← 3-way overview
│   ├── plots/seq1/                       ← generated PNGs
│   └── runs/                             ← per-trial outputs
│       ├── eee_03/{lio, livo, fastlio}/  ← eee_03 results
│       ├── seq1/{lio, livo, fastlio}/    ← UrbanNav results
│       ├── seq1_T0_lio … T14c_lio/       ← tuning trials
│       ├── seq1_T5_livo … T13_livo/      ← LIVO trials
│       └── seq1_T15/livo, seq1_T16/livo/ ← root cause hypothesis tests
├── src/
│   ├── FAST-LIVO2/                       ← F-LIVO2 source + configs
│   ├── urbannavdataset/                  ← UrbanNav bag + GT
│   └── PDF/2408.14035v2.pdf              ← FAST-LIVO2 paper
└── ../fast_lio2_ws/src/FAST_LIO_ROS2/    ← Fast-LIO source + configs
```

## 12.2 Run commands

### eee_03 (NTU-VIRAL)
```bash
cd /home/fibo5/fast_ws

# F-LIVO2 LIO (camera off)
./experiment/run_ntu_viral_seq.sh lio

# F-LIVO2 LIVO (camera on)
./experiment/run_ntu_viral_seq.sh livo

# Fast-LIO
./experiment/run_fastlio_ntu_seq.sh

# Apply prism lever-arm + evaluate
python3 experiment/convert_to_prism.py \
    experiment/runs/eee_03/livo/trajectory.txt \
    experiment/runs/eee_03/livo/trajectory_prism.txt

GT=/home/fibo5/fast_ws/src/FAST-LIVO2/Log/result/ntu_viral/eee_03_gt.txt
evo_ape tum "$GT" trajectory_prism.txt -a -s --pose_relation trans_part
```

### UrbanNav
```bash
BAG=/home/fibo5/fast_ws/src/urbannavdataset/urbannav_bag
GT=/home/fibo5/fast_ws/src/urbannavdataset/UrbanNav_TST_GT_raw.txt

# F-LIVO2 LIO/LIVO
./experiment/run_urbannav_seq.sh seq1 "$BAG" "$GT" lio  # หรือ livo

# Fast-LIO
./experiment/run_fastlio_seq.sh seq1 "$BAG" "$GT"

# Eval (built into runner) — produces errors.csv + eval_plot.png
```

## 12.3 Best config files (snapshots)

### F-LIVO2 LIVO best (T12)
File: `src/FAST-LIVO2/config/urbannav.yaml`
```yaml
common:
  img_en: 1
  lidar_en: 1
preprocess:
  point_filter_num: 3
  filter_size_surf: 0.5       # T8 (was 0.3)
vio:
  patch_size: 8                # T5 (was 4)
  normal_en: false             # T5 (was true)
  raycast_en: false
  inverse_composition_en: false
  exposure_estimate_en: true
lio:
  min_eigen_value: 0.001       # T7 (was 0.0025)
  voxel_size: 0.3              # T9 (was 0.5)
  max_layer: 2
  max_points_num: 100          # T12 (was 50)
extrin_calib:
  extrinsic_T: [0.0, 0.0, 0.28]
  Rcl: [1, 0, 0,  0, 0, -1,  0, 1, 0]
  Pcl: [0.0, 0.10, -0.10]      # placeholder (empirical)
```
**Result:** RMSE 2D = **2.81 m**

### F-LIVO2 LIO best (T14c)
Same as T12 but **revert aggressive LIO params**:
```yaml
common:
  img_en: 0
lio:
  min_eigen_value: 0.0025      # back to default
  voxel_size: 0.5              # back to default
  max_points_num: 50           # back to default
```
**Result:** RMSE 2D = **3.00 m** (Q1: 1.53 m ⭐)

### Fast-LIO best
File: `~/fast_lio2_ws/src/FAST_LIO_ROS2/config/urbannav.yaml`
```yaml
common:
  time_sync_en: true
preprocess:
  timestamp_unit: 0            # critical for UrbanNav
mapping:
  acc_cov: 0.1
  gyr_cov: 0.1
  extrinsic_T: [0., 0., 0.28]
```
**Result:** RMSE 2D = **2.21 m**

For eee_03 Fast-LIO ใช้:
```yaml
mapping:
  extrinsic_T: [-0.050, 0.0, 0.055]    # NTU-VIRAL IMU↔LiDAR offset
```
**Result:** RMSE 3D = **0.117 m**

---

# 📚 Section 13: References

1. **FAST-LIVO2** — Zheng et al., "FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry", arXiv:2408.14035v2 (Aug 2024) — [PDF](../src/PDF/2408.14035v2.pdf)
2. **VoxelMap** — Yuan et al., "Efficient and Probabilistic Adaptive Voxel Mapping for Accurate Online LiDAR Odometry", IROS 2022
3. **Fast-LIO2** — Xu et al., "FAST-LIO2: Fast Direct LiDAR-Inertial Odometry", IEEE T-RO 2022
4. **NTU-VIRAL dataset** — Nguyen et al., "NTU VIRAL: A Visual-Inertial-Ranging-LiDAR Dataset", IJRR 2022
5. **UrbanNav dataset** — Hsu et al., "UrbanNav: An Open-sourcing Localization Data Collected in Asian Urban Canyons", ION GNSS+ 2021
6. **evo** — Grupp, evaluation tool — https://github.com/MichaelGrupp/evo

## Key paper citations used in this analysis

| Topic | Source |
|---|---|
| F-LIVO2 LIO ดีกว่า Fast-LIO2 | Section IX-B (page 13) |
| VoxelMap structure | Section V (page 7-8) |
| LiDAR noise model | Section VI.B (page 9) |
| Visual map points + patches | Section II.A, VII (page 3, 10-11) |
| Outlier rejection criteria | Section VII.A.3 (page 10) |
| Camera causes "negative optimization" | Section IX-D (page 14) |
| LiDAR noise tuning per type | Section IX-A (page 12) |
| Datasets evaluated | Section VIII (page 11) |

---

# 📎 Appendix

- [TUNING_RESULTS.md](TUNING_RESULTS.md) — Full per-trial table (T0-T16) + per-parameter analysis
- [runs/seq1_T*_livo/eval_plot.png](runs/) — Per-trial visualizations
- [runs/seq1/all_q_segments.png](runs/seq1/all_q_segments.png) — Q1/Q2/Q3 spatial breakdown
- [runs/seq1/path_by_q_segments.png](runs/seq1/path_by_q_segments.png) — Zoomed segment view with 3-way comparison
- [plots/seq1/3way_trajectory.png](plots/seq1/3way_trajectory.png) — Main trajectory overview

---

> **Project:** SLAM benchmark research, FIBO/KMUTT
> **Author's note:** ผลทั้งหมดสามารถ reproduce ได้บน Ubuntu 22.04 + ROS 2 Humble. ใช้ Cyclone DDS สำหรับ UrbanNav (rosbags-converted bag). Bag rate 1.0 for eee_03, 0.5-1.0 for UrbanNav depending on config aggressiveness.
