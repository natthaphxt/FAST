# 📚 FAST-LIVO2 — คู่มืออธิบายแบบเข้าใจง่าย

> เอกสารอธิบาย **FAST-LIVO2** (Zheng et al., 2024) แบบเล่าเรื่อง — ไม่มีสมการ ไม่มีโค้ด เน้น "ภาพในหัว" และเปรียบเทียบกับ SLAM packages อื่น ๆ
>
> เหมาะสำหรับนักศึกษาที่กำลังเริ่มทำวิจัย SLAM และอยากเข้าใจหลักการก่อนลงลึกที่ paper

---

## 📖 สารบัญ

- [Part 1 — ปูพื้น: SLAM คืออะไร ทำไมยาก](#part-1--ปูพื้น-slam-คืออะไร-ทำไมยาก)
- [Part 2 — ทำไมต้องใช้ 3 sensors?](#part-2--ทำไมต้องใช้-3-sensors)
- [Part 3 — สถาปัตยกรรมรวมของ FAST-LIVO2](#part-3--สถาปัตยกรรมรวมของ-fast-livo2)
- [Part 4 — เจาะลึก 4 Modules](#part-4--เจาะลึก-4-modules)
  - [Module 1: ESIKF (Section IV)](#module-1--esikf-section-iv)
  - [Module 2: Local Mapping (Section V)](#module-2--local-mapping-section-v)
  - [Module 3: LiDAR Measurement Model (Section VI)](#module-3--lidar-measurement-model-section-vi)
  - [Module 4: Visual Measurement Model (Section VII)](#module-4--visual-measurement-model-section-vii)
- [Part 5 — เปรียบเทียบกับวิธีอื่น](#part-5--เปรียบเทียบกับวิธีอื่น)
- [Part 6 — เมื่อไหร่ FAST-LIVO2 ชนะ และเมื่อไหร่ไม่ชนะ](#part-6--เมื่อไหร่-fast-livo2-ชนะ-และเมื่อไหร่ไม่ชนะ)
- [Part 7 — สรุปและแหล่งข้อมูลเพิ่ม](#part-7--สรุปและแหล่งข้อมูลเพิ่ม)

---

# Part 1 — ปูพื้น: SLAM คืออะไร ทำไมยาก

## 🎒 ลองนึกภาพ

คุณถูกปล่อยลงในห้างสรรพสินค้าที่ไม่เคยเข้ามาก่อน **โดยปิดตา** แล้วต้องตอบ 2 คำถามนี้ตลอดเวลา:

1. **"ตอนนี้ฉันอยู่ตรงไหน?"** ← *Localization*
2. **"รอบ ๆ ฉันมีอะไร?"** ← *Mapping*

ทั้ง 2 คำถามต้องตอบ **พร้อมกัน** เพราะ:
- รู้ว่าตัวเองอยู่ไหน → ต้องอาศัยแผนที่
- สร้างแผนที่ได้ → ต้องรู้ตำแหน่งตัวเองก่อน

นี่คือคำว่า **SLAM** (**S**imultaneous **L**ocalization **A**nd **M**apping)

## ⚠️ ทำไมยาก?

### ปัญหา 1: Sensor ทุกตัวมี "noise" และ "drift"

ไม่มี sensor ใดสมบูรณ์แบบ — ทุกการวัดมีค่าผิดเพี้ยน + ถ้ารวมการวัดหลายครั้ง error ก็สะสม

### ปัญหา 2: Real-time

หุ่นยนต์ที่เคลื่อนที่ต้องตอบ "อยู่ตรงไหน" ภายใน **10–100 มิลลิวินาที** — ช้ากว่านี้ก็ชน

### ปัญหา 3: Scenario หลากหลาย

- ในร่ม / กลางแดด / มืดสนิท
- ผนังเรียบเปล่า ๆ vs สวนป่าที่มีรายละเอียดเยอะ
- ตึกที่หมุนซ้ำ ๆ (cathedral effect)

ไม่มี "วิธีเดียวฟิตทุกฉาก" → ต้องผสม sensor หลายชนิด

---

# Part 2 — ทำไมต้องใช้ 3 sensors?

ลองเปรียบกับประสาทสัมผัสคน:

| Sensor | เปรียบเหมือน | ✅ จุดแข็ง | ❌ จุดอ่อน |
|--------|--------------|------------|------------|
| **IMU** (Inertial Measurement Unit) | ความรู้สึกในร่างกาย / หูชั้นใน | เร็วมาก (200 ครั้ง/วิ) รู้ทุกการขยับเล็ก ๆ | สะสม error ไปเรื่อย ๆ — เดิน 10 วิ ก็เริ่มเพี้ยน |
| **LiDAR** (Light Detection And Ranging) | ค้างคาวที่ใช้ echolocation | วัดระยะแม่นระดับ cm — ได้ point cloud 3D | ไม่มีสี/texture — ผนังเรียบยาว ๆ จะ "หลง" |
| **Camera** | ตา | เห็นสี/ลวดลาย/รายละเอียดเยอะ | ไม่รู้ระยะตรง ๆ + มืดก็ดับสนิท |

## 🎯 หัวใจของ FAST-LIVO2

> **ใช้ทั้ง 3 sensors แบบ "tightly-coupled"** ในระบบเดียว — ไม่แยกประมวลผลแล้วเอาผลมารวม
>
> เปรียบเหมือน "การฟังเพลงสดด้วยหูทั้งสองข้าง" vs "ฟังหูซ้ายแล้วฟังหูขวา แล้วรวมเสียงในใจ" — แบบแรกได้ภาพรวมที่สอดคล้องกว่า

## 🌟 ตัวอย่างเชิงรูปธรรม — เดินในห้างที่กล้องบอด

| สถานการณ์ | LiDAR | Camera | IMU | ใครช่วยใคร? |
|-----------|-------|--------|-----|--------------|
| ทางเดินมีของตกแต่งเยอะ | ✅ | ✅ | ✅ | ทั้ง 3 ช่วยกัน |
| ทางเดินผนังขาวยาว | ❌ degeneration | ✅ ลวดลายช่วย | ✅ | **Camera ช่วย LiDAR** |
| เดินผ่านโซนมืด ไฟดับ | ✅ | ❌ มืด | ✅ | **LiDAR ช่วย Camera** |
| ขึ้นลิฟต์ ผนังเปล่า | ❌ | ❌ | ⚠️ พอลากไปได้ | IMU ลากไปก่อน |

---

# Part 3 — สถาปัตยกรรมรวมของ FAST-LIVO2

## 🏗️ ภาพรวม

```
   ┌─────┐    ┌─────┐    ┌────────┐
   │ IMU │    │LiDAR│    │ Camera │
   └──┬──┘    └──┬──┘    └───┬────┘
      │          │            │
      └──────────┴────────────┘
                 │
                 ▼
   ┌────────────────────────────────────┐
   │      Module 1: ESIKF (สมอง)        │  
   │   ทำนาย → เช็ค → ปรับ → ทำซ้ำ      │
   └────────────────────────────────────┘
                 │
        ┌────────┼────────┐
        │        │        │
        ▼        ▼        ▼
   ┌────────┐ ┌────┐ ┌─────────┐
   │Module 3│ │Map │ │Module 4 │
   │ LiDAR  │←→│M.2 │←→│ Visual  │
   │ Model  │ │    │ │  Model  │
   └────────┘ └────┘ └─────────┘
        ▲        ▲        ▲
        └────────┴────────┘
              │
   ┌──────────────────────────┐
   │     Output: Pose + Map   │
   │  (10 Hz ตามจังหวะ Camera) │
   └──────────────────────────┘
```

**สรุปบทบาท:**
- **Module 1 (ESIKF)** = สมอง orchestrator — รวบรวม evidence จาก 3, 4
- **Module 2 (Map)** = ห้องเก็บความจำ — เก็บโครงสร้างโลก
- **Module 3 (LiDAR Model)** = วิธีอ่านหลักฐานจาก LiDAR
- **Module 4 (Visual Model)** = วิธีอ่านหลักฐานจากกล้อง

## 🔄 Cycle หนึ่งทำอะไร?

ในแต่ละรอบ (~10 ครั้งต่อวินาที ตามจังหวะกล้อง) ระบบทำตามนี้:

```
   ┌─────────────────────────────────────────────────┐
   │ 1. 📥 Sync data — รับข้อมูลทั้งหมดที่มา           │
   │ 2. 🔮 IMU Forward Propagation — เดา pose         │
   │ 3. ⏪ Backward Propagation — แก้บิดเบี้ยว LiDAR    │
   │ 4. ✏️ LiDAR Update — ปรับ pose จากระนาบ           │
   │ 5. 🖼️ Visual Update — ปรับ pose จากลวดลาย         │
   │ 6. 🗺️ Map Update — เพิ่มจุดใหม่ / ปรับระนาบเก่า    │
   │ 7. 📤 Publish — ส่ง pose + map ออกไป              │
   └─────────────────────────────────────────────────┘
```

> **เกร็ด:** ใน implementation จริง Step 4 กับ 5 รัน "อย่างใดอย่างหนึ่ง" ต่อ cycle (ขึ้นกับว่ารอบนี้มีรูปใหม่ไหม) แต่ผลลัพธ์ทาง mathematics เทียบเท่าทำต่อกัน — เพราะข้อมูลถูกแยกล่วงหน้าโดย "scan recombination"

---

# Part 4 — เจาะลึก 4 Modules

## Module 1 — ESIKF (Section IV)

> **ชื่อเต็ม:** **E**rror-**S**tate **I**terated **K**alman **F**ilter with Sequential Update
>
> ฟังดูยาว แต่จริง ๆ คือ Kalman Filter + 3 trick พิเศษ

### 🕵️ Kalman Filter คืออะไร — เปรียบเทียบกับ "นักสืบ"

> **Kalman Filter เปรียบเหมือน Sherlock Holmes ทำงานทุกวินาที**
>
> 1. **เดา (Predict)** — "จากที่เห็นเมื่อกี้ + นิสัยคนร้าย เขาน่าจะอยู่ตรงนี้ ±2 เมตร"
> 2. **เช็ค (Update)** — "อ้าว! มีคนเห็นรอยเท้าตรงนั้น แสดงว่าเดาเมื่อกี้ผิดไป → ปรับใหม่"
>
> ความแม่นของการเดาขึ้นกับ **ความมั่นใจ** (covariance) ถ้านักสืบมั่นใจมาก → เชื่อการเดามากกว่าหลักฐานใหม่

ใน FAST-LIVO2:
- **State** = "ตำแหน่ง + ทิศหัน + ความเร็ว + IMU bias + gravity + แสงสว่าง" → รวม 19 ตัวเลข
- **Predict** = ใช้ IMU เดินทำนายไปข้างหน้า
- **Update** = ใช้ LiDAR + Camera มาเช็คว่าเดาถูกไหม

### 🔑 Trick 1: "Error-State"

**ปัญหา:** Rotation อยู่บน manifold SO(3) (โลก 3 มิติเชิงทรงกลม) บวกกันตรง ๆ ไม่ได้

> **Analogy:** เข็มนาฬิกาที่ "11 น." บวก "3 ชม." → "2 น." (ไม่ใช่ "14 น.") เพราะเวลาวนเป็นวงกลม
>
> Rotation ก็เหมือนกัน — วน 360° แล้วกลับที่เดิม

**วิธีแก้:** แทนที่จะ track ค่า "จริง" ของ rotation → track **"error" จากค่าที่เดาไว้** ซึ่ง error เล็ก ๆ ใกล้ 0 → ใช้คณิตเส้นตรงปกติได้

```
   จริง:    หมุน 357°
   เดา:     หมุน 359°
   Error:   -2°   ← เล็กพอที่จะอยู่บน "ระนาบ" ตรง ๆ
```

### 🔄 Trick 2: "Iterated"

Kalman Filter ปกติ update ครั้งเดียวจบ — เหมาะกับระบบ **linear** เท่านั้น

แต่ FAST-LIVO2 มี rotation, projection — เป็น **non-linear** → update ครั้งเดียวไม่พอ

> **Analogy:** เหมือนยิงธนู — ครั้งแรกอาจพลาดเพราะคำนวณทิศลม + แรงโน้มถ่วงไม่ครบ → ดูว่าเฉไปทางไหน → ปรับ → ยิงใหม่ → ทำซ้ำจนกลางเป้า

ในโค้ด iterate ประมาณ **3 รอบ** ต่อระดับ pyramid — ก่อนหน้านี้ FAST-LIVO ต้อง 10 รอบ — version 2 ปรับให้ converge เร็วขึ้น

### 🎭 Trick 3: "Sequential" — ทำไมแบ่ง LiDAR กับ Camera?

**ปัญหา dimension mismatch:**

```
LiDAR ให้ measurement:    ~1,000 ตัว    (1 จุด = 1 residual)
Camera ให้ measurement:   ~30,000 ตัว   (8×8 patch × 4 levels × 100 patches)
```

ถ้ารวม 2 ชนิดนี้พร้อมกัน → ต้องคำนวณ **inverse ของ matrix ใหญ่ 31,000 × 31,000** → ช้ามาก

**วิธีแก้:** อาศัยข้อสมมติว่า noise ของ LiDAR กับ Camera **อิสระกัน** → fuse ทีละตัว ตามทฤษฎี Bayes ก็ได้ผลเหมือนกัน

```
   prior  ──→ LiDAR fuse ──→ posterior 1 ──→ Camera fuse ──→ ผลสุดท้าย
                                  ↑
                          (ใช้เป็น prior ของขั้นถัดไป)
```

> **Analogy:** นักสืบดูหลักฐาน A → เปลี่ยนความเชื่อ → ดูหลักฐาน B → เปลี่ยนความเชื่ออีก → ผลเหมือนดูพร้อมกัน

### 📜 4 ขั้นใน 1 cycle (ขยาย)

#### 1. Forward Propagation — เดาด้วย IMU

IMU บอกว่า "ตอนนี้กำลังเร่งไปข้างหน้า 2 m/s² และหันขวาเล็กน้อย" → ระบบเดา pose ใหม่ + เพิ่ม "ความไม่แน่นอน" (covariance)

```
   ความไม่แน่นอน:
   t=0    →   ●     ← จุดเดียว แม่นเป๊ะ
   t=0.1  →   ◯     ← วงเล็ก
   t=0.5  →   ⊃⊂   ← วงใหญ่ขึ้น (เพราะสะสม drift)
```

#### 2. Backward Propagation — แก้ point cloud บิดเบี้ยว

LiDAR scan 1 รอบใช้เวลา **100 ms** — ระหว่างนั้นคุณกำลังเดิน → จุดแรกของ scan กับจุดสุดท้ายถูก capture จากตำแหน่งต่างกัน

ถ้าเอามาวางพิกัดเดียวกัน → ผนังตรงจะดูโค้ง

```
   ก่อนแก้:                    หลังแก้:
   จุด A ─── จุด B ─── จุด C   ทุกจุดอยู่ที่เวลา t=t_end
   (ถูก scan คนละเวลา)         (ราวกับถ่ายรูป snapshot)
   → ผนัง "เบลอ"                → ผนังคมชัด
```

**Backward propagation = ใช้ IMU ย้อนเดินกลับ เพื่อ "ดึง" ทุกจุดมาที่เวลาเดียวกัน**

#### 3. LiDAR Update — ดูรายละเอียด Module 3
#### 4. Visual Update — ดูรายละเอียด Module 4

---

## Module 2 — Local Mapping (Section V)

> ส่วนที่จัดการ "**แผนที่**" — ทั้งโครงสร้าง วิธีเพิ่ม วิธีลบ

### 📦 Map Structure: Hash + Octree — "ตู้ล็อกเกอร์ในยิม"

ทำไมไม่เก็บเป็น array 3D ใหญ่ ๆ? เพราะแผนที่อาจเป็น km² → กินแรมมหาศาล

```
   Hash Map  =  สมุดโทรศัพท์ (lookup ตำแหน่ง → กล่อง ภายใน O(1))
            +
  Octree   =  ผังครอบครัว (แต่ละกล่องลูก 8 ก้อน ถ้าซับซ้อน)
```

**ขั้นตอน:**
1. ใส่จุด LiDAR เข้า hash → หา "root voxel" (0.5 m) ที่จุดตกอยู่
2. ถ้าจุดทั้งหมดในกล่องเรียงเป็นระนาบ → เก็บ **(normal n, จุดศูนย์ q)** พอ
3. ถ้าไม่เป็นระนาบ → ซอย 8 ก้อน (octree) ลงไปต่อ
4. ทำซ้ำได้ลึกสุด 3 ชั้น (ขนาดเล็กสุด ~0.06 m)

```
   ┌─┬─┬─┬─┬─┐
   ├─┼─┼─┼─┼─┤      ← root voxel ขนาด 0.5 m เรียงเต็มห้อง
   ├─┼─┼─┼─┼─┤
   └─┴─┴─┴─┴─┘
        ▼ บางกล่องที่ซับซ้อน:
   ┌─────┐
   │ ┌─┬─┐│      ← ซอยเป็น 8 octant (ในรูป 2D ดูเป็น 4)
   │ ├─┼─┤│
   │ └─┴─┘│
   └─────┘
```

> **ทำไมต้องเก็บเป็นระนาบ?** เพราะระนาบให้ทิศทาง (normal) ซึ่งใช้คำนวณว่าจุดใหม่ตรงตามแนวระนาบเดิมไหม + ประหยัดที่เก็บ (ผนัง 1 ผืน = 1 ระนาบ ไม่ใช่ล้านจุด)

### 🎯 Plane Fitting — ใช้ SVD ดูการกระจายของจุด

> **Analogy:** จุดในกล่องเปรียบเหมือน "ฝุ่นที่ลอยอยู่"
>
> - ฝุ่นกระจายเป็น **แผ่นแบน** → มี 1 ทิศที่กระจายน้อย = ตั้งฉากกับแผ่น = **normal**
> - ฝุ่นกระจายเป็น **เส้นตรง** → มี 2 ทิศกระจายน้อย → ไม่ใช่ระนาบ
> - ฝุ่นกระจาย **ทุกทิศเท่ากัน** → ไม่ใช่ระนาบ

```
        แผ่นแบน                        ลูกบอล
    ┌──────────┐                    ┌──────────┐
    │ • • • • •│  ← กระจายเฉพาะ      │  • ・ • ・ │  ← กระจายทุกทิศ
    │ • • • • •│    แกน X, Y          │  ・ • • • │
    │ • • • • •│    น้อยมากที่ Z      │  • • ・ • │
    └──────────┘                    └──────────┘
    
    eigenvalue เล็กสุด = ทิศ Z         eigenvalues ทั้ง 3 ใกล้กัน
    → normal = แกน Z                   → ไม่เป็นระนาบ
```

### 🎨 Visual Map Points — บางจุดได้เป็น "ดารา"

ในแต่ละ frame กล้อง บางจุด LiDAR จะถูกเลือกเป็น "**visual map point**" → แนบ patch ภาพไว้ด้วย

**เกณฑ์:**
1. มองเห็นจาก camera FoV ✓
2. **ลวดลายเด่น** (image gradient สูง — มุม ขอบ pattern ชัด) ✓

> **Analogy:** เหมือนเลือก "landmark" ที่ใช้บอกทาง — เราไม่จำผนังขาว ๆ แต่จำป้ายสี ๆ มุมห้อง รอยร้าวที่เด่น เพราะใช้ระบุตำแหน่งดีกว่า

### 🔄 Reference Patch — เลือก patch "ที่ดีที่สุด"

จุดเดียวกันอาจถูกถ่ายจากหลาย frame → เก็บ patch หลายอันได้
**คำถาม:** ใช้ patch ไหนเป็น "reference" สำหรับเทียบ?

**เกณฑ์ 2 ข้อ:**
1. **คล้าย patches อื่น ๆ** (NCC สูง) → กรองสิ่งของเคลื่อนไหว เช่น คน รถ
2. **มองตรง ๆ** (view direction ขนานกับ normal ของระนาบ) → เห็นลวดลายชัด

> **Analogy:** เหมือนเลือกรูปประจำตัวจาก album:
> - ❌ ไม่เอารูปเบลอ (สิ่งของเคลื่อนไหว)
> - ❌ ไม่เอารูปถ่ายเฉียง ๆ (ลวดลายไม่ชัด)
> - ✅ เอารูปคล้ายรูปอื่นในชุด + มองตรง ๆ ที่สุด

### 🚪 Local Map Slide — กันแผนที่โต

เก็บแค่ **กล่องรอบ ๆ ตัวเรา** (รัศมี ~50 m) — ที่อยู่ไกลกว่าลบทิ้ง

```
   เดินไปข้างหน้า →
   [........[##live area##]........]
              ←←  เลื่อน  →→
   [..[##live area##]..............]    ← พื้นที่หลัง = ลบ
```

> **Analogy:** เหมือน RAM ของเกม open-world — โหลดแผนที่รอบตัว พื้นที่ห่างไกลล้าง

---

## Module 3 — LiDAR Measurement Model (Section VI)

> ส่วนที่บอกว่า "**LiDAR เห็นอะไร และจะใช้มัน update pose ยังไง**"

### 🎯 Point-to-Plane — ทำไมไม่ใช้ point-to-point?

| | Point-to-Point (ICP คลาสสิก) | **Point-to-Plane (FAST-LIVO2)** |
|--|------------------------------|--------------------------------|
| **คำถาม** | จุดใหม่ใกล้จุดเก่าไหม? | จุดใหม่อยู่บนระนาบเก่าไหม? |
| **Residual** | ระยะระหว่าง 2 จุด | ระยะตั้งฉากจากจุดถึงระนาบ |
| **ปัญหา** | scan ใหม่อาจ "ไม่ยิงโดน" จุดเดียวกันเป๊ะ | – |
| **ข้อดี** | ง่าย | ทนต่อ scan pattern ที่ไม่ตรงกัน 100% |

```
   Point-to-Point: ต้องหา "จุดคู่"            Point-to-Plane: เทียบกับระนาบ
   
        ผนัง   ●  ← จุดเก่า                       ผนัง  ════════════════
              ↗                                          ↑
            ↗ ระยะคู่จุด                              dist (ตั้งฉาก)
          ●  ← จุดใหม่ (ไม่ตรงกับเก่า)                  ●  ← จุดใหม่
                                                  
        ปัญหา: scan ใหม่ไม่ยิงโดน             ดีกว่า: ผนังเป็นระนาบ
        จุดเก่าตรง ๆ                          จุดใหม่ "บนผนังที่ไหนก็ได้"
```

### 📐 Residual ทำงานยังไง

```
   pose ที่เดาไว้ (อาจผิดนิดหน่อย)
        ↓
   เอาจุด LiDAR ใหม่ 1 จุด → แปลงเข้า world frame
        ↓
   หา voxel ที่จุดนี้ตกอยู่ → ดึงระนาบ (normal n + จุดศูนย์ q)
        ↓
   วัด "ระยะตั้งฉากจากจุดถึงระนาบ" = residual r
        ↓
   ถ้า r = 0 → pose ถูก ✓
   ถ้า r ≠ 0 → pose ผิด → ใช้ r บอก Kalman filter ว่า "ปรับ pose ให้ r ลด"
```

ทำกับจุดทั้งพันจุด → Kalman filter หา pose ที่ลด r รวมน้อยสุด → iterate จนนิ่ง

### 🌫️ Noise Model — เลเซอร์ไม่ได้แม่นทุกจุดเท่ากัน

มี **3 ชนิด noise**:

```
  1. Range noise (δd)        — error ของระยะ ToF
  2. Bearing noise (δω)      — error ของทิศ encoder
  3. Beam divergence (θ)     — เลเซอร์ไม่ใช่เส้นเดี่ยว — มันแผ่เป็นกรวย!
```

**Beam divergence — ของใหม่ใน FAST-LIVO2:**

```
        LiDAR
          ●
         /│\
        / │ \    ← beam แผ่ออกเป็นกรวย มุม θ (~0.15°)
       /  │  \
      ─────┴─────  ← ผนัง
       ↑   ↑   ↑
       ผลตรง: ระยะที่วัดได้ "เฉลี่ย" จากบริเวณบนผนัง
```

- **กับผนัง** (ตั้งฉาก, มุม φ ใกล้ 0): กระทบเป็น **วงกลมเล็ก** → noise ต่ำ ✓
- **กับพื้น** (มุมเฉียง φ ใกล้ 90°): กระทบเป็น **วงรียาวมาก** → noise สูงมาก ⚠️

```
    ผนัง (ตั้งฉาก)                  พื้น (มุมเฉียง)
        │                               ─────────
        │   ●  (กระทบเล็ก)            ____●____
        │                              ↑________↑
                                          กระทบเป็นแถบยาว
        noise เล็ก                      noise ใหญ่
```

→ FAST-LIVO2 ปรับ **น้ำหนัก** ของแต่ละจุดตามมุมตกกระทบ — จุดบนพื้นได้รับน้ำหนักน้อยลงในการ update

---

## Module 4 — Visual Measurement Model (Section VII)

> ส่วนที่บอกว่า "**Camera เห็นอะไร และจะใช้มัน update pose ยังไง**"

แบ่งเป็น 2 ส่วนใหญ่:
- **(A) เลือก visual map points ที่จะใช้**
- **(B) คำนวณ residual จาก photometric error**

### 🔍 Part A — เลือก visual map points 3 ขั้น

#### ขั้น 1: Visible Voxel Query

**ปัญหา:** map ใหญ่มี visual map points หลายแสน แต่ในเฟรมปัจจุบันเห็นแค่ไม่กี่ร้อย → เช็คทุกจุดสิ้นเปลือง

**ไอเดีย:** ใช้ **scan LiDAR ของรอบนี้** เป็น "เครื่องมือ probe"

```
   ┌────────┬────────┬────────┬────────┐
   │  voxel │  voxel │  voxel │  voxel │  ← LiDAR ยิงโดน 4 voxel
   │  ●●●   │  ●●    │  ●●●   │  ●●●●  │     → query เฉพาะ 4 ตัวนี้
   └────────┴────────┴────────┴────────┘
                ↑
              camera FoV
```

#### ขั้น 2: On-Demand Raycasting

**ปัญหาที่เหลือ:** บางครั้ง camera FoV กว้างกว่า LiDAR → บางส่วนของภาพไม่มี visual map points

**ไอเดีย:** ตำแหน่งภาพที่ "ว่าง" — ยิง ray virtual จาก camera center → เจออะไรก่อนเอาตัวนั้น

```
   ภาพ (แบ่ง 30×30 px grid):
   ┌─────┬─────┬─────┬─────┐
   │ ●   │ ●   │     │ ●   │  ← cell ว่าง
   ├─────┼─────┼─────┼─────┤
   │ ●   │     │ ●   │ ●   │  ← cell ว่าง
   └─────┴─────┴─────┴─────┘
                ↓
            ยิง ray ออกจาก camera center
            ────────●────────●─── ●  ← เจออะไรบ้าง?
            d_min  เลือกตัวแรก    d_max
```

> **Analogy:** เหมือนหาของในห้องมืด — บางที่ที่ไฟส่องไม่ถึง → ใช้ไม้แหย่ดูว่าโดนอะไร

#### ขั้น 3: Outlier Rejection

ที่เหลือต้องคัด **จุดเสีย** ออก:

```
   ❌ Occluded            (มีของบังด้านหน้า)
   ❌ Depth-discontinuous (อยู่ขอบของวัตถุ — เห็นลวดลายไม่ครบ)
   ❌ Large view angle    (เห็นจากมุมเฉียงมาก > 60°)
```

**Occluded ตรวจยังไง?**

```
   camera ─→ ●─────●─────●(จริง ๆ ถูกบัง)
          pt1   pt2    pt3
                       ↑
   เปรียบ depth ของ pt3 กับเพื่อนรอบ ๆ 9×9 pixel
   ถ้าต่างกันมาก (> 50 cm) → pt3 อาจถูกบัง → ทิ้ง
```

### 🖼️ Part B — Photometric Error

#### Affine Warping — ทำไมต้อง warp ก่อนเทียบ?

**ปัญหา:** patch ที่เก็บไว้ใน map ถ่ายจาก **มุมเก่า** — ตอนนี้มุมต่างไป → ลวดลายยืด/หด/หมุน

> **Analogy:** ถ่ายผนังอิฐจากระยะใกล้ vs ไกล vs เฉียง → อิฐในภาพดูต่างกันมาก แต่จริง ๆ คือผนังเดียวกัน

```
   patch ตอนเก็บ (ถ่ายตรง ๆ)        patch ตอนนี้ (ถ่ายเฉียง)
        ┌─────────┐                    ┌──────────┐
        │ ▓▓▓ ▓▓▓ │     ตอนเทียบ       │ ▓▓ ▓▓ │     ← ดูแคบกว่า
        │ ▓░▓ ▓░▓ │  ← ต้อง warp ก่อน → │ ▓░ ▓░ │       เพราะมุมเฉียง
        │ ▓▓▓ ▓▓▓ │                    │▓▓ ▓▓  │
        └─────────┘                    └──────────┘
```

**Affine warping** = แปลง patch เก่าให้ "ดูเหมือนถ่ายจากมุมปัจจุบัน" ก่อนเทียบ

ใช้ **plane normal** จาก map (ที่ LiDAR ให้) → คำนวณ warp matrix ได้

> นี่คือเหตุผลที่ FAST-LIVO2 รวม LiDAR + camera ดีกว่าใช้กล้องอย่างเดียว — เพราะ LiDAR ให้ "**plane prior**" ฟรี ทำให้ warp ทำงานได้แม่น

#### Sparse Direct — ไม่ extract features

วิธี SLAM แบบเดิม (เช่น ORB-SLAM):
```
   ภาพ → extract ORB → match descriptors → solve pose
   (ช้า ใช้ข้อมูลทางอ้อม)
```

วิธี **Direct** ของ FAST-LIVO2:
```
   ภาพ → เลือก patch → warp + exposure → minimize |ภาพ - patch|²
   (เร็ว ใช้ raw pixel intensity ตรง ๆ)
```

**คำว่า "Sparse"** = ไม่ใช้ทุก pixel ของภาพ — ใช้แค่ patch รอบ ๆ visual map points → เร็วกว่า "dense direct" (เช่น DSO ที่ใช้ทุก pixel)

#### Photometric Error คำนวณยังไง

```
   patch อ้างอิง (จาก map)        patch ปัจจุบัน
       ┌─────────┐                    ┌─────────┐
       │ 100 120 │                    │ 95 118  │   ← pixel intensity
       │ 80  150 │   ← เทียบ →         │ 78 145  │
       │ 200 90  │                    │ 195 92  │
       └─────────┘                    └─────────┘
       
       residual = sum(|ปัจจุบัน - อ้างอิง|²)
                = (95-100)² + (118-120)² + ...
                
       → Kalman filter ปรับ pose จน residual ต่ำสุด
```

#### Pyramid 4 ระดับ — ทำไมต้องหลายชั้น

ถ้าเริ่มเทียบที่ภาพละเอียด → ถ้า pose เริ่มผิดมาก → **ติด local minimum** (ภาพไม่ overlap → คำนวณ gradient ผิดทาง)

**วิธีแก้:** เริ่มจากภาพ **หยาบ** ก่อน → หา pose คร่าว ๆ → ค่อย refine ละเอียดขึ้น

```
   Pyramid 4 ระดับ (coarse → fine):
   
   Level 3 (หยาบที่สุด)  →  Level 2  →  Level 1  →  Level 0 (ละเอียดสุด)
       8×8                  16×16        32×32         64×64
   เริ่มที่นี่                                         จบที่นี่
   pose เคลื่อนได้เยอะ                                 refine สุดท้าย
```

> **Analogy:** เหมือนปรับรูปในห้อง darkroom — เริ่มจากเงาคร่าว ๆ ค่อย ๆ ปรับให้คมขึ้นทีละนิด

#### Inverse Compositional — Trick เร่ง

**ปัญหา:** ทุก iteration ต้องคำนวณ Jacobian ใหม่ — ช้ามาก

**Trick:** แทนที่จะปรับ pose ของ **current frame** → ปรับ pose ของ **reference frame ในจินตนาการ**

ผลคือ Jacobian คำนวณจาก patch อ้างอิง (ไม่เปลี่ยน) → คำนวณ **ครั้งเดียว** ใช้ได้ทุก iteration → เร็วขึ้น 3×

#### Exposure Time Estimation — ทนต่อแสงแปร ๆ

**ปัญหา:** เดินจากในร่ม → กลางแดด → ภาพสว่างขึ้น แม้ลวดลายเดิม

```
   ในร่ม:                 กลางแดด:
   ┌─────┐                ┌─────┐
   │ 80  │                │ 200 │  ← intensity เพิ่ม 2.5×
   │ 100 │                │ 250 │     แม้เห็นผนังเดียวกัน
   └─────┘                └─────┘
```

ถ้าไม่ทำอะไร → ระบบจะคิดว่า pose ผิด

**วิธีแก้:** เก็บ **inverse exposure time τ** เข้าไปใน state ด้วย → ตอนเทียบ:

```
   τ_ปัจจุบัน × (intensity ปัจจุบัน)  vs  τ_อ้างอิง × (intensity อ้างอิง)
                       ↓
                ถ้า τ ถูก → ค่าจะใกล้เคียงกัน
```

→ Kalman filter ปรับทั้ง pose และ τ ไปพร้อมกัน → ทนแสงแปร ๆ

---

# Part 5 — เปรียบเทียบกับวิธีอื่น

นี่เป็นส่วนที่สำคัญมาก — เพราะถ้าเข้าใจว่า FAST-LIVO2 ต่างจากตัวอื่นยังไง จะรู้ว่าเมื่อไหร่ควรใช้

## 🥊 ภาพรวมการแข่งขันใน arena SLAM

```
                       Sensors used
                  ┌─────────────────────────┐
                  │  LiDAR  IMU  Camera     │
   ───────────────┼─────────────────────────┤
   FAST-LIO2      │  ✅    ✅    ❌         │  LIO only
   FAST-LIVO      │  ✅    ✅    ✅         │  LIVO รุ่นแรก  
   FAST-LIVO2     │  ✅    ✅    ✅         │  ★ ตัวที่เราอ่าน
   R3LIVE         │  ✅    ✅    ✅         │  LIV ดัง
   LVI-SAM        │  ✅    ✅    ✅         │  factor graph
   ORB-SLAM3      │  ❌    ✅    ✅         │  VIO only
   VINS-Mono      │  ❌    ✅    ✅         │  VIO only
```

## 1️⃣ FAST-LIO2 vs FAST-LIVO2 — ต้นตระกูล vs รุ่นใหม่

**FAST-LIO2** (HKU MaRS Lab, 2022) คือ **predecessor ที่ไม่มีกล้อง**

| ประเด็น | FAST-LIO2 | FAST-LIVO2 |
|---------|-----------|-------------|
| Sensors | LiDAR + IMU | LiDAR + IMU + **Camera** |
| Map structure | ikd-Tree (k-d tree เพิ่มจุดได้) | Voxel hash + Octree + planes |
| Filter | ESIKF | ESIKF + **sequential update** |
| Visual module | – | **Sparse direct + plane prior** |
| RMSE บน Hilti'22 | ~0.097 m | **0.044 m** (ดีขึ้น 2×) |

### 🎯 จุดแข็งของ FAST-LIVO2 ที่ FAST-LIO2 ไม่มี

> **Analogy:** FAST-LIO2 เหมือนคนตาบอดที่ใช้ไม้เท้าเก่ง — ในห้องที่มีกำแพงและของยังเดินได้ดี
>
> แต่ในทางเดินผนังขาวยาว ๆ (LiDAR degeneration) ไม้เท้าก็ "หลง" — เพราะไม่ว่าจะเลื่อนไปทางไหน ก็ยังโดนผนังเดียวกัน
>
> **FAST-LIVO2 = คนตาบอดที่กลับมามองเห็นได้บางที** — แม้ไม้เท้าหลง ก็ยังเห็นป้าย/ลวดลายผนังเป็นจุดอ้างอิง

### 📊 ฉากที่ FAST-LIVO2 ชนะชัดเจน

จากการทดลองใน paper:
- **HIT Graffiti Wall** (เดิน 800 m เลียบกำแพง gradient ต่ำ) — FAST-LIO2 ล้ม, FAST-LIVO2 รอด
- **Cupola, Construction Stairs** — FAST-LIVO2 ดีกว่า ~5×
- **Mining Tunnel** (มืดสนิท) — FAST-LIO2 ก็ยังทำได้ FAST-LIVO2 ทำได้เท่ากันเพราะกล้องดับ

### 📉 ฉากที่ FAST-LIO2 อาจเทียบเท่า

- **ในที่ texture น้อย + แสงน้อย** (เช่น Mining Tunnel) — กล้องไม่มีประโยชน์ → 2 ตัวคงเท่า ๆ กัน
- **ในที่มี LiDAR points เยอะ + ไม่ degenerate** — กล้องช่วยน้อย

---

## 2️⃣ FAST-LIVO vs FAST-LIVO2 — รุ่นเก่า vs รุ่นใหม่

**FAST-LIVO (รุ่นแรก)** ก็มีกล้องแล้ว แต่ FAST-LIVO2 แก้ปัญหาเดิมของรุ่นแรก:

| ปัญหาใน FAST-LIVO | วิธีแก้ใน FAST-LIVO2 |
|--------------------|----------------------|
| **Wild assumption:** "ทุก pixel ใน patch อยู่ที่ depth เดียวกัน" → warping ผิดเมื่อมุมเอียง | ใช้ **plane prior** จาก LiDAR → warp ตามระนาบจริง |
| Reference patch เลือกจาก patch ที่ใกล้ frame ปัจจุบัน → คุณภาพต่ำ | เลือก patch ที่ **คล้าย patches อื่น + มองตรง ๆ** ที่สุด |
| ไม่มี exposure estimation → ตายเมื่อแสงแปร | **Online exposure estimation** เก็บ τ เข้า state |
| ไม่มี raycasting → ในที่ LiDAR ใกล้วัตถุเกินไป มีพื้นที่ไม่มี map points | **On-demand raycasting** เติมจุด |
| ใช้ standard ESIKF (รวม LiDAR + image พร้อมกัน) → ช้า | **Sequential update** — fuse ทีละ sensor |

### 🎯 ผลของการปรับปรุง

> ตามตาราง Run Time ใน paper: FAST-LIVO2 ใช้ **30 ms/frame** บน Intel i7 — **iterate ลดจาก 10 รอบเหลือ 3 รอบ** ต่อ pyramid level

---

## 3️⃣ R3LIVE vs FAST-LIVO2 — คู่แข่งที่ใกล้ที่สุด

**R3LIVE** (Lin et al., 2022) ก็ใช้ LiDAR + IMU + Camera แบบ tightly-coupled

| ประเด็น | R3LIVE | FAST-LIVO2 |
|---------|--------|------------|
| Map structure | **Map แยก 2 อัน** (LiDAR map + radiance map) | **Map เดียว** ใช้ร่วม |
| Visual module | **Pixel-level** alignment (ทุก pixel) | **Patch-level** (8×8 รอบ feature point) |
| Visual processing | Dense — ช้า | **Sparse Direct** — เร็ว |
| ต้องมี frame-to-frame tracking? | ✅ ใช้ optical flow ตามคู่เฟรม | ❌ ตรง frame-to-map |
| Bayesian map update | ✅ มี (เพิ่มเวลาเมื่อ map โต) | ❌ ไม่มี (เร็วคงที่) |
| Photometric calibration | ✅ ต้อง pre-calibrate ก่อน | ❌ estimate online ระหว่างรัน |
| RMSE บน Hilti'22 | ~0.278 m | **0.044 m** (~6× ดีกว่า) |

### 🎯 ความต่างสำคัญ

> **Analogy:**
> - **R3LIVE** = นักวาดที่ผสมสีบนจาน (radiance map) แล้วลงผ้า — ละเอียดมาก แต่ใช้เวลา
> - **FAST-LIVO2** = นักวาดที่ใช้ stencil (patch) ลงตำแหน่งสำคัญ ๆ — เร็วกว่า แต่ต้องเลือก stencil ดี ๆ

**ข้อดีของ patch-level (FAST-LIVO2):**
- เร็วกว่า ~3-4× ในเวลาประมวลผล
- patch อยู่ที่ resolution เต็ม (8×8 px) แม่นกว่า R3LIVE ที่ผูกกับ resolution ของ point cloud

**ข้อดีที่ R3LIVE มีแต่ FAST-LIVO2 ไม่มี:**
- Colored point cloud ที่สีถูกต้องตามแสงจริง (radiance map)
- ใช้ทุก pixel → ในที่มี texture น้อย ๆ อาจดีกว่า

---

## 4️⃣ LVI-SAM vs FAST-LIVO2 — ESIKF vs Factor Graph

**LVI-SAM** (Shan et al., 2021) ใช้ approach ต่างไปคนละสาย — **factor graph** + **smoothing**

| ประเด็น | LVI-SAM | FAST-LIVO2 |
|---------|---------|------------|
| State estimation | **Factor graph + iSAM2** (smoothing) | **ESIKF** (filtering) |
| Use feature? | **ORB feature-based** | **Direct (no features)** |
| Coupling level | Loose ใน vision (frame-to-frame ก่อน) | Tight (frame-to-map ตรง ๆ) |
| Processing latency | สูงกว่า (factor graph optimize) | ต่ำกว่า (filter step เดียว) |
| Map | Pose graph + LiDAR map | Voxel + visual map points |
| Robust ต่อ texture น้อย? | ❌ ต้องมี features | ✅ ใช้ raw pixel ได้ |
| RMSE บน Hilti'22 | ~1.928 m (fail บน 9 sequences) | **0.044 m** |

### 🎯 ความต่าง

> **Analogy:**
> - **LVI-SAM** = นักการเงินที่บันทึก ledger ยาว ๆ ทุกธุรกรรม แล้ว reconcile ทั้งระบบทุกชั่วโมง — ละเอียด แต่ช้า
> - **FAST-LIVO2** = นักการเงินที่เก็บแค่ "ยอดล่าสุด + ความเชื่อมั่น" — เร็ว ใช้พื้นที่น้อย แต่ไม่ revisit ประวัติ

**Factor graph (LVI-SAM) ดีตรงไหน:** ถ้าเจอ loop closure → revise ทั้ง trajectory ได้
**ESIKF (FAST-LIVO2) ดีตรงไหน:** เร็วกว่ามาก, real-time guarantee ดีกว่า, ใช้ memory คงที่

---

## 5️⃣ ORB-SLAM3 / VINS-Mono vs FAST-LIVO2 — Visual-Inertial เพียว ๆ

ตัวเหล่านี้ **ไม่มี LiDAR** — เป็น VIO (Visual-Inertial Odometry)

### ปัญหาของ VIO ล้วน ๆ

| ปัญหา | สาเหตุ |
|-------|--------|
| **Scale drift** | กล้องเดียวไม่รู้สเกล — IMU pre-integration ช่วยได้ตอนเริ่ม แต่ drift เร็ว |
| **Depth ambiguity** | ต้อง triangulate จาก 2 frame → noise สะสม |
| **Texture-less environments** | เช่น ผนังสีเดียว → fail |
| **Dynamic objects** | คน/รถเคลื่อนไหว → matching ผิด |

### FAST-LIVO2 แก้ทุกข้อโดย "ดูด LiDAR เข้ามาช่วย"

```
   VIO เพียว ๆ:               FAST-LIVO2:
   - Scale ไม่แน่ ⚠️           - LiDAR ให้สเกลแม่น ✅
   - Depth ต้อง triangulate    - LiDAR ให้ depth ตรง ✅
   - Texture-less = fail       - LiDAR ไม่สนใจ texture ✅
   - Dynamic objects = noise   - LiDAR + IMU เป็น sanity check ✅
```

> **เปรียบ:** VIO = ตาเปล่า; FAST-LIVO2 = ตาเปล่า + ไม้เท้า + GPS แอร์โร ภายในร่างเดียวกัน

---

## 📋 ตารางสรุปทั้งหมด

| Method | Sensors | Approach | Strengths | Weaknesses |
|--------|---------|----------|-----------|------------|
| **FAST-LIO2** | L+I | ESIKF, ikd-Tree | เร็ว, แม่น | ❌ ไม่ได้กล้อง — fail ใน LiDAR-degeneration |
| **FAST-LIVO** | L+I+C | ESIKF + frame-to-frame | เพิ่ม robustness | ❌ wild patch assumption, ❌ ไม่มี exposure |
| **★ FAST-LIVO2** | L+I+C | **ESIKF + sequential** | แม่นที่สุด, robust ที่สุด, real-time | ปรับ tuning ยุ่งหน่อย |
| **R3LIVE** | L+I+C | Pixel-level alignment | Colored map สวย | ❌ ช้า เมื่อ map โต, ❌ ต้อง pre-calibrate |
| **LVI-SAM** | L+I+C | Factor graph + ORB | ดีถ้ามี loop closure | ❌ feature-based fail ใน texture-less |
| **ORB-SLAM3** | I+C | Bundle adjustment | สวย ถ้ามี features | ❌ ไม่มี LiDAR, fail texture-less |
| **VINS-Mono** | I+C | Sliding window opt. | เบา, real-time | ❌ scale drift, ❌ ไม่ดี indoor |

---

# Part 6 — เมื่อไหร่ FAST-LIVO2 ชนะ และเมื่อไหร่ไม่ชนะ

## ✅ เมื่อ FAST-LIVO2 จะชนะแบบเด็ดขาด

### 1. **LiDAR Degeneration + Camera ใช้ได้**

ฉากเช่น:
- ทางเดินผนังเรียบยาว (corridor)
- Mining tunnel ที่มีลวดลาย
- ทางเดินรถไฟใต้ดิน

> Camera ช่วย LiDAR เห็น "มุมการเคลื่อนที่" ตามแนวยาวที่ LiDAR หลง

### 2. **Visual Challenge + LiDAR ใช้ได้**

ฉากเช่น:
- มืด → สว่าง สลับเร็ว (cross indoor/outdoor)
- มีของเคลื่อนไหวเยอะ
- Texture-less ในบางส่วน

> LiDAR เป็น sanity check ให้กล้อง — ป้องกันไม่ให้ track สิ่งของเคลื่อนไหว

### 3. **Real-time + Embedded**

> 30 ms/frame บน Intel i7, 78 ms บน ARM Kryo585 — เหมาะกับ UAV onboard

## ❌ เมื่อ FAST-LIVO2 อาจไม่ดีกว่า

### 1. **เซ็นเซอร์ไม่ครบ**

ถ้าระบบมีแค่ LiDAR+IMU (ไม่มีกล้อง) → FAST-LIO2 น่าจะเทียบเท่า + เร็วกว่า

### 2. **Loop closure จำเป็น**

FAST-LIVO2 เป็น **odometry** เพียว ๆ — ไม่มี loop closure
ถ้าเดินวงใหญ่ ๆ แล้วต้องการแก้ drift → ต้อง integrate กับ pose graph backend หรือใช้ LVI-SAM แทน

### 3. **Fully Dynamic Scene**

เช่นในถนนที่มีรถ + คน + จักรยานเต็มไปหมด — ทั้ง LiDAR และ Camera เห็นแต่ของเคลื่อนไหว → SLAM ทุกตัวก็ลำบาก แต่ FAST-LIVO2 ไม่ได้ออกแบบมาให้ filter dynamic ดีเป็นพิเศษ

### 4. **Hardware Calibration ไม่ดี**

FAST-LIVO2 ต้องการ extrinsic LiDAR↔IMU↔Camera **แม่น** — ถ้า calibrate ไม่ดี (หลายเซ็นต์) → ผลแย่กว่าใช้ sensor เดียว

---

# Part 7 — สรุปและแหล่งข้อมูลเพิ่ม

## 🎯 5 ข้อสำคัญที่ควรจำ

1. **SLAM** = ตอบ "อยู่ที่ไหน + รอบ ๆ มีอะไร" พร้อมกัน
2. **FAST-LIVO2** รวม 3 sensors แบบแน่น ๆ ใน Kalman Filter ตัวเดียว ใช้ map ร่วมกัน
3. **Map** = ตู้ล็อกเกอร์ 0.5 m เก็บระนาบ + บางตู้แนบลวดลายภาพ
4. **Update pose** ด้วย "เดา → เช็ค → ปรับ" — หลักฐาน 2 ชนิด: จุด LiDAR ต้องตรงระนาบ + ลวดลายภาพต้องเหมือน patch
5. **ดีกว่ารุ่นเก่า/คู่แข่ง** เพราะ: sequential update, plane prior, exposure estimation, raycasting, sparse direct

## 📐 ในงานวิจัยของคุณ (camera on/off)

ข้อมูลจาก paper Section IX-B ablation:
- **เปิด camera (ปกติ)** → RMSE 0.044 m
- **ปิด exposure estimation** → RMSE 0.050 m (แย่ลง 6 mm)
- **ปิด normal refinement** → RMSE 0.045 m (แย่ลง 1 mm)

→ ใน UrbanNav (urban canyon) คุณน่าจะเจอ:
- ✅ ฉาก urban texture-rich → camera ช่วย
- ✅ ทางเดียวยาว ๆ → camera ช่วย LiDAR degeneration
- ⚠️ ฉากแสงแปร ๆ (เข้า-ออกอุโมงค์) → exposure estimation สำคัญ

แนะนำลองทดลอง:
1. `img_en=1` + default → baseline
2. `img_en=0` (ปิด camera) → เห็นผลของ LiDAR-only
3. `img_en=1` + ปิด exposure → เห็นว่าฉากแสงแปร ๆ พังหรือไม่

## 📚 อ่านต่อ

| Resource | จุดประสงค์ |
|----------|------------|
| [Paper FAST-LIVO2](https://arxiv.org/abs/2408.14035) | ของจริง — เปิดประกอบเอกสารนี้ |
| [Supplementary](FAST-LIVO2/Supplementary/LIVO2_supplementary.pdf) | Ablation studies เต็ม |
| [FAST-LIVO2_explained.md](FAST-LIVO2_explained.md) | เอกสารเดิม — เน้นเทียบ paper กับ code |
| [GitHub FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2) | source code, issues |
| [FAST-LIO2 paper](https://arxiv.org/abs/2107.06829) | ตัว predecessor (LiDAR-IMU เพียว ๆ) |
| [R3LIVE paper](https://arxiv.org/abs/2109.07982) | ตัวคู่แข่งหลัก |

## 🤔 คำถามสะสมที่อาจสนใจ

1. "Kalman gain (K) ตัดสินใจเชื่อ predict กับ update เท่าไรยังไง?"
2. "Affine warping ทำงาน step-by-step ยังไง?"
3. "ทำไมต้องมี IMU ทั้งที่ LiDAR ก็แม่น?"
4. "ในการทดลองของผม — ฉากแบบไหนที่ camera ช่วย และฉากแบบไหนไม่ช่วย?"
5. "Voxel size 0.5 m เลือกอย่างไร เปลี่ยนได้ไหม?"

ถามต่อได้เลยครับ — ตอบให้ภาพชัดทีละข้อ 📚
