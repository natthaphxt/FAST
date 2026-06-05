# FAST-LIVO2 อธิบายแบบเทียบ Paper กับ Code

> เอกสารฉบับนี้สรุปหลักการของ **FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry** (Zheng et al., 2024 — [arXiv:2408.14035v2](https://arxiv.org/abs/2408.14035)) โดยจับคู่สมการในเปเปอร์กับโค้ดต้นฉบับใน [FAST-LIVO2/](FAST-LIVO2/) ที่อยู่ใน workspace นี้
>
> ใช้สำหรับเปิดประกอบการอ่าน paper หน้า 4–11 (Sections IV–VII) — กดที่ลิงก์ `file.cpp:LINE` ใน VSCode เพื่อกระโดดเข้าโค้ดได้ทันที
-9
---

## สารบัญ

- [0. ภาพรวมระบบ](#0-ภาพรวมระบบ-system-overview)
- [1. State Vector & Manifold (IV-A)](#1-state-vector--manifold-iv-a)
- [2. Forward & Backward Propagation (IV-C)](#2-forward--backward-propagation-iv-c)
- [3. Sequential Update Algorithm (IV-D, Algorithm 1)](#3-sequential-update-algorithm-iv-d-algorithm-1)
- [4. Local Mapping – Voxel Map (V)](#4-local-mapping--voxel-map-v)
- [5. LiDAR Measurement Model (VI)](#5-lidar-measurement-model-vi)
- [6. Visual Measurement Model (VII)](#6-visual-measurement-model-vii)
- [7. สรุปจุดที่ Code ไม่ตรงกับ Paper](#7-สรุปจุดที่-code-ไม่ตรงกับ-paper)
- [8. ลำดับอ่านโค้ดสำหรับมือใหม่](#8-ลำดับอ่านโค้ดสำหรับมือใหม่)

---

## 0. ภาพรวมระบบ (System Overview)

FAST-LIVO2 เป็นระบบ **odometry แบบ tightly-coupled** ที่หลอม 3 sensors เข้าด้วยกันใน Error-State Iterated Kalman Filter (ESIKF) ตัวเดียว โดยใช้ **map ก้อนเดียว** (voxel map) ทั้งกับ LiDAR และ Visual

### 0.1 องค์ประกอบ 4 ส่วน (ตาม Fig. 2 ของ paper)

```
        +---------+              +-----------------+
  IMU --| Forward |---+----------| Backward Prop.  |--+
        |  Prop.  |   |          | (motion comp.)  |  |
        +---------+   |          +-----------------+  |
                      |                               |
                      v                               v
                +-----------+      +-----------------------+
                |   State   |<---  | LiDAR Update (ESIKF)  | <-- LiDAR
                |    x_k    |      |  point-to-plane res.  |
                |  19-dim   |      +-----------------------+
                |    P_k    |                ^
                |           |                | retrieve plane (q,n,Σ)
                |           |      +-----------------------+
                |           |<---  | Visual Update (ESIKF) | <-- Camera
                |           |      |  sparse photometric   |
                +-----------+      |  + affine warp + τ    |
                      |            +-----------------------+
                      v                       ^
              +---------------+               |
              |  Voxel Map    |---------------+
              |  hash + octree|
              |  + planes     |
              |  + visual map |
              |    points     |
              +---------------+
```

**4 modules ที่ paper จัดเป็น Sections:**

| Section | หัวข้อ | ตำแหน่งในโค้ด |
|---------|--------|----------------|
| IV | ESIKF (sequential update) | [LIVMapper.cpp](FAST-LIVO2/src/LIVMapper.cpp), [IMU_Processing.cpp](FAST-LIVO2/src/IMU_Processing.cpp) |
| V | Local Mapping (voxel + visual map points) | [voxel_map.cpp](FAST-LIVO2/src/voxel_map.cpp), [vio.cpp](FAST-LIVO2/src/vio.cpp) (ส่วน map-point gen) |
| VI | LiDAR Measurement Model | [voxel_map.cpp](FAST-LIVO2/src/voxel_map.cpp) (`build_single_residual`) |
| VII | Visual Measurement Model | [vio.cpp](FAST-LIVO2/src/vio.cpp) (`updateState`, `processFrame`) |

### 0.2 Main loop

จุดเริ่มของทุก cycle อยู่ที่ [`LIVMapper::run()`](FAST-LIVO2/src/LIVMapper.cpp#L603-L622):

```cpp
while (rclcpp::ok()) {
    rclcpp::spin_some(this->node);
    if (!sync_packages(LidarMeasures)) { rate.sleep(); continue; }   // L609
    handleFirstFrame();                                              // L614
    processImu();                                                    // L616  forward + backward prop
    stateEstimationAndMapping();                                     // L620  -> handleLIO() / handleVIO()
}
```

`stateEstimationAndMapping()` ที่ [LIVMapper.cpp:336-348](FAST-LIVO2/src/LIVMapper.cpp#L336-L348) จะแยกตาม flag:

```cpp
switch (LidarMeasures.lio_vio_flg) {
    case VIO:        handleVIO();  break;     // ใช้กล้องอัพเดท
    case LIO: case LO: handleLIO(); break;   // ใช้ LiDAR อัพเดท
}
```

> **หมายเหตุสำคัญ:** ในแต่ละ cycle จะรัน **อย่างใดอย่างหนึ่ง** เท่านั้น (LIO หรือ VIO) ไม่ใช่รันต่อกันแบบใน Algorithm 1 ของ paper — ดูรายละเอียดใน [Section 3](#3-sequential-update-algorithm-iv-d-algorithm-1) และ [ตารางสรุปข้อ 7](#7-สรุปจุดที่-code-ไม่ตรงกับ-paper)

### 0.3 Frame conventions (จาก Table I ของ paper)

| สัญลักษณ์ | ความหมาย | code |
|----------|---------|------|
| $G$ | global / world frame | – |
| $I$ | IMU body frame | `state.rot_end`, `state.pos_end` (IMU pose ใน $G$) |
| $L$ | LiDAR frame | – |
| $C$ | Camera frame | – |
| $^I T_L$ | extrinsic LiDAR→IMU | `Lid_rot_to_IMU`, `Lid_offset_to_IMU` |
| $^C T_I$ | extrinsic IMU→Camera | `Rci`, `Pci` ใน [vio.cpp](FAST-LIVO2/src/vio.cpp) |
| $\boxplus$ / $\boxminus$ | boxplus/boxminus บน manifold | `operator+`, `operator-` ของ `StatesGroup` |

---

## 1. State Vector & Manifold (IV-A)

### 1.1 สมการในเปเปอร์

State manifold:

$$
\mathcal{M} \triangleq SO(3) \times \mathbb{R}^{16},\quad \dim(\mathcal{M}) = 19
$$

State vector (eq. 2):

$$
\mathbf{x} \triangleq \big[\, ^G\mathbf{R}_I^T \;\; ^G\mathbf{p}_I^T \;\; ^G\mathbf{v}_I^T \;\; \mathbf{b}_g^T \;\; \mathbf{b}_a^T \;\; ^G\mathbf{g}^T \;\; \tau \,\big]^T
$$

โดย $\tau$ คือ **inverse exposure time** เทียบกับเฟรมแรก

Input และ noise:

$$
\mathbf{u} \triangleq [\boldsymbol{\omega}_m^T \;\; \mathbf{a}_m^T]^T,\quad
\mathbf{w} \triangleq [\mathbf{n}_g^T \;\; \mathbf{n}_a^T \;\; \mathbf{n}_{b_g}^T \;\; \mathbf{n}_{b_a}^T \;\; n_\tau]^T
$$

Continuous-time dynamics (eq. 2):

$$
f(\mathbf{x},\mathbf{u},\mathbf{w}) =
\begin{bmatrix}
\boldsymbol{\omega}_m - \mathbf{b}_g - \mathbf{n}_g \\
^G\mathbf{v}_I + \tfrac{1}{2}(^G\mathbf{R}_I(\mathbf{a}_m - \mathbf{b}_a - \mathbf{n}_a) + ^G\mathbf{g})\Delta t \\
^G\mathbf{R}_I(\mathbf{a}_m - \mathbf{b}_a - \mathbf{n}_a) + ^G\mathbf{g} \\
\mathbf{n}_{b_g} \\
\mathbf{n}_{b_a} \\
\mathbf{0}_{3\times 1} \\
n_\tau
\end{bmatrix}
$$

(แถวที่ 6 คือ $\mathbf{0}$ เพราะ gravity ถือเป็น constant ที่จะเอามา estimate)

Discrete-time update (eq. 1):

$$
\mathbf{x}_{i+1} = \mathbf{x}_i \boxplus \big(\Delta t \cdot f(\mathbf{x}_i, \mathbf{u}_i, \mathbf{w}_i)\big)
$$

### 1.2 โค้ดที่เก็บ state

อยู่ที่ struct [`StatesGroup`](FAST-LIVO2/include/common_lib.h#L129-L226) ใน [common_lib.h:129](FAST-LIVO2/include/common_lib.h#L129):

```cpp
struct StatesGroup {
    M3D    rot_end;        // ^G R_I  (3 dof in error-state)
    V3D    pos_end;        // ^G p_I
    V3D    vel_end;        // ^G v_I
    double inv_expo_time;  // τ
    V3D    bias_g;         // b_g
    V3D    bias_a;         // b_a
    V3D    gravity;        // ^G g
    Matrix<double, 19, 19> cov;  // P
};
```

จำนวน DoF รวม = 3+3+3+1+3+3+3 = **19** ตรงกับ `DIM_STATE` ที่นิยามใน [common_lib.h:33](FAST-LIVO2/include/common_lib.h#L33)

### 1.3 Boxplus operator $\boxplus$

[`operator+`](FAST-LIVO2/include/common_lib.h#L170-L183) ของ `StatesGroup` คือนิยามของ $\boxplus$:

```cpp
StatesGroup operator+(const Matrix<double, 19, 1> &δx) {
    StatesGroup a;
    a.rot_end = rot_end * Exp(δx(0), δx(1), δx(2));   // SO(3) update via exponential
    a.pos_end = pos_end + δx.block<3,1>(3,0);
    a.inv_expo_time = inv_expo_time + δx(6);
    a.vel_end = vel_end + δx.block<3,1>(7,0);
    a.bias_g  = bias_g  + δx.block<3,1>(10,0);
    a.bias_a  = bias_a  + δx.block<3,1>(13,0);
    a.gravity = gravity + δx.block<3,1>(16,0);
    return a;
}
```

> **ลำดับ index ของ error-state $\delta\mathbf{x}$ ในโค้ด:**
> `[0:3]` = δrot, `[3:6]` = δpos, `[6]` = δτ, `[7:10]` = δvel, `[10:13]` = δb_g, `[13:16]` = δb_a, `[16:19]` = δg
>
> **ระวัง:** ลำดับนี้ไม่เหมือนใน paper Table I — `inv_expo_time` ถูกแทรกระหว่าง `pos` กับ `vel` (index 6 อยู่ตรงกลางแทนที่จะอยู่ท้าย) เวลาคำนวณ Jacobian ต้องระวังตำแหน่ง

`operator-` ที่ [common_lib.h:197-209](FAST-LIVO2/include/common_lib.h#L197-L209) คือ $\boxminus$ — ใช้ `Log(R₁ᵀ R₂)` สำหรับ rotation

---

## 2. Forward & Backward Propagation (IV-C)

### 2.1 Forward Propagation

**สมการในเปเปอร์ (eq. 1)** : เดินสมการ dynamics ที่จุด IMU sample แต่ละจุด (โดย set $\mathbf{w}_i = 0$) เพื่อทำนาย mean และ covariance ที่จุดต่อไป

ในรูปของ ESIKF discrete linearization:

$$
\hat{\mathbf{x}}_{i+1} = \hat{\mathbf{x}}_i \boxplus \Delta t \cdot f(\hat{\mathbf{x}}_i, \mathbf{u}_i, 0)
$$

$$
\hat{\mathbf{P}}_{i+1} = \mathbf{F}_{\!x}\, \hat{\mathbf{P}}_i\, \mathbf{F}_{\!x}^T + \mathbf{F}_{\!w}\, \mathbf{Q}\, \mathbf{F}_{\!w}^T
$$

### 2.2 โค้ด Forward Propagation

หัวใจอยู่ใน [`ImuProcess::UndistortPcl()`](FAST-LIVO2/src/IMU_Processing.cpp#L240-L541) ภายใน loop ของ IMU samples ที่ [IMU_Processing.cpp:329-403](FAST-LIVO2/src/IMU_Processing.cpp#L329-L403):

```cpp
for (int i = 0; i < v_imu.size() - 1; i++) {
    // ค่าเฉลี่ย angular vel/acc ระหว่าง 2 IMU samples
    angvel_avr -= state_inout.bias_g;                                  // L354
    acc_avr     = acc_avr * G_m_s2 / mean_acc.norm() - state_inout.bias_a;

    // === Build F_x (Jacobian ของ f w.r.t. error-state) ===
    F_x.setIdentity();
    F_x.block<3,3>(0, 0)  = Exp(angvel_avr, -dt);                      // δrot/δrot
    F_x.block<3,3>(0,10)  = -Eye3d * dt;                               // δrot/δb_g
    F_x.block<3,3>(3, 7)  =  Eye3d * dt;                               // δpos/δvel
    F_x.block<3,3>(7, 0)  = -R_imu * acc_avr_skew * dt;                // δvel/δrot
    F_x.block<3,3>(7,13)  = -R_imu * dt;                               // δvel/δb_a
    F_x.block<3,3>(7,16)  =  Eye3d * dt;                               // δvel/δg

    // === Process noise covariance Q*dt² ===
    cov_w(6,6) = cov_inv_expo * dt * dt;                               // for τ
    cov_w.block<3,3>(0,0).diagonal()  = cov_gyr * dt * dt;
    cov_w.block<3,3>(7,7) = R_imu * cov_acc.asDiagonal() * R_imu.t() * dt * dt;
    cov_w.block<3,3>(10,10).diagonal()= cov_bias_gyr * dt * dt;
    cov_w.block<3,3>(13,13).diagonal()= cov_bias_acc * dt * dt;

    state_inout.cov = F_x * state_inout.cov * F_x.transpose() + cov_w; // L403
    // จากนั้นอัพเดท mean (rot/pos/vel) ตาม dynamics ปกติ
}
```

> **สังเกต:** โค้ดรวม $F_w Q F_w^T$ ไว้ใน `cov_w` แล้ว ทำให้บรรทัดเดียวจบ ($P \leftarrow F P F^T + Q'$) — ตรงกับ standard ESIKF

มีอีกฟังก์ชันสั้นๆ ชื่อ [`prop_imu_once()`](FAST-LIVO2/src/LIVMapper.cpp#L625-L643) ที่ใช้สำหรับ **mean-only propagation** สำหรับ publish odometry สดๆ ที่ความถี่ IMU (ไม่อัพเดท covariance)

### 2.3 Backward Propagation (Motion Compensation)

เนื่องจาก LiDAR แต่ละจุดถูก scan ที่ time stamp ต่างกัน เราต้อง "ดึง" ทุกจุดมาที่ time stamp เดียวกัน (end-of-scan) — ทำได้โดยเดิน IMU ย้อนหลังจาก scan-end → จุดที่ scan ได้

**สมการรวบยอด** สำหรับจุดที่ scan ที่เวลา $t_j$ ก่อน scan-end เป็นเวลา $\Delta t_j$:

$$
^I \mathbf{p}_j^{\text{compensated}} = \mathbf{R}_{jk} \cdot {}^I\mathbf{p}_j + \mathbf{p}_{jk}
$$

โดย $\mathbf{R}_{jk}$, $\mathbf{p}_{jk}$ คือการเลื่อนของ IMU pose ระหว่าง $t_j \to t_{\text{scan-end}}$

### 2.4 โค้ด Backward Propagation

อยู่ที่ปลาย [`UndistortPcl()`](FAST-LIVO2/src/IMU_Processing.cpp#L494-L541) — เดินจาก `IMUpose.end()` กลับหลังพร้อมๆ กับเดิน point cloud:

```cpp
auto it_pcl = pcl_out.points.end() - 1;
for (auto it_kp = IMUpose.end() - 1; it_kp != IMUpose.begin(); it_kp--) {
    // คำนวณ R_i, T_ei สำหรับ IMU pose ที่ keypoint
    // เดิน it_pcl ย้อน จากท้าย scan ลงมาจน curvature(time offset) > offs_t ของ keypoint
    for(; it_pcl->curvature / 1000.0 > offs_t; it_pcl--) {
        dt = it_pcl->curvature/1000 - offs_t;
        V3D P_compensate =
            extR_Ri * (R_i * (Lid_rot_to_IMU * P_i + Lid_offset_to_IMU) + T_ei)
            - exrR_extT;
        it_pcl->x = P_compensate(0);
        it_pcl->y = P_compensate(1);
        it_pcl->z = P_compensate(2);
        if (it_pcl == pcl_out.points.begin()) break;
    }
}
```

หลังขั้นนี้แล้ว ทุกจุดใน `pcl_out` จะอยู่บน LiDAR frame ที่ time = scan-end พร้อมส่งเข้า ESIKF update

### 2.5 Scan Recombination (IV-B, Fig. 3)

```
Time:        t_{k-1}                          t_k
Camera ─────●─────────────────────────────────●─────  (10 Hz, sampling moments)
IMU    ─●●●●●●●●●●●●●●●●●●●●●●●●●●●●●●●●●●●●●●●●●●  (100-250 Hz, forward prop)
LiDAR  ┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄┄  (100-500 kHz, async stream)
                                          ↑
                       split LiDAR points at camera time:
                       points before → process this cycle (LIO)
                       points after  → buffer for next cycle (VIO ready)
```

จุดที่แบ่งคือ `img_capture_time` และโค้ดทำใน [LIVMapper.cpp:1017-1120](FAST-LIVO2/src/LIVMapper.cpp#L1017-L1120):

```cpp
// LIVMapper.cpp ~L1087-L1110
for (auto p : *pcl_in) {
    if (p.time + lid_beg < img_capture_time) {
        pcl_proc_cur->push_back(p);   // แพ็คเข้า cycle นี้
    } else {
        pcl_proc_next->push_back(p);  // เก็บไว้ cycle หน้า
    }
}
```

`pcl_proc_cur` ถูกป้อนต่อไปยัง LiDAR backward propagation + ESIKF update

---

## 3. Sequential Update Algorithm (IV-D, Algorithm 1)

### 3.1 ปัญหา Dimension Mismatch

ใน 1 cycle, LiDAR อาจให้ residuals หลายร้อยถึงพันมิติ (1 point-to-plane = 1 scalar res), ส่วน Image patch (8×8 px × 4 levels × N patches) อาจให้หลายหมื่นมิติ — ถ้าใช้ standard EKF ที่ stack ทั้งหมด จะคำนวณ inverse $(\mathbf{H}^T\mathbf{R}^{-1}\mathbf{H} + \mathbf{P}^{-1})^{-1}$ ขนาดใหญ่มาก

**วิธีแก้ของ paper:** ใช้ **Bayesian sequential update** อาศัยสมมุติฐานว่า LiDAR noise และ Image noise เป็นอิสระกัน (eq. 5):

$$
p(\mathbf{x}|\mathbf{y}_l, \mathbf{y}_c) \propto p(\mathbf{y}_c|\mathbf{x})\,\underbrace{p(\mathbf{y}_l|\mathbf{x})\,p(\mathbf{x})}_{\propto\, p(\mathbf{x}|\mathbf{y}_l)}
$$

ดังนั้นทำสองสเต็ป:

1. (eq. 6) update ด้วย LiDAR ก่อน → ได้ posterior $p(\mathbf{x}|\mathbf{y}_l)$ ใช้เป็น prior สำหรับสเต็ปถัดไป
2. (eq. 7) update ด้วย Image ต่อ → ได้ final posterior $p(\mathbf{x}|\mathbf{y}_l, \mathbf{y}_c)$

### 3.2 ESIKF Iterated Update Form

ทุกครั้งที่ทำ update (eq. 11):

$$
\mathbf{K} = (\mathbf{H}^T \mathbf{R}^{-1} \mathbf{H} + \hat{\mathbf{P}}^{-1})^{-1} \mathbf{H}^T \mathbf{R}^{-1}
$$

$$
\hat{\mathbf{x}}^{\kappa+1} = \hat{\mathbf{x}}^{\kappa} \boxplus (-\mathbf{K} \mathbf{z}^\kappa - (\mathbf{I} - \mathbf{K}\mathbf{H}^\kappa)(\hat{\mathbf{x}}^\kappa \boxminus \hat{\mathbf{x}}))
$$

จนกว่าจะ converge ($\|\delta\hat{\mathbf{x}}\| < \epsilon$) แล้ว update covariance: $\bar{\mathbf{P}} = (\mathbf{I} - \mathbf{K}\mathbf{H})\hat{\mathbf{P}}$

### 3.3 Algorithm 1 ของ Paper จับคู่กับโค้ด

```
═══════════════════════════════════════════════════════════════════════
PAPER (Algorithm 1)                          CODE
─────────────────────────────────────────────────────────────────────
1.  Scan recombination                       LIVMapper.cpp:1017-1120
2.  Forward propagation → x̂, P̂              IMU_Processing.cpp:329-403
3.  Backward prop. (motion comp.)            IMU_Processing.cpp:494-541
─── // Point-to-plane LiDAR update ───
4.  κ = -1, x̂^κ=0 = x̂                       voxel_map.cpp:388 (κ init)
5-7. repeat: κ++, compute z, H, δx           voxel_map.cpp:388-516 (loop)
8.  until ||δx|| < ε                         voxel_map.cpp:506-509 (break)
9.  x̄ = x̂; P̄ = (I - KH) P̂                  voxel_map.cpp:520
─── // Sparse direct visual update ───
10. level = -1                               vio.cpp:785 computeJacobianAndUpdateEKF
11. repeat (per pyramid level):
12.   κ = -1, x̂^κ=0 = x̂; level++             vio.cpp:791 (level outer loop)
13.   repeat: κ++, compute z, H, δx          vio.cpp:1522-1671 (iter in updateState)
14.   until ||δx|| < ε                       vio.cpp:1672 (break)
15. until level >= 2                          vio.cpp:791 (level-end)
16. x̄ = x̂; P̄ = (I - KH) P̂                  vio.cpp:1671-1685
═══════════════════════════════════════════════════════════════════════
```

### 3.4 LiDAR Iterated Update — โค้ด

หัวใจที่ [`BuildResidualListOMP`](FAST-LIVO2/src/voxel_map.cpp#L659-L727) + main loop ที่ [voxel_map.cpp:354-527](FAST-LIVO2/src/voxel_map.cpp#L354-L527):

```cpp
for (int iterCount = 0; iterCount < NUM_MAX_ITERATIONS; iterCount++) {
    // 1) คำนวณ residual + Jacobian ทุกจุด (parallel-OMP)
    BuildResidualListOMP(...);                           // L405
    // 2) สร้าง H_T_H, H_T_z (lumped form)
    for (int i = 0; i < effct_feat_num; i++) {
        // Hsub.row(i) = [A_i | n_i]   (1x6)
        H_T_H.block<6,6>(0,0) += Hsub.row(i).T * Hsub.row(i) / σ_l;
    }
    // 3) Kalman gain (lumped)
    auto K_1 = (H_T_H + state_.cov.inverse()).inverse(); // L484
    // 4) update
    auto solution = - K_1 * H_T_z - ... ;                // L490
    state_ += solution;
    if (solution.norm() < ε) break;                       // converge check
}
state_.cov = (I - K * H) * state_.cov;
```

### 3.5 Visual Iterated Update (Pyramid) — โค้ด

Outer pyramid loop อยู่ใน [`computeJacobianAndUpdateEKF()`](FAST-LIVO2/src/vio.cpp#L785-L800) ที่เรียก [`updateState()`](FAST-LIVO2/src/vio.cpp#L1522-L1690) ทีละชั้น (coarse→fine):

```cpp
// vio.cpp:791
for (int level = patch_pyrimid_level - 1; level >= 0; level--) {  // coarse→fine
    updateState(img, level);                                       // L799
}

// vio.cpp:1522 inside updateState()
for (int iter = 0; iter < max_iters; iter++) {
        // สำหรับทุก visual map point → คำนวณ photometric residual + Jacobian
        // J_img (1×2), J_dpi (2×3 = pinhole proj), J_se3 = J_img * J_dpi * [-Rp]^∧
        // residual = τ_k * I_cur - τ_r * I_ref (ในโค้ด var "res")
        // สะสม H_sub^T H_sub และ H_sub^T z
        K_1 = (H_sub_T * H_sub + (state->cov / img_point_cov).inverse()).inverse(); // L1663
        solution = -K_1 * (...);
        (*state) += solution;                                                       // L1671
        if (solution.norm() < ε) break;
    }
}
```

### 3.6 Mismatches กับ Paper

| Paper | Code |
|-------|------|
| Algorithm 1 รัน LiDAR → Visual ในรอบเดียว | โค้ดเลือก **อย่างใดอย่างหนึ่ง** ตาม `lio_vio_flg` ([LIVMapper.cpp:336-348](FAST-LIVO2/src/LIVMapper.cpp#L336-L348)) — เพราะ scan recombination ทำให้ในรอบที่ marker เป็น LIO ก็ไม่มีรูปใหม่ และในรอบ VIO ก็ไม่มี LiDAR points ให้ใช้ |
| Kalman gain เขียนแบบ `(HᵀR⁻¹H + P⁻¹)⁻¹HᵀR⁻¹` | โค้ดแยกเป็น `K_1 = (HᵀR⁻¹H + P⁻¹)⁻¹` แล้วคูณ `H/R` ตอน apply (mathematically เหมือนกัน) |

---

## 4. Local Mapping – Voxel Map (V)

### 4.1 โครงสร้างหลัก: Hash + Octree

- **Root voxel** ขนาด `0.5 × 0.5 × 0.5 m` (default config — [voxel_map.cpp:41](FAST-LIVO2/src/voxel_map.cpp#L41))
- เก็บใน [`std::unordered_map<VOXEL_LOCATION, VoxelOctoTree*>`](FAST-LIVO2/include/voxel_map.h#L194) → O(1) lookup
- แต่ละ root voxel เป็น **octree** — ถ้าจุดใน voxel ไม่ลงเป็นระนาบเดียว ก็แตก 8 octants ลงไปต่อจนถึง `max_layer` (default = 3 — [voxel_map.cpp:40](FAST-LIVO2/src/voxel_map.cpp#L40))
- Hash function อยู่ที่ [voxel_map.h:108-118](FAST-LIVO2/include/voxel_map.h#L108-L118) ใช้ prime `116101`

```
   root voxel (0.5 m)
        │
   ┌────┴────┐ (ถ้าไม่ planar → split)
   │  ...    │
  octant   octant   ...      ← layer 1
   │         │
  ...       ...                ← layer 2 (max layer = 3)
```

### 4.2 Plane Fitting (V-B)

ทุก leaf voxel จะลอง fit ระนาบจากจุด LiDAR ภายใน:

**สมการที่ paper อธิบาย:**

จากชุดจุด $\{\mathbf{p}_i\}$ ภายใน voxel:

$$
\bar{\mathbf{q}} = \frac{1}{N}\sum_i \mathbf{p}_i,\quad
\boldsymbol{\Sigma} = \frac{1}{N}\sum_i (\mathbf{p}_i - \bar{\mathbf{q}})(\mathbf{p}_i - \bar{\mathbf{q}})^T
$$

ทำ **SVD/eigendecomposition** บน $\boldsymbol{\Sigma}$ — ถ้าค่า eigenvalue ที่เล็กที่สุด $\lambda_{\min}$ ต่ำกว่า threshold → ถือเป็นระนาบ และ `normal n` คือ eigenvector ของ $\lambda_{\min}$

นอกจากนี้ paper คำนวณ **plane covariance** $\boldsymbol{\Sigma}_{\mathbf{n},\mathbf{q}}$ จาก first-order propagation ของ noise ที่จุดทุกจุด

### 4.3 โค้ด Plane Fitting

ใน [`init_plane()`](FAST-LIVO2/src/voxel_map.cpp#L71-L151):

```cpp
void VoxelOctoTree::init_plane(const std::vector<pointWithVar>& points, VoxelPlane* plane) {
    // 1) accumulate mean & covariance
    for (auto p : points) {
        plane->covariance_ += p.point_w * p.point_w.T;
        plane->center_     += p.point_w;
    }
    plane->center_     /= N;
    plane->covariance_  = plane->covariance_/N - center*center.T;

    // 2) eigendecomposition
    EigenSolver<Matrix3d> es(plane->covariance_);
    auto eigvals = es.eigenvalues().real();
    int  evalsMin = argmin(eigvals);
    // 3) ถ้า min eigenvalue เล็กพอ → เป็นระนาบ
    if (eigvals[evalsMin] < planer_threshold_) {
        plane->normal_ = es.eigenvectors().col(evalsMin).real();
        plane->is_plane_ = true;
        // 4) คำนวณ plane covariance Σ_{n,q} (6×6) ด้วย Jacobian propagation
        //    plane->plane_var_ = J * Σ_pts * J^T
    } else {
        plane->is_plane_ = false;   // ต้องไป split octants ต่อ
    }
}
```

### 4.4 Octree Sub-division

ใน [`cut_octo_tree()`](FAST-LIVO2/src/voxel_map.cpp#L179-L233) — ถ้า voxel ไม่ planar และ layer ยังต่ำกว่า max:

```cpp
if (layer_ < max_layer_) {
    for each point:
        int idx = 4*xyz[0] + 2*xyz[1] + xyz[2];   // octant index
        leaves_[idx]->temp_points_.push_back(point);
    // recurse ลงไปทำ init_plane() ต่อในแต่ละ octant
}
```

### 4.5 Map Sliding (Ring Buffer, Fig. 4)

ป้องกัน hash table โตไม่จำกัด — เมื่อ LiDAR เคลื่อนเกิน `sliding_thresh` (default 8.0 m, [voxel_map.cpp:53](FAST-LIVO2/src/voxel_map.cpp#L53)) จะเลื่อน local map และลบ root voxels นอก `±half_map_size` (default 100, [voxel_map.cpp:52](FAST-LIVO2/src/voxel_map.cpp#L52))

โค้ด [`mapSliding()` + `clearMemOutOfMap()`](FAST-LIVO2/src/voxel_map.cpp#L940-L987):

```cpp
void VoxelMapManager::mapSliding() {
    if ((position_ - last_slide_pos).norm() < sliding_thresh) return;
    clearMemOutOfMap(...);  // ลบ voxel นอกหน้าต่าง
    last_slide_pos = position_;
}
```

### 4.6 Visual Map Point Generation (V-C)

หลังจาก LiDAR update เสร็จ ระบบจะหา **LiDAR points ที่จะกลายเป็น visual map points** ในเฟรมปัจจุบัน — เกณฑ์คือ:

1. มองเห็นจาก camera FoV
2. **Image gradient สูง** (textured)

โค้ดอยู่ใน [vio.cpp:805-899](FAST-LIVO2/src/vio.cpp#L805-L899):

```cpp
// คำนวณ Shi-Tomasi corner score
double score = computeShiTomasi(img, projected_uv);
if (score < 30.0) continue;                       // L825 threshold
// แบ่งภาพเป็น grid 30×30 px → เลือกจุดที่คะแนนดีที่สุดต่อ grid cell
// แนบ patch pyramid (3-4 level, 11×11 px) เป็น Feature
VisualPoint* vp = new VisualPoint(world_pos);     // L879-896
vp->addFrameRef(new Feature(patch, current_frame, ref_pose, exposure));
```

### 4.7 Reference Patch Update (V-D, eq. 12)

แต่ visual map point เก็บ **patch ที่ดีที่สุด** เป็น reference (สำหรับใช้คำนวณ photometric error)

**สมการเลือก reference patch (eq. 12):**

$$
\mathrm{NCC}(\mathbf{f}, \mathbf{g}) = \frac{\sum_{x,y}(\mathbf{f}-\bar{\mathbf{f}})(\mathbf{g}-\bar{\mathbf{g}})}{\sqrt{\sum(\mathbf{f}-\bar{\mathbf{f}})^2 \cdot \sum(\mathbf{g}-\bar{\mathbf{g}})^2}}
$$

$$
c = \frac{\mathbf{n}\cdot\mathbf{p}}{\|\mathbf{p}\|},\quad
\omega_1 = \frac{1}{1+e^{\mathrm{tr}(\boldsymbol{\Sigma}_{\!n})}}
$$

$$
S = (1-\omega_1)\cdot\frac{1}{n}\sum_{i=1}^n \mathrm{NCC}(\mathbf{f}, \mathbf{g}_i) + \omega_1 \cdot c
$$

แล้วเลือก patch ที่ $S$ สูงสุดเป็น reference

โค้ดอยู่ใน [vio.cpp:971-1102](FAST-LIVO2/src/vio.cpp#L971-L1102) — สังเกตว่าโค้ดใช้ `score = NCC + cos_angle` (อย่างง่าย ไม่ใช้ explicit weight $\omega_1$ ตามสูตร eq. 12 — ดูตาราง mismatch ข้อ 7)

### 4.8 Normal Refinement (V-E, eq. 14)

หลังจากมี reference patch แล้ว สามารถปรับ **plane normal** ของ visual map point ด้วยการ minimize photometric error ระหว่าง reference patch กับ patches อื่นที่ผ่านการ warp:

$$
^{I_r}\mathbf{n}^* = \arg\min_{n} \sum_{i\in S}\sum_{j=1}^{N^2}\big\| \tau_i \mathbf{I}_i(\mathbf{A}_r^i \mathbf{u}_r^j) - \tau_r \mathbf{I}_r(\mathbf{u}_r^j) \big\|_2
$$

โดย $\mathbf{A}_r^i$ คือ affine warp ที่ขึ้นกับ normal (ดู eq. 13 ใน Section 6)

โค้ดอยู่ใน [vio.cpp:971-1036](FAST-LIVO2/src/vio.cpp#L971-L1036) — เป็นส่วนหนึ่งของ `updateReferencePatch()`

> **หมายเหตุ Mismatch:** Paper บอกว่า normal refinement "can be performed in a separate thread" แต่โค้ดไม่ได้ทำใน thread แยก — รันบน main thread

---

## 5. LiDAR Measurement Model (VI)

### 5.1 Point-to-Plane Residual (VI-A, eq. 17–19)

จุด LiDAR ที่ผ่าน undistortion แล้ว ($^L\mathbf{p}_j$) ถูกแปลงเข้า world frame:

$$
^G\hat{\mathbf{p}}_j^\kappa = ^G\hat{\mathbf{T}}_I^\kappa \cdot {}^I\mathbf{T}_L \cdot {}^L\mathbf{p}_j \quad\text{(eq. 17)}
$$

จากนั้นหา root/sub-voxel ที่จุดนี้ตกลง — ถ้า voxel นั้นมีระนาบ (mature plane) ที่มี normal $\mathbf{n}_j$ และจุดศูนย์กลาง $\mathbf{q}_j$ ก็จะได้ measurement equation:

$$
\mathbf{0} = \mathbf{n}_j^T (^G\mathbf{T}_I \cdot {}^I\mathbf{T}_L \cdot {}^L\mathbf{p}_j - \mathbf{q}_j) \quad\text{(eq. 18)}
$$

โดยรวมเสียงรบกวน (LiDAR point noise $\delta\mathbf{p}$, plane normal $\delta\mathbf{n}$, plane center $\delta\mathbf{q}$):

$$
\underbrace{\mathbf{0}}_{=\mathbf{y}_l} = \underbrace{(\mathbf{n}_j \boxplus \delta\mathbf{n}_j)^T \big(^G\mathbf{T}_I \cdot {}^I\mathbf{T}_L (^L\mathbf{p}_j - \delta^L\mathbf{p}_j) - (\mathbf{q}_j - \delta\mathbf{q}_j)\big)}_{\mathbf{h}_l(\mathbf{x}, \mathbf{v}_l)} \quad\text{(eq. 19)}
$$

### 5.2 Jacobian $\mathbf{H}_l = \partial\mathbf{h}_l / \partial\delta\mathbf{x}$

ในโค้ดใช้รูป **6-dim** (เฉพาะ rotation 3 + position 3) — bias/grav/expo ไม่ได้อยู่ใน LiDAR Jacobian เพราะไม่มีผลต่อ point projection ผ่านสมการ:

$$
\mathbf{H}_l^{(j)} =
\begin{bmatrix}
-\mathbf{n}_j^T \cdot {}^G\mathbf{R}_I \big[\,^I\mathbf{p}_j\big]_\times & \mathbf{n}_j^T
\end{bmatrix}
$$

(Block 1 = derivative w.r.t. δrot, Block 2 = derivative w.r.t. δpos)

### 5.3 โค้ด Residual + Jacobian

อยู่ใน [`build_single_residual()`](FAST-LIVO2/src/voxel_map.cpp#L729-L802):

```cpp
double dis_to_plane = fabs( plane.normal_.x()*p_w.x()
                          + plane.normal_.y()*p_w.y()
                          + plane.normal_.z()*p_w.z()
                          + plane.d_ );                   // L741, residual
// ตรวจระยะ — ถ้าเกิน sigma_num*sqrt(σ_l) → ทิ้ง
double sigma_l = J_nq * plane.plane_var_ * J_nq.T
               + plane.normal_.T * point_cov * plane.normal_;
if (dis_to_plane < sigma_num * sqrt(sigma_l)) {           // L753 outlier gate
    // เลือก plane ที่ probability สูงสุด (recursive ลง octree ถ้ามี)
}
```

จุดที่สะสม Jacobian อยู่ใน main estimation loop ที่ [voxel_map.cpp:425-474](FAST-LIVO2/src/voxel_map.cpp#L425-L474):

```cpp
V3D point_crossmat = SkewSym(state_.rot_end.T * (-point_w + state_.pos_end));
M3D A = point_crossmat * state_.rot_end.T * normal_;       // L469
Hsub.row(i) << A.x(), A.y(), A.z(),
               normal_.x(), normal_.y(), normal_.z();      // L470
meas_vec(i) = -dis_to_plane;                                // residual z_i
```

### 5.4 LiDAR Noise Model (VI-B, eq. 20)

Paper เสนอให้รวม **3 ชนิดของ noise** สำหรับแต่ละจุด:

1. **Ranging noise** $\delta d$ — Time-of-flight error
2. **Bearing noise** $\delta\boldsymbol{\omega}$ — encoder error
3. **Beam-divergence noise** — เกิดจาก beam spread angle $\theta$

**สมการ (20)** สำหรับ ranging noise ที่เกิดจาก beam divergence (มุม $\varphi$ ระหว่าง bearing direction กับ surface normal):

$$
\delta d = L_2 - L_1 = d \left( \frac{\cos\varphi}{\cos(\theta+\varphi)} - \frac{\cos\varphi}{\cos(\theta-\varphi)} \right)
$$

นั่นคือ ที่จุดตกบนพื้น (มุม $\varphi$ ใหญ่) error จะมากกว่ามาก ↔ จุดบนผนัง (มุม $\varphi$ เล็ก) error ต่ำกว่า

### 5.5 โค้ด Noise Model

อยู่ที่ [`calcBodyCov()`](FAST-LIVO2/src/voxel_map.cpp#L15-L34):

```cpp
void calcBodyCov(V3D &pb, float range_inc, float degree_inc, M3D &cov) {
    float range = pb.norm();
    float range_var = range_inc * range_inc;            // L19   σ_d²
    Matrix2d direction_var;
    direction_var << sin²(degree_inc), 0,
                     0,                sin²(degree_inc); //  L21  σ_ω²

    Vector3d direction = pb.normalized();
    Matrix3d direction_hat = SkewSym(direction);
    // base_vector1, base_vector2 = สอง orthonormal vectors ตั้งฉากกับ direction
    Matrix<3,2> N << base1, base2;
    Matrix<3,2> A = range * direction_hat * N;

    cov = direction * range_var * direction.T            // ranging part
        + A * direction_var * A.T;                       // bearing part   L33
}
```

> **🚩 Mismatch สำคัญ:** โค้ดใช้แค่ **range_inc² + bearing_var × A·Aᵀ** แบบง่ายๆ — **ไม่มี term `cos φ / cos(θ±φ)`** ตามสมการ (20) ที่อยู่ใน paper section VI-B
>
> **ผลที่เกิด:** จุดบนพื้นกับจุดบนผนังถูก weight เท่ากัน — ในทางปฏิบัติ accuracy จะลดลงเล็กน้อยในฉากที่จุดส่วนใหญ่อยู่บนพื้น
>
> Paper ablation (Section IX-B) บอก beam divergence helps "more precise pose estimation" — ใน implementation ใน repo นี้คงไม่ได้รับประโยชน์เต็มที่

---

## 6. Visual Measurement Model (VII)

### 6.1 Visual Map Point Selection (VII-A) — 3 ขั้นตอน

#### (1) Visible Voxel Query (VII-A-1)

เพื่อหลีกเลี่ยงการ project ทุก visual map point ใน global map ลงเฟรมปัจจุบัน — เราใช้ **LiDAR point cloud ของ scan ปัจจุบัน** เป็น "probe" ดูว่า voxel ไหนถูกชน

โค้ด [vio.cpp:380-485](FAST-LIVO2/src/vio.cpp#L380-L485):

```cpp
for (auto p : current_lidar_scan) {
    VOXEL_LOCATION pos = quantize(p, voxel_size=0.5);
    auto it = voxel_map_->find(pos);
    if (it != voxel_map_->end()) {
        // ตรวจ FoV: project visual map points ลงภาพ
        for (auto vp : it->second->visual_points_) {
            if (isInFrame(vp->pos)) submap.push_back(vp);   // L468
        }
    }
}
```

#### (2) On-Demand Raycasting (VII-A-2)

ปัญหา: ถ้า camera มี FoV กว้างกว่า LiDAR หรือ LiDAR ใกล้วัตถุเกิน blind zone — บาง grid cell จะไม่มี visual map point — ต้องยิง ray หาเอง

โค้ด [vio.cpp:488-592](FAST-LIVO2/src/vio.cpp#L488-L592):

```cpp
// แบ่งภาพเป็น 30×30 px grid; ทุก cell ที่ยังว่าง ลอง raycast
for each empty grid cell {
    for (auto sample : rays_with_sample_points[grid_id]) {  // L504  pre-computed
        V3D world_pt = new_frame_->f2w(sample);             // L506  body→world
        VOXEL_LOCATION pos = quantize(world_pt);
        if (voxel_map_->find(pos) != end) {
            // ถ้าเจอ visual map point ที่ต้องของ cell นี้ → หยุด ray
        }
    }
}
```

`d_min = 0.1`, `d_max = 3.0`, step = 0.2 — pre-computed sample points บน rays ([vio.cpp:88-117](FAST-LIVO2/src/vio.cpp#L88-L117))

#### (3) Outlier Rejection (VII-A-3)

หลังได้ submap แล้ว ทิ้ง point ที่:
- **Occluded** หรือ **depth-discontinuous** — เปรียบ depth ของ visual map point กับ LiDAR depth ของ neighbors 9×9 pixels
- **View angle ใหญ่เกิน** — มุมระหว่าง normal patch กับ ray ปัจจุบัน

โค้ด [vio.cpp:620-641](FAST-LIVO2/src/vio.cpp#L620-L641):

```cpp
// depth-discontinuity check
double delta_dist = fabs(depth_at_uv - vp_depth);
if (delta_dist > 0.5) reject;                           // L633  > 50 cm
```

```
camera ─→ ●─────●─────●(occluded)
       ╲   pt1   pt2    pt3
        ╲          ↑ if depth(pt3) << depth(neighbors), reject
```

### 6.2 Affine Warping (V-D-1, eq. 13)

เมื่อ project reference patch จาก reference frame เข้า current frame เราต้อง **warp** เพื่อชดเชยการเปลี่ยน viewpoint:

$$
\mathbf{u}_i^j = \mathbf{A}_r^i \cdot \mathbf{u}_r^j
$$

$$
\mathbf{A}_r^i = \mathbf{P} \left( ^{I_r}\mathbf{R}_{I_i} + {}^{I_r}\mathbf{t}_{I_i} \cdot \frac{1}{^{I_r}\mathbf{n}^T \cdot {}^{I_r}\mathbf{p}}\, {}^{I_r}\mathbf{n}^T \right) \mathbf{P}^{-1}
$$

โดย $\mathbf{P}$ คือ projection (intrinsic), $^{I_r}\mathbf{n}$ คือ plane normal, $^{I_r}\mathbf{p}$ คือ visual point ใน reference frame

### 6.3 โค้ด Affine Warp

อยู่ที่ [`getWarpMatrixAffineHomography()`](FAST-LIVO2/src/vio.cpp#L253-L274):

```cpp
Matrix3d getWarpMatrixAffineHomography(const Camera &cam, const V3D &normal_ref,
                                       const V3D &xyz_ref, const SE3 &T_cur_ref, ...) {
    // H = R + t * n^T / (n^T · p)         ← ตรงกับ eq. 13 (ก่อน premul P, postmul P^-1)
    Matrix3d H = T_cur_ref.rotation_matrix() +
                 T_cur_ref.translation() * normal_ref.transpose() /
                 (normal_ref.dot(xyz_ref));
    // project 4 corners of ref patch ผ่าน H → fit 2x2 affine A_cur_ref
    return A_cur_ref;
}
```

### 6.4 Sparse Direct Photometric Error (VII-B, eq. 21–22)

จุด visual map point $^G\mathbf{p}_i$ — เมื่อ projected เข้าทั้ง current และ reference frame ค่า intensity ต้องเท่ากัน (ถ้าหักลบ exposure):

**สมการเต็มในฉบับ inverse-compositional (eq. 21):**

$$
\mathbf{0} = \tau_k \mathbf{I}_k^{gt}\Big(\boldsymbol{\pi}\big({}^C\mathbf{T}_I({}^G\mathbf{T}_I)^{-1}\, {}^G\mathbf{p}_i\big) + \Delta\mathbf{u}\Big)
        - \tau_r \mathbf{I}_r^{gt}\Big(\boldsymbol{\pi}\big({}^{C_r}\mathbf{T}_G\, {}^G\mathbf{p}_i\big)\Big) + \mathbf{A}_i^r \Delta\mathbf{u}
$$

หลังรวมเสียงรบกวน ADC noise (eq. 22):

$$
\underbrace{\mathbf{0}}_{=\mathbf{y}_c} = \tau_k(\mathbf{I}_k(\mathbf{u}_k + \Delta\mathbf{u}) - \delta\mathbf{I}_k) - \tau_r(\mathbf{I}_r(\mathbf{u}_r' + \mathbf{A}_i^r \Delta\mathbf{u}) - \delta\mathbf{I}_r)
$$

โดย $\tau_k$, $\tau_r$ คือ inverse exposure ของ current/reference frame

### 6.5 Inverse Compositional Trick (eq. 23)

แทนที่จะอัพเดต pose ของ current frame โดยตรง (forward), paper ใช้ **incremental warp ที่ฝั่ง reference**:

$$
{}^G\mathbf{T}_I = {}^G\hat{\mathbf{T}}_I^\kappa \cdot \mathrm{Exp}(\delta\mathbf{T})
$$

$$
\mathbf{u}_k = \boldsymbol{\pi}({}^C\mathbf{T}_I({}^G\hat{\mathbf{T}}_I^\kappa)^{-1} \, {}^G\mathbf{p}_i),\quad \mathbf{u}_r' = \boldsymbol{\pi}({}^{C_r}\mathbf{T}_G \cdot \mathrm{Exp}(\delta\mathbf{T})\, {}^G\mathbf{p}_i)
$$

ข้อดี: $\mathbf{u}_r'$ คงที่ระหว่าง iterations → คำนวณ Jacobian **ครั้งเดียวต่อ patch** (ลดเวลา ~3×)

### 6.6 โค้ด Photometric Residual + Jacobian

อยู่ใน [`updateState()`](FAST-LIVO2/src/vio.cpp#L1522-L1690) — ลึกใน inner loop คำนวณ residual + Jacobian:

```cpp
// Jacobian (inverse compositional)
Jimg  = image_gradient * state->inv_expo_time;            // L1614   τ_k * ∇I
Jdpi  = pinhole_jacobian(uv);                             //  2×3 projection
Jdphi = Jimg * Jdpi * SkewSym(p_in_camera);               // L1616   ∂/∂rot
Jdp   = -Jimg * Jdpi;                                     // L1617   ∂/∂trans

// Residual
double res = state->inv_expo_time * cur_value
           - inv_ref_expo * P[ref_pixel];                 // L1623
// rows คำนวณสะสม H^T H, H^T z แบบเดียวกับ LiDAR update
// state vector ที่อัพเดตคือ 7 dim: [δrot 3, δtrans 3, δτ 1]
```

> **`state->inv_expo_time` คือ τ ในสมการ** — exposure time estimation ทำใน ESIKF เลย ไม่ต้องมีกระบวนการแยก

### 6.7 Image Pyramid

- Paper บอก "3 levels" (Section IX-A: "image patch size is set as 8×8")
- Code default = **4 levels** (configurable, [LIVMapper.cpp:72](FAST-LIVO2/src/LIVMapper.cpp#L72)):

```cpp
node->declare_parameter<int>("vio.patch_pyrimid_level", 4);
node->declare_parameter<int>("vio.patch_size", 8);
```

Pyramid ถูก build ใน [`Frame::initFrame()` (frame.cpp:54-63)](FAST-LIVO2/src/frame.cpp#L54-L63):

```cpp
img_pyr_.resize(n_levels);
img_pyr_[0] = img;
for (int i = 1; i < n_levels; ++i)
    img_pyr_[i] = vk::halfSample(img_pyr_[i-1]);   // half-resolution per level
```

Iterate จากชั้น coarsest → finest ใน [`computeJacobianAndUpdateEKF()` (vio.cpp:791)](FAST-LIVO2/src/vio.cpp#L791-L800):

```cpp
for (int level = patch_pyrimid_level - 1; level >= 0; level--) {
    if (inverse_composition_en) updateStateInverse(img, level);
    else                          updateState(img, level);
}
```

---

## 7. สรุปจุดที่ Code ไม่ตรงกับ Paper

| # | หัวข้อ | Paper | Code | ผลกระทบ |
|---|--------|-------|------|---------|
| 1 | Beam-divergence noise (eq. 20) | $\delta d = d(\cos\varphi/\cos(\theta+\varphi) - \cos\varphi/\cos(\theta-\varphi))$ | `calcBodyCov` ใช้แค่ `range_var + bearing_var` แบบไม่มี $\cos$ term ([voxel_map.cpp:15-34](FAST-LIVO2/src/voxel_map.cpp#L15-L34)) | จุดบนพื้น (มุม $\varphi$ ใหญ่) ถูก weight เท่ากับจุดบนผนัง — accuracy ในฉาก ground-heavy ลดลงเล็กน้อย |
| 2 | Sequential update flow | LiDAR → Visual ในรอบเดียว (Algorithm 1) | branch แยก `handleLIO()` / `handleVIO()` ตาม `lio_vio_flg` ([LIVMapper.cpp:336-348](FAST-LIVO2/src/LIVMapper.cpp#L336-L348)) | ผลทาง mathematical เหมือนกัน เพราะ scan recombination แยก data ไว้แล้ว — แต่อ่านโค้ดเทียบ Algorithm 1 อาจงง |
| 3 | Normal refinement thread | "can be performed in a separate thread" (Section V-E) | รันบน main thread ใน `updateReferencePatch()` ([vio.cpp:971-1036](FAST-LIVO2/src/vio.cpp#L971-L1036)) | latency สูงขึ้นเล็กน้อย แต่ thread-safety ง่ายกว่า |
| 4 | Image pyramid levels | 3 levels (Section IX-A) | default = 4 (configurable, [LIVMapper.cpp:72](FAST-LIVO2/src/LIVMapper.cpp#L72)) | – (config-able) |
| 5 | Reference patch score $S$ (eq. 12) | Explicit $\omega_1 = 1/(1+e^{\mathrm{tr}(\Sigma_n)})$ weight | `score = NCC + cos_angle` แบบรวมไม่มี $\omega_1$ ([vio.cpp:1089](FAST-LIVO2/src/vio.cpp#L1089)) | tuning ทำได้ยากกว่า แต่ผลลัพธ์ใกล้เคียง |
| 6 | LiDAR Jacobian dim | (paper) คำนวณ $\partial h_l/\partial \delta\mathbf{x}$ ทั้ง 19 มิติ | โค้ดเก็บแค่ 6 มิติ (rot+pos) ใน `Hsub` ([voxel_map.cpp:470](FAST-LIVO2/src/voxel_map.cpp#L470)) | ถูกต้องแล้ว — bias/grav/τ ไม่มีผลต่อ point projection ในรอบ update เดียว |
| 7 | State error-state ordering | (paper Table I) [R, p, v, b_g, b_a, g, τ] | (code) [R, p, **τ**, v, b_g, b_a, g] ([common_lib.h:170-194](FAST-LIVO2/include/common_lib.h#L170-L194)) | ถ้าต้องเขียน Jacobian ใหม่ ต้องระวัง index ของ `δτ` อยู่ที่ตำแหน่ง 6 ไม่ใช่ 18 |

---

## 8. ลำดับอ่านโค้ดสำหรับมือใหม่

แนะนำเริ่มอ่านตามลำดับนี้:

1. **[main.cpp](FAST-LIVO2/src/main.cpp)** — ROS entry, สร้าง `LIVMapper` แล้วเรียก `run()`
2. **[LIVMapper.h](FAST-LIVO2/include/LIVMapper.h)** + **[LIVMapper.cpp](FAST-LIVO2/src/LIVMapper.cpp)** — orchestrator
   - อ่าน `run()` ก่อน → เห็น loop ใหญ่
   - แล้วไป `sync_packages()`, `processImu()`, `stateEstimationAndMapping()`
3. **[common_lib.h](FAST-LIVO2/include/common_lib.h)** — `StatesGroup`, `LidarMeasureGroup`, `MeasureGroup`
4. **[IMU_Processing.cpp](FAST-LIVO2/src/IMU_Processing.cpp)** — เริ่มจาก `Process2()` → `UndistortPcl()`
5. **[voxel_map.cpp](FAST-LIVO2/src/voxel_map.cpp)** — เริ่มจาก `StateEstimation()` (LIO update) → ลงไป `BuildResidualListOMP` → `build_single_residual` → `init_plane`
6. **[vio.cpp](FAST-LIVO2/src/vio.cpp)** — เริ่มจาก `processFrame()` → `updateState()` → `getWarpMatrixAffineHomography`
7. **[visual_point.h](FAST-LIVO2/include/visual_point.h)** + **[frame.cpp](FAST-LIVO2/src/frame.cpp)** — สำหรับเข้าใจ data structures ของ vision

### Tip การ debug

- เปิด `pub_plane_en: true` ใน [config](FAST-LIVO2/config/) เพื่อเห็น plane ที่ fit ได้ใน RViz
- ตั้ง `exposure_estimate_en: false` ก่อน ถ้าจะ tune param อื่นโดยไม่ปนกับ τ
- บน UrbanNav ที่ใช้กล้อง+LiDAR+IMU แนะนำตรวจ `extrinsic` ใน [config/](FAST-LIVO2/config/) ทั้ง LiDAR↔IMU และ Camera↔IMU ให้ตรงก่อนเริ่ม

---

> **อ้างอิง:**
> - Paper: `PDF/2408.14035v2.pdf` (อยู่ใน workspace นี้)
> - Supplementary: [FAST-LIVO2/Supplementary/LIVO2_supplementary.pdf](FAST-LIVO2/Supplementary/LIVO2_supplementary.pdf)
> - GitHub upstream: <https://github.com/hku-mars/FAST-LIVO2>
