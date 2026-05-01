#!/usr/bin/env python3
"""
UrbanNav Ground Truth Evaluator for FAST-LiVO2
================================================
Compares FAST-LiVO2 TUM trajectory against UrbanNav GPS ground truth.

Usage:
    python3 urbannav_evaluate.py \
        --gt  UrbanNav_TST_GT_raw.txt \
        --est urbannav_tst.txt

Requirements:
    pip install numpy pandas matplotlib scipy

How it works:
    1. Parse GT (GPS DMS → decimal degrees → local ENU meters)
    2. Parse SLAM TUM poses (already local metric XYZ)
    3. Temporally align: match each GT point to nearest SLAM timestamp
    4. SE3 Umeyama alignment (like evo_ape -a): find best R, t to align SLAM→GT
    5. Compute 2D / 3D errors and report metrics
    6. Save multi-panel plot as PNG
"""

import argparse
import sys
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.colors import Normalize
from matplotlib.cm import ScalarMappable
from scipy.spatial.transform import Rotation


# ═══════════════════════════════════════════════════════════════════════════════
# 1. PARSERS
# ═══════════════════════════════════════════════════════════════════════════════

def parse_gt(path):
    """Parse UrbanNav_TST_GT_raw.txt  →  DataFrame with decimal lat/lon/h."""
    rows = []
    with open(path) as f:
        for i, line in enumerate(f):
            if i < 2 or not line.strip():
                continue
            t = line.split()
            if len(t) < 20:
                continue
            utc = float(t[0])
            lat = float(t[3]) + float(t[4]) / 60.0 + float(t[5]) / 3600.0
            lon = float(t[6]) + float(t[7]) / 60.0 + float(t[8]) / 3600.0
            h   = float(t[9])
            q   = int(t[19])
            rows.append({'utc': utc, 'lat': lat, 'lon': lon, 'h': h, 'quality': q})
    df = pd.DataFrame(rows)
    print(f"  [GT]  {len(df)} points  |  "
          f"t={df['utc'].min():.2f} → {df['utc'].max():.2f}  "
          f"({df['utc'].max()-df['utc'].min():.1f} s)")
    return df


def parse_tum(path):
    """Parse TUM trajectory file  →  DataFrame with timestamp + xyz + quat."""
    rows = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            t = line.split()
            if len(t) < 8:
                continue
            rows.append({
                'ts': float(t[0]),
                'tx': float(t[1]), 'ty': float(t[2]), 'tz': float(t[3]),
                'qx': float(t[4]), 'qy': float(t[5]),
                'qz': float(t[6]), 'qw': float(t[7]),
            })
    df = pd.DataFrame(rows)
    print(f"  [EST] {len(df)} poses  |  "
          f"t={df['ts'].min():.2f} → {df['ts'].max():.2f}  "
          f"({df['ts'].max()-df['ts'].min():.1f} s)")
    return df


# ═══════════════════════════════════════════════════════════════════════════════
# 2. GPS → LOCAL ENU
# ═══════════════════════════════════════════════════════════════════════════════

def geodetic_to_enu(lat, lon, h, lat0, lon0, h0):
    """
    Convert arrays of (lat, lon, h) in degrees/meters to local ENU
    relative to origin (lat0, lon0, h0).
    Returns (E, N, U) arrays in meters.
    """
    a  = 6378137.0          # WGS84 semi-major axis
    e2 = 6.69437999014e-3   # WGS84 first eccentricity squared

    def to_ecef(la, lo, al):
        la, lo = np.radians(la), np.radians(lo)
        N = a / np.sqrt(1 - e2 * np.sin(la)**2)
        x = (N + al) * np.cos(la) * np.cos(lo)
        y = (N + al) * np.cos(la) * np.sin(lo)
        z = (N * (1 - e2) + al) * np.sin(la)
        return x, y, z

    x,  y,  z  = to_ecef(lat,  lon,  h)
    x0, y0, z0 = to_ecef(lat0, lon0, h0)
    dx, dy, dz = x - x0, y - y0, z - z0

    la0, lo0 = np.radians(lat0), np.radians(lon0)
    E = -np.sin(lo0)*dx + np.cos(lo0)*dy
    N = (-np.sin(la0)*np.cos(lo0)*dx
         - np.sin(la0)*np.sin(lo0)*dy
         + np.cos(la0)*dz)
    U = (np.cos(la0)*np.cos(lo0)*dx
         + np.cos(la0)*np.sin(lo0)*dy
         + np.sin(la0)*dz)
    return E, N, U


# ═══════════════════════════════════════════════════════════════════════════════
# 3. TEMPORAL ALIGNMENT
# ═══════════════════════════════════════════════════════════════════════════════

def temporal_align(gt_df, est_df, max_dt=0.6):
    """
    For each GT timestamp find the nearest EST timestamp.
    Drop pairs with time gap > max_dt seconds.
    Returns aligned DataFrame.
    """
    gt_ts  = gt_df['utc'].values
    est_ts = est_df['ts'].values

    matched_est_idx = []
    dts = []
    for t in gt_ts:
        idx = np.argmin(np.abs(est_ts - t))
        dt  = abs(est_ts[idx] - t)
        matched_est_idx.append(idx)
        dts.append(dt)

    dts = np.array(dts)
    valid = dts <= max_dt

    gt_matched  = gt_df.iloc[valid].reset_index(drop=True)
    est_matched = est_df.iloc[np.array(matched_est_idx)[valid]].reset_index(drop=True)

    print(f"  [ALIGN] {valid.sum()}/{len(gt_ts)} GT points matched  "
          f"(max_dt={max_dt}s, dropped {(~valid).sum()})")
    return gt_matched, est_matched


# ═══════════════════════════════════════════════════════════════════════════════
# 4. UMEYAMA SE3 ALIGNMENT  (scale=1, like evo_ape -a)
# ═══════════════════════════════════════════════════════════════════════════════

def umeyama_align(src, dst):
    """
    Find R, t such that  dst ≈ R @ src + t  (least-squares, no scale).
    src, dst : (N, 3) arrays
    Returns R (3×3), t (3,), src_aligned (N,3)
    """
    mu_src = src.mean(axis=0)
    mu_dst = dst.mean(axis=0)
    src_c  = src - mu_src
    dst_c  = dst - mu_dst

    H = src_c.T @ dst_c / len(src)
    U, S, Vt = np.linalg.svd(H)
    d = np.linalg.det(Vt.T @ U.T)
    D = np.diag([1, 1, d])
    R = Vt.T @ D @ U.T
    t = mu_dst - R @ mu_src

    src_aligned = (R @ src.T).T + t
    return R, t, src_aligned


# ═══════════════════════════════════════════════════════════════════════════════
# 5. ERROR METRICS
# ═══════════════════════════════════════════════════════════════════════════════

def compute_errors(gt_xyz, est_aligned):
    err3d = np.linalg.norm(gt_xyz - est_aligned, axis=1)
    err2d = np.linalg.norm(gt_xyz[:, :2] - est_aligned[:, :2], axis=1)
    errh  = np.abs(gt_xyz[:, 2] - est_aligned[:, 2])
    return err2d, err3d, errh


def print_metrics(err2d, err3d, errh):
    def pm(arr, name):
        print(f"  {'─'*42}")
        print(f"  {name}")
        print(f"  {'─'*42}")
        print(f"  {'RMS':<22} {np.sqrt((arr**2).mean()):>10.4f} m")
        print(f"  {'Mean':<22} {arr.mean():>10.4f} m")
        print(f"  {'Median':<22} {np.median(arr):>10.4f} m")
        print(f"  {'Std':<22} {arr.std():>10.4f} m")
        print(f"  {'P50':<22} {np.percentile(arr,50):>10.4f} m")
        print(f"  {'P68':<22} {np.percentile(arr,68):>10.4f} m")
        print(f"  {'P95':<22} {np.percentile(arr,95):>10.4f} m")
        print(f"  {'Max':<22} {arr.max():>10.4f} m")

    print(f"\n{'═'*46}")
    print(f"  POSITIONING ACCURACY REPORT")
    print(f"{'═'*46}")
    pm(err2d, "2D Horizontal Error (E-N plane)")
    pm(err3d, "3D Error")
    pm(errh,  "Height / Vertical Error")
    print(f"  {'═'*42}")


# ═══════════════════════════════════════════════════════════════════════════════
# 6. VISUALISATION
# ═══════════════════════════════════════════════════════════════════════════════

Q_COLOR = {1: '#2ecc71', 2: '#3498db', 3: '#f39c12', 4: '#e74c3c'}
Q_LABEL = {1: 'Fixed RTK', 2: 'Float RTK', 3: 'SBAS/DGNSS', 4: 'Single'}
BG      = '#0f1117'
AX_BG   = '#1a1d27'
GRID_C  = '#2a2d3a'


def sax(ax, title='', xl='', yl=''):
    ax.set_facecolor(AX_BG)
    ax.tick_params(colors='#aaa', labelsize=8)
    for sp in ax.spines.values():
        sp.set_edgecolor('#333')
    ax.set_title(title, color='#eee', fontsize=10, pad=6, weight='bold')
    if xl: ax.set_xlabel(xl, color='#aaa', fontsize=8)
    if yl: ax.set_ylabel(yl, color='#aaa', fontsize=8)
    ax.grid(True, color=GRID_C, linewidth=0.5, linestyle='--')


def plot_results(gt_enu, est_aligned, gt_matched, err2d, err3d, errh,
                 est_full_xyz, out_path):
    """Generate 10-panel analysis figure."""

    t = gt_matched['utc'].values - gt_matched['utc'].values[0]
    qualities = gt_matched['quality'].values
    q_colors  = [Q_COLOR.get(q, '#aaa') for q in qualities]

    fig = plt.figure(figsize=(20, 24), facecolor=BG)
    fig.suptitle(
        'FAST-LiVO2 vs UrbanNav Ground Truth  ·  Positioning Accuracy Report\n'
        'UrbanNav-HK-Medium-Urban-1  |  SE3 Aligned (Umeyama)',
        fontsize=15, color='white', y=0.99, weight='bold')

    gs = gridspec.GridSpec(4, 3, figure=fig,
                           hspace=0.45, wspace=0.35,
                           top=0.95, bottom=0.04,
                           left=0.07, right=0.97)

    # ── Panel 1: Matched-only trajectory overlay (2 cols wide) ───────────
    ax0 = fig.add_subplot(gs[0:2, 0:2])
    sax(ax0, 'Trajectory Overlay  (matched portion only)', 'East (m)', 'North (m)')
    # Only plot the matched GT segment
    ax0.plot(gt_enu[:, 0], gt_enu[:, 1],
             color='#3498db', lw=1.4, label='Ground Truth (matched)', zorder=3)
    ax0.plot(est_full_xyz[:, 0], est_full_xyz[:, 1],
             color='#e74c3c', lw=1.1, alpha=0.85, label='FAST-LiVO2 (aligned)', zorder=2)
    # Draw error lines at matched points (every 5th)
    for i in range(0, len(gt_enu), 5):
        ax0.plot([gt_enu[i, 0], est_aligned[i, 0]],
                 [gt_enu[i, 1], est_aligned[i, 1]],
                 color='#f1c40f', lw=0.4, alpha=0.45, zorder=1)
    # Start / end markers on matched segment
    ax0.plot(gt_enu[0, 0],  gt_enu[0, 1],  'w*', ms=14, zorder=5, label='Start')
    ax0.plot(gt_enu[-1, 0], gt_enu[-1, 1], 'wX', ms=12, zorder=5, label='End (SLAM stopped)')
    ax0.legend(fontsize=8, framealpha=0.35, labelcolor='white',
               facecolor=AX_BG, edgecolor='#444')

    # ── Panel 2: 2D error over time, colored by quality ───────────────────
    ax1 = fig.add_subplot(gs[0, 2])
    sax(ax1, '2D Horizontal Error over Time', 'Time (s)', 'Error (m)')
    ax1.scatter(t, err2d, c=q_colors, s=3, zorder=3)
    ax1.axhline(err2d.mean(),              color='#f1c40f', lw=1.2,
                linestyle='--', label=f'Mean {err2d.mean():.2f}m')
    ax1.axhline(np.percentile(err2d, 95), color='#e74c3c', lw=1.0,
                linestyle=':', label=f'P95  {np.percentile(err2d,95):.2f}m')
    ax1.legend(fontsize=7, framealpha=0.3, labelcolor='white',
               facecolor=AX_BG, edgecolor='#444')
    # quality legend patches
    from matplotlib.patches import Patch
    patches = [Patch(color=c, label=Q_LABEL[q]) for q, c in Q_COLOR.items()
               if q in np.unique(qualities)]
    ax1.legend(handles=patches, fontsize=6, framealpha=0.3, labelcolor='white',
               facecolor=AX_BG, edgecolor='#444', loc='upper right')

    # ── Panel 3: Height error over time ───────────────────────────────────
    ax2 = fig.add_subplot(gs[1, 2])
    sax(ax2, 'Height (Vertical) Error over Time', 'Time (s)', 'Error (m)')
    ax2.plot(t, errh, color='#9b59b6', lw=0.8)
    ax2.fill_between(t, 0, errh, alpha=0.2, color='#9b59b6')
    ax2.axhline(errh.mean(), color='#f1c40f', lw=1, linestyle='--',
                label=f'Mean {errh.mean():.2f}m')
    ax2.legend(fontsize=7, framealpha=0.3, labelcolor='white',
               facecolor=AX_BG, edgecolor='#444')

    # ── Panel 4: CDF 2D ───────────────────────────────────────────────────
    ax3 = fig.add_subplot(gs[2, 0])
    sax(ax3, 'CDF of 2D Horizontal Error', '2D Error (m)', 'Cumulative Probability')
    se = np.sort(err2d)
    ax3.plot(se, np.arange(1, len(se)+1)/len(se), color='#1abc9c', lw=2)
    for pct, col in [(50, '#f39c12'), (68, '#e67e22'), (95, '#e74c3c')]:
        v = np.percentile(err2d, pct)
        ax3.axvline(v, color=col, lw=1, linestyle='--',
                    label=f'P{pct}={v:.2f}m')
    ax3.legend(fontsize=8, framealpha=0.3, labelcolor='white',
               facecolor=AX_BG, edgecolor='#444')

    # ── Panel 5: 2D error histogram ───────────────────────────────────────
    ax4 = fig.add_subplot(gs[2, 1])
    sax(ax4, '2D Error Histogram', '2D Error (m)', 'Count')
    ax4.hist(err2d, bins=60, color='#e67e22', edgecolor=BG, lw=0.3)
    ax4.axvline(err2d.mean(), color='#f1c40f', lw=1.2, linestyle='--',
                label=f'Mean {err2d.mean():.2f}m')
    ax4.axvline(np.percentile(err2d, 95), color='#e74c3c', lw=1,
                linestyle=':', label=f'P95 {np.percentile(err2d,95):.2f}m')
    ax4.legend(fontsize=7, framealpha=0.3, labelcolor='white',
               facecolor=AX_BG, edgecolor='#444')

    # ── Panel 6: 3D error histogram ───────────────────────────────────────
    ax5 = fig.add_subplot(gs[2, 2])
    sax(ax5, '3D Error Histogram', '3D Error (m)', 'Count')
    ax5.hist(err3d, bins=60, color='#8e44ad', edgecolor=BG, lw=0.3)
    ax5.axvline(err3d.mean(), color='#f1c40f', lw=1.2, linestyle='--',
                label=f'Mean {err3d.mean():.2f}m')
    ax5.legend(fontsize=7, framealpha=0.3, labelcolor='white',
               facecolor=AX_BG, edgecolor='#444')

    # ── Panel 7: Error heatmap on trajectory ──────────────────────────────
    ax6 = fig.add_subplot(gs[3, 0:2])
    sax(ax6, 'Error Heatmap on Trajectory  (color = 2D error magnitude)',
        'East (m)', 'North (m)')
    norm = Normalize(vmin=0, vmax=np.percentile(err2d, 95))
    sc   = ax6.scatter(gt_enu[:, 0], gt_enu[:, 1],
                       c=err2d, cmap='RdYlGn_r', norm=norm, s=8, zorder=3)
    cbar = fig.colorbar(sc, ax=ax6, pad=0.01)
    cbar.ax.yaxis.set_tick_params(color='#aaa', labelsize=7)
    cbar.set_label('2D Error (m)', color='#aaa', fontsize=8)
    plt.setp(cbar.ax.yaxis.get_ticklabels(), color='#aaa')

    # ── Panel 8: Summary metrics box ──────────────────────────────────────
    ax7 = fig.add_subplot(gs[3, 2])
    ax7.set_facecolor(AX_BG)
    ax7.set_xlim(0, 1); ax7.set_ylim(0, 1)
    for sp in ax7.spines.values(): sp.set_edgecolor('#333')
    ax7.tick_params(left=False, bottom=False, labelleft=False, labelbottom=False)
    ax7.set_title('Accuracy Summary', color='#eee', fontsize=10, pad=6, weight='bold')

    def rms(a): return np.sqrt((a**2).mean())
    metrics = [
        ('Matched Points',      f'{len(err2d):,}'),
        ('── 2D Horizontal ──', ''),
        ('  RMS',               f'{rms(err2d):.3f} m'),
        ('  Mean',              f'{err2d.mean():.3f} m'),
        ('  Median',            f'{np.median(err2d):.3f} m'),
        ('  P68',               f'{np.percentile(err2d,68):.3f} m'),
        ('  P95',               f'{np.percentile(err2d,95):.3f} m'),
        ('  Max',               f'{err2d.max():.3f} m'),
        ('── 3D ──',            ''),
        ('  RMS',               f'{rms(err3d):.3f} m'),
        ('── Height ──',        ''),
        ('  RMS',               f'{rms(errh):.3f} m'),
        ('  Mean',              f'{errh.mean():.3f} m'),
    ]
    for i, (k, v) in enumerate(metrics):
        y = 0.97 - i * 0.071
        col = '#f39c12' if k.startswith('──') else '#aaa'
        ax7.text(0.03, y, k, color=col, fontsize=8, va='top', style='italic' if '──' in k else 'normal')
        ax7.text(0.97, y, v, color='#fff', fontsize=8, va='top', ha='right', weight='bold')

    plt.savefig(out_path, dpi=150, bbox_inches='tight', facecolor=BG)
    print(f"\n  [PLOT] Saved → {out_path}")


# ═══════════════════════════════════════════════════════════════════════════════
# 7. MAIN
# ═══════════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description='UrbanNav × FAST-LiVO2 trajectory evaluator',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    parser.add_argument('--gt',      required=True,
                        help='Path to UrbanNav_TST_GT_raw.txt')
    parser.add_argument('--est',     required=True,
                        help='Path to urbannav_tst.txt (TUM format from FAST-LiVO2)')
    parser.add_argument('--max_dt',  type=float, default=0.6,
                        help='Max time gap (s) for matching GT↔EST (default: 0.6)')
    parser.add_argument('--out',     default='urbannav_eval_result.png',
                        help='Output PNG filename (default: urbannav_eval_result.png)')
    parser.add_argument('--csv',     default='urbannav_errors.csv',
                        help='Output per-epoch error CSV (default: urbannav_errors.csv)')
    args = parser.parse_args()

    print('\n' + '═'*50)
    print('  UrbanNav × FAST-LiVO2 Evaluator')
    print('═'*50)

    # ── Load ─────────────────────────────────────────────────────────────
    print('\n[1] Loading files …')
    gt_df  = parse_gt(args.gt)
    est_df = parse_tum(args.est)

    # ── ENU origin = first GT point ───────────────────────────────────────
    lat0, lon0, h0 = gt_df['lat'].iloc[0], gt_df['lon'].iloc[0], gt_df['h'].iloc[0]
    print(f'\n[2] ENU origin: lat={lat0:.6f}°  lon={lon0:.6f}°  h={h0:.3f}m')

    E, N, U = geodetic_to_enu(gt_df['lat'].values, gt_df['lon'].values,
                               gt_df['h'].values, lat0, lon0, h0)
    gt_df['E'], gt_df['N'], gt_df['U'] = E, N, U
    gt_full_enu = np.column_stack([E, N, U])

    # ── Temporal alignment ────────────────────────────────────────────────
    print('\n[3] Temporal alignment …')
    gt_m, est_m = temporal_align(gt_df, est_df, max_dt=args.max_dt)

    gt_enu  = np.column_stack([gt_m['E'].values,  gt_m['N'].values,  gt_m['U'].values])
    est_xyz = np.column_stack([est_m['tx'].values, est_m['ty'].values, est_m['tz'].values])
    est_full_xyz_raw = np.column_stack([est_df['tx'].values,
                                        est_df['ty'].values,
                                        est_df['tz'].values])

    # ── SE3 Umeyama alignment ─────────────────────────────────────────────
    print('\n[4] SE3 Umeyama alignment (SLAM → GT ENU) …')
    R, t, est_aligned = umeyama_align(est_xyz, gt_enu)
    # Apply same transform to full SLAM trajectory for plotting
    est_full_aligned = (R @ est_full_xyz_raw.T).T + t
    print(f'  Translation: {t}')
    print(f'  Rotation (deg): {np.degrees(Rotation.from_matrix(R).as_euler("xyz"))}')

    # ── Errors ────────────────────────────────────────────────────────────
    print('\n[5] Computing errors …')
    err2d, err3d, errh = compute_errors(gt_enu, est_aligned)
    print_metrics(err2d, err3d, errh)

    # ── Save CSV ──────────────────────────────────────────────────────────
    out_df = gt_m[['utc', 'lat', 'lon', 'h', 'quality', 'E', 'N', 'U']].copy()
    out_df['est_E']   = est_aligned[:, 0]
    out_df['est_N']   = est_aligned[:, 1]
    out_df['est_U']   = est_aligned[:, 2]
    out_df['err_2d']  = err2d
    out_df['err_3d']  = err3d
    out_df['err_h']   = errh
    out_df.to_csv(args.csv, index=False)
    print(f'\n  [CSV]  Saved → {args.csv}')

    # ── Plot ──────────────────────────────────────────────────────────────
    print('\n[6] Generating plots …')
    plot_results(gt_enu, est_aligned, gt_m,
                 err2d, err3d, errh,
                 est_full_aligned,
                 args.out)

    print('\n' + '═'*50)
    print('  DONE')
    print('═'*50 + '\n')


if __name__ == '__main__':
    main()