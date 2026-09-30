#!/usr/bin/env python3
"""Generate the Real-World Validation figures and metrics for the S3VO report.

Reads the per-trial CSVs written by extract_bags.py plus config/report_trials.yaml,
and writes into <report_dir>:
  figures/realworld/<bag>_timeseries.pdf   trajectory + distance/surge/yaw-rate plots
  figures/realworld/<bag>_aerial.jpg       3-frame aerial strip (approach, CPA, recovery)
  figures/realworld/<bag>_vspace.jpg       Foxglove velocity-space / obstacle snapshot
  realworld_metrics.tex                    \\newcommand macros + summary table

Velocities plotted are the waypoint controller's desired_vel and S3VO's cmd_vel
(never the odometry velocity). Times are seconds since Guided engagement.

Usage: python3 make_report.py <data_dir> <report_dir> [--config ../config/report_trials.yaml]
"""
import argparse
import math
import os
import subprocess

import matplotlib
import numpy as np
import yaml

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.gridspec import GridSpec  # noqa: E402

from trialdata import Trial, load_csv  # noqa: E402

LILY_C = "#2a78d6"
NAUT_C = "#eb6834"
INK = "#0b0b0b"
INK2 = "#52514e"
MUTED = "#9a9892"
GRID = "#e4e3df"
S_ENU = 111319.5  # same flat-earth scale as go_to_gps_waypoint.cpp
INTERVENTION_TOL = 0.05  # |cmd - desired| above this (m/s or rad/s) counts as S3VO acting
FOX_PANELS = (840, 326, 1920, 950)  # Velocity Samples + Obstacle Representation panels

plt.rcParams.update({
    "font.family": "serif", "font.size": 8, "axes.titlesize": 8, "axes.labelsize": 8,
    "legend.fontsize": 7, "xtick.labelsize": 7, "ytick.labelsize": 7,
    "axes.edgecolor": INK2, "axes.linewidth": 0.6, "xtick.color": INK2, "ytick.color": INK2,
    "axes.labelcolor": INK, "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.5,
    "axes.spines.top": False, "axes.spines.right": False, "lines.linewidth": 1.4,
})


# ---------------------------------------------------------------- analysis

def origin_offset(data_dir, bag):
    """NAUTILUS local-ENU origin expressed in LILY's local-ENU frame."""
    lo = load_csv(os.path.join(data_dir, bag, "lily_origin.csv"))
    no = load_csv(os.path.join(data_dir, bag, "naut_origin.csv"))
    lo, no = np.atleast_1d(lo)[0], np.atleast_1d(no)[0]
    lat0 = float(lo["lat"])
    return ((float(no["lon"]) - float(lo["lon"])) * math.cos(math.radians(lat0)) * S_ENU,
            (float(no["lat"]) - float(lo["lat"])) * S_ENU)


def detect_window(T):
    """Guided run = last sustained non-zero thruster segment; ends at the last thrust change."""
    n = min(len(T.thrust_l), len(T.thrust_r))
    t = T.thrust_l["t"][:n] - T.t0
    left, right = T.thrust_l["v"][:n], T.thrust_r["v"][:n]
    active = (np.abs(left) + np.abs(right)) > 1e-3
    starts = np.flatnonzero(np.diff(active.astype(int)) == 1) + 1
    if active[0]:
        starts = np.r_[0, starts]
    ends = np.flatnonzero(np.diff(active.astype(int)) == -1)
    segs = []
    for s in starts:
        e = ends[ends >= s]
        e = e[0] if len(e) else n - 1
        if t[e] - t[s] > 10.0:
            segs.append((t[s], t[e]))
    start = segs[-1][0]
    changing = np.flatnonzero((np.abs(np.diff(left)) + np.abs(np.diff(right))) > 1e-6)
    end = t[changing[-1] + 1]
    return start, end


def analyse(cfg, data_dir, tc):
    T = Trial(data_dir, tc["bag"])
    ox, oy = origin_offset(data_dir, tc["bag"])
    auto = detect_window(T)
    win = [w if w is not None else a for w, a in zip(tc.get("window") or [None, None], auto)]
    t_s, t_e = win

    L = T.lily
    tl = L["t"] - T.t0
    m = (tl >= t_s) & (tl <= t_e)
    t = tl[m]
    lx, ly, lyaw = L["x"][m], L["y"][m], L["yaw"][m]
    N = T.naut
    tn = N["t"] - T.t0
    nx_all, ny_all = N["x"] + ox, N["y"] + oy
    nx, ny = np.interp(t, tn, nx_all), np.interp(t, tn, ny_all)

    d = np.hypot(lx - nx, ly - ny)
    k = int(np.argmin(d))
    r_naut = float(np.median(T.elements["r"][T.elements["dynamic"] == 1])) / 2.0  # size.x is a diameter
    r_b = r_naut + cfg["lily_radius"]

    deploy = tc.get("type") == "deploy"
    goal = np.array(tc.get("goal_enu") or cfg["goal_enu"])
    p0 = np.array([lx[0], ly[0]])
    u = (goal - p0) / max(np.linalg.norm(goal - p0), 1e-6)
    cross_track = (lx - p0[0]) * u[1] - (ly - p0[1]) * u[0]  # >0: right (starboard) of nominal line
    path_len = float(np.sum(np.hypot(np.diff(lx), np.diff(ly))))
    straight = float(np.hypot(*(np.array([lx[-1], ly[-1]]) - p0)))

    # NAUTILUS velocity from its GPS track (odom twist is body-frame; 1 Hz broadcast).
    nvx, nvy = np.gradient(nx_all, tn), np.gradient(ny_all, tn)
    mn = (tn >= t_s) & (tn <= t_e)
    naut_speed = float(np.median(np.hypot(nvx[mn], nvy[mn])))
    # Encounter angle: NAUTILUS course relative to LILY's course when it first comes within 25 m.
    j = int(np.argmax(d < 25.0)) if np.any(d < 25.0) else 0
    nv = np.array([np.interp(t[j], tn, nvx), np.interp(t[j], tn, nvy)])
    if np.linalg.norm(nv) > 0.3:
        rel_course = math.degrees(math.atan2(nv[1], nv[0]) - lyaw[j])
        rel_course = (rel_course + 180.0) % 360.0 - 180.0
    else:
        rel_course = float("nan")  # obstacle effectively stationary
    # Side on which NAUTILUS is passed (in LILY body frame at CPA; FLU => y>0 is port).
    rx, ry = nx[k] - lx[k], ny[k] - ly[k]
    y_body = -math.sin(lyaw[k]) * rx + math.cos(lyaw[k]) * ry
    naut_side = "port" if y_body > 0 else "starboard"

    D, C = T.desired, T.cmd
    tcmd = C["t"] - T.t0
    mc = (tcmd >= t_s) & (tcmd <= t_e)
    td = D["t"] - T.t0
    cvx, cwz = C["vx"][mc], C["wz"][mc]
    dvx, dwz = np.interp(tcmd[mc], td, D["vx"]), np.interp(tcmd[mc], td, D["wz"])
    acting = (np.abs(cvx - dvx) > INTERVENTION_TOL) | (np.abs(cwz - dwz) > INTERVENTION_TOL)
    dev = np.hypot(cvx - dvx, cwz - dwz)

    # DURIUS (static element) distance, for completeness.
    E = T.elements
    st = E[E["dynamic"] == 0]
    if len(st):
        i = np.clip(np.searchsorted(L["t"], st["t"]) - 1, 0, None)
        yw = L["yaw"][i]
        dx = L["x"][i] + np.cos(yw) * st["x"] - np.sin(yw) * st["y"]
        dy = L["y"][i] + np.sin(yw) * st["x"] + np.cos(yw) * st["y"]
        durius = np.array([np.median(dx), np.median(dy)])
        durius_r = float(np.median(st["r"])) / 2.0
        durius_clear = float(np.min(np.hypot(lx - durius[0], ly - durius[1]))) - durius_r
    else:
        durius, durius_r, durius_clear = None, 0.0, float("nan")
    excursion = np.hypot(lx - goal[0], ly - goal[1])

    return dict(
        T=T, key=tc["key"], bag=tc["bag"], title=tc["title"], cfg=tc,
        t_s=t_s, t_e=t_e, auto_window=auto, t=t - t_s, lx=lx, ly=ly, nx=nx, ny=ny,
        naut_track=(tn - t_s, nx_all, ny_all), d=d, k=k, r_b=r_b, r_naut=r_naut,
        p0=p0, goal=goal, cross_track=cross_track, deploy=deploy,
        durius=durius, durius_r=durius_r,
        tcmd=tcmd[mc] - t_s, cvx=cvx, cwz=cwz, dvx=dvx, dwz=dwz, acting=acting, dev=dev,
        metrics=dict(
            duration=t_e - t_s,
            dmin=float(d[k]), tcpa=float(t[k] - t_s), clearance=float(d[k] - r_b),
            naut_speed=naut_speed, rel_course=rel_course, naut_side=naut_side,
            max_xtrack=float(np.max(np.abs(cross_track))),
            path_ratio=path_len / straight,
            max_surge_red=float(np.max(dvx - cvx)),
            max_yaw_dev=float(np.max(np.abs(cwz - dwz))),
            acting_pct=100.0 * float(np.mean(acting)),
            goal_dist_end=float(np.hypot(lx[-1] - goal[0], ly[-1] - goal[1])),
            durius_clear=durius_clear,
            max_excursion=float(np.max(excursion)),
            max_reverse=float(max(0.0, -np.min(cvx))),
        ),
    )


# ---------------------------------------------------------------- figures

def acting_spans(t, acting, min_gap=1.0):
    """Intervals where S3VO modifies the command, merging gaps shorter than min_gap seconds."""
    edges = np.flatnonzero(np.diff(np.r_[0, acting.astype(int), 0]))
    spans = []
    for a, b in zip(edges[::2], edges[1::2]):
        s, e = t[a], t[min(b, len(t) - 1)]
        if spans and s - spans[-1][1] < min_gap:
            spans[-1][1] = e
        else:
            spans.append([s, e])
    return spans


def shade_acting(ax, t, acting):
    for s, e in acting_spans(t, acting):
        ax.axvspan(s, e, color=LILY_C, alpha=0.10, lw=0)


def plot_timeseries(R, out):
    fig = plt.figure(figsize=(7.0, 3.9))
    gs = GridSpec(3, 2, figure=fig, width_ratios=[1.0, 1.25], wspace=0.28, hspace=0.18)
    ax = fig.add_subplot(gs[:, 0])
    p0 = R["goal"] if R["deploy"] else R["p0"]
    t, lx, ly = R["t"], R["lx"] - p0[0], R["ly"] - p0[1]
    tn, nx, ny = R["naut_track"]
    mn = (tn >= -1.0) & (tn <= t[-1] + 1.0)
    g = R["goal"] - p0
    if R["deploy"]:
        if R["durius"] is not None:
            c = R["durius"] - p0
            ax.add_patch(plt.Circle(c, R["durius_r"], color=MUTED, alpha=0.25, lw=0, label="DURIUS obstacle"))
            ax.add_patch(plt.Circle(c, R["durius_r"] + R["r_b"] - R["r_naut"], fill=False, ls=":",
                                    color=INK2, lw=0.8))
    else:
        ax.plot([0, g[0]], [0, g[1]], ls="--", color=MUTED, lw=0.9, label="Nominal path")
    ax.plot(nx[mn] - p0[0], ny[mn] - p0[1], color=NAUT_C, lw=1.4, label="NAUTILUS")
    ax.plot(lx, ly, color=LILY_C, lw=1.8, label="LILY")
    ax.plot(lx[0], ly[0], "o", color=LILY_C, ms=4)
    ax.plot(*g, marker="*", color=INK, ms=9, ls="none",
            label="Deployment point" if R["deploy"] else "Goal")
    # Matching time ticks every 5 s on both tracks.
    for ts in np.arange(0, t[-1], 5.0):
        i = np.searchsorted(t, ts)
        ax.plot(lx[i], ly[i], "o", ms=3.2, mfc="white", mec=LILY_C, mew=0.9, zorder=4)
        ax.plot(np.interp(ts, tn, nx) - p0[0], np.interp(ts, tn, ny) - p0[1], "s", ms=3.0,
                mfc="white", mec=NAUT_C, mew=0.9, zorder=4)
    k = R["k"]
    cx, cy = R["nx"][k] - p0[0], R["ny"][k] - p0[1]
    ax.plot([lx[k], cx], [ly[k], cy], color=INK, lw=0.8, zorder=5)
    ax.add_patch(plt.Circle((cx, cy), R["r_b"], fill=False, ls=":", color=NAUT_C, lw=1.0))
    ax.plot(lx[k], ly[k], "o", color=LILY_C, ms=4.5, zorder=6)
    ax.plot(cx, cy, "s", color=NAUT_C, ms=4.5, zorder=6)
    ax.annotate(f"CPA {R['metrics']['dmin']:.1f} m", ((lx[k] + cx) / 2, (ly[k] + cy) / 2),
                xytext=(6, -2), textcoords="offset points", fontsize=7, color=INK)
    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel("East [m]")
    ax.set_ylabel("North [m]")
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.13), ncol=2, frameon=False, handlelength=1.6)

    ax1 = fig.add_subplot(gs[0, 1])
    ax2 = fig.add_subplot(gs[1, 1], sharex=ax1)
    ax3 = fig.add_subplot(gs[2, 1], sharex=ax1)
    tc = R["tcmd"]
    tcpa = R["metrics"]["tcpa"]
    for a in (ax1, ax2, ax3):
        shade_acting(a, tc, R["acting"])
        a.axvline(tcpa, color=INK2, lw=0.7, ls=":")
    ax1.plot(t, R["d"], color=INK, lw=1.4)
    ax1.axhline(R["r_b"], color=NAUT_C, lw=0.9, ls=":")
    ax1.text(t[-1], R["r_b"], "  $R_B$", va="center", ha="left", fontsize=7, color=INK2)
    ax1.set_ylim(0, None)
    ax1.set_ylabel("Range [m]")
    ax2.plot(tc, R["dvx"], color=MUTED, ls="--", lw=1.2, label="Desired")
    ax2.plot(tc, R["cvx"], color=LILY_C, lw=1.4, label="S3VO command")
    ax2.set_ylabel("Surge [m/s]")
    ax2.legend(loc="best", frameon=False, ncol=2)
    ax3.plot(tc, R["dwz"], color=MUTED, ls="--", lw=1.2)
    ax3.plot(tc, R["cwz"], color=LILY_C, lw=1.4)
    ax3.set_ylabel("Yaw rate [rad/s]")
    ax3.set_xlabel("Time since Guided engaged [s]")
    for a in (ax1, ax2):
        plt.setp(a.get_xticklabels(), visible=False)
    ax1.set_xlim(0, t[-1])
    fig.align_ylabels([ax1, ax2, ax3])
    fig.savefig(out, bbox_inches="tight")
    plt.close(fig)


def grab_frame(video, t_video, crop=None, size=None):
    vf = []
    if crop:
        vf.append("crop={2}:{3}:{0}:{1}".format(*crop))
    w, h = (crop[2], crop[3]) if crop else size
    cmd = ["ffmpeg", "-loglevel", "error", "-ss", f"{max(t_video, 0):.2f}", "-i", video,
           "-map", "0:v:0", "-frames:v", "1"] + (["-vf", ",".join(vf)] if vf else []) + \
          ["-f", "rawvideo", "-pix_fmt", "rgb24", "-"]
    raw = subprocess.run(cmd, check=True, capture_output=True).stdout
    return np.frombuffer(raw, np.uint8).reshape(h, w, 3)


def plot_aerial(R, videos_dir, out):
    a = R["cfg"]["aerial"]
    tcpa = R["metrics"]["tcpa"]
    times = a.get("times") or [max(tcpa - 9.0, 0.0), tcpa, min(tcpa + 9.0, R["t"][-1])]
    video = os.path.join(videos_dir, a["file"])
    fig, axs = plt.subplots(1, 3, figsize=(7.0, 1.62))
    labels = ["Approach", "Closest approach", "Recovery"]
    for ax, ts, lab in zip(axs, times, labels):
        img = grab_frame(video, R["t_s"] + ts + a["offset"], crop=a["crop"])
        ax.imshow(img)
        ax.set_axis_off()
        ax.set_title(f"{lab} ($t\\approx{ts:.0f}$ s)", pad=2)
    fig.subplots_adjust(left=0, right=1, bottom=0, top=0.88, wspace=0.02)
    fig.savefig(out, dpi=300, pil_kwargs={"quality": 88})
    plt.close(fig)
    return times


def plot_vspace(R, videos_dir, out):
    f = R["cfg"]["foxglove"]
    ts = f.get("time")
    if ts is None:
        # Strongest S3VO intervention during the encounter (up to shortly after CPA).
        tcpa = R["metrics"]["tcpa"]
        m = (R["tcmd"] >= tcpa - 15.0) & (R["tcmd"] <= tcpa + 3.0)
        ts = float(R["tcmd"][m][int(np.argmax(R["dev"][m]))])
    video = os.path.join(videos_dir, f["file"])
    x0, y0, x1, y1 = FOX_PANELS
    img = grab_frame(video, R["t_s"] + ts + f["offset"], crop=[x0, y0, x1 - x0, y1 - y0])
    img = img[: int(f.get("keep_height", 0.72) * img.shape[0])]  # lower part is usually empty
    half = (x1 - x0) // 2
    fig, axs = plt.subplots(1, 2, figsize=(7.0, 4.15 * img.shape[0] / (y1 - y0) + 0.05))
    for ax, part, lab in zip(axs, (img[:, :half], img[:, half:]),
                             ("Velocity samples", "Obstacle representation")):
        ax.imshow(part)
        ax.set_axis_off()
        ax.set_title(lab, pad=2)
    fig.subplots_adjust(left=0, right=1, bottom=0, top=0.94, wspace=0.02)
    fig.savefig(out, dpi=250, pil_kwargs={"quality": 88})
    plt.close(fig)
    return ts


# ---------------------------------------------------------------- LaTeX

def fmt(v, nd=1):
    return "--" if v is None or (isinstance(v, float) and math.isnan(v)) else f"{v:.{nd}f}"


def write_tex(results, extras, path):
    lines = ["% Auto-generated by s3vo_analysis/scripts/make_report.py -- do not edit by hand.", ""]
    for R in results:
        k, M = R["key"], R["metrics"]
        vals = dict(Duration=fmt(M["duration"], 0), Dmin=fmt(M["dmin"]), Tcpa=fmt(M["tcpa"], 0),
                    Clearance=fmt(M["clearance"]), NautSpeed=fmt(M["naut_speed"]),
                    RelCourse=fmt(abs(M["rel_course"]) if not math.isnan(M["rel_course"]) else M["rel_course"], 0),
                    NautSide=M["naut_side"], MaxXtrack=fmt(M["max_xtrack"]),
                    PathRatio=fmt(M["path_ratio"], 2), SurgeRed=fmt(M["max_surge_red"], 2),
                    YawDev=fmt(M["max_yaw_dev"], 2), Acting=fmt(M["acting_pct"], 0),
                    GoalDist=fmt(M["goal_dist_end"]), DuriusClear=fmt(M["durius_clear"], 0),
                    Rb=fmt(R["r_b"]), Start=fmt(R["t_s"], 0),
                    Excursion=fmt(M["max_excursion"]), Reverse=fmt(M["max_reverse"], 2))
        vals.update(extras.get(k, {}))
        for name, v in vals.items():
            lines.append(f"\\newcommand{{\\rw{name}{k}}}{{{v}}}")
        lines.append("")
    durius = min(R["metrics"]["durius_clear"] for R in results if not R["deploy"])
    lines += [f"\\newcommand{{\\rwDuriusClearMin}}{{{fmt(durius, 0)}}}", ""]
    lines += [
        "\\newcommand{\\rwMetricsTable}{%",
        "\\begin{table}[htbp]",
        "\\centering",
        "\\caption{Per-trial metrics computed over the autonomous (Guided) window of each real-world trial. "
        "$d_{\\min}$ is the minimum centre-to-centre distance between LILY and NAUTILUS; clearance is "
        "$d_{\\min} - R_B$; cross-track is the largest lateral deviation from the straight start--goal line; "
        "intervention is the share of S3VO cycles whose command differs from the desired velocity by more "
        f"than {INTERVENTION_TOL:g}~m/s or rad/s.}}",
        "\\label{tab:rw_metrics}",
        "\\small",
        "\\setlength{\\tabcolsep}{4pt}",
        "\\begin{tabular}{l r r r r r r r r}",
        "\\toprule",
        "Trial & $\\|\\mathbf{v}_B\\|$ & $d_{\\min}$ & Clearance & Cross-track & Path & "
        "Max.\\ surge & Interv. & Duration \\\\",
        " & [m/s] & [m] & [m] & [m] & ratio & red.\\ [m/s] & [\\%] & [s] \\\\",
        "\\midrule",
    ]
    for R in (r for r in results if not r["deploy"]):
        M = R["metrics"]
        lines.append(f"{R['cfg'].get('short', R['title'])} & {fmt(M['naut_speed'])} & {fmt(M['dmin'])} & {fmt(M['clearance'])} & "
                     f"{fmt(M['max_xtrack'])} & {fmt(M['path_ratio'], 2)} & {fmt(M['max_surge_red'], 2)} & "
                     f"{fmt(M['acting_pct'], 0)} & {fmt(M['duration'], 0)} \\\\")
    lines += ["\\bottomrule", "\\end{tabular}", "\\end{table}", "}", ""]
    lines += [
        "\\newcommand{\\rwDeployTable}{%",
        "\\begin{table}[htbp]",
        "\\centering",
        "\\caption{Metrics of the deployment trials over the autonomous window. Excursion is the largest distance "
        "of LILY from the deployment point; reverse is the largest backwards speed commanded by S3VO; "
        "final is the distance to the deployment point at the end of the run.}",
        "\\label{tab:rw_deploy_metrics}",
        "\\small",
        "\\begin{tabular}{l r r r r r r r r}",
        "\\toprule",
        "Trial & $\\|\\mathbf{v}_B\\|$ & $d_{\\min}$ & Clearance & Excursion & Reverse & Interv. & Final & Duration \\\\",
        " & [m/s] & [m] & [m] & [m] & [m/s] & [\\%] & [m] & [s] \\\\",
        "\\midrule",
    ]
    for R in (r for r in results if r["deploy"]):
        M = R["metrics"]
        lines.append(f"{R['cfg'].get('short', R['title'])} & {fmt(M['naut_speed'])} & {fmt(M['dmin'])} & "
                     f"{fmt(M['clearance'])} & {fmt(M['max_excursion'])} & {fmt(M['max_reverse'], 2)} & "
                     f"{fmt(M['acting_pct'], 0)} & {fmt(M['goal_dist_end'])} & {fmt(M['duration'], 0)} \\\\")
    lines += ["\\bottomrule", "\\end{tabular}", "\\end{table}", "}", ""]
    with open(path, "w") as f:
        f.write("\n".join(lines))


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser()
    ap.add_argument("data_dir")
    ap.add_argument("report_dir")
    ap.add_argument("--config", default=os.path.join(here, "..", "config", "report_trials.yaml"))
    ap.add_argument("--no-video", action="store_true", help="skip aerial/Foxglove frame extraction")
    args = ap.parse_args()

    cfg = yaml.safe_load(open(args.config))
    videos_dir = os.path.expanduser(cfg["videos_dir"])
    fig_dir = os.path.join(args.report_dir, "figures", "realworld")
    os.makedirs(fig_dir, exist_ok=True)

    results, extras = [], {}
    for tc in cfg["trials"]:
        R = analyse(cfg, args.data_dir, tc)
        results.append(R)
        ex = extras.setdefault(R["key"], {})
        plot_timeseries(R, os.path.join(fig_dir, f"{R['bag']}_timeseries.pdf"))
        if not args.no_video and tc.get("aerial"):
            times = plot_aerial(R, videos_dir, os.path.join(fig_dir, f"{R['bag']}_aerial.jpg"))
            ex.update(AerialA=fmt(times[0], 0), AerialB=fmt(times[1], 0), AerialC=fmt(times[2], 0))
        if not args.no_video and tc.get("foxglove"):
            ts = plot_vspace(R, videos_dir, os.path.join(fig_dir, f"{R['bag']}_vspace.jpg"))
            ex["Snapshot"] = fmt(ts, 0)
        M = R["metrics"]
        print(f"{R['bag']:20s} window {R['t_s']:6.1f}-{R['t_e']:6.1f} (auto {R['auto_window'][0]:.1f}-"
              f"{R['auto_window'][1]:.1f})  dmin {M['dmin']:.2f} @ {M['tcpa']:.1f}s  clear {M['clearance']:.2f}  "
              f"vB {M['naut_speed']:.2f}  course {M['rel_course']:.0f}  NAUT on {M['naut_side']}  "
              f"xtrack {M['max_xtrack']:.1f}  ratio {M['path_ratio']:.2f}  surge-red {M['max_surge_red']:.2f}  "
              f"yawdev {M['max_yaw_dev']:.2f}  acting {M['acting_pct']:.0f}%  goal {M['goal_dist_end']:.1f}  "
              f"durius {M['durius_clear']:.0f}")
    write_tex(results, extras, os.path.join(args.report_dir, "realworld_metrics.tex"))


if __name__ == "__main__":
    main()
