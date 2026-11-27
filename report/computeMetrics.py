import os, math
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
from rclpy.time import Time
from rosbag2_reader_py import Rosbag2Reader

BAGS = {
    "task1": ("Task 1", os.path.expanduser("~/labEkf_ws/real_task1")),
    "task2": ("Task 2", os.path.expanduser("~/labEkf_ws/real_task2")),
}
OUT_DIR = os.path.expanduser("~/labEkf_ws/report")

# True for real robot (map + real landmarks); False for simulation (grid + sim landmarks)
IS_REAL = True
PGM_PATH = os.path.expanduser("~/labEkf_ws/map.pgm")
YAML_PATH = os.path.expanduser("~/labEkf_ws/map.yaml")

LANDMARKS_REAL = {
    0: (4.927, 0.772), 1: (3.758, 1.431), 2: (1.851, 0.379),
    4: (1.616, -0.025), 6: (2.609, 0.239), 7: (3.924, 0.599),
    10: (1.150, 1.227), 11: (3.058, 0.159), 14: (4.179, 0.324),
}

# Simulation landmarks 
LANDMARKS_SIM = {
    11: (-1.1, -1.1), 12: (-1.1, 0.0), 13: (-1.1, 1.1),
    21: (0.0, -1.1),  22: (0.0, 0.0),  23: (0.0, 1.1),
    31: (1.1, -1.1),  32: (1.1, 0.0),  33: (1.1, 1.1),
}

wrap = lambda a: (a + np.pi) % (2 * np.pi) - np.pi
yaw = lambda q: math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def load_map():
    x0, y0, res = -3.2154, -1.3259, 0.05
    with open(YAML_PATH) as f:
        lines = [l.strip() for l in f if l.strip()]
    for i, l in enumerate(lines):
        if l.startswith("resolution:"):
            res = float(l.split(":")[1])
        elif l.startswith("origin:"):
            x0, y0 = [float(x[1:]) for x in lines[i + 1 : i + 3]]
    with open(PGM_PATH, "rb") as f:
        f.readline()
        toks = []
        while len(toks) < 3:
            toks += f.readline().split(b"#")[0].split()
        w, h = int(toks[0]), int(toks[1])
        img = np.frombuffer(f.read(w * h), dtype=np.uint8).reshape((h, w))
    return img, [x0, x0 + w * res, y0, y0 + h * res]


def metrics(ref, est):
    xy = interp1d(ref["t"], ref["xy"], axis=0, kind="nearest", fill_value="extrapolate")(est["t"])
    th = wrap(interp1d(ref["t"], np.unwrap(ref["th"]), kind="nearest", fill_value="extrapolate")(est["t"]))
    err = np.column_stack((est["xy"] - xy, wrap(est["th"] - th)))
    return np.mean(np.abs(err), axis=0), np.sqrt(np.mean(err ** 2, axis=0))


os.makedirs(OUT_DIR, exist_ok=True)
labels = {"/odom": "Odom", "/ekf": "EKF", "/ground_truth": "Ground Truth"}
landmarks = LANDMARKS_REAL if IS_REAL else LANDMARKS_SIM
map_data = load_map() if (IS_REAL and os.path.exists(PGM_PATH)) else None

for prefix, (title, bag_path) in BAGS.items():
    if not os.path.isdir(bag_path) or not os.listdir(bag_path):
        print(f"Salto {title}: percorso bag vuoto o non impostato ({bag_path})")
        continue

    reader = Rosbag2Reader(bag_path)
    wanted = [t for t in ["/ground_truth", "/odom", "/ekf"] if t in reader.all_topics]
    reader.set_filter(wanted)

    data = {t: {"t": [], "xy": [], "th": []} for t in wanted}
    for topic, msg, _ in reader:
        data[topic]["t"].append(Time.from_msg(msg.header.stamp).nanoseconds * 1e-9)
        data[topic]["xy"].append((msg.pose.pose.position.x, msg.pose.pose.position.y))
        data[topic]["th"].append(yaw(msg.pose.pose.orientation))

    t0 = min(data[t]["t"][0] for t in wanted if data[t]["t"])
    for t in wanted:
        data[t] = {k: np.array(v) - (t0 if k == "t" else 0) for k, v in data[t].items()}

    # 1. 2D Trajectory
    fig, ax = plt.subplots(figsize=(7.5, 7))
    if map_data:
        ax.imshow(map_data[0], cmap="gray", origin="upper", extent=map_data[1], alpha=0.85)
    for t in wanted:
        ax.plot(data[t]["xy"][:, 0], data[t]["xy"][:, 1], lw=1.8, label=labels[t])
    if landmarks:
        lx, ly = zip(*landmarks.values())
        ax.scatter(lx, ly, c="k", s=50, zorder=5, label="Landmark")
        for k, (x, y) in landmarks.items():
            ax.annotate(str(k), (x, y), xytext=(5, 4), textcoords="offset points", fontsize=9)
    ax.set(title=f"2D Trajectory (X-Y) with Landmark - {title}", xlabel="X [m]", ylabel="Y [m]")
    ax.set_aspect("equal", adjustable="datalim")
    ax.grid(True, alpha=0.7)
    ax.legend(title="Signal")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT_DIR, f"{prefix}_traj.png"), dpi=200)

    # 2. Time comparison x(t), y(t), theta(t)
    fig, axs = plt.subplots(3, 1, figsize=(10, 7.5), sharex=True)
    for t in wanted:
        for i in range(3):
            axs[i].plot(data[t]["t"], data[t]["xy"][:, i] if i < 2 else data[t]["th"], lw=1.4, label=labels[t])
    for ax, yl in zip(axs, ["x [m]", "y [m]", "theta [rad]"]):
        ax.set_ylabel(yl)
        ax.grid(True, alpha=0.7)
        ax.legend(loc="upper right")
    axs[0].set_title(f"State evolution x(t), y(t), theta(t) - {title}")
    axs[2].set_xlabel("Time [s]")
    fig.tight_layout()
    fig.savefig(os.path.join(OUT_DIR, f"{prefix}_states.png"), dpi=200)

    # 3. MAE / RMSE table
    pairs = [("/ground_truth", t) for t in ["/odom", "/ekf"] if t in data] if "/ground_truth" in data else [("/odom", "/ekf")]
    for ref_t, est_t in pairs:
        if ref_t in data and est_t in data:
            mae, rmse = metrics(data[ref_t], data[est_t])
            print(f"\n=== {title}: {labels[est_t]} vs {labels[ref_t]} ===")
            print(f"MAE  (x, y, theta): {mae[0]:.4f}, {mae[1]:.4f}, {mae[2]:.4f}")
            print(f"RMSE (x, y, theta): {rmse[0]:.4f}, {rmse[1]:.4f}, {rmse[2]:.4f}")

plt.show()