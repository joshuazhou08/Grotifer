import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, FFMpegWriter

# === Load CSVs ===
orientation_df = pd.read_csv("logs/orientation.csv", header=None)
profile_df = pd.read_csv("logs/profile_orientation.csv", header=None)
angvel_df = pd.read_csv("logs/angular_velocity.csv", header=None)
profile_angvel_df = pd.read_csv("logs/profile_angular_velocity.csv", header=None)

def extract_rotations(df):
    data = df.to_numpy()
    return data[:, 0], data[:, 1:].reshape(-1, 3, 3)

def extract_angvel(df):
    data = df.to_numpy()
    return data[:, 0], data[:, 1:4]

# === Parse data ===
t_orient, R_orient = extract_rotations(orientation_df)
t_prof, R_prof = extract_rotations(profile_df)
t_ang, W_ang = extract_angvel(angvel_df)
t_prof_ang, W_prof = extract_angvel(profile_angvel_df)

# === Downsample orientation for speed ===
skip = max(1, len(t_orient) // 300)
t_orient = t_orient[::skip]
R_orient = R_orient[::skip]

# === Compute angular velocity error at matching timestamps (no interpolation) ===
error_mag = []
for t in t_ang:
    if len(t_prof_ang) == 0:
        error_mag.append(0)
        continue
    prof_idx = np.searchsorted(t_prof_ang, t)
    if prof_idx >= len(W_prof):
        prof_idx = len(W_prof) - 1
    err = np.linalg.norm(W_ang[np.searchsorted(t_ang, t)] - W_prof[prof_idx])
    error_mag.append(err)
error_mag = np.array(error_mag)

# === Compute position error (rotation angle between R_actual and R_profile) ===
position_error = []
for t in t_orient:
    if len(t_prof) == 0:
        position_error.append(0)
        continue
    prof_idx = np.searchsorted(t_prof, t)
    if prof_idx >= len(R_prof):
        prof_idx = len(R_prof) - 1
    Ra = R_orient[np.searchsorted(t_orient, t)]
    Rp = R_prof[prof_idx]
    dR = Rp.T @ Ra
    trace_val = np.clip((np.trace(dR) - 1) / 2, -1.0, 1.0)
    theta = np.arccos(trace_val)
    position_error.append(theta)
position_error = np.array(position_error)

# === Setup figure ===
fig = plt.figure(figsize=(8, 9))
gs = fig.add_gridspec(2, 1, height_ratios=[2, 1])
ax3d = fig.add_subplot(gs[0], projection="3d")
axerr = fig.add_subplot(gs[1])

# === 3D Setup ===
ax3d.set_box_aspect([1, 1, 1])
ax3d.set_xlim([-1.2, 1.2])
ax3d.set_ylim([-1.2, 1.2])
ax3d.set_zlim([-1.2, 1.2])
ax3d.set_xlabel("X")
ax3d.set_ylabel("Y")
ax3d.set_zlabel("Z")
ax3d.view_init(elev=-10, azim=0, roll=90)

# Dark, distinct colors
color_orient_actual = '#8B0000'   # dark red
color_orient_profile = '#003366'  # dark blue
color_w_actual = '#B22222'        # firebrick red
color_w_profile = '#1E90FF'       # dodger blue

# Orientation axes
lines_actual = [ax3d.plot([0, R_orient[0][0, i]], [0, R_orient[0][1, i]],
                          [0, R_orient[0][2, i]], color=color_orient_actual, lw=2)[0] for i in range(3)]
lines_profile = [ax3d.plot([0, 0], [0, 0], [0, 0], color=color_orient_profile, lw=2)[0] for _ in range(3)]

# Angular velocity vectors
w_actual = ax3d.quiver(0, 0, 0, 0, 0, 0, color=color_w_actual, length=0.8, normalize=True)
w_profile = ax3d.quiver(0, 0, 0, 0, 0, 0, color=color_w_profile, length=0.8, normalize=True)

# === Error plot ===
axerr.set_xlim(t_ang[0], t_ang[-1])

# zoom in to smaller errors (90th percentile)
max_err = max(np.percentile(error_mag, 90), np.percentile(position_error, 90))
axerr.set_ylim(0, max_err * 1.2)

axerr.set_xlabel("Time (s)")
axerr.set_ylabel("Error (rad)")
axerr.grid(True, alpha=0.4)
line_werr, = axerr.plot([], [], color="purple", lw=2, label="‖ω_actual − ω_profile‖ (rad/s)")
line_poserr, = axerr.plot([], [], color="green", lw=2, label="Rotation Error (rad)")
axerr.legend()

# === Animation update ===
def update(frame):
    global w_actual, w_profile
    t = t_orient[frame]
    ax3d.set_title(f"t = {t:.2f}s")

    # Orientation axes
    R = R_orient[frame]
    for i in range(3):
        lines_actual[i].set_data([0, R[0, i]], [0, R[1, i]])
        lines_actual[i].set_3d_properties([0, R[2, i]])

    if len(t_prof):
        prof_idx = np.searchsorted(t_prof, t)
        if 0 <= prof_idx < len(R_prof):
            R_p = R_prof[prof_idx]
            for i in range(3):
                lines_profile[i].set_data([0, R_p[0, i]], [0, R_p[1, i]])
                lines_profile[i].set_3d_properties([0, R_p[2, i]])

    # Angular velocity vectors — discrete, not interpolated
    if w_actual:
        w_actual.remove()
    if w_profile:
        w_profile.remove()

    idx_a = np.searchsorted(t_ang, t)
    idx_p = np.searchsorted(t_prof_ang, t)
    if 0 <= idx_a < len(W_ang):
        w_actual = ax3d.quiver(0, 0, 0, *W_ang[idx_a],
                               color=color_w_actual, length=0.8, normalize=True)
    if 0 <= idx_p < len(W_prof):
        w_profile = ax3d.quiver(0, 0, 0, *W_prof[idx_p],
                                color=color_w_profile, length=0.8, normalize=True)

    # Error plots
    idx_e = np.searchsorted(t_ang, t)
    line_werr.set_data(t_ang[:idx_e], error_mag[:idx_e])
    idx_pos = frame
    line_poserr.set_data(t_orient[:idx_pos], position_error[:idx_pos])

    return lines_actual + lines_profile + [w_actual, w_profile, line_werr, line_poserr]

ani = FuncAnimation(fig, update, frames=len(t_orient), interval=50, blit=False, repeat=False)

# === Save ===
writer = FFMpegWriter(fps=15, bitrate=1200)
ani.save("animation.mp4", writer=writer)
print("Saved as orientation_angvel_pos_error_zoomed.mp4")
