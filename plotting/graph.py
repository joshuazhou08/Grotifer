import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path


def load_rotations(path):
    data = np.loadtxt(path, delimiter=",")
    if data.size == 0:
        return np.array([]), np.empty((0, 3, 3))
    data = np.atleast_2d(data)
    times = data[:, 0]
    rotations = data[:, 1:10].reshape(-1, 3, 3)
    return times, rotations


AXIS_NAMES = ("X", "Y", "Z")


def load_vectors(path):
    data = np.loadtxt(path, delimiter=",")
    if data.size == 0:
        return np.array([]), np.empty((0, 3))
    data = np.atleast_2d(data)
    times = data[:, 0]
    vectors = data[:, 1:4]
    return times, vectors


def load_profile_scalar(path):
    data = np.loadtxt(path, delimiter=",")
    if data.size == 0:
        return np.array([]), np.array([]), np.array([])
    data = np.atleast_2d(data)
    times = data[:, 0]
    position = data[:, 1]
    velocity = data[:, 2]
    return times, position, velocity


def rotation_axis_angle(rotations):
    if rotations.size == 0:
        return np.empty((0, 3)), np.array([])
    axes = np.zeros((rotations.shape[0], 3))
    angles = np.zeros(rotations.shape[0])
    for i, rot in enumerate(rotations):
        trace = np.trace(rot)
        cos_angle = (trace - 1.0) * 0.5
        cos_angle = np.clip(cos_angle, -1.0, 1.0)
        angle = np.arccos(cos_angle)
        angles[i] = angle
        if angle < 1e-6:
            axes[i] = np.array([1.0, 0.0, 0.0])
            continue
        denom = 2.0 * np.sin(angle)
        axis = np.array([
            rot[2, 1] - rot[1, 2],
            rot[0, 2] - rot[2, 0],
            rot[1, 0] - rot[0, 1],
        ]) / denom
        axes[i] = axis
    return axes, angles


def main():
    orientation_path = Path("logs/orientation.csv")
    profile_path = Path("logs/profile_orientation.csv")
    profile_scalar_path = Path("logs/profile.csv")
    angvel_path = Path("logs/angular_velocity.csv")
    angvel_profile_path = Path("logs/profile_angular_velocity.csv")
    torp_left_actual_path = Path("logs/torp_left_actual.csv")
    torp_left_profile_path = Path("logs/torp_left_profile.csv")
    torp_right_actual_path = Path("logs/torp_right_actual.csv")
    torp_right_profile_path = Path("logs/torp_right_profile.csv")

    if (
        not orientation_path.is_file()
        and not profile_path.is_file()
        and not profile_scalar_path.is_file()
        and not angvel_path.is_file()
        and not angvel_profile_path.is_file()
        and not torp_left_actual_path.is_file()
        and not torp_left_profile_path.is_file()
        and not torp_right_actual_path.is_file()
        and not torp_right_profile_path.is_file()
    ):
        raise FileNotFoundError(
            "Missing logs/orientation.csv, logs/profile_orientation.csv, logs/profile.csv, "
            "logs/angular_velocity.csv, logs/profile_angular_velocity.csv, "
            "logs/torp_left_actual.csv, logs/torp_left_profile.csv, "
            "logs/torp_right_actual.csv, and logs/torp_right_profile.csv"
        )

    t_actual, r_actual = load_rotations(orientation_path) if orientation_path.is_file() else (np.array([]), np.empty((0, 3, 3)))
    t_profile, r_profile = load_rotations(profile_path) if profile_path.is_file() else (np.array([]), np.empty((0, 3, 3)))

    plots_dir = Path("plots")
    plots_dir.mkdir(parents=True, exist_ok=True)

    for axis_idx, axis_name in enumerate(AXIS_NAMES):
        fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True, constrained_layout=True)
        fig.suptitle(f"{axis_name}-Axis Components (Actual vs Profile)")

        comps_actual = r_actual[:, :, axis_idx] if r_actual.size else np.empty((0, 3))
        comps_profile = r_profile[:, :, axis_idx] if r_profile.size else np.empty((0, 3))

        for comp_idx, comp_name in enumerate(AXIS_NAMES):
            ax = axes[comp_idx]
            if t_actual.size and comps_actual.size:
                ax.step(t_actual, comps_actual[:, comp_idx], where="post", color="#8B0000", lw=1.5, label="actual")
            if t_profile.size and comps_profile.size:
                ax.step(t_profile, comps_profile[:, comp_idx], where="post", color="#003366", lw=1.5, label="profile")
            ax.set_ylabel(f"{axis_name} axis {comp_name}")
            ax.grid(True, alpha=0.3)

        axes[0].legend(loc="upper right")
        axes[-1].set_xlabel("Time (s)")

        output_path = plots_dir / f"{axis_name.lower()}_axis_components.png"
        fig.savefig(output_path, dpi=150)
        plt.close(fig)

    axis_actual, angle_actual = rotation_axis_angle(r_actual) if r_actual.size else (np.empty((0, 3)), np.array([]))
    axis_profile, angle_profile = rotation_axis_angle(r_profile) if r_profile.size else (np.empty((0, 3)), np.array([]))

    if t_actual.size or t_profile.size:
        fig, axes = plt.subplots(4, 1, figsize=(10, 10), sharex=True, constrained_layout=True)
        fig.suptitle("Rotation Axis Components and Angle (Actual vs Profile)")

        for idx, name in enumerate(AXIS_NAMES):
            ax = axes[idx]
            if t_actual.size and axis_actual.size:
                ax.step(t_actual, axis_actual[:, idx], where="post", color="#8B0000", lw=1.5, label="actual")
            if t_profile.size and axis_profile.size:
                ax.step(t_profile, axis_profile[:, idx], where="post", color="#003366", lw=1.5, label="profile")
            ax.set_ylabel(f"Axis {name}")
            ax.grid(True, alpha=0.3)
            if idx == 0:
                ax.legend(loc="upper right")

        ax_angle = axes[3]
        if t_actual.size and angle_actual.size:
            ax_angle.step(t_actual, angle_actual, where="post", color="#8B0000", lw=1.5, label="actual")
        if t_profile.size and angle_profile.size:
            ax_angle.step(t_profile, angle_profile, where="post", color="#003366", lw=1.5, label="profile")
        ax_angle.set_ylabel("Angle (rad)")
        ax_angle.set_xlabel("Time (s)")
        ax_angle.grid(True, alpha=0.3)

        output_path = plots_dir / "rotation_axis_angle.png"
        fig.savefig(output_path, dpi=150)
        plt.close(fig)

    if profile_scalar_path.is_file():
        t_prof, angle, velocity = load_profile_scalar(profile_scalar_path)
        if t_prof.size:
            fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True, constrained_layout=True)
            axes[0].step(t_prof, angle, where="post", color="#003366", lw=1.5)
            axes[0].set_ylabel("Angle (rad)")
            axes[0].grid(True, alpha=0.3)

            axes[1].step(t_prof, velocity, where="post", color="#8B0000", lw=1.5)
            axes[1].set_ylabel("Velocity")
            axes[1].set_xlabel("Time (s)")
            axes[1].grid(True, alpha=0.3)

            fig.suptitle("Profile Angle and Velocity")
            output_path = plots_dir / "profile_angle_velocity.png"
            fig.savefig(output_path, dpi=150)
            plt.close(fig)

    if angvel_path.is_file() or angvel_profile_path.is_file():
        t_ang, w_actual = load_vectors(angvel_path) if angvel_path.is_file() else (np.array([]), np.empty((0, 3)))
        t_ang_prof, w_profile = load_vectors(angvel_profile_path) if angvel_profile_path.is_file() else (np.array([]), np.empty((0, 3)))

        fig, axes = plt.subplots(3, 1, figsize=(10, 8), sharex=True, constrained_layout=True)
        fig.suptitle("Angular Velocity (rad/s) (Actual vs Profile)")

        for idx, name in enumerate(AXIS_NAMES):
            ax = axes[idx]
            if t_ang.size and w_actual.size:
                ax.step(t_ang, w_actual[:, idx], where="post", lw=1.5, color="#8B0000", label="actual")
            if t_ang_prof.size and w_profile.size:
                ax.step(t_ang_prof, w_profile[:, idx], where="post", lw=1.5, color="#003366", label="profile")
            ax.set_ylabel(f"{name} ω")
            ax.grid(True, alpha=0.3)
            if idx == 0:
                ax.legend(loc="upper right")

        axes[-1].set_xlabel("Time (s)")

        output_path = plots_dir / "angular_velocity_actual_vs_profile.png"
        fig.savefig(output_path, dpi=150)
        plt.close(fig)

    for label, actual_path, profile_path in (
        ("left", torp_left_actual_path, torp_left_profile_path),
        ("right", torp_right_actual_path, torp_right_profile_path),
    ):
        if not actual_path.is_file() and not profile_path.is_file():
            continue

        t_torp_actual, pos_actual, vel_actual = (np.array([]), np.array([]), np.array([]))
        t_torp_profile, pos_profile, vel_profile = (np.array([]), np.array([]), np.array([]))
        if actual_path.is_file():
            t_torp_actual, pos_actual, vel_actual = load_profile_scalar(actual_path)
        if profile_path.is_file():
            t_torp_profile, pos_profile, vel_profile = load_profile_scalar(profile_path)

        fig, axes = plt.subplots(2, 1, figsize=(10, 6), sharex=True, constrained_layout=True)
        fig.suptitle(f"Torp {label.capitalize()} Position and Velocity (Actual vs Profile)")

        if t_torp_actual.size:
            axes[0].step(t_torp_actual, pos_actual, where="post", color="#8B0000", lw=1.5, label="actual")
        if t_torp_profile.size:
            axes[0].step(t_torp_profile, pos_profile, where="post", color="#003366", lw=1.5, label="profile")
        axes[0].set_ylabel("Position")
        axes[0].grid(True, alpha=0.3)
        axes[0].legend(loc="upper right")

        if t_torp_actual.size:
            axes[1].step(t_torp_actual, vel_actual, where="post", color="#8B0000", lw=1.5, label="actual")
        if t_torp_profile.size:
            axes[1].step(t_torp_profile, vel_profile, where="post", color="#003366", lw=1.5, label="profile")
        axes[1].set_ylabel("Velocity")
        axes[1].set_xlabel("Time (s)")
        axes[1].grid(True, alpha=0.3)

        output_path = plots_dir / f"torp_{label}_position_velocity.png"
        fig.savefig(output_path, dpi=150)
        plt.close(fig)


if __name__ == "__main__":
    main()
