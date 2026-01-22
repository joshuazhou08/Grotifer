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


def main():
    orientation_path = Path("logs/orientation.csv")
    profile_path = Path("logs/profile_orientation.csv")
    profile_scalar_path = Path("logs/profile.csv")
    angvel_path = Path("logs/angular_velocity.csv")
    angvel_profile_path = Path("logs/profile_angular_velocity.csv")

    if (
        not orientation_path.is_file()
        and not profile_path.is_file()
        and not profile_scalar_path.is_file()
        and not angvel_path.is_file()
        and not angvel_profile_path.is_file()
    ):
        raise FileNotFoundError(
            "Missing logs/orientation.csv, logs/profile_orientation.csv, logs/profile.csv, "
            "logs/angular_velocity.csv, and logs/profile_angular_velocity.csv"
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

    if profile_scalar_path.is_file():
        profile_scalar = np.loadtxt(profile_scalar_path, delimiter=",")
        if profile_scalar.size:
            profile_scalar = np.atleast_2d(profile_scalar)
            t_prof = profile_scalar[:, 0]
            angle = profile_scalar[:, 1]
            velocity = profile_scalar[:, 2]

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


if __name__ == "__main__":
    main()
