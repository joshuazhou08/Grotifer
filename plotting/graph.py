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


def main():
    orientation_path = Path("logs/orientation.csv")
    profile_path = Path("logs/profile_orientation.csv")

    if not orientation_path.is_file() and not profile_path.is_file():
        raise FileNotFoundError("Missing logs/orientation.csv and logs/profile_orientation.csv")

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
                ax.plot(t_actual, comps_actual[:, comp_idx], color="#8B0000", lw=1.5, label="actual")
            if t_profile.size and comps_profile.size:
                ax.plot(t_profile, comps_profile[:, comp_idx], color="#003366", lw=1.5, label="profile")
            ax.set_ylabel(f"{axis_name} axis {comp_name}")
            ax.grid(True, alpha=0.3)

        axes[0].legend(loc="upper right")
        axes[-1].set_xlabel("Time (s)")

        output_path = plots_dir / f"{axis_name.lower()}_axis_components.png"
        fig.savefig(output_path, dpi=150)
        plt.close(fig)


if __name__ == "__main__":
    main()
