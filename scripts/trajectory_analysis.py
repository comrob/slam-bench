import os
import copy
import sys
import yaml
import numpy as np
import matplotlib.pyplot as plt

from evo.tools import file_interface
from evo.core import sync, metrics, trajectory
from evo.core.trajectory import PoseTrajectory3D

# =============================================================================
# Configuration and Data Loading
# =============================================================================
def get_file_paths():
    """
    Define and return file paths for the bagfiles.
    """
    dataset_name = os.getenv("DATASET_NAME")
    base_path = os.path.join("/rosbag_files", dataset_name)
    gt_file = os.getenv("REFERENCE_TRAJECTORY_FILE")
    gt_file = os.path.join(base_path, gt_file)
    
    base_est_file = "/trajectory_files"
    est_file = os.getenv("ESTIMATED_TRAJECTORY_FILE", "estimated_trajectory.txt")
    est_file = os.path.join(base_est_file, est_file)

    return base_path, gt_file, est_file

def load_trajectories(gt_file, est_file):
    """
    Load reference and estimated trajectories using the file_interface.
    """
    traj_ref = file_interface.read_tum_trajectory_file(gt_file)
    traj_est = file_interface.read_tum_trajectory_file(est_file)
    return traj_ref, traj_est

# =============================================================================
# Trajectory Processing: Synchronization, Alignment & Orientation
# =============================================================================
def synchronize_trajectories(traj_ref, traj_est, max_diff=0.05):
    """
    Synchronize the reference and estimated trajectories based on a maximum time difference.
    """
    return sync.associate_trajectories(traj_ref, traj_est, max_diff)

def align_trajectories(traj_ref_sync, traj_est_sync):
    """
    Align the estimated trajectory to the reference trajectory.
    """
    # Create deep copies to avoid modifying the originals
    traj_ref_aligned = copy.deepcopy(traj_ref_sync)
    traj_est_aligned = copy.deepcopy(traj_est_sync)
    traj_est_aligned.align(traj_ref_sync, correct_scale=False, correct_only_scale=False)
    return traj_ref_aligned, traj_est_aligned

def set_identity_orientations(traj):
    """
    Reset the trajectory's orientations to identity (no rotation).
    """
    num_poses = len(traj.positions_xyz)
    # Quaternion format: [w, x, y, z] where identity is [1, 0, 0, 0]
    identity_quats = np.zeros((num_poses, 4))
    identity_quats[:, 0] = 1  # set w=1
    return PoseTrajectory3D(
        positions_xyz=traj.positions_xyz,
        orientations_quat_wxyz=identity_quats,
        timestamps=traj.timestamps
    )

def process_trajectories():
    """
    Load, synchronize, align, and reset orientation for both trajectories.
    Returns the processed (reference, estimated) trajectory pair.
    """
    base_path, gt_file, est_file = get_file_paths()
    # check if the files exist
    if not os.path.exists(gt_file) or not os.path.exists(est_file):
        if not os.path.exists(gt_file):
            print (f"File {gt_file} does not exist (gt_file)")
        if not os.path.isfile(est_file):
            print (f"File {est_file} does not exist (est_file)")
        exit(1)
        
    
    try:
        traj_ref, traj_est = load_trajectories(gt_file, est_file)
    except Exception as e:
        print(f"Error loading trajectories: {e}")
        print(f"file paths: {gt_file} {est_file}")
        exit(1)
    
    traj_ref_sync, traj_est_sync = synchronize_trajectories(traj_ref, traj_est)
    traj_ref_aligned, traj_est_aligned = align_trajectories(traj_ref_sync, traj_est_sync)
    
    # Reset orientations to identity
    traj_ref_final = set_identity_orientations(traj_ref_sync)
    traj_est_final = set_identity_orientations(traj_est_aligned)
    
    return base_path, (traj_ref_final, traj_est_final)

# =============================================================================
# Metric Computation: APE and RPE
# =============================================================================
def compute_ape(traj_pair):
    """
    Compute Absolute Pose Error (APE) using the translation part.
    """
    pose_relation = metrics.PoseRelation.translation_part
    ape_metric = metrics.APE(pose_relation)
    ape_metric.process_data(traj_pair)
    return ape_metric.get_statistic(metrics.StatisticsType.rmse), ape_metric.get_all_statistics()

def compute_rpe_for_delta(traj_pair, delta_meters):
    """
    Compute Relative Pose Error (RPE) for a given delta (in meters).
    Returns the computed statistics or None if processing fails.
    """
    pose_relation = metrics.PoseRelation.translation_part
    delta_unit = metrics.Unit.meters
    rpe_metric = metrics.RPE(pose_relation, delta_meters, delta_unit, all_pairs=True)
    
    try:
        rpe_metric.process_data(traj_pair)
    except Exception as e:
        print(f"Error processing RPE for delta {delta_meters}: {e}")
        return None
    
    return rpe_metric.get_all_statistics()

def compute_rpe_set(traj_pair, delta_list):
    """
    Compute RPE for a list of delta values.
    Returns a dictionary mapping delta to its statistics.
    """
    results = {}
    for delta in delta_list:
        stats = compute_rpe_for_delta(traj_pair, delta)
        if stats is not None:
            results[delta] = stats
        else:
            print(f"Skipping delta {delta} due to processing error.")
    return results

def create_rpe_table(rpe_results):
    """
    Create a table (list of lists) summarizing the RPE results.
    Also computes the average relative RPE.
    """
    table_data = []
    relative_rpe_values = []
    for delta, stats in rpe_results.items():
        rel_rpe = (stats['rmse'] / delta) * 100  # percentage
        relative_rpe_values.append(rel_rpe)
        table_data.append([
            f"{delta}m",
            f"RMSE: {rel_rpe:.2f}%\nSTD: {(stats['std'] / delta) * 100:.2f}%\n"
            f"MIN: {(stats['min'] / delta) * 100:.2f}%\nMAX: {(stats['max'] / delta) * 100:.2f}%",
            f"RMSE: {stats['rmse']:.3f} m\nSTD: {stats['std']:.3f} m\n"
            f"MIN: {stats['min']:.3f} m\nMAX: {stats['max']:.3f} m"
        ])
    avg_relative_rpe = float(np.mean(relative_rpe_values))
    return table_data, avg_relative_rpe

def compute_ate_rmse(rpe_results):
    """
    Compute an aggregated ATE RMSE value from RPE results.
    """
    rmse_values = [stats['rmse'] for stats in rpe_results.values()]
    return float(np.sqrt(np.mean(np.square(rmse_values))))

def export_results_to_yaml(filename, avg_relative_rpe, ate_rmse, rpe_results):
    """
    Save the computed metrics (average RPE and ATE RMSE along with detailed RPE stats) to a YAML file.
    """
    rpe_details = {
        f"{delta}m": {
            'rmse_meters': float(stats['rmse']),
            'std_meters': float(stats['std']),
            'min_meters': float(stats['min']),
            'max_meters': float(stats['max'])
        }
        for delta, stats in rpe_results.items()
    }
    data = {
        'results': {
            'rpe_avg_rmse_percentage': avg_relative_rpe,
            'ate_rmse_meters': ate_rmse
        },
        'rpe_details': rpe_details
    }
    with open(filename, 'w') as file:
        yaml.dump(data, file)

# =============================================================================
# Visualization Functions
# =============================================================================
def plot_trajectory_xy(ax, traj_ref, traj_est):
    """
    Plot the XY trajectories (reference and estimated) on the given axis.
    """
    ax.plot(traj_ref.positions_xyz[:, 0], traj_ref.positions_xyz[:, 1],
            label="Reference", linestyle='-', marker='o', markersize=2)
    ax.plot(traj_est.positions_xyz[:, 0], traj_est.positions_xyz[:, 1],
            label="Estimated", linestyle='-', marker='x', markersize=2)
    ax.set_xlabel("X Position (m)")
    ax.set_ylabel("Y Position (m)")
    ax.set_title("XY Trajectory Plot")
    ax.legend()
    ax.grid()
    ax.set_aspect('equal', adjustable='datalim')

def set_equal_aspect_3d(ax, positions):
    """
    Set equal aspect ratio for a 3D plot based on trajectory positions.
    """
    x_limits = [np.min(positions[:, 0]), np.max(positions[:, 0])]
    y_limits = [np.min(positions[:, 1]), np.max(positions[:, 1])]
    z_limits = [np.min(positions[:, 2]), np.max(positions[:, 2])]
    max_range = max(np.ptp(x_limits), np.ptp(y_limits), np.ptp(z_limits))
    mid_x, mid_y, mid_z = np.mean(x_limits), np.mean(y_limits), np.mean(z_limits)
    ax.set_xlim(mid_x - max_range/2, mid_x + max_range/2)
    ax.set_ylim(mid_y - max_range/2, mid_y + max_range/2)
    ax.set_zlim(mid_z - max_range/2, mid_z + max_range/2)

def plot_trajectory_3d(ax, traj_ref, traj_est):
    """
    Plot the 3D trajectories on the given axis.
    """
    ax.plot(traj_ref.positions_xyz[:, 0], traj_ref.positions_xyz[:, 1], traj_ref.positions_xyz[:, 2],
            label="Reference")
    ax.plot(traj_est.positions_xyz[:, 0], traj_est.positions_xyz[:, 1], traj_est.positions_xyz[:, 2],
            label="Estimated")
    ax.set_xlabel("X Position (m)")
    ax.set_ylabel("Y Position (m)")
    ax.set_zlabel("Z Position (m)")
    ax.set_title("3D Trajectory Plot")
    ax.legend()
    ax.grid()
    set_equal_aspect_3d(ax, traj_ref.positions_xyz)

def plot_summary_table(ax, avg_relative_rpe, ate_rmse):
    """
    Plot a summary table of the computed metrics.
    """
    ax.axis('tight')
    ax.axis('off')
    ax.set_title("SUMMARY METRICS", fontsize=12, fontweight='bold')
    table_data = [[f"{avg_relative_rpe:.2f} %", f"{ate_rmse:.3f} m"]]
    col_labels = ["AVG RMSE RPE (%)", "ATE RMSE (m)"]
    table = ax.table(cellText=table_data, colLabels=col_labels, loc='center', cellLoc='center')
    table.auto_set_font_size(False)
    table.set_fontsize(10)
    table.scale(1.0, 2.4)

def plot_rpe_details_table(ax, rpe_table):
    """
    Plot a table showing detailed RPE results.
    """
    ax.axis('off')
    table = ax.table(cellText=rpe_table,
                     colLabels=["Delta", "Relative RPE", "Absolute RPE [m]"],
                     loc='center', cellLoc='center')
    table.auto_set_font_size(False)
    table.set_fontsize(9)
    table.scale(1.0, 4.0)

def create_figure(traj_ref, traj_est, rpe_table, avg_relative_rpe, ate_rmse, save_path):
    """
    Create and save a figure with:
    - XY trajectory plot
    - Summary metrics table
    - 3D trajectory plot
    - RPE details table
    """
    fig, axs = plt.subplots(2, 2, figsize=(12, 10))

    # XY Trajectory Plot
    plot_trajectory_xy(axs[0, 0], traj_ref, traj_est)

    # Summary Table
    plot_summary_table(axs[0, 1], avg_relative_rpe, ate_rmse)

    # 3D Trajectory Plot (added as subplot 3)
    ax_3d = fig.add_subplot(2, 2, 3, projection='3d')
    plot_trajectory_3d(ax_3d, traj_ref, traj_est)

    # RPE Details Table
    plot_rpe_details_table(axs[1, 1], rpe_table)

    plt.tight_layout()
    plt.savefig(save_path, format="pdf", dpi=300)
    plt.close()

# =============================================================================
# Main Execution
# =============================================================================
if __name__ == '__main__':
    # Process trajectories
    base_path, traj_pair = process_trajectories()
    
    # Compute metrics
    ape_rmse, ape_stats = compute_ape(traj_pair)
    
    # Define deltas for RPE computation
    TEST_DELTAS = [1, 2, 5, 10, 20, 50, 100]
    EVALUAITON_DELTAS = [5, 100, 200, 300, 400, 500, 600, 700, 800]
    
    TEST_MODE = os.getenv("TEST_MODE", 0)
    print(f"Test mode: {TEST_MODE}")
    is_test = (TEST_MODE == "1")
    print (f"Is test: {is_test}")
    
    DELTAS = TEST_DELTAS if is_test else EVALUAITON_DELTAS
    
    rpe_results = compute_rpe_set(traj_pair, DELTAS)
    
    if len(rpe_results) == 0:
       print("\033[91mToo big deltas! Try turning on the test mode in `.env` file: TEST_MODE=1\033[0m", file=sys.stderr)
       sys.exit(1)
       exit(1)

    
    # Build RPE summary table and aggregated metrics
    rpe_table, avg_relative_rpe = create_rpe_table(rpe_results)
    ate_rmse = compute_ate_rmse(rpe_results)
    
    base_path, gt_file, est_file = get_file_paths()
    # Export results to YAML
    # get the folder name of the est_file
    yaml_dir = os.path.dirname(est_file)
    yaml_filename = os.path.join(yaml_dir, "trajectory_analysis.yaml")
    export_results_to_yaml(yaml_filename, avg_relative_rpe, ate_rmse, rpe_results)
    
    # Create and save visualization
    pdf_filename = os.path.join(yaml_dir, "trajectory_analysis.pdf")
    create_figure(traj_pair[0], traj_pair[1], rpe_table, avg_relative_rpe, ate_rmse, pdf_filename)