import argparse
import pickle
import pandas as pd
import numpy as np
from pathlib import Path


def parse_args():
    parser = argparse.ArgumentParser(description="Convert SMPL pkl poses to kinematics CSV")
    parser.add_argument(
        "--subject",
        required=True,
        help="Subject name (e.g. walking3_1103)"
    )
    parser.add_argument(
        "--pkl-dir",
        default="results",
        help="Directory containing {subject}_skel.pkl (default: current directory)"
    )
    parser.add_argument(
        "--out-dir",
        default="results",
        help="Directory to save {subject}_kinematics.csv (default: current directory)"
    )
    parser.add_argument(
        "--dt",
        type=float,
        default=0.01,
        help="Time step between frames (default: 0.01)"
    )
    return parser.parse_args()


def main():
    args = parse_args()

    subject = args.subject
    pkl_path = Path(args.pkl_dir) / f"{subject}/{subject}_skel.pkl"
    csv_path = Path(args.out_dir) / f"{subject}/{subject}_kinematics.csv"

    if not pkl_path.exists():
        raise FileNotFoundError(f"PKL file not found: {pkl_path}")

    # pkl 파일 로드
    with open(pkl_path, "rb") as f:
        data = pickle.load(f)

    if "poses" not in data:
        raise KeyError("The pkl file does not contain the key 'poses'.")

    poses_data = np.asarray(data["poses"])
    num_frames = poses_data.shape[0]

    # time 벡터
    time = np.arange(num_frames) * args.dt

    df = pd.DataFrame(poses_data)

    column_names = ['pelvis_tilt', #0
                    'pelvis_list', #1
                    'pelvis_rotation', #2
                    'hip_flexion_r', #3
                    'hip_adduction_r', #4
                    'hip_rotation_r', #5
                    'knee_angle_r', #6
                    'ankle_angle_r', #7
                    'subtalar_angle_r', #8
                    'mtp_angle_r', #9
                    'hip_flexion_l', #10
                    'hip_adduction_l', #11
                    'hip_rotation_l', #12
                    'knee_angle_l', #13
                    'ankle_angle_l', #14
                    'subtalar_angle_l', #15
                    'mtp_angle_l', #16
                    'lumbar_bending', #17
                    'lumbar_extension', #18
                    'lumbar_twist', #19
                    'thorax_bending', #20
                    'thorax_extension', #21
                    'thorax_twist', #22
                    'head_bending', #23
                    'head_extension', #24
                    'head_twist', #25
                    'scapula_abduction_r', #26
                    'scapula_elevation_r', #27
                    'scapula_upward_rot_r', #28
                    'shoulder_r_x', #29
                    'shoulder_r_y', #30
                    'shoulder_r_z', #31
                    'elbow_flexion_r', #32
                    'pro_sup_r', #33
                    'wrist_flexion_r', #34
                    'wrist_deviation_r', #35
                    'scapula_abduction_l', #36
                    'scapula_elevation_l', #37
                    'scapula_upward_rot_l', #38
                    'shoulder_l_x', #39
                    'shoulder_l_y', #40
                    'shoulder_l_z', #41
                    'elbow_flexion_l', #42
                    'pro_sup_l', #43
                    'wrist_flexion_l', #44
                    'wrist_deviation_l', #45
                    ]
    
    if df.shape[1] != len(column_names):
        raise ValueError(
            f"DOF mismatch: data has {df.shape[1]} columns, "
            f"but {len(column_names)} names were provided."
        )

    df.columns = column_names
    df.insert(0, "time", time)

    csv_path.parent.mkdir(parents=True, exist_ok=True)
    df.to_csv(csv_path, index=False)

    print(f"✅ CSV saved: {csv_path}")


if __name__ == "__main__":
    main()
