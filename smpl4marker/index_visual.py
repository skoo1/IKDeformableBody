import csv
import argparse
from pathlib import Path
import numpy as np
import sys
import pickle

sys.path.append('./SMPL3/smpl_webuser')
from serialization import load_model


def get_base_smpl_path(gender: str) -> Path:
    g = gender.lower()
    if g in ("m", "male"):
        return Path("basicmodel_m_lbs_10_207_0_v1.1.0.pkl")
    elif g in ("f", "female"):
        return Path("basicmodel_f_lbs_10_207_0_v1.1.0.pkl")
    else:
        raise ValueError("gender must be m/f or male/female")


def export_vertices_from_result(
    obj_name: str,
    gender: str,
    results_root: str = "results"
) -> Path:
    """
    result pkl에는 반드시 다음이 들어있다고 가정:
      - trans : (3,)
      - pose  : (72,)
      - betas : (10,)
    """

    subject = Path(obj_name).stem
    result_dir = Path(results_root) / subject
    result_pkl = result_dir / f"{subject}.pkl"

    if not result_pkl.exists():
        raise FileNotFoundError(f"[ERROR] Result pkl not found: {result_pkl}")

    # 1️⃣ 결과 pkl에서 파라미터 로드
    with open(result_pkl, "rb") as f:
        params = pickle.load(f, encoding="latin1")

    for k in ["trans", "pose", "betas"]:
        if k not in params:
            raise KeyError(f"[ERROR] '{k}' not found in result pkl keys: {list(params.keys())}")

    trans = np.asarray(params["trans"]).reshape(3,)
    pose  = np.asarray(params["pose"]).reshape(-1)
    betas = np.asarray(params["betas"]).reshape(-1)

    print("[INFO] Loaded parameters from pkl")
    print("  trans:", trans)
    print("  pose shape :", pose.shape)
    print("  betas shape:", betas.shape)

    # 2️⃣ base SMPL 모델 로드
    base_smpl_pkl = get_base_smpl_path(gender)
    if not base_smpl_pkl.exists():
        raise FileNotFoundError(f"[ERROR] Base SMPL model not found: {base_smpl_pkl}")

    smpl = load_model(str(base_smpl_pkl))
    print(f"[INFO] Loaded base SMPL model: {base_smpl_pkl}")

    # 3️⃣ 파라미터 적용 (중요: slice 대입)
    smpl.trans[:] = trans
    smpl.pose[:]  = pose
    smpl.betas[:10] = betas

    # 4️⃣ vertex 계산
    vertices = np.asarray(smpl.r)
    if vertices.size == 0:
        raise ValueError("[ERROR] smpl.r is empty after applying parameters.")

    # 5️⃣ CSV 저장
    out_csv = result_dir / f"{subject}_vertices.csv"
    result_dir.mkdir(parents=True, exist_ok=True)

    with open(out_csv, mode="w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["Vertex Index", "X", "Y", "Z"])
        for idx, (x, y, z) in enumerate(vertices):
            writer.writerow([idx, float(x), float(y), float(z)])

    print(f"[INFO] Saved vertices CSV: {out_csv}")
    return out_csv


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--obj", required=True)
    parser.add_argument("--gender", required=True, choices=["m", "f", "male", "female"])
    parser.add_argument("--results_root", default="results")

    args = parser.parse_args()

    export_vertices_from_result(
        args.obj,
        args.gender,
        results_root=args.results_root
    )
