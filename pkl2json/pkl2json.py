import warnings
warnings.filterwarnings(
    "ignore",
    category=DeprecationWarning,
    message=".*scipy.sparse.*"
)

import pickle
import json
import os

import chumpy as ch
from scipy.sparse import csc_matrix, csr_matrix, coo_matrix
import numpy as np


def make_json(obj):
    """
    JSON 직렬화가 가능한 형태로 변환해주는 재귀 함수.
    """
    if isinstance(obj, ch.Ch):
        arr = np.array(obj.r)
        return make_json(arr)

    elif isinstance(obj, np.ndarray):
        return obj.tolist()

    elif isinstance(obj, (csc_matrix, csr_matrix, coo_matrix)):
        return obj.toarray().tolist()

    elif isinstance(obj, dict):
        return {k: make_json(v) for k, v in obj.items()}

    elif isinstance(obj, list):
        return [make_json(x) for x in obj]

    elif isinstance(obj, tuple):
        return [make_json(x) for x in obj]  # JSON 호환을 위해 list로 변환

    return obj


# =========================
# 설정
# =========================
pkl_dir = "./pkl_files"     # pkl 파일들이 들어 있는 폴더
json_dir = "./json_files"   # json 저장 폴더

os.makedirs(json_dir, exist_ok=True)

# =========================
# 변환 루프
# =========================
for filename in os.listdir(pkl_dir):
    if not filename.endswith(".pkl"):
        continue

    pkl_path = os.path.join(pkl_dir, filename)
    json_path = os.path.join(
        json_dir,
        filename.replace(".pkl", ".json")
    )

    try:
        with open(pkl_path, "rb") as f:
            data = pickle.load(f, encoding="latin1")

        data_friendly = make_json(data)

        with open(json_path, "w", encoding="utf-8") as f:
            json.dump(data_friendly, f, ensure_ascii=False, indent=4)

        print(f"✅ 변환 완료: {filename} → {os.path.basename(json_path)}")

    except Exception as e:
        print(f"❌ 실패: {filename}")
        print(f"   오류 내용: {e}")
