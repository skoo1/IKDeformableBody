import time
import numpy as np
from typing import Tuple
from pathlib import Path
import argparse
import warnings
import sys
sys.path.append('./SMPL3/smpl_webuser')

import cma
import trimesh

from serialization import load_model, save_model
from ruamel.yaml import YAML, dump, RoundTripDumper
from RaisimGymVecEnv import RaisimGymVecEnv as VecEnv

########## pyd 설정 ##########
def make_env(gender: str):
    if gender.lower().startswith("m"):
        from scan2smpl_male import RaisimGymEnv
    else:
        from scan2smpl_female import RaisimGymEnv
    return RaisimGymEnv

########## obj 파일 탐색 ##########
def resolve_obj_path(obj_arg: str, base_dir="scan_files") -> Tuple[str, Path]:
    """
    obj_arg:
      - "P002"                -> scan_files/P002.obj
      - "P002.obj"            -> scan_files/P002.obj
      - "scan_files/P002.obj" -> 그대로 사용
      - "/abs/path/P002.obj"  -> 그대로 사용

    return: (subject, obj_path)
    """
    p = Path(obj_arg)
    subject = p.stem  # 항상 stem 기준으로 subject 결정

    # 1) 사용자가 준 경로가 실제 파일이면 최우선
    if p.suffix == ".obj" and p.exists():
        return subject, p

    # 2) base_dir에서 찾기
    candidate = Path(base_dir) / f"{subject}.obj"
    if candidate.exists():
        return subject, candidate

    # 3) 둘 다 없으면 경고 + base_dir 경로 반환
    warnings.warn(
        f"[OBJ NOT FOUND]\n"
        f"  input arg : {obj_arg}\n"
        f"  tried     : {p.resolve() if p.suffix == '.obj' else 'N/A'}\n"
        f"  tried     : {candidate.resolve()}\n",
        RuntimeWarning
    )

    return subject, candidate

########## 성별 설정 ##########
def get_smpl_model_path(gender: str) -> str:
    """
    gender: 'm'/'male' or 'f'/'female'
    아래 경로는 프로젝트에 맞게 수정하세요.
    """
    g = gender.strip().lower()
    if g in ("m", "male"):
        return "basicmodel_m_lbs_10_207_0_v1.1.0.pkl"
    if g in ("f", "female"):
        return "basicmodel_f_lbs_10_207_0_v1.1.0.pkl"
    raise ValueError(f"Unknown gender: {gender} (use m/f)")

########## obj 파일 저장 ##########
def save_obj(smpl, total_time, outmesh_path, parameter_path) :
    with open(outmesh_path, 'w') as fp:
        for v in smpl.r:
            fp.write( 'v %f %f %f\n' % (v[0], v[1], v[2]) )

        for f in smpl.f+1:
            fp.write( 'f %d %d %d\n' %  (f[0], f[1], f[2]) )

    with open(parameter_path, 'w') as file:
        file.write(f"total process time : {total_time}\n")
        file.write(f"trans :\n{smpl.trans[:]}\n")
        file.write(f"pose :\n{smpl.pose[:]}\n")
        file.write(f"beta :\n{smpl.betas[:]}\n")

    return 0

########## Main Begins Here ##########
def main(obj_arg: str, gender: str, out_root="results", scan_root="scan_files"):
    subject, obj_path = resolve_obj_path(obj_arg, base_dir=scan_root)

    if not obj_path.exists():
        raise FileNotFoundError(f"obj not found: {obj_path}")
    
    model_name = subject  # 이제 model 개념 없음
    CMAES_POPSIZE = 20

    obj_path = Path("scan_files") / f"{subject}.obj"
    if not obj_path.exists():
        print(f"[SKIP] obj not found: {obj_path}")
        return

    obj_mesh = trimesh.load(str(obj_path))
    obj_vertices = np.ascontiguousarray(obj_mesh.vertices, dtype=np.float32)
    obj_vertex_normals = np.ascontiguousarray(obj_mesh.vertex_normals, dtype=np.float32)

    ########## pyd_cost_calculator ##########
    cfg = YAML().load(open("cfg.yaml", 'r'))
    cfg['environment']['num_envs'] = CMAES_POPSIZE
    dump_cfg = dump(cfg['environment'], Dumper=RoundTripDumper)
    EnvClass = make_env(gender)
    MyRaisimGymEnv = EnvClass("/rsc", dump_cfg)
    env = VecEnv(MyRaisimGymEnv)
    env.build(obj_vertices, obj_vertex_normals)

    ########## Run Main Code ##########
    print(f"SMPL model ::: {model_name} (gender={gender}) ::: Fitting START ...")
    start_time = time.time()

    trans0 = np.zeros(3)
    pose0 = np.zeros(72)
    betas0 = np.zeros(10)
    cma_param_initpose = np.concatenate([trans0, pose0, betas0])
    cma_param_sigma0 = 0.05
    cma_options = {
        'maxiter': 50000,
        'bounds': [np.concatenate([[-0.1] * len(trans0), [-np.pi] * len(pose0), [-5] * len(betas0)]),  # 하한
                   np.concatenate([[0.1] * len(trans0), [np.pi] * len(pose0), [5] * len(betas0)])],  # 상한
        'verb_disp': 100,
        'popsize': CMAES_POPSIZE,
        'seed': 234,
    }

    es = cma.CMAEvolutionStrategy(cma_param_initpose, cma_param_sigma0, cma_options)
    while not es.stop():
        solutions = es.ask()
        rewards, _ = env.step(np.array(solutions, dtype=np.float32))
        es.tell(solutions, rewards)
        es.logger.add()
        es.disp()

        best_solution = es.result.xbest
        trans = best_solution[:3]
        pose = best_solution[3:75]
        betas = best_solution[75:]

    total_time = time.time() - start_time

    smpl_model_path = get_smpl_model_path(gender)
    smpl = load_model(smpl_model_path)
    smpl.trans[:] = trans
    smpl.pose[:] = pose
    smpl.betas[:10] = betas

    # 결과 폴더 생성: results/P002/
    out_dir = Path(out_root) / subject
    out_dir.mkdir(parents=True, exist_ok=True)

    outsmpl_path = out_dir / f"{model_name}.pkl"
    outmesh_path = out_dir / f"{model_name}.obj"
    parameter_path = out_dir / f"{model_name}.txt"
    save_model(smpl, str(outsmpl_path))
    save_obj(smpl, total_time, str(outmesh_path), str(parameter_path))

    print("Process Complete, Loss :", es.best.f)
    print("All process time :", total_time)
    print("Saved to:", out_dir)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--obj", required=True,
                        help="OBJ path or subject name. e.g., scan_files/P002.obj or P002")
    parser.add_argument("--gender", required=True, choices=["m", "f", "n", "male", "female", "neutral"],
                        help="SMPL gender: m/f/n (or male/female/neutral)")
    parser.add_argument("--scan_root", default="scan_files",
                        help="Base directory when --obj is subject name (default: scan_files)")
    parser.add_argument("--out_root", default="results",
                        help="Output root directory (default: results)")

    args = parser.parse_args()
    main(args.obj, args.gender, out_root=args.out_root, scan_root=args.scan_root)