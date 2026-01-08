import argparse
from pathlib import Path

import numpy as np
import open3d as o3d
import trimesh
from scipy.spatial import cKDTree
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.cm import ScalarMappable


def load_vertices_from_obj_anyhow(obj_path: str) -> np.ndarray:
    """
    OBJ가 triangle mesh가 아니어도 'vertices'만 확보하는 로더.
    1) Open3D triangle mesh로 시도 -> vertices 있으면 사용
    2) 실패(또는 vertices=0) -> trimesh로 fallback (Scene도 처리)
    """
    # 1) Open3D 시도
    mesh = o3d.io.read_triangle_mesh(obj_path)
    v = np.asarray(mesh.vertices)
    if v.size > 0:
        return v

    # 2) trimesh fallback
    tm = trimesh.load(obj_path, process=False)

    # OBJ가 여러 geometry로 들어오면 Scene이 될 수 있음
    if isinstance(tm, trimesh.Scene):
        verts = []
        for geom in tm.geometry.values():
            if hasattr(geom, "vertices") and len(geom.vertices) > 0:
                verts.append(np.asarray(geom.vertices))
        if len(verts) == 0:
            return np.empty((0, 3), dtype=np.float64)
        return np.vstack(verts)

    if hasattr(tm, "vertices") and len(tm.vertices) > 0:
        return np.asarray(tm.vertices)

    return np.empty((0, 3), dtype=np.float64)


def resolve_paths(obj_name: str, gender: str,
                  scan_root: str = "scan_files",
                  results_root: str = "results"):
    p = Path(obj_name)
    subject = p.stem

    scan_root_p = Path(scan_root)

    # ---- dst: scan_files ----
    dst_candidates = []

    if p.suffix.lower() == ".obj":
        # 1) 사용자가 준 경로가 실제 파일이면 그대로
        if p.exists():
            dst_candidates.append(p)

        # 2) 파일명만 준 경우(예: P001_Scanner.obj) -> scan_root에서 찾기
        dst_candidates.append(scan_root_p / p.name)
    else:
        # 3) subject만 준 경우 -> scan_root/subject.obj
        dst_candidates.append(scan_root_p / f"{obj_name}.obj")
        dst_candidates.append(scan_root_p / f"{subject}.obj")

    dst_path = next((c for c in dst_candidates if c.exists()), None)
    if dst_path is None:
        # scan_root에 뭐가 있는지 같이 보여주면 디버깅이 매우 쉬움
        available = sorted([x.name for x in scan_root_p.glob("*.obj")])
        raise FileNotFoundError(
            f"[ERROR] Scan OBJ (dst) not found.\n"
            f"  input      : {obj_name}\n"
            f"  tried      : " + ", ".join(str(c) for c in dst_candidates) + "\n"
            f"  scan_root   : {scan_root_p.resolve()}\n"
            f"  available   : {available[:30]}" + (" ..." if len(available) > 30 else "")
        )

    # ---- src: results/<subject>/ ----
    g = gender.strip().lower()
    if g in ("m", "male"):
        gender_tag = "male"
    elif g in ("f", "female"):
        gender_tag = "female"
    else:
        raise ValueError("gender must be m/f (or male/female)")

    subj_dir = Path(results_root) / subject
    src_candidates = [
        subj_dir / f"{subject}_{gender_tag}.obj",
        subj_dir / f"{subject}.obj",
    ]
    src_path = next((c for c in src_candidates if c.exists()), src_candidates[0])

    return subject, dst_path, src_path, subj_dir, gender_tag


def main(subject: str, dst_path: Path, src_path: Path, out_dir: Path, tag: str):
    # -------------------------------------------------
    # 1. 두 OBJ 메시 불러오기 (SMPL = 참값, result = 계산값)
    # -------------------------------------------------
    if not dst_path.exists():
        raise FileNotFoundError(f"[ERROR] Scan OBJ (dst) not found: {dst_path}")
    if not src_path.exists():
        raise FileNotFoundError(f"[ERROR] Result OBJ (src) not found: {src_path}")
    
    scan_mesh = o3d.io.read_triangle_mesh(str(dst_path))
    result_mesh = o3d.io.read_triangle_mesh(str(src_path))

    scan_points = load_vertices_from_obj_anyhow(str(dst_path))
    if scan_points.size == 0:
        raise ValueError(
            f"[ERROR] dst(scan) loaded but has 0 vertices.\n"
            f"  path: {dst_path}\n"
            f"  hint: scan OBJ may not contain vertices/faces in a readable form."
        )

    # src(result): 기존대로 triangle mesh로 읽어서 컬러/ply/뷰어 유지
    result_mesh = o3d.io.read_triangle_mesh(str(src_path))
    result_points = np.asarray(result_mesh.vertices)
    if result_points.size == 0:
        raise ValueError(f"[ERROR] src(result) has 0 vertices: {src_path}")


    print(f"[INFO] dst(scan) : '{dst_path}' vertices: {len(scan_points)}")
    print(f"[INFO] src(result): '{src_path}' vertices: {len(result_points)}")

    # -------------------------------------------------
    # 2. KDTree로 최근접 거리 계산 및 저장 (미터 → 밀리미터)
    # -------------------------------------------------
    kdtree = cKDTree(scan_points)
    distances_m = kdtree.query(result_points, k=1)[0]  # shape=(M,)
    distances_mm = distances_m * 1000                  # mm

    min_mm = distances_mm.min()
    max_mm = distances_mm.max()
    avg_mm = distances_mm.mean()

    print(f"Avg distance : {avg_mm:.4f} mm")
    print(f"Max distance : {max_mm:.4f} mm")
    print(f"Min distance : {min_mm:.4f} mm")

    out_dir.mkdir(parents=True, exist_ok=True)

    result_txt = out_dir / f"{subject}_distance_{tag}.txt"
    with open(result_txt, "w", encoding="utf-8") as file:
        file.write(f"Distance validation result\n")
        file.write(f"dst(scan)  : {dst_path}\n")
        file.write(f"src(result): {src_path}\n\n")
        file.write(f"Avg distance : {avg_mm:.4f} mm\n")
        file.write(f"Max distance : {max_mm:.4f} mm\n")
        file.write(f"Min distance : {min_mm:.4f} mm\n")

    # -------------------------------------------------
    # 3. 거리 기반 컬러맵으로 result_mesh에 버텍스 컬러 할당
    # -------------------------------------------------
    cmap = plt.get_cmap("rainbow")
    norm = plt.Normalize(vmin=min_mm, vmax=max_mm)
    # RGBA -> RGB
    vertex_colors = cmap(norm(distances_mm))[:, :3]  # (M,3)
    result_mesh.vertex_colors = o3d.utility.Vector3dVector(vertex_colors)

    # -------------------------------------------------
    # 4. 결과 저장: PLY 파일로 저장
    # -------------------------------------------------
    out_ply = out_dir / f"{subject}_distance_colored_{tag}.ply"
    o3d.io.write_triangle_mesh(str(out_ply), result_mesh)
    print(f"[INFO] Saved colored mesh: {out_ply}")

    # -------------------------------------------------
    # 5. Open3D 뷰어: 메시 시각화
    #    (창을 닫아야 다음 코드로 진행됨)
    # -------------------------------------------------
    print("[INFO] --- Open3D Viewer ---")
    print("[INFO] --- Close the window to continue to the Matplotlib step. ---")
    o3d.visualization.draw_geometries([result_mesh])

    # -------------------------------------------------
    # 6. Matplotlib로 3D 메시 + 컬러바 표시
    # -------------------------------------------------
    faces = np.asarray(result_mesh.triangles)       # (F,3)
    vertices = np.asarray(result_mesh.vertices)     # (M,3)
    colors = np.asarray(result_mesh.vertex_colors)  # (M,3)

    polygons = []
    face_colors = []
    for tri in faces:
        i, j, k = tri
        # (x, y, z) 좌표
        polygons.append([vertices[i], vertices[j], vertices[k]])
        # 면 색상: 세 버텍스 컬러의 평균 (Flat Shading)
        avg_col = (colors[i] + colors[j] + colors[k]) / 3.0
        face_colors.append(avg_col)
    face_colors = np.array(face_colors)  # (F,3)

    fig = plt.figure(figsize=(8,6))
    ax = fig.add_subplot(111, projection='3d')

    poly3d = Poly3DCollection(polygons, facecolors=face_colors, edgecolors=None)
    ax.add_collection3d(poly3d)

    # 축 범위 설정
    x_min, y_min, z_min = vertices.min(axis=0)
    x_max, y_max, z_max = vertices.max(axis=0)
    ax.set_xlim(x_min, x_max)
    ax.set_ylim(y_min, y_max)
    ax.set_zlim(z_min, z_max)
    ax.set_box_aspect((x_max - x_min, y_max - y_min, z_max - z_min))

    # 카메라 각도 (예: Y축이 위로 보이도록)
    ax.view_init(elev=120, azim=-170, roll=-80)

    # 축/그리드를 가리고 싶다면:
    ax.set_axis_off()

    # ---- 여기서 제목 추가 ----
    ax.set_title(f"Result Mesh (distance, {tag})")

    # -------------------------------------------------
    # 6. 컬러바
    # -------------------------------------------------
    sm = ScalarMappable(norm=norm, cmap=cmap)
    sm.set_array([])  # 실제 데이터 없이도 가능
    # 'ax=ax'를 꼭 지정해야 3D축에 컬러바가 붙으면서 오류가 안 남
    cbar = fig.colorbar(sm, ax=ax, fraction=0.03, pad=0.05)
    cbar.set_label("Distance (mm)")

    # -------------------------------------------------
    # 7. 결과 이미지 저장 (옵션)
    # -------------------------------------------------
    out_png = out_dir / f"{subject}_distance_img_{tag}.png"
    plt.savefig(str(out_png), dpi=300, bbox_inches="tight")
    print(f"[INFO] Saved image: {out_png}")

    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--obj", required=True,
                        help="Input scan OBJ name or path. e.g., P002.obj or scan_files/P002.obj or P002")
    parser.add_argument("--gender", required=True, choices=["m", "f", "male", "female"],
                        help="Gender tag to select result mesh (m/f)")
    parser.add_argument("--scan_root", default="scan_files",
                        help="Directory for scan OBJs (dst). Default: scan_files")
    parser.add_argument("--results_root", default="results",
                        help="Directory for fitted results (src). Default: results")

    args = parser.parse_args()

    subject, dst_path, src_path, out_dir, gender_tag = resolve_paths(
        args.obj, args.gender, scan_root=args.scan_root, results_root=args.results_root
    )

    print(f"[INFO] subject    : {subject}")
    print(f"[INFO] dst(scan)  : {dst_path}")
    print(f"[INFO] src(result): {src_path}")
    main(subject, dst_path, src_path, out_dir, gender_tag)