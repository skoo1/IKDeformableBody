import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
from matplotlib.cm import ScalarMappable

def main(subject, dst_path, src_path):
    # -------------------------------------------------
    # 1. 두 OBJ 메시 불러오기 (SMPL = 참값, result = 계산값)
    # -------------------------------------------------
    smpl_mesh = o3d.io.read_triangle_mesh(dst_path)   # 예: random_smpl.obj
    result_mesh = o3d.io.read_triangle_mesh(src_path) # 예: result.obj

    smpl_points = np.asarray(smpl_mesh.vertices)     # (N,3)
    result_points = np.asarray(result_mesh.vertices) # (M,3)
    print(f"[INFO] '{dst_path}' 버텍스 개수: {len(smpl_points)}")
    print(f"[INFO] '{src_path}' 버텍스 개수: {len(result_points)}")

    # -------------------------------------------------
    # 2. KDTree로 최근접 거리 계산 및 저장 (미터 → 밀리미터)
    # -------------------------------------------------
    kdtree = cKDTree(smpl_points)
    distances_m = kdtree.query(result_points, k=1)[0]  # shape=(M,)
    distances_mm = distances_m * 1000                  # mm

    min_mm = distances_mm.min()
    max_mm = distances_mm.max()
    avg_mm = distances_mm.mean()

    print(f"Avg distance : {avg_mm:.4f} mm")
    print(f"Max distance : {max_mm:.4f} mm")
    print(f"Min distance : {min_mm:.4f} mm")

    result_path =f"results/{subject}/{subject}_distance_{model}.txt"
    with open(result_path, 'w') as file:
        file.write(f"random_smpl{model}.obj validation result\n")
        file.write(f"Avg distance : {avg_mm:.4f} mm\n")
        file.write(f"Max distance : {max_mm:.4f} mm\n")
        file.write(f"Min distance : {min_mm:.4f} mm")

    # -------------------------------------------------
    # 3. 거리 기반 컬러맵으로 result_mesh에 버텍스 컬러 할당
    # -------------------------------------------------
    cmap = plt.get_cmap("rainbow")
    norm = plt.Normalize(vmin=min_mm, vmax=max_mm)
    # norm = plt.Normalize(vmin=0.0, vmax=3.0)
    # RGBA -> RGB
    vertex_colors = cmap(norm(distances_mm))[:, :3]  # (M,3)
    result_mesh.vertex_colors = o3d.utility.Vector3dVector(vertex_colors)

    # -------------------------------------------------
    # 4. 결과 저장: PLY 파일로 저장
    # -------------------------------------------------
    o3d.io.write_triangle_mesh(f"results/{subject}/{subject}_distance_colored_{model}.ply", result_mesh)
    print("[INFO] 컬러가 입혀진 result_colored.ply 저장 완료.")

    # -------------------------------------------------
    # 5. Open3D 뷰어: 메시 시각화
    #    (창을 닫아야 다음 코드로 진행됨)
    # -------------------------------------------------
    print("[INFO] --- Open3D Viewer 실행. ---")
    print("[INFO] --- 창 닫으면 Matplotlib 단계로 넘어갑니다. ---")
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
    ax.set_title("Result Mesh")

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
    plt.savefig(f"./results/{subject}/{subject}_distance_img_{model}.png", dpi=300, bbox_inches='tight')

    print("[INFO] --- Matplotlib 창을 띄웁니다. ---")
    plt.show()

if __name__ == "__main__":
    for model in ['RecFusion'] :     # ['Scanner', 'RecFusion']
        subject = "P002"
        dst_path = f"dst/{subject}_{model}.obj"
        src_path = f"results/{subject}/{subject}_{model}.obj"
        main(subject, dst_path, src_path)
